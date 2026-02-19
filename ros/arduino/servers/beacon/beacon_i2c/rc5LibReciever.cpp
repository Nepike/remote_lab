  /*
   * ===================================================
   * ===================================================
   * MODIFIED BY robofob/asanmalyshev(Александр Малышев)
   * ===================================================
   * ===================================================
   *
   * IRremote
   * Version 0.11 August, 2009
   * Copyright 2009 Ken Shirriff
   * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
   *
   * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
   * Modified  by Mitra Ardron <mitra@mitra.biz> 
   * Added Sanyo and Mitsubishi controllers
   * Modified Sony to spot the repeat codes that some Sony's send
   *
   * Interrupt code based on NECIRrcv by Joe Knapp
   * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
   * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
   *
   */

  #include "rc5LibReciever.h"
  #include "rc5LibRecieverInterupts.h"

  // Provides ISR
  #include <avr/interrupt.h>

#ifndef rc5libint_sndr_h
  volatile irparams_t irparams[NUMRECV];  
#endif

  IRrecv::IRrecv(void)
  {
  }

  IRrecv::IRrecv(int n, int recvpin)
  {
    SetPin(n, recvpin);
  }

  void IRrecv::SetPin(int n, int recvpin)
  {
    num = n;
    irparams[num].recvpin = recvpin;
    irparams[num].blinkflag = 0;

    // initialize state machine variables
    irparams[num].rcvstate = STATE_IDLE;
    irparams[num].rawlen = 0;

    // set pin modes
    pinMode(irparams[num].recvpin, INPUT);
  }


  // initialization
  void enableIRIn(void) 
  {
    cli();
    // setup pulse clock timer interrupt
    //Prescale /8 (16M/8 = 0.5 microseconds per tick)
    // Therefore, the timer interval can range from 0.5 to 128 microseconds
    // depending on the reset value (255 to 0)
    TIMER_CONFIG_NORMAL();

    //Timer2 Overflow Interrupt Enable
    TIMER_ENABLE_INTR;

    TIMER_RESET;

    sei();  // enable interrupts

  }

  // TIMER2 interrupt code to collect raw data.
  // Widths of alternating SPACE, MARK are recorded in rawbuf.
  // Recorded in ticks of 50 microseconds.
  // rawlen counts the number of entries recorded so far.
  // First entry is the SPACE between transmissions.
  // As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
  // As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts
  ISR(TIMER_INTR_NAME)
  {
    TIMER_RESET;

    uint8_t irdata;

    for(int i=0;i<NUMRECV;i++)
    {
      irdata = (uint8_t)digitalRead(irparams[i].recvpin);

      irparams[i].timer++; // One more 50us tick
      if (irparams[i].rawlen >= RAWBUF) {
        // Buffer overflow
        irparams[i].rcvstate = STATE_STOP;
      }
      switch(irparams[i].rcvstate) 
      {
      case STATE_IDLE: // In the middle of a gap
        if (irdata == MARK) {
          if (irparams[i].timer < GAP_TICKS) {
            // Not big enough to be a gap.
            irparams[i].timer = 0;
          } 
          else {
            // gap just ended, record duration and start recording transmission
            irparams[i].rawlen = 0;
            irparams[i].rawbuf[irparams[i].rawlen++] = irparams[i].timer;
            irparams[i].timer = 0;
            irparams[i].rcvstate = STATE_MARK;
          }
        }
        break;
      case STATE_MARK: // timing MARK
        if (irdata == SPACE) {   // MARK ended, record time
          irparams[i].rawbuf[irparams[i].rawlen++] = irparams[i].timer;
          irparams[i].timer = 0;
          irparams[i].rcvstate = STATE_SPACE;
        }
        break;
      case STATE_SPACE: // timing SPACE
        if (irdata == MARK) { // SPACE just ended, record it
          irparams[i].rawbuf[irparams[i].rawlen++] = irparams[i].timer;
          irparams[i].timer = 0;
          irparams[i].rcvstate = STATE_MARK;
        } 
        else { // SPACE
          if (irparams[i].timer > GAP_TICKS) {
            // big SPACE, indicates gap between codes
            // Mark current code as ready for processing
            // Switch to STOP
            // Don't reset timer; keep counting space width
            irparams[i].rcvstate = STATE_STOP;
          } 
        }
        break;
      case STATE_STOP: // waiting, measuring gap
        if (irdata == MARK) { // reset gap timer
          irparams[i].timer = 0;
        }
        break;
      }
    }
  }

  void IRrecv::resume() 
  {
    irparams[num].rcvstate = STATE_IDLE;
    irparams[num].rawlen = 0;
  }


  // Decodes the received IR message
  // Returns 0 if no data ready, 1 if data ready.
  // Results of decoding are stored in results
  int IRrecv::decode(decode_results *results) 
  {
    results->rawbuf = irparams[num].rawbuf;
    results->rawlen = irparams[num].rawlen;
    if (irparams[num].rcvstate != STATE_STOP) 
    {
      return ERR;
    }

    if (decodeRC5(results)) 
    {
      results->value= results->value >> 1;
      return DECODED ;
    }

    resume();
    return ERR;
  }


  // Gets one undecoded level at a time from the raw buffer.
  // The RC5/6 decoding is easier if the data is broken into time intervals.
  // E.g. if the buffer has MARK for 2 time intervals and SPACE for 1,
  // successive calls to getRClevel will return MARK, MARK, SPACE.
  // offset and used are updated to keep track of the current position.
  // t1 is the time interval for a single bit in microseconds.
  // Returns -1 for error (measured time interval is not a multiple of t1).
  int IRrecv::getRClevel(decode_results *results, int *offset, int *used, int t1) {
    if (*offset >= results->rawlen) {
      // After end of recorded buffer, assume SPACE.
      return SPACE;
    }
    int width = results->rawbuf[*offset];
    int val = ((*offset) % 2) ? MARK : SPACE;
    int correction = (val == MARK) ? MARK_EXCESS : - MARK_EXCESS;

    int avail;
    if (MATCH(width, t1 + correction)) {
      avail = 1;
    }
    else if (MATCH(width, 2*t1 + correction)) {
      avail = 2;
    }
    else if (MATCH(width, 3*t1 + correction)) {
      avail = 3;
    } 
    else {
      return -1;
    }

    (*used)++;
    if (*used >= avail) {
      *used = 0;
      (*offset)++;
    }
    return val;
  }

  long IRrecv::decodeRC5(decode_results *results) {
    if (irparams[num].rawlen < MIN_RC5_SAMPLES + 2) {
      return ERR;
    }
    int offset = 1; // Skip gap space
    long data = 0;
    int used = 0;
    // Get start bits
    if (getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
    if (getRClevel(results, &offset, &used, RC5_T1) != SPACE) return ERR;
    if (getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
    int nbits;
    for (nbits = 0; offset < irparams[num].rawlen; nbits++) {
      int levelA = getRClevel(results, &offset, &used, RC5_T1); 
      int levelB = getRClevel(results, &offset, &used, RC5_T1);
      if (levelA == SPACE && levelB == MARK) {
        // 1 bit
        data = (data << 1) | 1;
      } 
      else if (levelA == MARK && levelB == SPACE) {
        // zero bit
        data <<= 1;
      } 
      else {
        return ERR;
      }
    }

    // Success
    results->bits = nbits;
    results->value = data;
    results->decode_type = RC5;
    return DECODED;
  }




