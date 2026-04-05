/*
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
 * LP 06.02.2023
 *
 */

#include "rc5lib.h"
#include "rc5libint.h"

// Provides ISR
#include <avr/interrupt.h>

volatile irparams_t irparams[NUMRECV];

//------------------------------------------------------------------------------

void IRsend::sendRaw(unsigned int buf[], int len, int hz)
{
  enableIROut(hz);
  for (int i = 0; i < len; i++) {
    if (i & 1) {
      space(buf[i]);
    } 
    else {
      mark(buf[i]);
    }
  }
  space(0); // Just to be sure
}

// Note: first bit must be a one (start bit)
// PIN 3 is used
void IRsend::sendRC5(unsigned long data, int nbits)
{
  enableIROut(36);
  data = data << (32 - nbits);
  mark(RC5_T1); // First start bit
  space(RC5_T1); // Second start bit
  mark(RC5_T1); // Second start bit
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space(RC5_T1); // 1 is space, then mark
      mark(RC5_T1);
    } 
    else {
      mark(RC5_T1);
      space(RC5_T1);
    }
    data <<= 1;
  }
  space(0); // Turn off at end
}


void IRsend::mark(int time) {
  // Sends an IR mark for the specified number of microseconds.
  // The mark output is modulated at the PWM frequency.
  TIMER_ENABLE_PWM; // Enable pin 3 PWM output
  delayMicroseconds(time);
}

/* Leave pin off for time (given in microseconds) */
void IRsend::space(int time) {
  // Sends an IR space for the specified number of microseconds.
  // A space is no output, so the PWM output is disabled.
  TIMER_DISABLE_PWM; // Disable pin 3 PWM output
  delayMicroseconds(time);
}

void IRsend::enableIROut(int khz) {
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  // The IR output will be on pin 3 (OC2B).
  // This routine is designed for 36-40KHz; if you use it for other values, it's up to you
  // to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
  // TIMER2 is used in phase-correct PWM mode, with OCR2A controlling the frequency and OCR2B
  // controlling the duty cycle.
  // There is no prescaling, so the output frequency is 16MHz / (2 * OCR2A)
  // To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
  // A few hours staring at the ATmega documentation and this will all make sense.
  // See my Secrets of Arduino PWM at http://arcfn.com/2009/07/secrets-of-arduino-pwm.html for details.


  // Disable the Timer2 Interrupt (which is used for receiving IR)
  TIMER_DISABLE_INTR; //Timer2 Overflow Interrupt

  pinMode(TIMER_PWM_PIN, OUTPUT);
  digitalWrite(TIMER_PWM_PIN, LOW); // When not sending PWM, we want it low

  // COM2A = 00: disconnect OC2A
  // COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
  // WGM2 = 101: phase-correct PWM with OCRA as top
  // CS2 = 000: no prescaling
  // The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR2A.
  TIMER_CONFIG_KHZ(khz);
}

//------------------------------------------------------------------------------

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

// enable/disable blinking of pin 13 on IR processing
void IRrecv::blink13(int blinkflag)
{
  irparams[num].blinkflag = blinkflag;
  if (blinkflag)
    pinMode(BLINKLED, OUTPUT);
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
    //sic Что-то происходит при наложении сигналов. Видимо, с памятью. Разбираться лень, поэтому так.
    if (irparams[i].rawlen >= RAWBUF-10)
    //if (irparams[i].rawlen >= RAWBUF)
    {
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

    if (irparams[i].blinkflag) {
      if (irdata == MARK) {
        BLINKLED_ON();  // turn pin 13 LED on
      } 
      else {
        BLINKLED_OFF();  // turn pin 13 LED off
      }
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
    return DECODED;
  }
  // decodeHash returns a hash on any input.
  // Thus, it needs to be last in the list.
  // If you add any decodes, add them before this.
/*** sic */
  if (decodeHash(results))
  {
    return DECODED;
  }
/***/
  // Throw away and start over
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

/* -----------------------------------------------------------------------
 * hashdecode - decode an arbitrary IR code.
 * Instead of decoding using a standard encoding scheme
 * (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
 *
 * The algorithm: look at the sequence of MARK signals, and see if each one
 * is shorter (0), the same length (1), or longer (2) than the previous.
 * Do the same with the SPACE signals.  Hszh the resulting sequence of 0's,
 * 1's, and 2's to a 32-bit value.  This will give a unique value for each
 * different code (probably), for most code systems.
 *
 * http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */

// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
int IRrecv::compare(unsigned int oldval, unsigned int newval)
{
  if (newval < oldval * .8)
  {
    return 0;
  } 
  else if (oldval < newval * .8)
  {
    return 2;
  }
  else
  {
    return 1;
  }
}

// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

/* Converts the raw code values into a 32-bit hash code.
 * Hopefully this code is unique for each button.
 * This isn't a "real" decoding, just an arbitrary value.
 */
long IRrecv::decodeHash(decode_results *results) {
  // Require at least 6 samples to prevent triggering on noise
  if (results->rawlen < 6) {
    return ERR;
  }
  long hash = FNV_BASIS_32;
  for (int i = 1; i+2 < results->rawlen; i++) {
    int value =  compare(results->rawbuf[i], results->rawbuf[i+2]);
    // Add value into the hash
    hash = (hash * FNV_PRIME_32) ^ value;
  }
  results->value = hash;
  results->bits = 32;
  results->decode_type = UNKNOWN;
  return DECODED;
}

