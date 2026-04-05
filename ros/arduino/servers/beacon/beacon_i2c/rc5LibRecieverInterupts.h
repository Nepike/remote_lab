/*
 * ===================================================
 * ===================================================
 * MODIFIED BY robofob/asanmalyshev(Александр Малышев)
 * ===================================================
 * ===================================================
 *
 * IRremote
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */

#ifndef rc5LibRecieverInterupts_h
#define rc5LibRecieverInterupts_h
//#define DEBUG

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifdef F_CPU
#define SYSCLOCK F_CPU     // main Arduino clock
#else
#define SYSCLOCK 16000000  // main Arduino clock
#endif

#define ERR 0
#define DECODED 1

// // defines for setting and clearing register bits
// #ifndef cbi
// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif
// #ifndef sbi
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif

#define RC5_T1		889
#define RC5_RPT_LENGTH	46000


#define TOLERANCE 25  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.) 
#define UTOL (1.0 + TOLERANCE/100.) 

#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)

#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))

#ifndef DEBUG
int MATCH(int measured, int desired) {return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
int MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
int MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}
// Debugging versions are in IRremote.cpp
#endif

// receiver states
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5

#ifndef rc5libint_sndr_h
  // information for the interrupt handler
  typedef struct {
    uint8_t recvpin;           // pin for IR data from detector
    uint8_t rcvstate;          // state machine
    uint8_t blinkflag;         // TRUE to enable blinking of pin 13 on IR processing
    unsigned int timer;     // state timer, counts 50uS ticks.
    unsigned int rawbuf[RAWBUF]; // raw data
    uint8_t rawlen;         // counter of entries in rawbuf
  } 
  irparams_t;

  #define NUMRECV 4
  // Defined in IRremote.cpp
  extern volatile irparams_t irparams[];
#endif

// IR detector output is active low
#define MARK  0
#define SPACE 1

#define TOPBIT 0x80000000

#define MIN_RC5_SAMPLES 11

// defines for timer2 (8 bits)
#define TIMER_RESET
#define TIMER_ENABLE_PWM     (TCCR2A |= _BV(COM2B1))
#define TIMER_DISABLE_PWM    (TCCR2A &= ~(_BV(COM2B1)))
#define TIMER_ENABLE_INTR    (TIMSK2 = _BV(OCIE2A))
#define TIMER_DISABLE_INTR   (TIMSK2 = 0)
#define TIMER_INTR_NAME      TIMER2_COMPA_vect
#define TIMER_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
  TCCR2A = _BV(WGM20); \
  TCCR2B = _BV(WGM22) | _BV(CS20); \
  OCR2A = pwmval; \
  OCR2B = pwmval / 3; \
})
#define TIMER_COUNT_TOP      (SYSCLOCK * USECPERTICK / 1000000)
#if (TIMER_COUNT_TOP < 256)
#define TIMER_CONFIG_NORMAL() ({ \
  TCCR2A = _BV(WGM21); \
  TCCR2B = _BV(CS20); \
  OCR2A = TIMER_COUNT_TOP; \
  TCNT2 = 0; \
})
#else
#define TIMER_CONFIG_NORMAL() ({ \
  TCCR2A = _BV(WGM21); \
  TCCR2B = _BV(CS21); \
  OCR2A = TIMER_COUNT_TOP / 8; \
  TCNT2 = 0; \
})
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define TIMER_PWM_PIN        9  /* Arduino Mega */
#else
  #define TIMER_PWM_PIN        3  /* etc */
#endif

#endif
