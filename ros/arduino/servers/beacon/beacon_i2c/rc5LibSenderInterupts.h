/*
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

#ifndef rc5LibSenderInterupts_h
  #define r5LibSenderInterupts_h
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
//   #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif

// #ifndef sbi
//   #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif

#define RC5_T1		889
#define RC5_RPT_LENGTH	46000

#define TOLERANCE 25  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.) 
#define UTOL (1.0 + TOLERANCE/100.) 

// #define _GAP 5000 // Minimum map between transmissions
// #define GAP_TICKS (_GAP/USECPERTICK)

#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))

// #ifndef DEBUG
//   int MATCH(int measured, int desired) {return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
//   int MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
//   int MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}
//   // Debugging versions are in IRremote.cpp
// #endif

// // receiver states
// #define STATE_IDLE     2
// #define STATE_MARK     3
// #define STATE_SPACE    4
// #define STATE_STOP     5

// #ifndef rc5libint_rcv_h
//   // information for the interrupt handler
//   typedef struct {
//     uint8_t recvpin;           // pin for IR data from detector
//     uint8_t rcvstate;          // state machine
//     uint8_t blinkflag;         // TRUE to enable blinking of pin 13 on IR processing
//     unsigned int timer;     // state timer, counts 50uS ticks.
//     unsigned int rawbuf[RAWBUF]; // raw data
//     uint8_t rawlen;         // counter of entries in rawbuf
//   } 
//   irparams_t;

//   #define NUMRECV 4
//   // Defined in IRremote.cpp
//   extern volatile irparams_t irparams[];
// #endif 

// // IR detector output is active low
// #define MARK  0
// #define SPACE 1

#define TOPBIT 0x80000000

// #define MIN_RC5_SAMPLES 11

// ========================================================
// ------------- defines for timer2 (8 bits) ------------- 
// ========================================================

#define TIMER2_RESET
#define TIMER2_ENABLE_PWM     (TCCR2A |= _BV(COM2B1))
#define TIMER2_DISABLE_PWM    (TCCR2A &= ~(_BV(COM2B1)))
#define TIMER2_ENABLE_INTR    (TIMSK2 = _BV(OCIE2A))
#define TIMER2_DISABLE_INTR   (TIMSK2 = 0)
#define TIMER2_INTR_NAME      TIMER2_COMPA_vect
#define TIMER2_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
  TCCR2A = _BV(WGM20); \
  TCCR2B = _BV(WGM22) | _BV(CS20); \
  OCR2A = pwmval; \
  OCR2B = pwmval / 3; \
})
#define TIMER2_COUNT_TOP      (SYSCLOCK * USECPERTICK / 1000000)

#if (TIMER_COUNT_TOP < 256)
  #define TIMER2_CONFIG_NORMAL() ({ \
    TCCR2A = _BV(WGM21); \
    TCCR2B = _BV(CS20); \
    OCR2A = TIMER_COUNT_TOP; \
    TCNT2 = 0; \
  })
#else
  #define TIMER2_CONFIG_NORMAL() ({ \
    TCCR2A = _BV(WGM21); \
    TCCR2B = _BV(CS21); \
    OCR2A = TIMER_COUNT_TOP / 8; \
    TCNT2 = 0; \
  })
#endif

#if defined(CORE_OC2B_PIN)
  #define TIMER2_PWM_PIN        CORE_OC2B_PIN  /* Teensy */
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define TIMER2_PWM_PIN        9  /* Arduino Mega */
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
  #define TIMER2_PWM_PIN        14 /* Sanguino */
#else
  #define TIMER2_PWM_PIN        3  /* Arduino Duemilanove, Diecimila, LilyPad, etc */
#endif

// ========================================================
// ------------- defines for timer1 (16 bits) ------------- 
// ========================================================

#define TIMER1_RESET
#define TIMER1_ENABLE_PWM     (TCCR1A |= _BV(COM1A1))
#define TIMER1_DISABLE_PWM    (TCCR1A &= ~(_BV(COM1A1)))

#if defined(__AVR_ATmega8P__) || defined(__AVR_ATmega8__)
  #define TIMER1_ENABLE_INTR    (TIMSK = _BV(OCIE1A))
  #define TIMER1_DISABLE_INTR   (TIMSK = 0)
#else
  #define TIMER1_ENABLE_INTR    (TIMSK1 = _BV(OCIE1A))
  #define TIMER1_DISABLE_INTR   (TIMSK1 = 0)
#endif

#define TIMER1_INTR_NAME      TIMER1_COMPA_vect
#define TIMER1_CONFIG_KHZ(val) ({ \
  const uint16_t pwmval = SYSCLOCK / 2000 / (val); \
  TCCR1A = _BV(WGM11); \
  TCCR1B = _BV(WGM13) | _BV(CS10); \
  ICR1 = pwmval; \
  OCR1A = pwmval / 3; \
})
#define TIMER1_CONFIG_NORMAL() ({ \
  TCCR1A = 0; \
  TCCR1B = _BV(WGM12) | _BV(CS10); \
  OCR1A = SYSCLOCK * USECPERTICK / 1000000; \
  TCNT1 = 0; \
})

#if defined(CORE_OC1A_PIN)
  #define TIMER1_PWM_PIN        CORE_OC1A_PIN  /* Teensy */
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define TIMER1_PWM_PIN        11  /* Arduino Mega */
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
  #define TIMER1_PWM_PIN        13 /* Sanguino */
#else
  #define TIMER1_PWM_PIN        9  /* Arduino Duemilanove, Diecimila, LilyPad, etc */
#endif

#if defined(__AVR_ATmega2560__)
// ========================================================
// ------------- defines for timer3 (16 bits) ------------- 
// ========================================================

  #define TIMER3_RESET
  #define TIMER3_ENABLE_PWM     (TCCR3A |= _BV(COM3A1))
  #define TIMER3_DISABLE_PWM    (TCCR3A &= ~(_BV(COM3A1)))
  #define TIMER3_ENABLE_INTR    (TIMSK3 = _BV(OCIE3A))
  #define TIMER3_DISABLE_INTR   (TIMSK3 = 0)
  #define TIMER3_INTR_NAME      TIMER3_COMPA_vect
  #define TIMER3_CONFIG_KHZ(val) ({ \
    const uint16_t pwmval = SYSCLOCK / 2000 / (val); \
    TCCR3A = _BV(WGM31); \
    TCCR3B = _BV(WGM33) | _BV(CS30); \
    ICR3 = pwmval; \
    OCR3A = pwmval / 3; \
  })
  #define TIMER3_CONFIG_NORMAL() ({ \
    TCCR3A = 0; \
    TCCR3B = _BV(WGM32) | _BV(CS30); \
    OCR3A = SYSCLOCK * USECPERTICK / 1000000; \
    TCNT3 = 0; \
  })

  #if defined(CORE_OC3A_PIN)
    #define TIMER3_PWM_PIN        CORE_OC3A_PIN  /* Teensy */
  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define TIMER3_PWM_PIN        5  /* Arduino Mega */
  #else
    #error "Please add OC3A pin number here\n"
  #endif

// ========================================================
// ------------- defines for timer4 (16 bits) -------------
// ========================================================

  #define TIMER4_RESET
  #define TIMER4_ENABLE_PWM     (TCCR4A |= _BV(COM4A1))
  #define TIMER4_DISABLE_PWM    (TCCR4A &= ~(_BV(COM4A1)))
  #define TIMER4_ENABLE_INTR    (TIMSK4 = _BV(OCIE4A))
  #define TIMER4_DISABLE_INTR   (TIMSK4 = 0)
  #define TIMER4_INTR_NAME      TIMER4_COMPA_vect
  #define TIMER4_CONFIG_KHZ(val) ({ \
    const uint16_t pwmval = SYSCLOCK / 2000 / (val); \
    TCCR4A = _BV(WGM41); \
    TCCR4B = _BV(WGM43) | _BV(CS40); \
    ICR4 = pwmval; \
    OCR4A = pwmval / 3; \
  })
  #define TIMER4_CONFIG_NORMAL() ({ \
    TCCR4A = 0; \
    TCCR4B = _BV(WGM42) | _BV(CS40); \
    OCR4A = SYSCLOCK * USECPERTICK / 1000000; \
    TCNT4 = 0; \
  })
  #if defined(CORE_OC4A_PIN)
    #define TIMER4_PWM_PIN        CORE_OC4A_PIN
  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define TIMER4_PWM_PIN        6  /* Arduino Mega */
  #else
    #error "Please add OC4A pin number here\n"
  #endif

// ========================================================
// ------------- defines for timer5 (16 bits) ------------- 
// ========================================================

  #define TIMER5_RESET
  #define TIMER5_ENABLE_PWM     (TCCR5A |= _BV(COM5A1))
  #define TIMER5_DISABLE_PWM    (TCCR5A &= ~(_BV(COM5A1)))
  #define TIMER5_ENABLE_INTR    (TIMSK5 = _BV(OCIE5A))
  #define TIMER5_DISABLE_INTR   (TIMSK5 = 0)
  #define TIMER5_INTR_NAME      TIMER5_COMPA_vect
  #define TIMER5_CONFIG_KHZ(val) ({ \
    const uint16_t pwmval = SYSCLOCK / 2000 / (val); \
    TCCR5A = _BV(WGM51); \
    TCCR5B = _BV(WGM53) | _BV(CS50); \
    ICR5 = pwmval; \
    OCR5A = pwmval / 3; \
  })
  #define TIMER5_CONFIG_NORMAL() ({ \
    TCCR5A = 0; \
    TCCR5B = _BV(WGM52) | _BV(CS50); \
    OCR5A = SYSCLOCK * USECPERTICK / 1000000; \
    TCNT5 = 0; \
  })

  #if defined(CORE_OC5A_PIN)
    #define TIMER5_PWM_PIN        CORE_OC5A_PIN
  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define TIMER5_PWM_PIN        46  /* Arduino Mega */
  #else
    #error "Please add OC5A pin number here\n"
  #endif

#endif // __AVR_ATmega2560__
  
#endif // IRremoteint

  
