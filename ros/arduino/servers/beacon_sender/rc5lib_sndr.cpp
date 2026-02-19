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
 */

#include "rc5lib_sndr.h"
#include "rc5libint_sndr.h"

// Provides ISR
#include <avr/interrupt.h>

volatile irparams_t irparams[NUMRECV];

//------------------------------------------------------------------------------

byte IRsend::getPin(){
  return _pin;
};

IRsend::IRsend(byte timer) {

  _timer=timer; 

  if(_timer==1){
    _pin = TIMER1_PWM_PIN;
  }
  else if (_timer==2){
    _pin = TIMER2_PWM_PIN; 
  }

#if defined(__AVR_ATmega2560__)
  else if (_timer==3){
    _pin = TIMER3_PWM_PIN; 
  }
  else if (_timer==4){
    _pin = TIMER4_PWM_PIN; 
  }
  else if (_timer==5){
    _pin = TIMER5_PWM_PIN;
  }
#endif

}

// Note: first bit must be a one (start bit)
void IRsend::sendRC5(unsigned long data, int nbits)
{
  if(_timer==1){
    IRsend::sendRC51(data, nbits);
  } 
  else if (_timer==2){ 
    IRsend::sendRC52(data, nbits);
  }

#if defined(__AVR_ATmega2560__) 
  else if (_timer==3){
    IRsend::sendRC53(data, nbits);
  }
  else if (_timer==4){
    IRsend::sendRC54(data, nbits);
  }
  else if (_timer==5){
    IRsend::sendRC55(data, nbits);
  }
#endif

}

// ----------------------------------------------------------------------------

// --------------------------------------
// *************** TIMER1 ***************
// --------------------------------------

// Note: first bit must be a one (start bit)
void IRsend::sendRC51(unsigned long data, int nbits)
{
  enableIROut1(36);
  data = data << (32 - nbits);
  mark1(RC5_T1);          // First start bit
  space1(RC5_T1);         // Second start bit
  mark1(RC5_T1);          // Second start bit
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space1(RC5_T1);     // 1 is space, then mark
      mark1(RC5_T1);
    } 
    else {
      mark1(RC5_T1);
      space1(RC5_T1);
    }
    data <<= 1;
  }
  space1(RC5_T1);     // end bit
  mark1(RC5_T1);
  space1(0); // Turn off at end
}

void IRsend::mark1(int time) {
  // Sends an IR mark for the specified number of microseconds.
  // The mark output is modulated at the PWM frequency.
  TIMER1_ENABLE_PWM; // Enable pin PWM output
  delayMicroseconds(time);
}

/* Leave pin off for time (given in microseconds) */
void IRsend::space1(int time) {
  // Sends an IR space for the specified number of microseconds.
  // A space is no output, so the PWM output is disabled.
  TIMER1_DISABLE_PWM; // Disable pin PWM output
  delayMicroseconds(time);
}

void IRsend::enableIROut1(int khz) {
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  // This routine is designed for 36-40KHz; if you use it for other values, it's up to you
  // to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
  // There is no prescaling
  // To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
  // A few hours staring at the ATmega documentation and this will all make sense.
  // See my Secrets of Arduino PWM at http://arcfn.com/2009/07/secrets-of-arduino-pwm.html for details.

  // Disable the Timer Interrupt (which is used for receiving IR)
  TIMER1_DISABLE_INTR; //Timer1 Overflow Interrupt

  pinMode(TIMER1_PWM_PIN, OUTPUT);
  digitalWrite(TIMER1_PWM_PIN, LOW); // When not sending PWM, we want it low

  TIMER1_CONFIG_KHZ(khz);
}


// --------------------------------------
// *************** TIMER2 ***************
// *** Comments are similar to TIMER1 ***
// --------------------------------------

void IRsend::sendRC52(unsigned long data, int nbits)
{
  enableIROut2(36);
  data = data << (32 - nbits);
  mark2(RC5_T1); // First start bit
  space2(RC5_T1); // Second start bit
  mark2(RC5_T1); // Second start bit
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space2(RC5_T1); // 1 is space, then mark
      mark2(RC5_T1);
    } 
    else {
      mark2(RC5_T1);
      space2(RC5_T1);
    }
    data <<= 1;
  }
  space2(RC5_T1);     // end bit
  mark2(RC5_T1);
  space2(0); // Turn off at end
}

void IRsend::mark2(int time) {

  TIMER2_ENABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::space2(int time) {
  TIMER2_DISABLE_PWM;   
  delayMicroseconds(time);
}

void IRsend::enableIROut2(int khz) {

  TIMER2_DISABLE_INTR;

  pinMode(TIMER2_PWM_PIN, OUTPUT);
  digitalWrite(TIMER2_PWM_PIN, LOW);

  TIMER2_CONFIG_KHZ(khz);
}

#if defined(__AVR_ATmega2560__)
// --------------------------------------
// *************** TIMER3 ***************
// *** Comments are similar to TIMER1 ***
// --------------------------------------

void IRsend::sendRC53(unsigned long data, int nbits)
{
  enableIROut3(36);
  data = data << (32 - nbits);
  mark3(RC5_T1); // First start bit
  space3(RC5_T1); // Second start bit
  mark3(RC5_T1); // Second start bit
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space3(RC5_T1); // 1 is space, then mark
      mark3(RC5_T1);
    } 
    else {
      mark3(RC5_T1);
      space3(RC5_T1);
    }
    data <<= 1;
  }
  space3(RC5_T1);     // end bit
  mark3(RC5_T1);
  space3(0); // Turn off at end
}

void IRsend::mark3(int time) {
  
  TIMER3_ENABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::space3(int time) {
  TIMER3_DISABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::enableIROut3(int khz) {

  TIMER3_DISABLE_INTR;

  pinMode(TIMER3_PWM_PIN, OUTPUT);
  digitalWrite(TIMER3_PWM_PIN, LOW);

  TIMER3_CONFIG_KHZ(khz);
}

// --------------------------------------
// *************** TIMER4 ***************
// *** Comments are similar to TIMER1 ***
// --------------------------------------

void IRsend::sendRC54(unsigned long data, int nbits)
{
  enableIROut4(36);
  data = data << (32 - nbits);
  mark4(RC5_T1); // First start bit
  space4(RC5_T1); // Second start bit
  mark4(RC5_T1); // Second start bit
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space4(RC5_T1); // 1 is space, then mark
      mark4(RC5_T1);
    } 
    else {
      mark4(RC5_T1);
      space4(RC5_T1);
    }
    data <<= 1;
  }
  space4(RC5_T1);     // end bit
  mark4(RC5_T1);
  space4(0); // Turn off at end
}

void IRsend::mark4(int time) {
  
  TIMER4_ENABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::space4(int time) {
  TIMER4_DISABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::enableIROut4(int khz) {

  TIMER4_DISABLE_INTR;

  pinMode(TIMER4_PWM_PIN, OUTPUT);
  digitalWrite(TIMER4_PWM_PIN, LOW);

  TIMER4_CONFIG_KHZ(khz);
}

// --------------------------------------
// *************** TIMER5 ***************
// *** Comments are similar to TIMER1 ***
// --------------------------------------

void IRsend::sendRC55(unsigned long data, int nbits)
{
  enableIROut5(36);
  data = data << (32 - nbits);
  mark5(RC5_T1); // First start bit
  space5(RC5_T1); // Second start bit
  mark5(RC5_T1); // Second start bit
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      space5(RC5_T1); // 1 is space, then mark
      mark5(RC5_T1);
    } 
    else {
      mark5(RC5_T1);
      space5(RC5_T1);
    }
    data <<= 1;
  }
  space5(RC5_T1);     // end bit
  mark5(RC5_T1);
  space5(0); // Turn off at end
}

void IRsend::mark5(int time) {
  
  TIMER5_ENABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::space5(int time) {
  TIMER5_DISABLE_PWM; 
  delayMicroseconds(time);
}

void IRsend::enableIROut5(int khz) {

  TIMER5_DISABLE_INTR;

  pinMode(TIMER5_PWM_PIN, OUTPUT);
  digitalWrite(TIMER5_PWM_PIN, LOW);

  TIMER5_CONFIG_KHZ(khz);
}

#endif
