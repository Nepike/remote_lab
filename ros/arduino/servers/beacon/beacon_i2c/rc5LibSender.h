/*
 * IRremote
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.htm http://arcfn.com
 * Edited by Mitra to add new controller SANYO
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */

#ifndef rc5LibSender_h
#define rc5LibSender_h

#include <Arduino.h>

// The following are compile-time library options.
// If you change them, recompile the library.
// If DEBUG is defined, a lot of debugging output will be printed during decoding.
// TEST must be defined for the IRtest unittests to work.  It will make some
// methods virtual, which will be slightly slower, which is why it is optional.
// #define DEBUG
// #define TEST


// Only used for testing; can remove virtual for shorter code
#ifdef TEST
#define VIRTUAL virtual
#else
#define VIRTUAL
#endif


class IRsend
{
  public:
    IRsend(byte timer);
    void sendRC5(unsigned long data, int nbits); 
    byte getPin(); 

  private:
    byte _timer;
    byte _pin;
     
    // TIMER1 
    void sendRC51(unsigned long data, int nbits);
    void enableIROut1(int khz);
    VIRTUAL void mark1(int usec);
    VIRTUAL void space1(int usec);

    // TIMER2
    void sendRC52(unsigned long data, int nbits);
    void enableIROut2(int khz);
    VIRTUAL void mark2(int usec);
    VIRTUAL void space2(int usec);

#if defined(__AVR_ATmega2560__)
    // TIMER3  
    void sendRC53(unsigned long data, int nbits);
    void enableIROut3(int khz);
    VIRTUAL void mark3(int usec);
    VIRTUAL void space3(int usec);

    // TIMER4
    void sendRC54(unsigned long data, int nbits);
    void enableIROut4(int khz);
    VIRTUAL void mark4(int usec);
    VIRTUAL void space4(int usec);

    // TIMER5
    void sendRC55(unsigned long data, int nbits);
    void enableIROut5(int khz);
    VIRTUAL void mark5(int usec);
    VIRTUAL void space5(int usec);
#endif  
    
};

// Some useful constants

#define USECPERTICK 50  // microseconds per clock interrupt tick
#define RAWBUF 100 // Length of raw duration buffer

// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
// #define MARK_EXCESS 100

#endif // IRremote_h
