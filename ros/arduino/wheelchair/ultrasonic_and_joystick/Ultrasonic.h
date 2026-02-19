/*
  Ultrasonic.h - Library for HR-SC04 Ultrasonic Ranging Module.
  Created by ITead studio. Alex, Apr 20, 2010.
  iteadstudio.com
  Ed by RoboFob
  LP 08.11.2015
*/


#ifndef Ultrasonic_h
#define Ultrasonic_h

#include <Arduino.h>

#define CM 1
#define INC 0

class Ultrasonic
{
  public:
    // TP - Trigger Pin
    // EP - Echo Pin
    // tm - pulseIn timeout in microsec
    //    defailt value is 100000 us (0.1 sec)
    Ultrasonic(int TP, int EP, unsigned long tm = 100000);
    long Timing();
    long Ranging(int sys);

  private:
    int Trig_pin;
    int Echo_pin;
    long duration, distacne_cm, distance_inc;
    unsigned long timeout;
};

#endif
