/*
  Ultrasonic.h - Library for HR-SC04 Ultrasonic Ranging Module.
  Created by ITead studio. Alex, Apr 20, 2010.
  iteadstudio.com
  Ed by RoboFob
  V 1.02
  08.11.2015
  LP 19.03.2023
*/


#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <Arduino.h>

#define CM 1
#define INC 0

class TUltrasonic
{
  public:
    // TP - Trigger Pin
    // EP - Echo Pin
    // tm - pulseIn timeout in microsec
    //    defailt value is 100000 us (0.1 sec)
    TUltrasonic(int TP, int EP, unsigned long tm = 100000)
    {
      pinMode(TP,OUTPUT);
      pinMode(EP,INPUT);
      Trig_pin=TP;
      Echo_pin=EP;
      timeout = tm;
    }
    long Timing(void)
    {
      digitalWrite(Trig_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(Trig_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(Trig_pin, LOW);
      duration = pulseIn(Echo_pin, HIGH, timeout);
      return duration;
    }
    long Ranging(int sys)
    {
      Timing();
      distacne_cm = duration / 29 / 2;
      distance_inc = duration / 74 / 2;
      if (sys)
        return distacne_cm;
      else
        return distance_inc;
    }

  private:
    int Trig_pin;
    int Echo_pin;
    long duration, distacne_cm, distance_inc;
    unsigned long timeout;
};

#endif
