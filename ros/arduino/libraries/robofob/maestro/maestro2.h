/*
Project : Управление сервоприводами с помощью 
          контроллера сервоприводов Pololu Mini Maestro
Version : 1.03
Date    : 13.11.2014
LP      : 10.01.2020
Author  :
Company : Robofob Lab
Comments: 

*/
#ifndef _MAESTRO_2_H_
#define _MAESTRO_2_H_

#include <Arduino.h>

// Минимальное и максимальное значение параметра target
// T_MIN =  992 620 608 992
// T_MAX = 2000 3008 2000
extern unsigned int T_MIN;
extern unsigned int T_MAX;

//----------------------------------------------------------

struct TMServo
{

  TMServo(void mputcharPtr(unsigned char c),
          unsigned char mgetcharPtr(void), int amin = 0, int amax = 90)
  {
    mputchar = mputcharPtr;
    mgetchar = mgetcharPtr;

    ANGLE_MIN = amin;
    ANGLE_MAX = amax;
  }

  int SetTarget(unsigned char id_dev, unsigned char channel, unsigned int target);
  int SetAng(unsigned char id_dev, unsigned char channel, byte angle);
  int SetSpeed(unsigned char id_dev, unsigned char channel, unsigned int cspeed);
  int SetAcceleration(unsigned char id_dev, unsigned char channel, unsigned int acceleration);
  int SetPWM(unsigned char id_dev, unsigned int time, unsigned int period);
  int GetPosition(unsigned char id_dev, unsigned char channel);
  int GetMovingState(unsigned char id_dev);
  int GetErrors(unsigned char id_dev);
  int GoHome(unsigned char id_dev);

private:
  void (*mputchar)(unsigned char c);
  unsigned char (*mgetchar)(void);
  // Минимальный и максимальный углы поворота
  int ANGLE_MIN, ANGLE_MAX;

  unsigned int a2t(byte angle);
};

#endif
