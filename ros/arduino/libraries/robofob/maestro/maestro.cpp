/*
Project : Управление сервоприводами с помощью 
          контроллера сервоприводов Pololu Mini Maestro
Version : 1.02 
Date    : 13.11.2014
Author  :
Company : Robofob Lab
Comments: 

*/

#include "maestro.h"

// Минимальное и максимальное значение параметра target
const unsigned int T_MIN =  620; //  608; // 992
const unsigned int T_MAX = 2000; // 3008; // 2000

inline void mputchar(unsigned char c)
{
  Serial.write(c);
}

unsigned char mgetchar(void)
{
  unsigned char c;
  c = Serial.read();
  return c;
}

//----------------------------------------------------------

unsigned int a2t(byte angle)
{
  unsigned int t;
  //t = (unsigned int)(((long)angle - (long)ANGLE_MIN) * (long)(T_MAX - T_MIN) / ((long)ANGLE_MAX - (long)ANGLE_MIN) + T_MIN);
  t = map(angle, ANGLE_MIN, ANGLE_MAX, T_MIN, T_MAX);
  return t;
}

int maestroSetTarget(unsigned char id_dev, unsigned char channel, unsigned int target)
{   
  target = target*4;
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x04);
  mputchar(channel);          
  mputchar(target & 0x7F);
  mputchar((target >> 7) & 0x7F);
  return 0;
}

int maestroSetAng(unsigned char id_dev, unsigned char channel, byte angle)
{   
  unsigned int target = a2t(angle);
  maestroSetTarget(id_dev, channel, target);
  return 0;
}

//----------------------------------------------------------

int maestroSetSpeed(unsigned char id_dev, unsigned char channel, unsigned int cspeed)
{
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x07);
  mputchar(channel);
  mputchar(cspeed & 0x7F);
  mputchar((cspeed >> 7) & 0x7F);
  return 0;
}

//----------------------------------------------------------

int maestroSetAcceleration(unsigned char id_dev, unsigned char channel, unsigned int acceleration)
{
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x09);
  mputchar(channel);
  mputchar(acceleration & 0x7F);
  mputchar((acceleration >> 7) & 0x7F);
  return 0;
}

//----------------------------------------------------------

int maestroSetPWM(unsigned char id_dev, unsigned int time, unsigned int period)
{
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x0A);
  mputchar(time & 0x7F);
  mputchar(time >> 7 & 0x7F);
  mputchar(period & 0x7F);
  mputchar((period >> 7) & 0x7F);
  return 0;
}

//----------------------------------------------------------

int maestroGetPosition(unsigned char id_dev, unsigned char channel)
{
  unsigned char response[2];
  
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x10);
  mputchar(channel);

  response[0] = mgetchar();
  response[1] = mgetchar();    

  return response[0] + 256*response[1];
}

//----------------------------------------------------------

int maestroGetMovingState(unsigned char id_dev)
{
  unsigned char response;
  
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x13);  
  response = mgetchar();  
  return response;
}

//----------------------------------------------------------

int maestroGetErrors(unsigned char id_dev) 
{
  unsigned char response[2];    

  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x21);

  response[0] = mgetchar();
  response[1] = mgetchar();

  return response[0] + 256*response[1];
}

//----------------------------------------------------------

int maestroGoHome(unsigned char id_dev)
{
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x22);
  return 0;
}
