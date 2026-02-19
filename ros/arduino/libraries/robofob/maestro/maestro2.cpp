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

#include "maestro2.h"

// Минимальное и максимальное значение параметра target
unsigned int T_MIN =  992; // 620 608 992
unsigned int T_MAX = 2000; // 3008 2000

//----------------------------------------------------------

unsigned int TMServo::a2t(byte angle)
{
  unsigned int t;
  //t = (unsigned int)(((long)angle - (long)ANGLE_MIN) * (long)(T_MAX - T_MIN) / ((long)ANGLE_MAX - (long)ANGLE_MIN) + T_MIN);
  t = map(angle, ANGLE_MIN, ANGLE_MAX, T_MIN, T_MAX);
  return t;
}

int TMServo::SetTarget(unsigned char id_dev, unsigned char channel, unsigned int target)
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

int TMServo::SetAng(unsigned char id_dev, unsigned char channel, byte angle)
{   
  unsigned int target = a2t(angle);
  SetTarget(id_dev, channel, target);
  return 0;
}

//----------------------------------------------------------

int TMServo::SetSpeed(unsigned char id_dev, unsigned char channel, unsigned int cspeed)
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

int TMServo::SetAcceleration(unsigned char id_dev, unsigned char channel, unsigned int acceleration)
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

int TMServo::SetPWM(unsigned char id_dev, unsigned int time, unsigned int period)
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

int TMServo::GetPosition(unsigned char id_dev, unsigned char channel)
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

int TMServo::GetMovingState(unsigned char id_dev)
{
  unsigned char response;
  
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x13);  
  response = mgetchar();  
  return response;
}

//----------------------------------------------------------

int TMServo::GetErrors(unsigned char id_dev) 
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

int TMServo::GoHome(unsigned char id_dev)
{
  mputchar(0xAA);
  mputchar(id_dev);
  mputchar(0x22);
  return 0;
}
