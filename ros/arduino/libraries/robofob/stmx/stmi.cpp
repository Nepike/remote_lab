/*
 *  Интерфейсные функции к stmdrv
 *  
 *  20.06.2020
 *  LP 04.04.2021
 * 
 */

#include <Arduino.h>
#include <Wire.h> 
#include "stmi.h"

void Set2Speed(signed char sp1, signed char sp2)
{
  byte buff[3];
  buff[0] = byte(2);
  buff[1] = byte(sp1);
  buff[2] = byte(sp2);

  Wire.beginTransmission(ADDR_STM_SERVER);
  Wire.write(buff,3);
  Wire.endTransmission();
}

void Set1Speed(signed char sp, byte n)
{
  byte buff[2];
  buff[0] = n;
  buff[1] = byte(sp);

  Wire.beginTransmission(ADDR_STM_SERVER);
  Wire.write(buff,2);
  Wire.endTransmission();
}

void SetLeftSpeed(signed char sp) { Set1Speed(sp, 0); }
void SetRightSpeed(signed char sp) { Set1Speed(sp, 1); }

//------------------------------------------------------------------------------

void stmMotorLeftGo(int speed)
{
  signed char V = speed;
  if (speed>100) V = 100;
  if (speed<-100) V = -100;
/*
  signed char V = 0;
  if (speed>0) V = 100;
  else
  if (speed<0) V = -100;
*/
  SetLeftSpeed(V);
}

void stmMotorRightGo(int speed)
{
  signed char V = speed;
  if (speed>100) V = 100;
  if (speed<-100) V = -100;
/*
  signed char V = 0;
  if (speed>0) V = 100;
  else
  if (speed<0) V = -100;
*/
  SetRightSpeed(V);
}

void stmMotorsInit(void) { Wire.begin(); }
