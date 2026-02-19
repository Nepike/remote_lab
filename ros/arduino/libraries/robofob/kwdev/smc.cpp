/*
 * Simple Motor Controller
 * 22.08.2018
 * LP 22.08.2018
 */
 
#include <Arduino.h>
#include "smc.h"

//------------------------------------------------------------------------------
// Диапазон скоростей
#define speed_min   0
#define speed_max 100

void Tsmc::Go(byte device, int sp)
{
  // speed should be a number from -3200 to 3200
  int speed = map(abs(sp), speed_min, speed_max, 0, 3200);

  CWrite(0xAA);
  CWrite(device);

  if (sp < 0)
    CWrite(0x06); // motor reverse command
  else
    CWrite(0x05); // motor forward command
  CWrite(speed & 0x1F);
  CWrite(speed >> 5);
}

