/*
Project : Управление сервоприводами с помощью 
          контроллера сервоприводов Pololu Mini Maestro
Version : 1.02 
Date    : 13.11.2014
Author  :
Company : Robofob Lab
Comments: 

*/
#ifndef _MAESTRO_H_
#define _MAESTRO_H_

#include <Arduino.h>

// Минимальный и максимальный углы поворота
#define ANGLE_MIN   0
#define ANGLE_MAX  90 // 90

//----------------------------------------------------------

void mputchar(unsigned char c);

unsigned char mgetchar(void);

//----------------------------------------------------------

int maestroSetTarget(unsigned char id_dev, unsigned char channel, unsigned int target);

int maestroSetAng(unsigned char id_dev, unsigned char channel, byte angle);

int maestroSetSpeed(unsigned char id_dev, unsigned char channel, unsigned int cspeed);

int maestroSetAcceleration(unsigned char id_dev, unsigned char channel, unsigned int acceleration);

int maestroSetPWM(unsigned char id_dev, unsigned int time, unsigned int period);

int maestroGetPosition(unsigned char id_dev, unsigned char channel);

int maestroGetMovingState(unsigned char id_dev);

int maestroGetErrors(unsigned char id_dev);

int maestroGoHome(unsigned char id_dev);

#endif
