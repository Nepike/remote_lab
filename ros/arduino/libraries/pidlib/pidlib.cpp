/**
 *  Progect:
 *    ПИД-регулирование
 *  Author: Robofob inc.
 *  Version 1.04
 *  Date: 23.02.2014
 *  24.08.2017
 *  Last Change: 20.08.2021
 */

#include <Arduino.h>

#include "pidlib.h"

//----------------------------------------------------------
// ПИД-регулятор-1
//----------------------------------------------------------
float pidlib::minU = -255;
float pidlib::maxU =  255;
// Максимальная "энкодерная скорость"
float pidlib::MAX_ENC_SPEED = 350.0;

float pidlib::minSpeed = 0; // Скорость трогания
float pidlib::maxSpeed = 0; // Максимальная скорость движения. Величина определяется с помощью процедуры DefineMaxSpeed
float pidlib::maxErr = 0;


float TPID::Eval(float err)
{
  float y;

  integral = integral + err;   // добавить ошибку в сумму ошибок
  if(integral>max_integral) integral=max_integral;
  if(integral<min_integral) integral=min_integral;

  float rdiff = Kd*(err - pred_err);
  // вычисление управляющего воздействия
  y = (Kp*err + Ki*integral + rdiff);

  pred_err = err; // текущая ошибка стала "прошлой ошибкой" для след. вычисления 

  return y;
}

float TPID::y2u(float y)
{
  float u;
  float maxY = Kp*maxErr + Ki*max_integral + Kd*maxErr;
  float minY = -maxY;

  u = (y-minY)/(maxY-minY)*(maxU-minU)+minU;

  if(u>maxU) u = maxU;
  if(u<minU) u = minU;

  return u;
}

//----------------------------------------------------------
// ПИД-регулятор-2
//----------------------------------------------------------
float TPID2::Eval(float err)
// Ошибка err - величина в диапазоне [-1,1]
{
  float y;

  if(err>1) err = 1;
  if(err<-1) err = -1;

  integral = integral + Ki*err;   // добавить ошибку в сумму ошибок
  if(integral>1) integral = 1;
  if(integral<-1) integral = -1;

  float rdiff = err - pred_err;
  if(rdiff>1) rdiff = 1;
  if(rdiff<-1) rdiff = -1;
  // вычисление управляющего воздействия
  y = (Kp*err + integral + Kd*rdiff);

  pred_err = err; // текущая ошибка стала "прошлой ошибкой" для след. вычисления 

  return y;
}

float TPID2::y2u(float y)
{
  float u;
  float maxY = (Kp + 1.0 + Kd);
  float minY = -maxY;

  u = (y-minY)/(maxY-minY)*(maxU-minU)+minU;

  if(u>maxU) u = maxU;
  if(u<minU) u = minU;

  return u;
}
