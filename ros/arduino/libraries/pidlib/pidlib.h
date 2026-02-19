/**
 *  Progect:
 *    ПИД-регулирование
 *  Author: Robofob
 *  Version 1.04
 *  Date: 23.02.2014
 *  24.08.2017
 *  Last Change: 20.08.2021

 * Технология настройки ПИД-регулятора:
 *   1. Задать параметр pidlib::maxU (=250)
 *   2. Запустить процедуру, которая определит  максимальную скорость движения pidlib::maxSpeed
 *      Например, это может быть процедура DefineMaxSpeed
 *   3. ...
 *   4. ...
 */

#ifndef _PID_LIB_H_
#define _PID_LIB_H_

//----------------------------------------------------------
// ПИД-регулятор
//----------------------------------------------------------

struct TPID
{
  float Kp; // Коэффициенты П, И и Д - звеньев
  float Ki;
  float Kd;

  float integral;
  float
    min_integral,
    max_integral;

  float minU, maxU; // Минимальное и максимальное управляющее воздействие
  float maxErr;     // Максимальная ошибка

  // Временные переменые, хранящие предыдущие значения всяких разных сигналов
  float pred_err;
  float pred_u;
  int ct;
  void Init(float Kp0, float Ki0, float Kd0, float _nmini, float _nmaxi, float _nminU, float _nmaxU, float _nmaxErr)
  {
    Kp = Kp0;
    Ki = Ki0;
    Kd = Kd0;
    max_integral = _nmaxi;
    min_integral = _nmini;
    minU = _nminU;
    maxU = _nmaxU;
    maxErr = _nmaxErr;
    Reset();
  }
  void SetParams(float Kp0, float Ki0, float Kd0)
  {
    Kp = Kp0;
    Ki = Ki0;
    Kd = Kd0;
    Reset();
  }
  void Reset(void)
  {
    integral = 0;
    pred_err = 0;
    pred_u = 0;
    ct = 0;
  }
  TPID(void)
  {
    Init(0, 0, 0, 0, 0, 0, 0, 0);
  }
  TPID(float Kp0, float Ki0, float Kd0, float _nmini, float _nmaxi, float _nminU, float _nmaxU, float _nmaxErr)
  {
    Init(Kp0, Ki0, Kd0, _nmini, _nmaxi, _nminU, _nmaxU, _nmaxErr);
  };
  float Eval(float err); // Выход ПИД-регулятора
  float y2u(float y);    // Вычисление управляющего воздействия
};

namespace pidlib
{
// Минимальное и максимальное управление (величина от -255 до 255)
extern float minU;
extern float maxU;

extern float minSpeed; // Скорость трогания
extern float maxSpeed; // Максимальная скорость движения. Величина определяется с помощью процедуры DefineMaxSpeed
extern float maxErr;

// Максимальная "энкодерная скорость"
// Устанавливается эмпирически. Очень слабое место
// Требуются предварительные эксперименты
extern float MAX_ENC_SPEED;
};

//----------------------------------------------------------
// ПИД-регулятор-2
//----------------------------------------------------------

struct TPID2
{
  float Kp; // Коэффициенты П, И и Д - звеньев
  float Ki;
  float Kd;

  float integral;
  float minU, maxU; // Минимальное и максимальное управляющее воздействие

  // Временные переменые, хранящие предыдущие значения всяких разных сигналов
  float pred_err;
  float pred_u;
  void Init(float Kp0, float Ki0, float Kd0, float _nminU, float _nmaxU)
  {
    Kp = Kp0;
    Ki = Ki0;
    Kd = Kd0;
    minU = _nminU;
    maxU = _nmaxU;
    Reset();
  }
  void SetParams(float Kp0, float Ki0, float Kd0)
  {
    Kp = Kp0;
    Ki = Ki0;
    Kd = Kd0;
    Reset();
  }
  void Reset(void)
  {
    integral = 0;
    pred_err = 0;
    pred_u = 0;
  }
  TPID2(void)
  {
    Init(0, 0, 0, 0, 0);
  }
  TPID2(float Kp0, float Ki0, float Kd0, float _nminU, float _nmaxU)
  {
    Init(Kp0, Ki0, Kd0, _nminU, _nmaxU);
  };
  float Eval(float err); // Выход ПИД-регулятора. Ошибка err - величина в диапазоне [-1,1]
  float y2u(float y);    // Вычисление управляющего воздействия
};

#endif
