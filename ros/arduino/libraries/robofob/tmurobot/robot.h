/** \file robot.h
 * Progect:
 * Робот с двумя ПИД-регуляторами
 * \author Robofob
 * \version 1.10
 *
 * \date 23.02.2014
 * \date Last Change: 29.06.2020
 *
 */

#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <Arduino.h>
#include <math.h>

#include <TimerOne.h>
#include <EEPROM.h>
#include "pidlib.h"

extern HardwareSerial &PSERIAL;

/**
 * Счетчики
 * Обработчики прерываний
 */

// Простые счетчики энкодеров. На управление не влияют.
// В функции doEncoder() тупо увеличивают свое значение. При этом
// пытаются определять направление движения колес.

union TEncoderCnt
{
  int val;
  byte d[2];
};

namespace stdrobot
{
extern volatile unsigned char debug_output;

extern volatile TEncoderCnt
  GlobalEncoderLeftCnt,
  GlobalEncoderRightCnt;

extern volatile int
  RealSpeedLeft,
  RealSpeedRight;

extern volatile int T_WCNT;     /**< Глобальный счетчик времени */

extern volatile int CTIME;      /**Счетчик времени до остановки робота после потери связи**/

// Направление вращения колес. Эти значения должны быть заданы в функциях
// MotorLeftGo(pwmsp) и MotorRightGo(-pwmsp)
extern volatile short int
  dir_left, dir_right;
};


void CtlSpeed(TPID &pid, int goalSpeed, int realSpeed, int motornum);

/**
 * Основной класс TRobot
 *
 */
class TRobot
{
public:
  /// Геометрия робота
  int ecnt;              /**< Количество импульсов энкодеров на 1 оборот колеса */
  float wheel_diameter;  /**< Диаметр колес */
  float rwidth;          /**< Расстояние между колесами */

  TPID pidLeft;
  TPID pidRight;

  void Init(unsigned long tns, int _necnt, float _nwheel_diameter, float _nrwidth, int enc1, int enc2);
  void InitPID(float Kp0, float Ki0, float Kd0, float _nmini, float _nmaxi, float _nminU, float _nmaxU, float _nmaxErr)
  {
    pidLeft.Init(Kp0, Ki0, Kd0, _nmini, _nmaxi, _nminU, _nmaxU, _nmaxErr);
    pidRight.Init(Kp0, Ki0, Kd0, _nmini, _nmaxi, _nminU, _nmaxU, _nmaxErr);
  };

  void SetMotorFunc(void (*_fMotorsInit)(), void (*_fMotorLeftGo)(int), void (*_fMotorRightGo)(int))
  {
    fptrMotorLeftGo = _fMotorLeftGo;
    fptrMotorRightGo = _fMotorRightGo;
    fptrMotorsInit = _fMotorsInit;
    fptrMotorsInit();
  }

  /**
   * Преобразование угла в путь, который должно проделать колесо
   * @param angle угол, град
   * @retval int длина отрезка, мм
   */
  int A2L(float angle) { return abs(angle)*M_PI*rwidth/360.0; };

  /**
   * Преобразование угла в количество импульсов энкодера
   * @param angle угол, град
   * @retval int количество импульсов энкодера
   */
  int A2Cnt(float angle) { return ecnt*fabs(angle)*rwidth/(wheel_diameter*360.0); };

  /**
   * Преобразование длины пути в количество импульсов энкодера
   * @param L расстояние, мм.
   */
  int L2Cnt(float L) { return L*ecnt/(M_PI*wheel_diameter); };

  /**
   * Преобразование числа импульсов энкодера n в расстояние (см.)
   */
  int Cnt2L(int n) { return ((float)n*M_PI*wheel_diameter/ecnt)/10; };

  /**
   * Движение со скоростями speedLeft, speedRight
   * @param speedLeft
   * @param speedRight
   */
  void Go2(int speedLeft, int speedRight)
  {
    CtlSpeed(pidLeft, speedLeft, stdrobot::RealSpeedLeft, 0);
    CtlSpeed(pidRight, speedRight, stdrobot::RealSpeedRight, 1);
  };

  /** Останов робота */
  void Stop(void)
  {
    GoalSpeedLeft = GoalSpeedRight = 0;
    robotStop();  
    pidReset(); 
  };

  /**
   * Прямое управление двигателями
   * Базовые двигательные функции
   * Определяются в основной программе
   * pwmspeed - в диапазоне [-255, +255]
   */
  void (*fptrMotorLeftGo)(int);
  void (*fptrMotorRightGo)(int);
  void (*fptrMotorsInit)();

  void goFwd(int pwmsp) { fptrMotorLeftGo(pwmsp);  fptrMotorRightGo(pwmsp); }
  void goBack(int pwmsp) { fptrMotorLeftGo(-pwmsp); fptrMotorRightGo(-pwmsp); }
  void goLeft(int pwmsp) { fptrMotorLeftGo(0); fptrMotorRightGo(pwmsp); }
  void goFastLeft(int pwmsp) { fptrMotorLeftGo(-pwmsp); fptrMotorRightGo(pwmsp); }
  void goRight(int pwmsp) { fptrMotorLeftGo(pwmsp); fptrMotorRightGo(0); }
  void goFastRight(int pwmsp) { fptrMotorLeftGo(pwmsp); fptrMotorRightGo(-pwmsp); }
  void robotStop(void) { fptrMotorLeftGo(0); fptrMotorRightGo(0); }

  /**
   * Движение с ускорением/торможением
   * @param L длина отрезка, мм
   */
  int Go(int L, int SpeedLeft, int SpeedRight, int (*event)(void));

  /**
   * Поворот с ускорением/торможением
   *
   */
  int Rotate(int A, int Speed, int (*event)(void));

  void pidReset(void) {  pidLeft.reset();  pidRight.reset(); };

  /**
   * @param cnt количество импульсов энкодера
   */
  int Go2Enc(int cnt, int SpeedLeft, int SpeedRight, int (*event)(void));

  /**
   * @param L длина отрезка, мм
   */
  int Go2(int L, int SpeedLeft, int SpeedRight, int (*event)(void));

  int Rotate2(int A, int Speed, int (*event)(void));

  /**
   * Простой тест ПИД-регулятора
   */
  void TestPID(int SpeedLeft, int SpeedRight);

  /**
   * Простой тест энкодеров
   */
  void TestEncoders();

  /**
   * @note Реентерабельные функции движения
   *
   */

  /**
   * Состояния движения
   */
  enum MoveStatus
  {
    MVC_READY = 0,      /**< Движение закончено*/
    MVC_BOOSTING,       /**< Разгон */
    MVC_START_MOVELINE, /**< Подготовка к движению на основном скоростном участке*/
    MVC_MOVELINE,       /**< Движение*/
    MVC_ERROR           /**< Ошибка*/
  };

  /**
   * Подготовка к прямолинейному движению
   * @param L длина отрезка
   * Если L<=0, то едем с заданной скоростью
   */
  void reStartMove(int L, int SpeedLeft, int SpeedRight);

  /**
   * Подготовка к повороту
   * @param A угол поворота в градусах
   * @param Speed скорость
   */
  void reStartRotate(int A, int Speed);

  /**
   * Шаг движения
   * @retval short int состояние движения MoveStatus
   * Вызывается в основном цикле
   */
  short int reMoveStep(void);

public:
  /** Текущее состояние движения  */
  byte mvstatus;
  int GoalSpeedLeft, GoalSpeedRight;
  // Режим прямого задания скорости
  byte set_speed_regime;

private:
  int LCNT;
  int dvLeft, dvRight;
  int vLeft, vRight;
  int ctime;
  byte checkL; // Флаг проверки дистанции (для функций reStartMove и reMoveStep)
};

extern TRobot robot;

namespace stdrobot
{

/**
 * Сервисные функции
 */

/**
 * Определение максимальной скорости при управлении U
 * Функция нужна для определения параметра pidlib::maxSpeed.
 * Для этого нужно вызывть ее с аргументом maxU (TestMaxSpeed(pidlib::maxU))
 * и далее посмотреть в терминале выдаваемые значения. Это и будет величина pidlib::maxSpeed
 */
void DefineMaxSpeed(int U);

// Выводим значение скорости каждые 10 тактов таймера
void ShowCurrSpeed(void);

// Простой тест ПИД-регуляторов
void TestPID(int Speed);

/**
  Тест подключения двигателей
    1. Вперед
    2. Назад
    3. Налево быстро
    4. Направо быстро
*/
void TestDelayMotors(int pwm);

/**
  Комплексный тест ПИД-регуляторов
    1. Go 1000 mm
    2. Rotate 90
    3. Rotate -90
    4. Rotate 180
    5. Rotate 360
*/
void TestComplexPID(int mspeed, int rspeed);
};

#endif
