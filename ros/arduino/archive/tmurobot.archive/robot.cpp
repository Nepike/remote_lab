/** \file robot.cpp
 * Progect:
 * Робот с двумя ПИД-регуляторами
 * \author Robofob
 * \version 1.09
 *
 * \date 23.02.2014
 * \date Last Change: 30.11.2017
 *
 */

#include <Arduino.h>
#include <math.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include "pidlib.h"
#include "robot.h"

//----------------------------------------------------------
//
//----------------------------------------------------------
/**
 * Подключение энкодеров
 */
struct TRobotConnections
{
  int PIN_encoderLeft;
  int PIN_encoderRight;
};

TRobot robot;
TRobotConnections RobotConnections;

/**
 * Счетчики
 */

volatile TEncoderCnt
  stdrobot::GlobalEncoderLeftCnt;

volatile TEncoderCnt
  stdrobot::GlobalEncoderRightCnt;

volatile int
  stdrobot::RealSpeedLeft = 0,
  stdrobot::RealSpeedRight = 0;

volatile short int
  stdrobot::dir_left = 0,
  stdrobot::dir_right = 0;

volatile unsigned char stdrobot::debug_output = 0;

static volatile int
  encoderLeftCnt = 0,
  encoderRightCnt = 0;

static volatile int
  cntSEncLeft = 0,
  cntSEncRight = 0;

/**
 * Обработчики прерываний
 */

/**
 * Системная функция. Обработчик прерывания для энкодеров
 * Не совсем корректно работает: регистрируются изменения сигнала (переходы с 0 на 1 и с 1 на 0),
 * а не сам сигнал, поэтому показания могут быть в 2 раза выше, чем на самом деле
 */
void doEncoder(void)
{
  static byte pred_eLeft = 0;
  static byte pred_eRight = 0;
  byte eLeft = digitalRead(RobotConnections.PIN_encoderLeft);
  byte eRight = digitalRead(RobotConnections.PIN_encoderRight);
  if(eLeft != pred_eLeft)
  {
    pred_eLeft = eLeft;
    encoderLeftCnt++;
    cntSEncLeft++;
    // Пытаемся учесть направление вращения
    if(stdrobot::dir_left>0)
      stdrobot::GlobalEncoderLeftCnt.val++;
    else
      stdrobot::GlobalEncoderLeftCnt.val--;
    
  }
  if(eRight != pred_eRight)
  {
    pred_eRight = eRight;
    encoderRightCnt++;
    cntSEncRight++;    
    // Пытаемся учесть направление вращения
    if(stdrobot::dir_right>0)
      stdrobot::GlobalEncoderRightCnt.val++;
    else
      stdrobot::GlobalEncoderRightCnt.val--;
  }
}


volatile int stdrobot::T_WCNT = 0; /**< Глобальный счетчик времени */

void timerIsr()
{
  stdrobot::T_WCNT++;
  stdrobot::RealSpeedLeft = cntSEncLeft;
  stdrobot::RealSpeedRight = cntSEncRight;
  cntSEncLeft = cntSEncRight = 0;
}

//---------------------------------------------------------------------

void goFwd(int pwmsp)
{
  MotorLeftGo(pwmsp);
  MotorRightGo(pwmsp);
}

void goBack(int pwmsp)
{
  MotorLeftGo(-pwmsp);
  MotorRightGo(-pwmsp);
}

void goLeft(int pwmsp)
{
  MotorLeftGo(0);
  MotorRightGo(pwmsp);
}

void goFastLeft(int pwmsp)
{
  MotorLeftGo(-pwmsp);
  MotorRightGo(pwmsp);
}

void goRight(int pwmsp)
{
  MotorLeftGo(pwmsp);
  MotorRightGo(0);
}

void goFastRight(int pwmsp)
{
  MotorLeftGo(pwmsp);
  MotorRightGo(-pwmsp);
}

void robotStop(void)
{
  MotorLeftGo(0);
  MotorRightGo(0);
}

void CtlSpeed(TPID &pid, int goalSpeed, int realSpeed, int motornum)
{
  if(pid.ct == stdrobot::T_WCNT) return;
  pid.ct = stdrobot::T_WCNT;

  short int sign = (motornum==0) ? stdrobot::dir_left : stdrobot::dir_right;
  float err = goalSpeed - realSpeed;
  //float err = goalSpeed - sign*realSpeed;
  float y = pid.Eval(err);
  float u = pid.y2u(y);

  if(u!=pid.pred_u)
  {
    pid.pred_u = u;
    if(motornum==0)
      MotorLeftGo(u);
    else
      MotorRightGo(u);
  }
}

//----------------------------------------------------------
// Основной класс TRobot. Реализация методов
//----------------------------------------------------------

void TRobot::Init(unsigned long tns, int _necnt, float _nwheel_diameter, float _nrwidth, int enc1, int enc2)
{
  ecnt = _necnt;
  wheel_diameter = _nwheel_diameter;
  rwidth = _nrwidth;

  RobotConnections.PIN_encoderLeft = enc1;
  RobotConnections.PIN_encoderRight = enc2;

  //--------------------------------------------------------
  // Настройка портов
  //--------------------------------------------------------
  // Двигатели
  MotorsInit();

  // Настройка энкодеров
  pinMode(RobotConnections.PIN_encoderLeft, INPUT);
  digitalWrite(RobotConnections.PIN_encoderLeft, HIGH);    // turn on pullup resistor

  pinMode(RobotConnections.PIN_encoderRight, INPUT);
  digitalWrite(RobotConnections.PIN_encoderRight, HIGH);   // turn on pullup resistor

  //--------------------------------------------------------
  // Настройка прерываний
  //--------------------------------------------------------
  // Настройка прерываний для энкодеров
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder, CHANGE);  // encoder pin on interrupt 1 - pin 3

  // Таймер
  // Пример: tns = 50000: таймер будет прерываться с периодом 50'000 микросекунд (или 0.05 сек - или 20Hz)
  Timer1.initialize(tns);
  Timer1.attachInterrupt(timerIsr); // attach the service routine here
  mvstatus = MVC_READY;
  set_speed_regime = 0;

  stdrobot::dir_left = stdrobot::dir_right = 0;
  stdrobot::GlobalEncoderLeftCnt.val = 0;
  stdrobot::GlobalEncoderRightCnt.val = 0;

}

int TRobot::Go2Enc(int cnt, int SpeedLeft, int SpeedRight, int (*event)(void))
// cnt - количество импульсов энкодера
{
  int n;
  int ctime = stdrobot::T_WCNT;

  encoderLeftCnt = encoderRightCnt = 0;
  while((encoderLeftCnt<cnt) && (encoderRightCnt<cnt))
  {
    Go2(SpeedLeft, SpeedRight);
    if(event)
    {
      n = event();
      if(n) return n;
    }
    // Проверка того, что мы вообще поехали. А то скорости может не хватить и мы навсегда останемся в этом цикле
    if(stdrobot::T_WCNT-ctime>10)
      if(encoderLeftCnt == 0 && encoderRightCnt == 0) return -1;
  }
  return 0;
}

int TRobot::Go2(int L, int SpeedLeft, int SpeedRight, int (*event)(void))
// L - длина отрезка
{
  int cnt, n;
  cnt = L2Cnt(L);
  n = Go2Enc(cnt, SpeedLeft, SpeedRight, event);
  return n;
}

inline int sign(int n)
{
  return n>0?1:-1;
}

#define ENABLED_MOVE_INTERVAL 20 // 20
#define vLeft0  pidlib::minSpeed
#define vRight0 pidlib::minSpeed

int TRobot::Go(int L, int cnSpeedLeft, int cnSpeedRight, int (*event)(void))
// Движение с ускорением/торможением
{
  int n;
  int vLeft, vRight;
  int dvLeft, dvRight;
  int predT_WCNT = 0;

  // Ускорение
  int a = pidlib::maxSpeed/10;
  if(a<1) a = 1;

  dvLeft = a*sign(cnSpeedLeft);
  dvRight = a*sign(cnSpeedRight);

  int LCNT, LCNT1, LCNT2, LCNT3;
  int D11, D12, D21, D22, D31, D32;

  LCNT = L2Cnt(L); // Общая длина пути (в тиках энкодера)

  if(stdrobot::debug_output)
  {
    PSERIAL.print("LCNT: "); PSERIAL.print(LCNT);
  }
  int ctime = stdrobot::T_WCNT;
  //----------------------------------------------
  // Участок 1. Разгон
  //----------------------------------------------
  pidReset();
  vLeft = vLeft0*sign(cnSpeedLeft);
  vRight = vRight0*sign(cnSpeedRight);

  encoderLeftCnt = encoderRightCnt = 0;
  // Разгоняетмя и едем, пока не достигнем нужной скорости или не доедем до конца
  while( (encoderLeftCnt<LCNT && encoderRightCnt<LCNT) &&
         (abs(vLeft)<abs(cnSpeedLeft) || abs(vRight)<abs(cnSpeedRight)))
  {
    if(event)
    {
      n = event();
      if(n) return n;
    }
    Go2(vLeft, vRight);
    if(predT_WCNT != stdrobot::T_WCNT)
    {
      predT_WCNT = stdrobot::T_WCNT;
      if(stdrobot::T_WCNT%4==0)
      {
        if(abs(vLeft)<abs(cnSpeedLeft)) vLeft += dvLeft;
        if(abs(vRight)<abs(cnSpeedRight)) vRight += dvRight;
      }
    }
    // Проверка того, что мы вообще поехали. А то скорости может не хватить и мы навсегда останемся в этом цикле
    if(stdrobot::T_WCNT-ctime>ENABLED_MOVE_INTERVAL)
      if(encoderLeftCnt == 0 && encoderRightCnt == 0) return -1;
  }
  D11 = encoderLeftCnt; 
  D12 = encoderRightCnt;
  LCNT1 = max(D11, D12);

  //----------------------------------------------
  // Участок 2. Основной режим
  //----------------------------------------------
  // Вычисляем остаток пути
  LCNT2 = LCNT - LCNT1;
  if(LCNT2<=0)
  {
    Stop();
    return 0;
  }

  n = Go2Enc(LCNT2, cnSpeedLeft, cnSpeedRight, event);
  if(n) return n;
  D21 = encoderLeftCnt; 
  D22 = encoderRightCnt;

  //----------------------------------------------
  // Участок 3. Торможение
  //----------------------------------------------
  // Тормозить не будем

  Stop();

  if(stdrobot::debug_output)
  {
    PSERIAL.print(" (");
    PSERIAL.print(LCNT1); PSERIAL.print(" ");
    PSERIAL.print(LCNT2); PSERIAL.print(" ");
    PSERIAL.print(LCNT3); PSERIAL.println(") ");

    PSERIAL.print("1. ");  PSERIAL.print(D11);  PSERIAL.print(" ");  
    PSERIAL.print(D12); PSERIAL.print("   V: ");  PSERIAL.print(vLeft); 
    PSERIAL.print(" ");  PSERIAL.print(vRight); PSERIAL.println();
    PSERIAL.print("2. ");  PSERIAL.print(D21);  PSERIAL.print(" ");  PSERIAL.print(D22); PSERIAL.println();
    PSERIAL.print("Total: ");  PSERIAL.print(D11+D21+D31);  PSERIAL.print(" ");  PSERIAL.println(D12+D22+D32);
  }
  return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

void TRobot::reStartMove(int L, int cnSpeedLeft, int cnSpeedRight)
{
  checkL = (L>0);
  LCNT = L2Cnt(L); // Общая длина пути (в тиках энкодера)

  GoalSpeedLeft = cnSpeedLeft;
  GoalSpeedRight = cnSpeedRight;

  // Ускорение
  int a = pidlib::maxSpeed/10;
  if(a<1) a = 1;

  dvLeft = a*sign(GoalSpeedLeft);
  dvRight = a*sign(GoalSpeedRight);

  if(stdrobot::debug_output)
  {
    PSERIAL.print("LCNT: "); PSERIAL.print(LCNT);
  }
  ctime = stdrobot::T_WCNT;

  pidReset();
  vLeft = vLeft0*sign(GoalSpeedLeft);
  vRight = vRight0*sign(GoalSpeedRight);
  encoderLeftCnt = encoderRightCnt = 0;
  mvstatus = MVC_BOOSTING;
}

void TRobot::reStartRotate(int A, int Speed)
{
  int L = A2L(A);
  if(A<0) Speed = -Speed;
  reStartMove(L, -Speed, Speed);
}

short int TRobot::reMoveStep(void)
// Шаг движения
{
  static int LCNT2 = 0;
  static int predT_WCNT = 0;
  int cnt1;

  // Режим прямого управления скоростью
  if(set_speed_regime)
  {
    Go2(GoalSpeedLeft, GoalSpeedRight);
    ctime = stdrobot::T_WCNT;
    encoderLeftCnt = encoderRightCnt = 0;
    mvstatus = MVC_READY;
    return mvstatus;
  }

  switch(mvstatus)
  {
    // Участок 1. Разгон
    // Разгоняетмя и едем, пока не достигнем нужной скорости или не доедем до конца
    case MVC_BOOSTING:
      // Проверка того, что мы вообще поехали. А то скорости может не хватить, и мы навсегда останемся в этом цикле
      if(stdrobot::T_WCNT-ctime>ENABLED_MOVE_INTERVAL)
        if((encoderLeftCnt == 0 && encoderRightCnt == 0) && checkL) return MVC_ERROR;

      if( ((encoderLeftCnt<LCNT && encoderRightCnt<LCNT) || !checkL) && 
           (abs(vLeft)<abs(GoalSpeedLeft) || abs(vRight)<abs(GoalSpeedRight)))
      {
        Go2(vLeft, vRight);
        if(predT_WCNT!=stdrobot::T_WCNT)
        {
          predT_WCNT = stdrobot::T_WCNT;
          if(stdrobot::T_WCNT%4==0)
          {
            if(abs(vLeft)<abs(GoalSpeedLeft)) vLeft += dvLeft;
            if(abs(vRight)<abs(GoalSpeedRight)) vRight += dvRight;
          }
        }
      }
      else
      {
        mvstatus = MVC_START_MOVELINE;
        return mvstatus;
      }
      return MVC_BOOSTING;

    // Участок 2. Основной режим. Подготовка
    case MVC_START_MOVELINE:
      if(checkL)
      {
        cnt1 = max(encoderLeftCnt, encoderRightCnt);
        // Вычисляем остаток пути
        LCNT2 = LCNT - cnt1;
        if(LCNT2<=0)
        {
          Stop();
          return MVC_READY;
        }
      }
      ctime = stdrobot::T_WCNT;
      encoderLeftCnt = encoderRightCnt = 0;
      mvstatus = MVC_MOVELINE;
      return mvstatus;

    // Едем
    case MVC_MOVELINE:
      // Проверка того, что мы вообще поехали. А то скорости может не хватить, и мы навсегда останемся в этом цикле
      if(stdrobot::T_WCNT-ctime>ENABLED_MOVE_INTERVAL)
      if((encoderLeftCnt == 0 && encoderRightCnt == 0) && checkL) return MVC_ERROR;
      if((encoderLeftCnt<LCNT2) && (encoderRightCnt<LCNT2) || !checkL)
      {
        Go2(GoalSpeedLeft, GoalSpeedRight);
        return mvstatus;
      }
      else
      {
        checkL = 1;
        Stop();
        mvstatus = MVC_READY;
      }
  }
  return mvstatus;
}

//------------------------------------------------------------------------------

int TRobot::Rotate2(int A, int Speed, int (*event)(void))
{
  int L = A2L(A);
  if(A<0) Speed = -Speed;
  int cn = A2Cnt(A);
  int n = Go2(L, -Speed, Speed, event);
  robotStop();
  return n;
}

int TRobot::Rotate(int A, int Speed, int (*event)(void))
// Поворот с ускорением/торможением
{
  int L = A2L(A);
  if(A<0) Speed = -Speed;
  int n = Go(L, -Speed, Speed, event);
  return n;
}

void TRobot::TestPID(int SpeedLeft, int SpeedRight)
// Простой тест ПИД-регулятора
{
  PSERIAL.print("Test PID regulator at speed = ");
  PSERIAL.print(SpeedLeft);
  PSERIAL.print(" ");
  PSERIAL.println(SpeedRight);

  while(1)
  {
    Go2(SpeedLeft, SpeedRight);
    stdrobot::ShowCurrSpeed();
  }
}

void TRobot::TestEncoders(void)
// Простой тест энкодеров
{
  static int predT_WCNT = 0;

  PSERIAL.println("Manual encoders test");

  //robotStop();

  encoderLeftCnt = encoderRightCnt = 0;
  while(1)
  {
    PSERIAL.print('\r');
    if(stdrobot::T_WCNT%10==0 && predT_WCNT!=stdrobot::T_WCNT)
    {
      predT_WCNT = stdrobot::T_WCNT;
      PSERIAL.print(encoderLeftCnt); PSERIAL.print(" "); PSERIAL.println(encoderRightCnt);
    }
  }
}

//----------------------------------------------------------
// Сервисные функции
//----------------------------------------------------------

void stdrobot::ShowCurrSpeed(void)
// Выводим значение скорости каждые 10 тактов таймера
{
  static int predT_WCNT = -1;
  if(stdrobot::T_WCNT%10==0 && stdrobot::T_WCNT!=predT_WCNT)
  {
    predT_WCNT = stdrobot::T_WCNT;
    PSERIAL.print(stdrobot::RealSpeedLeft); PSERIAL.print(" "); PSERIAL.println(stdrobot::RealSpeedRight);
    PSERIAL.flush();
  }
}

void stdrobot::DefineMaxSpeed(int U)
// Определение максимальной скорости при управлении U
{
  PSERIAL.print("Define max speed at U = ");
  PSERIAL.println(U);

  goFwd(U);
  while(1)
    stdrobot::ShowCurrSpeed();
}

//----------------------------------------------------------
// Тестовые функции высокого уровня
//----------------------------------------------------------

void stdrobot::TestDelayMotors(int pwm)
// Тест подключения двигателей
{
  PSERIAL.print("pwmspeed = ");
  PSERIAL.println(pwm);

  encoderLeftCnt = encoderRightCnt = 0;

  #define TM 1000
  PSERIAL.println("go fwd");
  goFwd(pwm);
  delay(TM);
  PSERIAL.print(encoderLeftCnt); PSERIAL.print(" "); PSERIAL.println(encoderRightCnt);

  PSERIAL.println("go back");
  goBack(pwm);
  delay(TM);

  PSERIAL.println("go Fast left");
  goFastLeft(pwm);
  delay(TM);

  PSERIAL.println("go Fast right");
  goFastRight(pwm);
  delay(TM);

  PSERIAL.println("stop");
  robot.Stop();

  PSERIAL.println("Total cnt: ");
  PSERIAL.print(encoderLeftCnt); PSERIAL.print(" "); PSERIAL.println(encoderRightCnt);
}

void stdrobot::TestComplexPID(int mspeed, int rspeed)
// Комплексный тест ПИД-регуляторов
{
#define DTIME 3000
  int n;

  delay(DTIME);
  PSERIAL.println("\r\nGo 1000 mm:");
  n = robot.Go(1000, mspeed, mspeed, NULL);
  robot.Stop();
  PSERIAL.print("Done: "); PSERIAL.println(n);

  delay(DTIME);
  PSERIAL.println("\r\nRotate 90:");
  n = robot.Rotate(90, rspeed, NULL);
  PSERIAL.print("Done: "); PSERIAL.println(n);

  delay(DTIME);
  PSERIAL.println("\r\nRotate -90:");
  n = robot.Rotate(-90, rspeed, NULL);
  PSERIAL.print("Done: "); PSERIAL.println(n);

  delay(DTIME);
  PSERIAL.println("\r\nRotate 180:");
  n = robot.Rotate(180, rspeed, NULL);
  PSERIAL.print("Done: "); PSERIAL.println(n);

  delay(DTIME);
  PSERIAL.println("\r\nRotate 360:");
  n = robot.Rotate(360, rspeed, NULL);
  PSERIAL.print("Done: "); PSERIAL.println(n);
}

/** @mainpage
 *
 * Реентерабельные процедуры ПИД-регулирвания
 */
