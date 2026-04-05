 /**
 * \file lpctl.ino
 * Progect:
 *  Робот с колесными модулями IG-42GM
 *  DualVNH5019MotorShield
 *  Протокол RCX2
 *  Baudrate 57600/115200
 *  Все значения датчиков считываются в 8-разрядном формате
 *  Внимание! Инициализация системы занимает очень много времени - порядка 5 секунд.
 *  Chip ATmega328
 *  \author Robofob
 *  \version 1.16
 *  \date 10.04.2014
 *  \date Last Change: 27.08.2015
 */

/// Версия прошивки
#define IMVERSION 0x1E
#define Title "LPCTL LLW/IG-42 1.16"

#define USE_LCD // Работа с LCD
//#define USE_ADA_FRUIT

// Флаг режима вывода отладочной информации
//#define DEBUG_OUTPUT

#include <Wire.h>

#ifdef USE_LCD
  #include <LiquidCrystal_I2C.h>
#endif

#include <TimerOne.h>
#include <EEPROM.h>

#include "pidlib.h"
#include "robot.h"
#include "rcproto.h"
#include "i2cwctl.h"

//----------------------------------------------------------
// LCD
//
// Контакты и порты
//
// Базовые двигательные функции
//----------------------------------------------------------
#include "rcp1motorfunc.inc"

/**
 * Геометрия робота
 */
#define RECnt  500    /// Количество импульсов энкодеров на 1 оборот колеса 
#define RWD   123.0   /// Диаметр колеса, мм
#define RLW   380.0   /// Расстояние между колесами, мм

/**
 * ПИД-регулятор
 */
/// Робот IG42-GM
float Kp = 0.5, Ki = 1.0, Kd = 0.5; // 0.5 0.52 0.0.5
float minU = 0;
float maxU = 200; // 250
float minSpeed = 30;         /// Начальная скорость (скорость трогания)
float maxSpeed = 75;

#define maxErr  ((float)maxSpeed/2)    /// Максимальная ошибка управления
#define CSPEED  (maxSpeed*2/3)         /// Крейсерская скорость (maxSpeed*3/4)
#define RSPEED  (CSPEED*2/3)           /// Скорость разворота (CSPEED*2/3)

BYTE pwmspeed = (maxU*2/3); /// Скорость движения

#include "rcp2func.inc"

/// Количество регистров
#define NREG 15
/**
 * Регистры
 */
BYTE REG[NREG] = {
         1,  /**<  0: REG_ID:         ID робота */
 IMVERSION,  /**<  1: REG_VERSION:    Версия */
         5,  /**<  2: REG_ANG_STEP:   ANG_STEP Шаг угла поворота */
         4,  /**<  3: REG_STOP_SPEED: StopSpeed Скорость останова (коррекционное значение) */
        50,  /**<  4: REG_SPEED:      Speed Текущая скорость */
         1,  /**<  5: REG_LOC_ENABLE: Разрешение поворота локатора */
         1,  /**<  6: REG_ACK:        Флаг режима подтверждения */
         0,  /**<  7: REG_USR: */
         0,  /**<  8: REG_D1:         GlobalEncoderLeftCnt -> sm */
         0,  /**<  9: REG_D1:         GlobalEncoderRightCnt -> sm */

       150,  /**< 10: REG_BUMP_DIST:  Дистанция срабатывания цифровых (контактных) бамперов */
         5,  /**< 11: REG_USS_DIST:   Дистанция срабатывания аналоговых (УЗД) бамперов */

       250,  /**< 12: REG_MAXU:       maxU */
        75,  /**< 13: REG_MAXSPEED:   maxSpeed */
         0   /**< 14: REG_STATUS:     Регистр статуса */
};

#include "rcp3func.inc"

//----------------------------------------------------------

/// Тест рулевого колеса
void TestWheelCtl(void)
{
  delay(1000);
  I2CDCS::SendCommand2B(I2CLPW::ADDR, I2CLPW::DCMD_CALIBRATE, 0, 0);
  delay(5000);
  Beep(1);
  while(1)
  {
    I2CDCS::SendCommand2B(I2CLPW::ADDR, I2CLPW::DCMD_SET_ANG, 0, 0);
    delay(2000);
    I2CDCS::SendCommand2B(I2CLPW::ADDR, I2CLPW::DCMD_SET_ANG, -30, 0);
    delay(2000);
    I2CDCS::SendCommand2B(I2CLPW::ADDR, I2CLPW::DCMD_SET_ANG,  30, 0);
    delay(2000);
  }
}

//----------------------------------------------------------

/**
 * Setup function
 */
void setup()
{
  #define BAUD_RATE 57600 //115200

  #include "rcp4setup.inc"

  // ПИД-регулятор робота
  robot.InitPID(Kp, Ki, Kd, -100.0, 100.0, minU, (float)maxU, maxErr);
  // Геометрия и ноги энкодеров
  // Период прерываний таймера
  unsigned long tns = 50000;    //  50'000 микросекунд (или 0.05 сек - или 20Hz)

  robot.Init(tns, RECnt, RWD, RLW, PIN_encoderLeft, PIN_encoderRight);

  // Тестовые движения (проверка подключения двигателей)
  // Обратите внимание на последовательность движений:
  //   1. Вперед
  //   2. Назад
  //   3. Налево быстро
  //   4. Направо быстро
  
//  TestDelayMotors(pwmspeed);
//  robot.TestEncoders();           // Простой тест энкодеров (в ручном режиме)
//  TestMaxSpeed((float)maxU);      // Определение скорости при управлении U
//  robot.TestPID(CSPEED);          // Простой тест ПИД-регуляторов
//  TestComplexPID(CSPEED, RSPEED); // Комплексный тест ПИД-регуляторов

  Beep(1);
  robot.Stop();

  // Тест рулевого колеса
  //TestWheelCtl();

#ifdef USE_LCD
  lcd.setCursor(0, 1);
  lcd.print("                ");
#endif

  //robot.reStartMove(1000, CSPEED, CSPEED);
  //robot.reStartRotate(90, RSPEED);
  
}

//----------------------------------------------------------
// MAIN
//----------------------------------------------------------

#include "rcp5func.inc"

/**
  * Обнаружение препятствия
  */

BYTE DetectObstacle(void)
{
  BYTE obstval;

  BYTE L1 = Sens[NUM_BUMPER_LEFT].GetVal()<=RVBUMP_DIST;
  BYTE R1 = Sens[NUM_BUMPER_RIGHT].GetVal()<=RVBUMP_DIST;
  BYTE L2 = Sens[NUM_USF_LEFT].GetVal()<=RVUSS_DIST;
  BYTE R2 = Sens[NUM_USF_RIGHT].GetVal()<=RVUSS_DIST;

  obstval = L1 | (R1<<1) | (L2<<2) | (R2<<3);

  RVSTATUS = obstval;
  return obstval;
}

void WriteUsrData(BYTE addr)
// Выдача массива показаний локатора
{
  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_USR_DATA);
  rcWriteByte(0);
  rcWriteByte(CS_VALUE);
}

void loop()
{
  static BYTE LAST_MOVECMD = 0;

  // Чтение сенсоров
  ReadSensors();

  // Обнаружение препятствия
  int obst = DetectObstacle();
  digitalWrite(PIN_REFLEX_INDICATOR, obst);

  if(obst && LAST_MOVECMD != CMD_BACK && LAST_MOVECMD != CMD_BACK2)
  {
    LAST_MOVECMD = CMD_STOP;
    robot.mvstatus=TRobot::MVC_READY;
    robot.Stop();
  }

  #include "rcp7loop.inc"
  
  robot.reMoveStep();
}

