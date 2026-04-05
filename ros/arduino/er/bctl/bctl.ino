/**
 * \file bctl.ino
 * Progect:
 *  Робот с колесными модулями IG-42GM/LLW
 *  DualVNH5019MotorShield
 *  Протокол RCX2
 *  Baudrate 115200
 *  Все значения датчиков считываются в 8-разрядном формате
 *  Chip ATmega328
 *  \author Robofob
 *  \version 1.13
 *  \date 10.04.2014
 *  \date Last Change: 08.01.2015
 */

//-------------------------------------------
// Очень странный финт
//typedef unsigned char BYTE;
#define BYTE unsigned char
//-------------------------------------------

/// Версия прошивки
#define IMVERSION 0x1D

#define USE_LCD // Работа с LCD

#define PIN_REFLEX_INDICATOR 12 // Индикатор срабатывания рефлекса

/// Тип робота. Надо оставить только один макрос
#define TYPE_IG42GM  /// IG-42GM
//#define TYPE_LLW     /// LLW320

// Флаг режима вывода отладочной информации
//#define DEBUG_OUTPUT

#define Title "TM LLW/IG-42 1.13"

// Проверка того, что был установлен только один макрос выбора типа робота
#ifndef TYPE_LLW
  #ifndef TYPE_IG42GM
    #error "ERROR: define TYPE_LLW or TYPE_IG42GM"
  #endif
#endif

#ifdef TYPE_LLW
  #ifdef TYPE_IG42GM
    #error "ERROR: define only one type: TYPE_LLW or TYPE_IG42GM"
  #endif
#endif

#ifdef DEBUG_OUTPUT
  unsigned char robot_debug_output = 1;
#else
  unsigned char robot_debug_output = 0;
#endif

#ifdef USE_LCD
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
#endif

#include <TimerOne.h>
#include <EEPROM.h>

#include "pidlib.h"
#include "robot.h"
#include "rcproto.h"

//------------------------------------------------------------------

#ifdef USE_LCD
// Set the LCD address to 0x20 or 0x27 for a 16 chars and 2 line display
// Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
// Ноги A4 (SDA) и A5 (SCL) заняты
LiquidCrystal_I2C lcd(0x20,16,2); // Обратите внимание на адрес устройства - 0x20 или 0x27
#endif

//------------------------------------------------------------------

#define ANALOG_READ_DELAY 75 /**< Задержка между analogRead, мкс (75)*/

/// Внешнее напряжение питания (№ АЦП)
#define VCC1         A0
#define VCC2         A1

/// Бамперы (№ АЦП) (рефлексы)
#define BUMPER_LEFT  A2
#define BUMPER_RIGHT A3
/// Соответствующие номера в массиве Sens
#define NUM_BUMPER_LEFT  2
#define NUM_BUMPER_RIGHT 3

/// УЗД (рефлексы)
#define USF_LEFT      A6
#define USF_RIGHT     A7
/// Соответствующие номера в массиве Sens
#define NUM_USF_LEFT   6
#define NUM_USF_RIGHT  7

/// Подключение энкодеров
#define PIN_encoderLeft   3 /**< Левый энкодер */
#define PIN_encoderRight  2 /**< Правый энкодер */

/// Пищалка
#define PIN_BEEP         10

//----------------------------------------------------------
// Базовые двигательные функции
//----------------------------------------------------------
#ifdef TYPE_IG42GM

/// Подключение двигателей
#define PIN_MOTOR_RA      4  // M2INA
#define PIN_MOTOR_RPWM    5  // M2PWM (PWM)
#define PIN_MOTOR_LPWM    6  // M1PWM (PWM)
#define PIN_MOTOR_LA      7  // M1INA

#define PIN_MOTOR_LB      8  // M1INB
#define PIN_MOTOR_RB      9  // M2INB

void MotorLeftGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR_LA, HIGH);
    digitalWrite(PIN_MOTOR_LB, HIGH);
  }
  else
  if(pwmspeed>0)
  {
    digitalWrite(PIN_MOTOR_LA, LOW);
    digitalWrite(PIN_MOTOR_LB, HIGH);
    analogWrite(PIN_MOTOR_LPWM, pwmspeed);
  }
  else
  {
    digitalWrite(PIN_MOTOR_LA, HIGH);
    digitalWrite(PIN_MOTOR_LB, LOW);
    analogWrite(PIN_MOTOR_LPWM, -pwmspeed);
  }
}

void MotorRightGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR_RA, HIGH);
    digitalWrite(PIN_MOTOR_RB, HIGH);
  }
  else
  if(pwmspeed>0)
  {
    digitalWrite(PIN_MOTOR_RA, LOW);
    digitalWrite(PIN_MOTOR_RB, HIGH);
    analogWrite(PIN_MOTOR_RPWM, pwmspeed);
  }
  else
  {
    digitalWrite(PIN_MOTOR_RA, HIGH);
    digitalWrite(PIN_MOTOR_RB, LOW);
    analogWrite(PIN_MOTOR_RPWM, -pwmspeed);
  }
}

void MotorsInit(void)
{
  pinMode(PIN_MOTOR_LA, OUTPUT);
  pinMode(PIN_MOTOR_LB, OUTPUT);
  pinMode(PIN_MOTOR_LPWM, OUTPUT);
  pinMode(PIN_MOTOR_RA, OUTPUT);
  pinMode(PIN_MOTOR_RB, OUTPUT);
  pinMode(PIN_MOTOR_RPWM, OUTPUT);

  pinMode(PIN_BEEP, OUTPUT);

  // Подтягиваем входы бампера к "1"
  digitalWrite(BUMPER_LEFT, HIGH);  // set pullup on analog pin BUMPER_LEFT, BUMPER_RIGHT
  digitalWrite(BUMPER_RIGHT, HIGH);
}
#else
//**********************************************************
// LLW
//**********************************************************
/// Подключение двигателей
#define PIN_MOTOR_L1      7  // in1
#define PIN_MOTOR_L2      6  // in2 PWM

#define PIN_MOTOR_R1      5  // in3 PWM
#define PIN_MOTOR_R2      4  // in4

void MotorLeftGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR_L1, HIGH);
    digitalWrite(PIN_MOTOR_L2, HIGH);
  }
  else
  if(pwmspeed>0)
  {
    digitalWrite(PIN_MOTOR_L1, LOW);
    analogWrite(PIN_MOTOR_L2, pwmspeed);
  }
  else
  {
    digitalWrite(PIN_MOTOR_L1, HIGH);
    analogWrite(PIN_MOTOR_L2, (int)255+pwmspeed);
  }
}

void MotorRightGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR_R1, HIGH);
    digitalWrite(PIN_MOTOR_R2, HIGH);
  }
  else
  if(pwmspeed>0)
  {
    analogWrite(PIN_MOTOR_R1, (int)255-pwmspeed);
    digitalWrite(PIN_MOTOR_R2, HIGH);
  }
  else
  {
    analogWrite(PIN_MOTOR_R1, -pwmspeed);
    digitalWrite(PIN_MOTOR_R2, LOW);
  }
}

void MotorsInit(void)
{
  pinMode(PIN_MOTOR_L1, OUTPUT);
  pinMode(PIN_MOTOR_L2, OUTPUT);
  pinMode(PIN_MOTOR_R1, OUTPUT);
  pinMode(PIN_MOTOR_R2, OUTPUT);

  // Подтягиваем входы бампера к "1"
  digitalWrite(BUMPER_LEFT, HIGH);  // set pullup on analog pin BUMPER_LEFT, BUMPER_RIGHT
  digitalWrite(BUMPER_RIGHT, HIGH);
}

#endif

/**
 * Геометрия робота
 */
#ifdef TYPE_IG42GM
  #define RECnt  500    /// Количество импульсов энкодеров на 1 оборот колеса 
  #define RWD   123.0   /// Диаметр колеса, мм
  #define RLW   380.0   /// Расстояние между колесами, мм
#endif

#ifdef TYPE_LLW
  #define RECnt  2000   /// Количество импульсов энкодеров на 1 оборот колеса 
                        /// (по паспорту - 1000, но т.к. мы регистрируем изменения сигнала, а не сам сигнал, то будем писать в 2 раза больше)
  #define RWD    69.0   /// Диаметр колеса (68.0)
  #define RLW   180.0   /// Расстояние между колесами (176.0)
#endif

/**
 * ПИД-регулятор
 */
#ifdef TYPE_IG42GM
  /// Робот IG42-GM
  float Kp = 0.5, Ki = 1.0, Kd = 0.5; // 0.5 0.52 0.0.5
  float minU = 0;
  float maxU = 250;
  float minSpeed = 30;         /// Начальная скорость (скорость трогания)
  float maxSpeed = 75;
#endif

#ifdef TYPE_LLW
  /// Робот LLW
  float Kp = 0.5, Ki = 1.0, Kd = 0.5; // 0.5 0.52 0.0.5
  float minU = 0;
  float maxU = 250;
  float minSpeed = 20;         /// Начальная скорость (скорость трогания)
  float maxSpeed = 120; //100;
#endif

#define maxErr  ((float)maxSpeed/2)    /// Максимальная ошибка управления
#define CSPEED  (maxSpeed*2/3)         /// Крейсерская скорость (maxSpeed*3/4)
#define RSPEED  (CSPEED*2/3)           /// Скорость разворота (CSPEED*2/3)

BYTE pwmspeed = (maxU*2/3); /// Скорость движения

/**
 * Функции реализации протокола rxX2
 */

/// Адрес контроллера
BYTE MY_ADDR = 1;

TPckg pckg;

unsigned char rcReadByte(void) { return Serial.read(); }
void rcWriteByte(unsigned char c) { Serial.write(c); }
unsigned char rcWasByte(void) { return (Serial.available()>0); }
void rcError(unsigned char  n) 
{
#ifdef USE_LCD
  lcd.setCursor(0, 1);
  lcd.print("Error ");
  lcd.print(n);
#endif
}

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

#define RVBUMP_DIST REG[REG_BUMP_DIST]
#define RVUSS_DIST  REG[REG_USS_DIST]
#define RVSTATUS    REG[REG_STATUS]

void CtlVars2Regs(void)
{
  REG[REG_MAXU] = (BYTE)maxU;
  REG[REG_MAXSPEED] = (BYTE)maxSpeed;
}

void Regs2CtlVars(void)
{
  maxU = REG[REG_MAXU];
  maxSpeed = REG[REG_MAXSPEED];
  pwmspeed = (maxU*2/3); // Скорость движения
}

//----------------------------------------------------------

BYTE tdel;

void Beep(unsigned char  n)
{
  for(unsigned char  i=0;i<n;i++)
  {
    digitalWrite(PIN_BEEP, HIGH);
    delay(200);
    digitalWrite(PIN_BEEP, LOW);
    delay(200);
  }
}

/**
 * Setup function
 */
void setup()
{
  #define BAUD_RATE 115200 // 9600

  // Настройка com-порта на скорость BAUD_RATE
  Serial.begin(BAUD_RATE);

  if(robot_debug_output)
    Serial.println(Title);

  // Настройка портов.
  // Подтягиваем входы бампера к "1"
  digitalWrite(BUMPER_LEFT, HIGH);
  digitalWrite(BUMPER_RIGHT, HIGH);

  digitalWrite(USF_LEFT, HIGH);
  digitalWrite(USF_RIGHT, HIGH);

  pinMode(PIN_REFLEX_INDICATOR, OUTPUT);

  //------------------------------------------------------------------------
#ifdef USE_LCD
  // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd.home();

  // печатаем первую строку
  lcd.print(Title);
  // устанавливаем курсор в колонку 0, строку 1. Нумерация начинается с нуля.
  lcd.setCursor(0, 1);
  lcd.print("Test ...        ");
#endif

  //------------------------------------------------------------------------

  // Протокол rcX2
  TPckgInit(&pckg);
  REG[REG_ID] = EEPROM.read(0);
  MY_ADDR = REG[REG_ID];
  if(MY_ADDR==0 || MY_ADDR==255)
  {
    MY_ADDR = REG[REG_ID] = 1;
    EEPROM.write(0,MY_ADDR);
  }
  CtlVars2Regs();

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

#ifdef USE_LCD
  lcd.setCursor(0, 1);
  lcd.print("                ");
#endif

  //robot.reStartMove(1000, CSPEED, CSPEED);
  //robot.reStartRotate(90, RSPEED);

}

//----------------------------------------------------------
// Высокоуровневые функции
//----------------------------------------------------------
/// Здесь хранятся значения датчиков
#define ADCNUM 8

#define BUF_LEN 4 // Длина буфера
struct TSens
{
  BYTE buff[BUF_LEN];
  BYTE GetVal(void);
  void Add(BYTE n);
  BYTE cp;
  TSens(void);
};

TSens::TSens(void)
{
  for(BYTE i=0;i<BUF_LEN;i++)
    buff[i] = 0;
   cp = 0;
}

BYTE TSens::GetVal(void)
{
  int sum = 0;
  for(BYTE i=0;i<BUF_LEN;i++)
    sum+=buff[i];
  return (BYTE)(sum/BUF_LEN);
}

void TSens::Add(BYTE n)
{
  buff[cp] = n;
  cp++;
  if(cp>=BUF_LEN) cp = 0;
}

TSens Sens[ADCNUM];     /**< Сенсоры. "Сырые" данные */

//----------------------------------------------------------

/// Состояния управляющего автомата
enum {Q_S, Q_WAIT, Q_FWD, Q_BCK, Q_LEFT, Q_RIGHT, Q_FAST_LEFT, Q_FAST_RIGHT};
BYTE Q = Q_S; /**< Текущее состояние */

void EvaluateCond(void)
{
  if(Q!=Q_S && Q!=Q_WAIT)
  {
    T_WCNT = 0;
  }
  switch(Q)
  {
    case Q_FWD:
      goFwd(pwmspeed);
      Q = Q_WAIT;
      break;
    case Q_BCK:
      goBack(pwmspeed);
      Q = Q_WAIT;
      break;
    case Q_LEFT:
      goLeft(pwmspeed);
      Q = Q_WAIT;
      break;
    case Q_RIGHT:
      goRight(pwmspeed);
      Q = Q_WAIT;
      break;
    case Q_FAST_LEFT:
      goFastLeft(pwmspeed);
      Q = Q_WAIT;
      break;
    case Q_FAST_RIGHT:
      goFastRight(pwmspeed);
      Q = Q_WAIT;
      break;
    case Q_WAIT:
      if(T_WCNT>tdel)
      {
        Q = Q_S;
        robotStop();
      }
      break;
  }
}

void ReadSensors(void)
{
  for(BYTE i=0;i<ADCNUM;i++)
  {
    Sens[i].Add((BYTE)(analogRead(i)>>2));
    delayMicroseconds(ANALOG_READ_DELAY);
  }
}

void WriteSensors(BYTE addr)
{
  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_SENS);
  rcWriteByte(ADCNUM);
  for(BYTE i=0;i<ADCNUM;i++)
    rcWriteByte(Sens[i].GetVal());
  rcWriteByte(CS_VALUE);
}

void GetAllRegs(BYTE addr)
{
  CtlVars2Regs();

  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_ALL_REG);
  rcWriteByte(NREG);
  for(BYTE i=0;i<NREG;i++)
    rcWriteByte(REG[i]);
  rcWriteByte(CS_VALUE);
}

//----------------------------------------------------------
// MAIN
//----------------------------------------------------------

//            3  2  1  0
//  obstval: R2 L2 R1 L1
#define OBST_BUMP_LEFT  0x01
#define OBST_BUMP_RIGHT 0x02
#define OBST_USF_LEFT   0x04
#define OBST_USF_RIGHT  0x08

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

/// Функция, прерывающая выполнение движения
int fevent2(void)
{
  int wb = rcWasByte();
  int ob = DetectObstacle();
  if(wb || ob)
  {
    robot.Stop();
    return 1;
  }
  return 0;
};

inline BYTE IS_REENERABLE_MOVING(BYTE cmd)
{
  return (cmd==CMD_FWD2 || cmd==CMD_BACK2 || cmd==CMD_FAST_LEFT2 || cmd==CMD_FAST_RIGHT2);
}

void loop()
{
  unsigned char n, val;
  unsigned char from = 0;
  char c;
  unsigned char addr, CMD;
  int angle;
  int obst;
  static BYTE LAST_MOVECMD = 0;

  ReadSensors();
  // Обнаружение препятствия
  obst = DetectObstacle();
  digitalWrite(PIN_REFLEX_INDICATOR, obst);

  if(obst && LAST_MOVECMD != CMD_BACK && LAST_MOVECMD != CMD_BACK2)
  {
    LAST_MOVECMD = CMD_STOP;
    robot.mvstatus=TRobot::MVC_READY;
    robot.Stop();
  }

#ifdef USE_LCD
  lcd.setCursor(0, 1);
  lcd.print(obst);
  lcd.print(" ");
#endif

  if(rcReadPackage(&pckg)==DATA_READY)
  {
    addr = rcGetAddr(&pckg);
    if(addr!=MY_ADDR) return;
    from = rcGetFrom(&pckg);
    CMD = rcGetCmd(&pckg);
    if(IS_MOVE_CMD(CMD))
    {
      if(IS_REENERABLE_MOVING(LAST_MOVECMD))
      {
        robot.Stop();
        robot.mvstatus=TRobot::MVC_READY;
      }
      LAST_MOVECMD = CMD;
    }

    switch(CMD)
    {
      case CMD_PING:
        break;
      case CMD_STOP:
        robot.Stop();
        break;
      case CMD_FWD:
        if(obst) break;
        goFwd(pwmspeed);
        break;
      case CMD_FWD2:
        if(obst) break;
        rcGet1B(&pckg, &val);
        //robot.Go(10*val, CSPEED, CSPEED, fevent2);
        robot.reStartMove(10*val, CSPEED, CSPEED);
        break;
      case CMD_BACK:
        goBack(pwmspeed);
        break;
      case CMD_BACK2:
        rcGet1B(&pckg, &val);
        //robot.Go(10*val, -CSPEED, -CSPEED, fevent2);
        robot.reStartMove(10*val, -CSPEED, -CSPEED);
        break;
      case CMD_LEFT:
        if((obst & OBST_BUMP_RIGHT)) break;
        goLeft(pwmspeed);
        break;
      case CMD_RIGHT:
        if((obst & OBST_BUMP_LEFT)) break;
        goRight(pwmspeed);
        break;
      case CMD_FAST_LEFT:
        goFastLeft(pwmspeed);
        break;
      case CMD_FAST_LEFT2:
        rcGet1B(&pckg, &val);
        angle = val;
        //robot.Rotate(angle, RSPEED, fevent2);
        robot.reStartRotate(angle, RSPEED);
        break;
      case CMD_FAST_RIGHT:
        goFastRight(pwmspeed);
        break;
      case CMD_FAST_RIGHT2:
        rcGet1B(&pckg, &val);
        angle = -val;
        //robot.Rotate(angle, RSPEED, fevent2);
        robot.reStartRotate(angle, RSPEED);
        break;
      case CMD_BEEP:
        break;
      case CMD_BEEP_ON:
        digitalWrite(PIN_BEEP, HIGH);
        break;
      case CMD_BEEP_OFF:
        digitalWrite(PIN_BEEP, LOW);
        break;
      case CMD_DEBUG:
        break;
      case CMD_GET_SENS:
        WriteSensors(from);
        break;
      case CMD_SET_REG:
        rcGet2B(&pckg, &n, &val); 
        REG[n] = val;
        if(n==REG_ID) // Т.к. MY_ADDR хранится в EEPROM
        {
          MY_ADDR = val;
          EEPROM.write(0, MY_ADDR);
        }
       Regs2CtlVars();
       break;
      case CMD_GET_REG:
        REG[REG_D1] = robot.Cnt2L(GlobalEncoderLeftCnt);
        REG[REG_D2] = robot.Cnt2L(GlobalEncoderRightCnt);
        rcGet1B(&pckg, &n); 
        val = REG[n];
        rcSend1B(&pckg, 0, CMD_ANS_GET_REG, val);
        break;
      case CMD_GET_ALL_REG:
        REG[REG_D1] = robot.Cnt2L(GlobalEncoderLeftCnt);
        REG[REG_D2] = robot.Cnt2L(GlobalEncoderRightCnt);
        GetAllRegs(from);
        break;
      default:
        rcError(ERR_ILLEGAL_CMD);
    }
    if((!IS_NOT_ACK_CMD(CMD) && REG[REG_ACK]) || CMD==CMD_PING)
      rcSendACK(&pckg, 0);
  }
  robot.reMoveStep();
//  EvaluateCond();
}
