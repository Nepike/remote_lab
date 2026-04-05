/**
 * \file mpctl_mw.ino
 * Progect:
 *  Макет подвижной платформы
 *  Мотор-колесо
 *  Протокол RCX2
 *  Baudrate 57600/115200
 *  Все значения датчиков считываются в 8-разрядном формате
 *  Внимание! Инициализация системы занимает очень много времени - порядка 5 секунд.
 *  Chip ATmega328
 *  \author Robofob
 *  \version 3.25
 *  \date 10.04.2014
 *  \date Last Change: 07.09.2016
 */

#include <Arduino.h>
#include <math.h>

#include <TimerOne.h>
#include <EEPROM.h>

/// Версия прошивки
#define IMVERSION 0x33
#define Title "MPMWCTL 3.25"

#define USE_LCD // Работа с LCD
#define USE_ADA_FRUIT

// Флаг режима вывода отладочной информации
//#define DEBUG_OUTPUT

#include <Wire.h>

#ifdef USE_LCD
#ifdef USE_ADA_FRUIT
  #include <Adafruit_MCP23017.h>
  #include <Adafruit_RGBLCDShield.h>
#else
  #include <LiquidCrystal_I2C.h>
#endif
#endif

#include "rcproto.h"
#include "i2cwctl.h"

//----------------------------------------------------------
// LCD
//
// Контакты и порты
//
// Базовые двигательные функции
//----------------------------------------------------------
// Очень странный финт
#define BYTE unsigned char
//-------------------------------------------

/// Адрес контроллера
BYTE MY_ADDR = 1;

#ifdef DEBUG_OUTPUT
  unsigned char robot_debug_output = 1;
#else
  unsigned char robot_debug_output = 0;
#endif

//----------------------------------------------------------
// LCD
//----------------------------------------------------------
#ifdef USE_LCD

#ifdef USE_ADA_FRUIT
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#else
// Set the LCD address to 0x20 or 0x27 for a 16 chars and 2 line display
// Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
// Ноги A4 (SDA) и A5 (SCL) заняты
LiquidCrystal_I2C lcd(0x20,16,2); // Обратите внимание на адрес устройства - 0x20 или 0x27
#endif

#endif

//----------------------------------------------------------
// Контакты и порты
//----------------------------------------------------------
#define ANALOG_READ_DELAY 75 /**< Задержка между analogRead, мкс (75)*/

#define PIN_REFLEX_INDICATOR 12 // Индикатор срабатывания рефлекса

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

/// Пищалка
#define PIN_BEEP      11

//----------------------------------------------------------
// Базовые двигательные функции
//----------------------------------------------------------

/// Подключение двигателей
#define PIN_MOTOR_L_PWM        10
#define PIN_MOTOR_L_STOP       8
#define PIN_MOTOR_L_REVERSE    7
// Emulate inverter for reverse movement
// #define PIN_MOTOR_L_REVERSE_INV 9

#define PIN_MOTOR_R_PWM        9
#define PIN_MOTOR_R_STOP       4
#define PIN_MOTOR_R_REVERSE    3
// Emulate inverter for reverse movement
// #define PIN_MOTOR_R_REVERSE_INV 2

/**
 * Геометрия робота
 */
#define RWD   350   /// Диаметр колеса, мм
#define RLW   820   /// Расстояние между колесами, мм

// Минимальный и максимальный ШИМ сигналы скорости колёс,
// соответствующие диапазону напряжений 1.0 - 2.5 В
const int MIN_PWM = 50;
const int MAX_PWM = 125;

// Шаг регуляции скорости
int K_V = 1;

int CSPEED = 100 * 2./3;     /// Крейсерская скорость 
int RSPEED = CSPEED * 2./3;   /// Скорость разворота


int T_WCNT = 0; /**< Глобальный счетчик времени */

void timerIsr()
{
  T_WCNT++;
}

class TRobot
{
public:
  TRobot(void)
  {
    MotorsInit();
    currSpeedL = currSpeedR = goalSpeedL = goalSpeedR = 0;
  }
  void goFwd(int speed);
  void goBack(int speed);
  void goLeft(int speed);
  void goRight(int speed);
  void goFastLeft(int speed);
  void goFastRight(int speed);
  void Stop(void);
  void MoveStep(void);
  void MotorLeftGo(int speed);
  void MotorRightGo(int speed);
  void SetSpeedLeft(int speed);
  void SetSpeedRight(int speed);

//private:
  int currSpeedL, currSpeedR;
  int goalSpeedL, goalSpeedR;

  void MotorsInit(void);
  void SetLeftStop(int n) { digitalWrite(PIN_MOTOR_L_STOP, n); }
  void SetRightStop(int n) { digitalWrite(PIN_MOTOR_R_STOP, n); }
  void SetLeftReverse(int n) 
  {
    digitalWrite(PIN_MOTOR_L_REVERSE, n); 
   // digitalWrite(PIN_MOTOR_L_REVERSE_INV, !n); // for hardware emulation
  }
  void SetRightReverse(int n) 
  {
    digitalWrite(PIN_MOTOR_R_REVERSE, n);  
   // digitalWrite(PIN_MOTOR_R_REVERSE_INV, !n); // for hardware emulation
  }
};

TRobot robot;

void TRobot::MoveStep(void)
{
  // static int ct = -1;

  //if(ct==T_WCNT) return;

  // Обработка скоростей
  if(currSpeedL < goalSpeedL)
    currSpeedL += K_V;
  else if(currSpeedL > goalSpeedL)
    currSpeedL -= K_V;
  MotorLeftGo(currSpeedL);

  if(currSpeedR < goalSpeedR)
    currSpeedR += K_V;
  else if(currSpeedR > goalSpeedR)
    currSpeedR -= K_V;
  MotorRightGo(currSpeedR);
}

/* Sends speed command to motors after converting it to an appropriate
 * format and setting stop and reverse flags (LEFT WHEEL).
 */
void TRobot::MotorLeftGo(int speed)
{
  // check reverse and stop conditions
  SetLeftReverse((speed<0));
  SetLeftStop((speed == 0));  

  // gets converted speed to PWM format
  int pwmspeed = getPWMFromSpeed(speed);

  // send command to the wheel
  analogWrite(PIN_MOTOR_L_PWM, pwmspeed);
}

/* Sends speed command to motors after converting it to an appropriate
 * format and setting stop and reverse flags (RIGHT WHEEL).
 */
void TRobot::MotorRightGo(int speed)
{
  // check reverse and stop conditions
  SetRightReverse((speed<0));
  SetRightStop((speed == 0));  

  // gets converted speed to PWM format
  int pwmspeed = getPWMFromSpeed(speed);

  // send command to the wheel
  analogWrite(PIN_MOTOR_R_PWM, pwmspeed);
}

/* Converts speed from [-100;100] to [MIN_PWM;MAX_PWM]
 */
int getPWMFromSpeed(int speed)
{
   // convert
  int pwmspeed = MIN_PWM + (MAX_PWM - MIN_PWM) * abs(speed) / 100.;

  // check absolute values
  if(pwmspeed<MIN_PWM) pwmspeed = MIN_PWM;
  if(pwmspeed>MAX_PWM) pwmspeed = MAX_PWM;

  return pwmspeed;
}

void TRobot::MotorsInit(void)
{
  pinMode(PIN_MOTOR_L_STOP, OUTPUT);
  pinMode(PIN_MOTOR_L_REVERSE, OUTPUT);
  // pinMode(PIN_MOTOR_L_REVERSE_INV, OUTPUT);
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);

  pinMode(PIN_MOTOR_R_STOP, OUTPUT);
  pinMode(PIN_MOTOR_R_REVERSE, OUTPUT);
  // pinMode(PIN_MOTOR_R_REVERSE_INV, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);

  // Настройка портов.
  // Подтягиваем входы бампера к "1"
  digitalWrite(BUMPER_LEFT, HIGH);
  digitalWrite(BUMPER_RIGHT, HIGH);

  digitalWrite(USF_LEFT, HIGH);
  digitalWrite(USF_RIGHT, HIGH);

  pinMode(PIN_BEEP, OUTPUT);
  pinMode(PIN_REFLEX_INDICATOR, OUTPUT);

  // изменение множителя счётчика на частоту 31 кГц на выходах 9 (PIN_MOTOR_L_PWM), 10 (PIN_MOTOR_R_PWM); (http://arduino-info.wikispaces.com/Arduino-PWM-Frequency) 
  TCCR1B = TCCR1B & B11111000 | B00000001;
}

void TRobot::goFwd(int speed) { goalSpeedL = goalSpeedR = speed; }
void TRobot::goBack(int speed) { goalSpeedL = goalSpeedR = -speed; }
void TRobot::goLeft(int speed) { goalSpeedL = 0; goalSpeedR = speed; }
void TRobot::goRight(int speed) { goalSpeedL = speed; goalSpeedR = 0; }
void TRobot::goFastLeft(int speed) { goalSpeedL = -speed; goalSpeedR = speed; }
void TRobot::goFastRight(int speed) { goalSpeedL = speed; goalSpeedR = -speed; }
void TRobot::Stop(void) 
{ 
  currSpeedL = currSpeedR =  goalSpeedL = goalSpeedR = 0; 
  SetLeftStop(1);
  SetRightStop(1);
};
void TRobot::SetSpeedLeft(int speed){  goalSpeedL = speed;}
void TRobot::SetSpeedRight(int speed){  goalSpeedR = speed;}

//-----------------------------------------------------------------------------------

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
 * Функции реализации протокола rxX2
 */

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

// Проверка таймаута. Возвращает 0, если истекло время ожидания очередного символа пакета
uchar rcTimeoutEvent(void) { return 0; }

// Сброс счетчика таймаута
void rcResetTimeoutCnt(void) { return; }

//----------------------------------------------------------
// Регистры
//----------------------------------------------------------

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

       100,  /**< 10: REG_BUMP_DIST:  Рефлекс: Дистанция срабатывания цифровых (контактных) бамперов */
       100,  /**< 11: REG_USS_DIST:   Рефлекс: Дистанция срабатывания аналоговых (УЗД) бамперов */

       250,  /**< 12: REG_MAXU:       maxU */
       150,  /**< 13: REG_MAXSPEED:   maxSpeed */
         0   /**< 14: REG_STATUS:     Регистр статуса */
};

/**
 * Работа с регистрами. Скорость
 */

#define RVBUMP_DIST  REG[REG_BUMP_DIST]
#define RVUSS_DIST   REG[REG_USS_DIST]
#define RVSTATUS     REG[REG_STATUS]
#define RVLOC_ENABLE REG[REG_LOC_ENABLE]

void CtlVars2Regs(void)
{
  REG[REG_MAXU] = (BYTE)MAX_PWM;
  REG[REG_MAXSPEED] = (BYTE)MAX_PWM;
}

void Regs2CtlVars(void)
{
  //  WARNING! DO NOT SET PWM FROM REGISTERS!
  //  THEY LIMIT MOTOR VOLTAGE. MAY DESTROY.
  //  MAX_PWM = REG[REG_MAXSPEED];
  CSPEED = (MAX_PWM*2/3); // Скорость движения
}

//----------------------------------------------------------
void TestDelayMotors(void)
// Тест подключения двигателей
// Не работает в версии с MoveStep, т.к. обработка двигателей идёт в основном цикле
{
  Serial.print("pwmspeed = ");
  Serial.println(CSPEED);

  #define TM 2000
  Serial.println("go fwd");

  robot.goFwd(CSPEED);
  delay(TM);

  Serial.println("go back");
  robot.goBack(CSPEED);
  delay(TM);

  Serial.println("go Fast left");
  robot.goFastLeft(RSPEED);
  delay(TM);

  Serial.println("go Fast right");
  robot.goFastRight(RSPEED);
  delay(TM);

  Serial.println("stop");
  robot.Stop();

}

/**
 * Setup function
 */
void setup()
{
  #define BAUD_RATE 115200 // 115200 (last - 57600)
  
  // Настройка com-порта на скорость BAUD_RATE
  Serial.begin(BAUD_RATE);

  if(robot_debug_output)
    Serial.println(Title);

  //------------------------------------------------------------------------
  // Инициализация lcd
  // Внимание! Инициализация идет очень долго - больше секунды.
#ifdef USE_LCD

#ifdef USE_ADA_FRUIT
  #define WHITE  0x7
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
#else
  lcd.init();
  lcd.backlight();
  lcd.home();
#endif

  // Печатаем первую строку
  // Устанавливаем курсор в колонку 0, строку 0. Нумерация начинается с нуля.
  lcd.setCursor(0, 0);
  lcd.print(Title);
  // Устанавливаем курсор в колонку 0, строку 1
  lcd.setCursor(0, 1);
  lcd.print("Test ...        ");

#else
  Wire.begin();
#endif

  //------------------------------------------------------------------------
  // Протокол rcX2
  //------------------------------------------------------------------------
  TPckgInit(&pckg);
  REG[REG_ID] = EEPROM.read(0);
  MY_ADDR = REG[REG_ID];
  if(MY_ADDR==0 || MY_ADDR==255)
  {
    MY_ADDR = REG[REG_ID] = 1;
    EEPROM.write(0,MY_ADDR);
  }
  CtlVars2Regs();
  robot.MotorsInit();

  //------------------------------------------------------------------------
  //*** Тестовые движения (проверка подключения двигателей)
  // Обратите внимание на последовательность движений:
  //   1. Вперед
  //   2. Назад
  //   3. Налево быстро
  //   4. Направо быстро
  //------------------------------------------------------------------------

//  TestDelayMotors();

  Beep(1);
  
/*
  robot.Stop();
  while(1)
  {
    robot.MotorLeftGo(100);
    robot.MotorRightGo(100);
    delay(4000);
  
    robot.MotorLeftGo(0);
    robot.MotorRightGo(0);
    delay(2000);

    robot.MotorLeftGo(-100);
    robot.MotorRightGo(-100);
    delay(4000);

    robot.MotorLeftGo(0);
    robot.MotorRightGo(0);
    delay(2000);
  }

*/
#ifdef USE_LCD
  lcd.setCursor(0, 1);
  lcd.print("                ");
#endif

}
/**
 * Работа с буферами. Определение TSens
 * Функции чтения/записи сенсорной информации, WriteI2CData
 */

/// Здесь хранятся значения датчиков
#define ADCNUM 8
#define NUMSENS (ADCNUM+2)

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

TSens Sens[NUMSENS];     /**< Сенсоры. "Сырые" данные */

/// ----------------------------------------------------------------

void ReadSensors(void)
// Чтение сенсоров. Читать лучше не в цикле, а по одному
{
  static BYTE n = 0;
  Sens[n].Add((BYTE)(analogRead(n)>>2));
  n++;
  if(n>=ADCNUM) n = 0;

  Sens[NUM_BUMPER_LEFT].Add((BYTE)digitalRead(BUMPER_LEFT));
  Sens[NUM_BUMPER_RIGHT].Add((BYTE)digitalRead(BUMPER_RIGHT));
}

void WriteSensors(BYTE addr)
{
  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_SENS);
  rcWriteByte(NUMSENS);
  for(BYTE i=0;i<NUMSENS;i++)
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

/// ----------------------------------------------------------------
//            3  2  1  0
//  obstval: R2 L2 R1 L1
#define OBST_BUMP_LEFT  0x01
#define OBST_BUMP_RIGHT 0x02
#define OBST_USF_LEFT   0x04
#define OBST_USF_RIGHT  0x08

BYTE DetectObstacle(void);

/// ----------------------------------------------------------------

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

/// ----------------------------------------------------------------

/// Буфер данных, получаемых от I2C-серверов
/// Полагаем, что I2CDCS::DATALEN - максимальный размер данных I2CDataServer и I2CUSonicServer

unsigned char DSDATA[I2CDCS::DATALEN];

void WriteI2CData(BYTE addr, unsigned char i2caddr, unsigned int size, unsigned char buff[])
{
  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_I2C_DATA);
  rcWriteByte((unsigned char)size+1); // Длина пакета данных (учтем адрес)
  rcWriteByte(i2caddr); // Сначала посылаем i2c адрес, чтоб было понятно, от какого устройства эти данные
  for(BYTE i=0;i<size;i++)
    rcWriteByte(buff[i]);
  rcWriteByte(CS_VALUE);
}

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

//----------------------------------------------------------
// MAIN
//----------------------------------------------------------

void loop()
{
  static BYTE LAST_MOVECMD = 0;

  // Чтение сенсоров
  ReadSensors();

  // Обнаружение препятствия
  //int obst = DetectObstacle(); // DEBUG
  int obst = 0; // DEBUG (ignore obstacles)
  digitalWrite(PIN_REFLEX_INDICATOR, obst);

  if(obst && LAST_MOVECMD != CMD_BACK && LAST_MOVECMD != CMD_BACK2)
  {
    LAST_MOVECMD = CMD_STOP;
    robot.Stop();
  }

#ifdef USE_LCD
  lcd.setCursor(0, 1);
  lcd.print(obst);
  lcd.print(" ");
#endif

  unsigned char addr, from = 0;
  unsigned char CMD;
  unsigned char i2caddr, i2ccmd, b1, b2;
  int angle;
  byte databuff[10];
  byte datalen;
  unsigned char n, val;

  if(rcReadPackage(&pckg)==DATA_READY)
  {
    addr = rcGetAddr(&pckg);
    if(addr!=MY_ADDR) return;
    from = rcGetFrom(&pckg);
    CMD = rcGetCmd(&pckg);
    if(IS_MOVE_CMD(CMD))
    {
      if(IS_REENTERABLE_MOVING(LAST_MOVECMD))
      {
        robot.Stop();
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
        robot.goFwd(CSPEED);
        break;
      case CMD_FWD2:
        if(obst) break;
        rcGet1B(&pckg, &val);
        //robot.Go(10*val, CSPEED, CSPEED, fevent2);
        //robot.reStartMove(10*val, CSPEED, CSPEED);
        break;
      case CMD_BACK:
        robot.goBack(CSPEED);
        break;
      case CMD_BACK2:
        rcGet1B(&pckg, &val);
        //robot.Go(10*val, -CSPEED, -CSPEED, fevent2);
        //robot.reStartMove(10*val, -CSPEED, -CSPEED);
        break;
      case CMD_LEFT:
        if((obst & OBST_BUMP_RIGHT)) break;
        robot.goLeft(RSPEED);
        break;
      case CMD_RIGHT:
        if((obst & OBST_BUMP_LEFT)) break;
        robot.goRight(RSPEED);
        break;
      case CMD_FAST_LEFT:
        robot.goFastLeft(RSPEED);
        break;
      case CMD_FAST_LEFT2:
        rcGet1B(&pckg, &val);
        angle = val;
        //robot.Rotate(angle, RSPEED, fevent2);
        //robot.reStartRotate(angle, RSPEED);
        break;
      case CMD_FAST_RIGHT:
        robot.goFastRight(RSPEED);
        break;
      case CMD_FAST_RIGHT2:
        rcGet1B(&pckg, &val);
        angle = -val;
        //robot.Rotate(angle, RSPEED, fevent2);
        //robot.reStartRotate(angle, RSPEED);
        break;
      case CMD_SET_SPEED:
        rcGet2B(&pckg, &b1, &b2); 
        robot.SetSpeedLeft((signed char)b1);
        robot.SetSpeedRight((signed char)b2);
        //robot.reStartMove(0, (signed char)b1, (signed char)b2);      
        break;
      case CMD_BEEP:
        Beep(1);
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
      case CMD_GET_USR_DATA:
        WriteUsrData(from);
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
        REG[REG_D1] = 0; //robot.Cnt2L(GlobalEncoderLeftCnt);
        REG[REG_D2] = 0; //robot.Cnt2L(GlobalEncoderRightCnt);
        rcGet1B(&pckg, &n); 
        val = REG[n];
        rcSend1B(&pckg, 0, CMD_ANS_GET_REG, val);
        break;
      case CMD_GET_ALL_REG:
        REG[REG_D1] = 0; //robot.Cnt2L(GlobalEncoderLeftCnt);
        REG[REG_D2] = 0; //robot.Cnt2L(GlobalEncoderRightCnt);
        GetAllRegs(from);
        break;
      //*** Работа с I2C-устройствами (серверами)
      case CMD_I2C: // Команда I2C-устройству. Формат: <i2c-адрес устройства> <команда> <количество аргументов> <аргумент1> <аргумент2> ...
        rcGetI2CDataBuff(&pckg, &i2caddr, &i2ccmd, &datalen, databuff);
        I2CDCS::SendCommand(i2caddr, i2ccmd, datalen, databuff);
        break;
      case CMD_GET_I2C_DATA: // Запрос: Получить массив данных от i2c-устройства. При этом надо заранее знать размер запрашиваемых данных
        rcGet2B(&pckg, &i2caddr, &b1);
        I2CDCS::ReadData(i2caddr, b1, DSDATA);
        WriteI2CData(from, i2caddr, b1, DSDATA);
        break;
      default:
        rcError(ERR_ILLEGAL_CMD);
    }
    if((!IS_NOT_ACK_CMD(CMD) && REG[REG_ACK]) || CMD==CMD_PING)
      rcSendACK(&pckg, 0);
  }
  // Шаг
  robot.MoveStep();
}
