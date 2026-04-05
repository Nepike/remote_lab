/**
 * \file mpctl_mw.ino
 * Progect:
 *  Макет подвижной платформы
 *  Инвалид-колесо
 *  Протокол RCX2
 *  Baudrate 57600/115200
 *  Все значения датчиков считываются в 8-разрядном формате
 *  Внимание! Инициализация системы занимает очень много времени - порядка 5 секунд.
 *  Chip ATmega328
 *  \author Robofob
 *  \version 3.25
 *  \date 10.04.2014
 *  \date Last Change: 20.06.2017
 */

#include <Arduino.h>
#include <math.h>

#include <TimerOne.h>
#include <EEPROM.h>

/// Версия прошивки
#define IMVERSION 0x32
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

#include "rcproto2.h"
#include "tmucmd.h"
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
// #define ANALOG_READ_DELAY 75 /**< Задержка между analogRead, мкс (75)*/

#define PIN_REFLEX_INDICATOR 12 // Индикатор срабатывания рефлекса

/// Внешнее напряжение питания (№ АЦП)
// #define VCC1         A0
// #define VCC2         A1

// Считывание УЗ датчиков
#define US_START_READING 7
#define REFLEX_SWITCH 6

/// Бамперы (№ АЦП) (рефлексы)
#define BUMPER_LEFT  A0
#define BUMPER_RIGHT A1
/// Соответствующие номера в массиве Sens
#define NUM_BUMPER_LEFT  0
#define NUM_BUMPER_RIGHT 1

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
// Управление скоростью
int PIN_TAN = 10; // поступательное
int PIN_ROT = 9; // вращательное

double V_TO_ADC = 255 / 5.0; // перевод управляющего напряжения в соотв. значение для analogWrite

double MIN_V = 1.1; // минимальное напряжение управляющего сигнала скорости,
                    // соответствующее 0 скорости
double MAX_V = 3.8; // максимальное напряжение управляющего сигнала скорости,
                    // соответствующее максимальной скорости
                    
// Минимальный и максимальный ШИМ сигналы скорости колёс,
// соответствующие диапазону напряжений MIN_V - MAX_V В
const int MIN_PWM = MIN_V * V_TO_ADC;
const int MAX_PWM = MAX_V * V_TO_ADC;

const int DELTA_PWM = 0.15 * V_TO_ADC; // поправка на потери в цепи (0.15 * )

// Шаг регуляции скорости
int K_V = 1;

int CSPEED = 100 * 2./3;     /// Крейсерская скорость 
int RSPEED = CSPEED;// * 2./3;   /// Скорость разворота

bool Reflex_Flag = false;

/**
 * Геометрия робота
 */
#define RWD   350   /// Диаметр колеса, мм
#define RLW   820   /// Расстояние между колесами, мм

class TRobot
{
public:
  TRobot(void)
  {
    MotorsInit();
    currSpeedRot = currSpeedTan = goalSpeedRot = goalSpeedTan = 0;
  }
  void goFwd(int speed);
  void goBack(int speed);
  void goLeft(int speed);
  void goRight(int speed);
  void goFastLeft(int speed);
  void goFastRight(int speed);
  void Stop(void);
  void MoveStep(void);
  void goUni(int rot, int linear);
  
  void MotorSetRot(int speed);
  void MotorSetTan(int speed);
  
//private:
  // rotational and tangential speeds
  int currSpeedRot, currSpeedTan;
  int goalSpeedRot, goalSpeedTan;
  
  void MotorsInit(void);
  
};

TRobot robot;

void TRobot::MoveStep(void)
{
  // Обработка скоростей
  if(currSpeedRot < goalSpeedRot)
    currSpeedRot += K_V;
  else if(currSpeedRot > goalSpeedRot)
    currSpeedRot -= K_V;
  MotorSetRot(currSpeedRot);

  if(currSpeedTan < goalSpeedTan)
    currSpeedTan += K_V;
  else if(currSpeedTan > goalSpeedTan)
    currSpeedTan -= K_V;
  MotorSetTan(currSpeedTan);
}


/* Converts speed from [-100;100] to [MAX_PWM;MIN_PWM] (100 corresponds to forward motion 
 *  and to right rotation)
 *  
 *  Conversion considers:
 *  -100 -> MIN_PWM
 *  0 -> (MIN_PWM + MAX_PWM) / 2
 *  100 -> MAX_PWM
 */
int getPWMFromSpeed(int speed)
{
   // convert
  int pwmspeed = MIN_PWM + (MAX_PWM - MIN_PWM) * ((1 + speed / 100.) / 2.);
  
  // поправка на потери в цепи
  pwmspeed += DELTA_PWM;
  
  // check constraints
  if(pwmspeed<MIN_PWM) pwmspeed = MIN_PWM;
  if(pwmspeed>MAX_PWM) pwmspeed = MAX_PWM;
  
  return pwmspeed;
}

/* Sends rotation speed command to motor controller after converting it to an appropriate
 * format.
 */
void TRobot::MotorSetRot(int speed)
{  
  // gets converted speed to PWM format
  int pwmspeed = getPWMFromSpeed(speed);
  
  // send command to the wheel
  analogWrite(PIN_ROT, pwmspeed);

  Serial.print(pwmspeed);
  Serial.print(" ");
}

/* Sends tangential speed command to motors after converting it to an appropriate
 * format.
 */
void TRobot::MotorSetTan(int speed)
{
  // gets converted speed to PWM format
  int pwmspeed = getPWMFromSpeed(-speed); // NOTE: minus sign because forward
    // and backward direction is inverted in the current version

  if(Reflex_Flag && speed < 0)
    pwmspeed = getPWMFromSpeed(0);
    
  // send command to the wheel
  analogWrite(PIN_TAN, pwmspeed);
  Serial.println(pwmspeed);
}

void TRobot::MotorsInit(void)
{
  pinMode(PIN_ROT, OUTPUT);
  pinMode(PIN_TAN, OUTPUT);
  
  // изменение множителя счётчика на частоту 31 кГц на выходах 9 (PIN_ROT), 10 (PIN_TAN); (http://arduino-info.wikispaces.com/Arduino-PWM-Frequency) 
  TCCR1B = TCCR1B & B11111000 | B00000001;
}

void TRobot::goUni(int rot, int linear) 
{
  goalSpeedTan = linear; goalSpeedRot = rot;
  currSpeedTan = linear; currSpeedRot = rot;
}

void TRobot::goFwd(int speed) 
{ 
  goalSpeedTan = speed; goalSpeedRot = 0;
  currSpeedTan = speed; currSpeedRot = 0;
}
void TRobot::goBack(int speed) 
{ 
  goalSpeedTan = -speed; goalSpeedRot = 0; 
  currSpeedTan = -speed; currSpeedRot = 0; 
}
void TRobot::goLeft(int speed) 
{ 
  goalSpeedTan = speed/2; goalSpeedRot = -speed/2; 
  currSpeedTan = speed/2; currSpeedRot = -speed/2; 
}
void TRobot::goRight(int speed) 
{ 
  goalSpeedTan = speed/2; goalSpeedRot = speed/2; 
  currSpeedTan = speed/2; currSpeedRot = speed/2; 
}
void TRobot::goFastLeft(int speed) 
{ 
  goalSpeedTan = 0; goalSpeedRot = -speed; 
  currSpeedTan = 0; currSpeedRot = -speed;
}
void TRobot::goFastRight(int speed) 
{ 
  goalSpeedTan = 0; goalSpeedRot = speed; 
  currSpeedTan = 0; currSpeedRot = speed;
}
void TRobot::Stop(void) 
{ 
  goalSpeedTan = goalSpeedRot = 0; 
  currSpeedTan = currSpeedRot = 0; 
};
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

// Проверка наличия пришедшего символа
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
       40,  /**< 11: REG_USS_DIST:   Рефлекс: Дистанция срабатывания аналоговых (УЗД) бамперов */

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


void TestMotors(void)
{
    // fwd
    robot.MotorSetRot(0);
    robot.MotorSetTan(100);
    delay(4000);
  
    robot.MotorSetRot(0);
    robot.MotorSetTan(0);
    delay(2000);

    // back
    robot.MotorSetRot(0);
    robot.MotorSetTan(-100);
    delay(4000);
    
    robot.MotorSetRot(0);
    robot.MotorSetTan(0);
    delay(2000);

    // left
    robot.MotorSetRot(-100);
    robot.MotorSetTan(0);
    delay(4000);
    
    robot.MotorSetRot(0);
    robot.MotorSetTan(0);
    delay(2000);

    // right
    robot.MotorSetRot(100);
    robot.MotorSetTan(0);
    delay(4000);
    
    robot.MotorSetRot(0);
    robot.MotorSetTan(0);
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
  TPckgInit(&pckg, MY_ADDR, rcError, rcReadByte, rcWriteByte, rcWasByte, rcTimeoutEvent, rcResetTimeoutCnt);
  /*
  REG[REG_ID] = EEPROM.read(0);
  MY_ADDR = REG[REG_ID];
  if(MY_ADDR==0 || MY_ADDR==255)
  {
    MY_ADDR = REG[REG_ID] = 1;
    EEPROM.write(0,MY_ADDR);
  }
  */
  CtlVars2Regs();
  robot.MotorsInit();
  
  // Настройка портов.
  pinMode(REFLEX_SWITCH, INPUT_PULLUP);
  pinMode(US_START_READING, OUTPUT);
  digitalWrite(US_START_READING, LOW);
  
  // turn US array on
  digitalWrite(US_START_READING, HIGH);
  //delayMicroseconds(30); // 20 us < this delay < 48 us
  delay(30);
  digitalWrite(US_START_READING, LOW);
  pinMode(US_START_READING, INPUT);

  pinMode(PIN_BEEP, OUTPUT);
  pinMode(PIN_REFLEX_INDICATOR, OUTPUT);

  // joystick init
  robot.MotorSetRot(0);
  robot.MotorSetTan(0);
  delay(3000);
  Beep(1);

  //------------------------------------------------------------------------
  //*** Тестовые движения (проверка подключения двигателей)
  // Обратите внимание на последовательность движений:
  //   1. Вперед
  //   2. Назад
  //   3. Налево быстро
  //   4. Направо быстро
  //------------------------------------------------------------------------
  TestMotors();
  Beep(1);

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
  /*
  static BYTE n = 0;
  Sens[n].Add((BYTE)(analogRead(n)>>2));
  n++;
  if(n>=ADCNUM) n = 0;
*/

  // NOTE: US array will be reading data only if it has been 
  // activated previously with a digitalWrite to its RX

  // NOTE: US array of LV-MAXSonar-EZ gets one US reading per 
  // 49 ms, which means ~= 5*50 = 250 ms for 5 sensors, which
  // effectively means 4 Hz. Reading with analogRead faster than
  // 4 Hz will lead to only partial updates being faster, and 
  // reading faster than 20 Hz will give no benefits at all.
  Sens[NUM_USF_LEFT].Add((BYTE)analogRead(USF_LEFT));
  Sens[NUM_USF_RIGHT].Add((BYTE)analogRead(USF_RIGHT));  
  Sens[NUM_BUMPER_LEFT].Add((BYTE)analogRead(BUMPER_LEFT));
  Sens[NUM_BUMPER_RIGHT].Add((BYTE)analogRead(BUMPER_RIGHT));
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
  
#define SKEWED_THRESHOLD 60
#define STRAIGHT_THRESHOLD 40
BYTE DetectObstacle(void)
{
  // if obstacle switch is on, do not detect them
  if (digitalRead(REFLEX_SWITCH) == LOW)
    return 0;
    
  BYTE obstval;

  BYTE L1 = Sens[NUM_BUMPER_LEFT].GetVal()<=SKEWED_THRESHOLD;//RVBUMP_DIST;
  BYTE R1 = Sens[NUM_BUMPER_RIGHT].GetVal()<=SKEWED_THRESHOLD;//RVBUMP_DIST;
  BYTE L2 = Sens[NUM_USF_LEFT].GetVal()<=STRAIGHT_THRESHOLD;
  BYTE R2 = Sens[NUM_USF_RIGHT].GetVal()<=STRAIGHT_THRESHOLD;

  obstval = L1 | (R1<<1) | (L2<<2) | (R2<<3);
/*
  Serial.print('%3ud',Sens[NUM_BUMPER_LEFT].GetVal());
  Serial.print(' ');
  Serial.print('%3d',Sens[NUM_BUMPER_RIGHT].GetVal());
  Serial.print(' ');
  Serial.print('%3d',Sens[NUM_USF_LEFT].GetVal());
  Serial.print(' ');
  Serial.println('%3d',Sens[NUM_USF_RIGHT].GetVal());
*/
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
  Reflex_Flag = false;

  // Чтение сенсоров
  ReadSensors();

  // Обнаружение препятствия
  int obst = DetectObstacle(); // DEBUG
  obst = 0; // DEBUG (ignore obstacles)
  digitalWrite(PIN_REFLEX_INDICATOR, obst);

  if(obst)
  {
    Reflex_Flag = true;
  }

#ifdef USE_LCD  
  lcd.setCursor(0, 1);
  lcd.print("o");
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
    //if ( (addr != MY_ADDR) && (addr != 255) )
    //   return;
    from = rcGetFrom(&pckg);
    CMD = rcGetCmd(&pckg);

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
        robot.goUni((signed char)b1, (signed char)b2);
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
