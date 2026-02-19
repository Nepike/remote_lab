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
 *  \version 5.01
 *  \date 10.04.2014
 *  \date Last Change: 27.06.2017
 */

#include <Arduino.h>
#include <math.h>

#include <TimerOne.h>
#include <EEPROM.h>

/// Версия прошивки
#define IMVERSION 0x42
#define Title "MPMWCTL 5.01"

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
#include "pidlib.h"
#include "robot.h"
#include "joyslib.h"

//----------------------------------------------------------
// LCD
//
// Контакты и порты
//
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
#define PIN_REFLEX_INDICATOR 12 // Индикатор срабатывания рефлекса

/// Внешнее напряжение питания (№ АЦП)
// #define VCC1         A0
// #define VCC2         A1

// Считывание УЗ датчиков
#define US_START_READING  7
#define REFLEX_SWITCH     9

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
#define PIN_BEEP       9

/// Подключение энкодеров
#define PIN_encoderLeft   3 /**< Левый энкодер */
#define PIN_encoderRight  2 /**< Правый энкодер */

// Подключение двигателей. Управление скоростью
int PIN_ROT = 5;  // вращательное
int PIN_TAN = 6;  // поступательное

//----------------------------------------------------------
// Базовые параметры и двигательные функции
//----------------------------------------------------------
double V_TO_ADC = 255 / 5.0; // перевод управляющего напряжения в соотв. значение для analogWrite

double MIN_V = 0;//1.1; // минимальное напряжение управляющего сигнала скорости,
                    // соответствующее 0 скорости
double MAX_V = 5;//3.8; // максимальное напряжение управляющего сигнала скорости,
                    // соответствующее максимальной скорости
                    
// Минимальный и максимальный выходной ШИМ сигналы скорости колёс,
// соответствующие диапазону напряжений MIN_V - MAX_V В
const int jstMIN_PWM = MIN_V * V_TO_ADC;
const int jstMAX_PWM = MAX_V * V_TO_ADC;

// Диапазон регулирования внутренних скоростей (все скорости от -255 до +255
#define MIN_PWM   0
#define MAX_PWM 255

// Значения нижеследующих переменных будут определены в функции Regs2CtlVars(void)
// Они являются производными от значений регистров
float minU = 0, maxU = 0;
float minSpeed = 0;         /// Начальная (скорость трогания) и максимальная скорости
float maxSpeed = 0;

float maxErr = 0;    /// Максимальная ошибка управления

const int DELTA_PWM = 0.15 * V_TO_ADC; // поправка на потери в цепи (0.15 * )

int CSPEED = 0;  /// Крейсерская скорость 
int RSPEED = 0;  /// Скорость разворота

bool Reflex_Flag = false;

//---------------------------------------------------------------------------------------------------
//
//---------------------------------------------------------------------------------------------------
TJoystick JST(-100, 100,
              -100, 100,
              -255, 255);

//---------------------------------------------------------------------------------------------------
/**
 * Геометрия робота
 */
#define RWD     350   /// Диаметр колеса, мм
#define RLW     820   /// Расстояние между колесами, мм
#define RECnt  2000   /// Количество импульсов энкодеров на 1 оборот колеса
                      /// (по паспорту - 1000, но т.к. мы регистрируем изменения сигнала, а не сам сигнал, то будем писать в 2 раза больше)
/**
 * ПИД-регулятор
 */
float Kp = 0.5, Ki = 1.0, Kd = 0.5; // 0.5 0.52 0.0.5

//---------------------------------------------------------------------------------------------------

/* Converts speed from [-100;100] to [jstMIN_PWM; jstMAX_PWM] (-100 corresponds to forward motion 
 *  and to right rotation)
 * speed: [-100;+100]
 *  
 *  Conversion considers:
 *  -100 -> jstMIN_PWM
 *  0 -> (jstMIN_PWM + jstMAX_PWM) / 2
 *  100 -> jstMAX_PWM
 */
int getPWMFromSpeed(int speed)
{
  // convert
  int pwmspeed = map(speed, -100, 100, jstMIN_PWM, jstMAX_PWM);
  
  // поправка на потери в цепи
  //pwmspeed += DELTA_PWM;
  
  // check constraints
  if(pwmspeed<jstMIN_PWM) pwmspeed = jstMIN_PWM;
  if(pwmspeed>jstMAX_PWM) pwmspeed = jstMAX_PWM;
  
  return pwmspeed;
}

/* Sends rotation speed command to motor controller after converting it to an appropriate format.
 * speed: [-100;+100]
 */
void MotorSetRot(int speed)
{  
  // gets converted speed to PWM format
  int pwmspeed = getPWMFromSpeed(speed);
  
  // send command to the wheel
  analogWrite(PIN_ROT, pwmspeed);
#ifdef DEBUG_OUTPUT
  Serial.print("r = "); Serial.print(pwmspeed); Serial.print(" ");
#endif
}

/* Sends tangential speed command to motors after converting it to an appropriate format.
 * speed: [-100;+100]
 */
void MotorSetTan(int speed)
{
  // gets converted speed to PWM format
  int pwmspeed = getPWMFromSpeed(-speed); // NOTE: minus sign because forward
    // and backward direction is inverted in the current version

  if(Reflex_Flag && speed < 0)
    pwmspeed = getPWMFromSpeed(0);
    
  // send command to the wheel
  analogWrite(PIN_TAN, pwmspeed);
#ifdef DEBUG_OUTPUT
  Serial.print("t = "); Serial.println(pwmspeed);
#endif
}

//-----------------------------------------------------------------------------------

void MotorsInit(void)
{
  pinMode(PIN_ROT, OUTPUT);
  pinMode(PIN_TAN, OUTPUT);  
}

/**
 * Базовые двигательные функции
 */
#define INCORRECT_SPEED 1000
int cxLeftSpeed = INCORRECT_SPEED; // Начальное значение является заведомо некорректным
int cxRightSpeed = INCORRECT_SPEED;

void ExecuteMCtl(void)
{
  if(cxRightSpeed == INCORRECT_SPEED) return;
  if(cxLeftSpeed == INCORRECT_SPEED) return;

  int joystick_ROT, joystick_TAN;
  // Получить положение джойстика по значению сигналов колёс робота.
  int res = JST.u2j(cxLeftSpeed, cxRightSpeed, &joystick_ROT, &joystick_TAN);
#ifdef DEBUG_OUTPUT
  Serial.print("Mctl: "); Serial.print(cxLeftSpeed); Serial.print(","); Serial.print(cxRightSpeed); Serial.print(" -> ");
#endif
  if(res==0)
  {    
#ifdef DEBUG_OUTPUT
    Serial.print(joystick_ROT); Serial.print(","); Serial.println(joystick_TAN);
#endif    
    MotorSetRot(joystick_ROT);
    MotorSetTan(joystick_TAN);
  }
#ifdef DEBUG_OUTPUT
  else
    Serial.println("????");
#endif
}

void MotorLeftGo(int pwmspeed)
{
  cxLeftSpeed =  pwmspeed;
  cxRightSpeed = INCORRECT_SPEED;
  ExecuteMCtl();
}

void MotorRightGo(int pwmspeed)
{
  cxRightSpeed = pwmspeed;
  ExecuteMCtl();
}

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
  minU = REG[REG_MINU];
  maxU = REG[REG_MAXU];
  minSpeed = REG[REG_MINSPEED];  // Скорость трогания
  maxSpeed = REG[REG_MAXSPEED];
  //pwmspeed = REG[REG_PWMSPEED];  // Скорость движения

  // Производные величины
  maxErr = ((float)maxSpeed/2);  // Максимальная ошибка управления
  //  MAX_PWM = REG[REG_MAXSPEED];
  CSPEED = (MAX_PWM*2/3);  // Скорость движения
  RSPEED = CSPEED;         // Скорость разворота (CSPEED*2/3)
}

void TestMotors1(void)
{
  #define T 2000
  // fwd
  MotorSetRot(0);
  MotorSetTan(100);
  delay(T);
  
  MotorSetRot(0);
  MotorSetTan(0);
  delay(T);

  // back
  MotorSetRot(0);
  MotorSetTan(-100);
  delay(T);
    
  MotorSetRot(0);
  MotorSetTan(0);
  delay(T);

  // left
  MotorSetRot(-100);
  MotorSetTan(0);
  delay(T);
    
  MotorSetRot(0);
  MotorSetTan(0);
  delay(T);

  // right
  MotorSetRot(100);
  MotorSetTan(0);
  delay(T);
    
  MotorSetRot(0);
  MotorSetTan(0);
}

void TestMotors2(void)
{
  Serial.println("TestMotors");
  Serial.print("Cspeed = "); Serial.print(CSPEED);  
  Serial.print(" Rspeed = "); Serial.println(RSPEED);

  #define T 2000
  Serial.println("goFwd");
  goFwd(CSPEED);
  delay(T);
  
  Serial.println("goBack");
  goBack(CSPEED);
  delay(T);
  
  Serial.println("goLeft");
  goLeft(RSPEED);
  delay(T);
  
  Serial.println("goRight");
  goRight(RSPEED);
  delay(T);
  
  Serial.println("goFastLeft");
  goFastLeft(RSPEED);
  delay(T);
  
  Serial.println("goFastRight");
  goFastRight(RSPEED);   
  delay(T);

  Serial.println("Stop");
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

#ifdef DEBUG_OUTPUT
    Serial.println(Title);
#endif
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
  
  REG[REG_ID] = EEPROM.read(0);
  MY_ADDR = REG[REG_ID];
  if(MY_ADDR==0 || MY_ADDR==255)
  {
    MY_ADDR = REG[REG_ID] = 1;
    EEPROM.write(0,MY_ADDR);
  }
  CtlVars2Regs();
  Regs2CtlVars();

  // ПИД-регулятор робота
  robot.InitPID(Kp, Ki, Kd, -100.0, 100.0, minU, (float)maxU, maxErr);
  // Геометрия и ноги энкодеров
  // Период прерываний таймера
  unsigned long tns = 50000;    //  50'000 микросекунд (или 0.05 сек - или 20Hz)
  robot.Init(tns, RECnt, RWD, RLW, PIN_encoderLeft, PIN_encoderRight);
  
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

  MotorsInit();
  // joystick init
  robot.Stop();
  delay(2000);
  Beep(1);

  //------------------------------------------------------------------------
  //*** Тестовые движения (проверка подключения двигателей)
  // Обратите внимание на последовательность движений:
  //   1. Вперед
  //   2. Назад
  //   3. Налево быстро
  //   4. Направо быстро
  //------------------------------------------------------------------------
  //TestMotors1();
  //TestMotors2();
  //robot.TestEncoders();
  //TestComplexPID(CSPEED, RSPEED);

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
    //if(addr != MY_ADDR && addr != 255) return;
    from = rcGetFrom(&pckg);
    CMD = rcGetCmd(&pckg);

    if(IS_MOVE_CMD(CMD)) //sic
    {
      // Прямое задание скорости
      robot.set_speed_regime = (CMD == CMD_SET_SPEED);
      if(IS_REENTERABLE_MOVING(LAST_MOVECMD) && (CMD != CMD_SET_SPEED))
      {
        robot.Stop();
        robot.mvstatus=TRobot::MVC_READY;
      }
      LAST_MOVECMD = CMD;
    }

    REG[REG_D1] = robot.Cnt2L(GlobalEncoderLeftCnt);
    REG[REG_D2] = robot.Cnt2L(GlobalEncoderRightCnt);
    switch(CMD)
    {
      case CMD_PING:
        break;
      case CMD_STOP:
        robot.Stop();
        break;
      case CMD_FWD:
        if(obst) break;
        goFwd(CSPEED);
        break;
      case CMD_FWD2:
        if(obst) break;
        rcGet1B(&pckg, &val);
        robot.reStartMove(10*val, CSPEED, CSPEED);
        break;
      case CMD_BACK:
        goBack(CSPEED);
        break;
      case CMD_BACK2:
        rcGet1B(&pckg, &val);
        robot.reStartMove(10*val, -CSPEED, -CSPEED);
        break;
      case CMD_LEFT:
        if((obst & OBST_BUMP_RIGHT)) break;
        goLeft(RSPEED);
        break;
      case CMD_RIGHT:
        if((obst & OBST_BUMP_LEFT)) break;
        goRight(RSPEED);
        break;
      case CMD_FAST_LEFT:
        goFastLeft(RSPEED);
        break;
      case CMD_FAST_LEFT2:
        rcGet1B(&pckg, &val);
        angle = val;
        robot.reStartRotate(angle, RSPEED);
        break;
      case CMD_FAST_RIGHT:
        goFastRight(RSPEED);   
        break;
      case CMD_FAST_RIGHT2:
        rcGet1B(&pckg, &val);
        angle = -val;
        robot.reStartRotate(angle, RSPEED);
        break;
      case CMD_SET_SPEED:
        rcGet2B(&pckg, &b1, &b2); 
        robot.GoalSpeedLeft = (signed char)b1;
        robot.GoalSpeedRight = (signed char)b2;
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
        if(n==REG_D1) GlobalEncoderLeftCnt = 0;
        if(n==REG_D2) GlobalEncoderRightCnt = 0;
        Regs2CtlVars();
        break;
      case CMD_GET_REG:
        rcGet1B(&pckg, &n);
        val = REG[n];
        rcSend1B(&pckg, 0, CMD_ANS_GET_REG, val);
        break;
      case CMD_GET_ALL_REG:
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
  robot.reMoveStep();
  // Регистр статуса
  REG[REG_STATUS] = (robot.mvstatus << 4) | obst;
}

