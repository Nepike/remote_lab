 /**
 * \file mpmanip.ino
 * Progect:
 *  Макет подвижной платформы
 *  Управление манипулятором
 *  Протокол RCX2
 *  Baudrate 57600
 *  Все значения датчиков считываются в 8-разрядном формате
 *  Chip ATmega328
 *  \author Robofob, Воробьев В.В.
 *  \version 1.03
 *  \date 28.07.2015
 *  \date Last Change: 03.11.2015
 */

//-------------------------------------------
// Очень странный финт
//typedef unsigned char BYTE;
#define BYTE unsigned char
//-------------------------------------------

#include <TimerOne.h>

#include "rcproto.h"
#include "maestro.h"
#include "Servo.h"

/// Адрес контроллера
BYTE MY_ADDR = 3;

//------------------------------------------------------------------

/// Пищалка
#define PIN_BEEP  10

//----------------------------------------------------------
// Базовые двигательные функции
//----------------------------------------------------------

// Минимальная и максимальная скорости двигателя, выдвигающего манипулятор
#define MIN_SPEED  -250
#define MAX_SPEED   250
#define STOP 0

// Подключение двигателей. № пинов
#define PIN_MOTOR1   7
#define PIN_MOTOR2   6
#define PIN_SERV1    9
#define PIN_SERV2   10

// Подключение концевых датчиков двигателя. № пинов
#define SENSOR1  11
#define SENSOR2  12

// Параметры сервомашинок
#define ANGSTEP   5     // Шаг сервомашинки (град)
#define INTERVAL 50     // Интервал срабатывания (мкс)
#define SERVSTARTPOS 90 // Стартовая позиция сервомашинки (град)
#define MAXANGLE 160    // Максимальный угол поворота сервомашинки
#define MINANGLE 10     // Минимальный угол поворота сервомашинки

#define BAUD_RATE 57600 // 115200 // Скорость com-порта

/// Здесь хранятся значения датчиков
#define ADCNUM 8

/// Количество регистров
#define NREG 15

int flag = 0;

// Класс, описывающий сервомашинку
class Sweeper
{
  private:  
    int _Step; // Шаг сервомашинки (град)
    int _UpdateInterval; // Интервал срабатывания (мкс)
    unsigned long _LastUpdate; // Последнее время обновления
    int _MaxAngle; // Максимальный угол поворота сервомашинки
    int _MinAngle; // Минимальный угол поворота сервомашинки
    int _Angle; // Текущее положение сервы  
  public: 
    int _Continue;  
    Servo _Servo; // Объект servo
    Sweeper(void)
    {
      _UpdateInterval = INTERVAL;
      _Step = ANGSTEP;
      _MaxAngle = MAXANGLE;
      _MinAngle = MINANGLE;
      _Continue = 0;
    }  
    void Attach(int pin)
    {
      _Servo.attach(pin);
    }      
    void SetPosition(int angle)
    {
      _Servo.write(angle);
      _Angle = angle;
    }   
    void Update(unsigned long currentmillis)
    {
      if((currentmillis - _LastUpdate) > _UpdateInterval) // пришло время обновляться
      {
        _LastUpdate = millis();
        _Angle += _Step;
        _Servo.write(_Angle);
        if ((_Angle >= _MaxAngle) || (_Angle <= _MinAngle)) // конец вращения
        {
         // обратное направление
         _Step = -_Step;
         _Continue++;
        }
      }
    }
    ~Sweeper(void)
    {
      _Servo.detach();
    }
};

Sweeper sweeper1;
Sweeper sweeper2;

TPckg pckg;

enum {Q_START, Q_GO_HOME, Q_GO_WORK, Q_AT_HOME, Q_AT_WORK, Q_CALIB1, Q_CALIB2};

BYTE QA;

void MotorsInit(void)
{
  pinMode(PIN_MOTOR1, OUTPUT);
  pinMode(PIN_MOTOR2, OUTPUT);
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  digitalWrite(SENSOR1, HIGH);
  digitalWrite(SENSOR2, HIGH);
}

void BaseActions(void)
{
  MotorGo(MAX_SPEED);
  delay(1000);
  MotorGo(STOP);
  
  MotorGo(MIN_SPEED);
  delay(1000);
  MotorGo(STOP);  
}

//----------------------------------------------------------

void MotorGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR1, HIGH);
    digitalWrite(PIN_MOTOR2, HIGH);
  }
  else
    if(pwmspeed>0)
    {
      digitalWrite(PIN_MOTOR1, LOW);
      analogWrite(PIN_MOTOR2, pwmspeed);
    }
    else
    {
      digitalWrite(PIN_MOTOR1, HIGH);
      analogWrite(PIN_MOTOR2, (int)255+pwmspeed);
    }
}

int GetSensStat(void)
{
  int s1 = !digitalRead(SENSOR1);
  int s2 = !digitalRead(SENSOR2);

  return s1+s2*2;
}

void TestSensors(void)
{
  int s;
  while(1)
  {
    s = GetSensStat();
    Serial.println(s);
  }
}

//----------------------------------------------------------

void setup()
{
  Serial.begin(BAUD_RATE);
  
  sweeper1.Attach(PIN_SERV1);
  sweeper2.Attach(PIN_SERV2);
  sweeper1.SetPosition(SERVSTARTPOS);
  sweeper2.SetPosition(SERVSTARTPOS);
  delay(1000);

  MotorsInit(); // Инициализация пинов контроллера
  
  //BaseActions(); // Базовые действия манипулятора
  
  MotorGo(STOP);
  QA = Q_START;
  
  // Протокол rcX2
  TPckgInit(&pckg);

  // Тест датчиков
  //TestSensors();  
 
}
//------------------------------------------------------------------------

unsigned char rcReadByte(void) { return Serial.read(); }
void rcWriteByte(unsigned char c) { Serial.write(c); }
unsigned char rcWasByte(void) { return (Serial.available()>0); }
void rcError(unsigned char  n) {}

//----------------------------------------------------------

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

void WriteSensors(BYTE addr)
{
  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_SENS);
  rcWriteByte(ADCNUM);

  BYTE s = GetSensStat();

  rcWriteByte(QA);
  for(BYTE i=1;i<ADCNUM;i++)
    rcWriteByte(s);
  rcWriteByte(CS_VALUE);
}

void GetAllRegs(BYTE addr)
{
  rcWriteByte(HDR_BYTE);
  rcWriteByte(HDR_BYTE);
  rcWriteByte(addr);
  rcWriteByte(MY_ADDR);
  rcWriteByte(CMD_ANS_GET_ALL_REG);
  rcWriteByte(NREG);
  for(BYTE i=0;i<NREG;i++)
    rcWriteByte(QA);
  rcWriteByte(CS_VALUE);
}

int ManipIn(void)
{
  int s;
  s = GetSensStat();
  if (s != 1)
    MotorGo(MIN_SPEED);
  else
    MotorGo(STOP); 
  return (s==1);
}

int ManipOut(void)
{
  int s;
  s = GetSensStat();
  if (s != 2)
    MotorGo(MAX_SPEED); 
  else
    MotorGo(STOP); 
  return (s==2);
}

//----------------------------------------------------------
// Управляющий автомат
//----------------------------------------------------------

void ExecuteAutomaton(void)
{
  int n = 0;
  switch(QA)
  {
    case Q_START:
      MotorGo(0);
      break;
    case Q_GO_HOME:
      n = ManipIn();
      if(n) QA = Q_AT_HOME;
      break;
    case Q_GO_WORK:
      n = ManipOut();
      if(n) QA = Q_AT_WORK;
      break;
    case Q_CALIB1:
      // Timer0 уже используется millis() - прерываемся где-то
      // посередине и вызываем ниже функцию "Compare A"
      MotorGo(STOP);
      OCR0A = 0xAF;
      TIMSK0 |= _BV(OCIE0A);
      break; 
    case Q_AT_HOME:
    case Q_AT_WORK:
      MotorGo(0);
      QA = Q_START;  
      break;
  }
}

SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  if (QA == Q_CALIB1)
  {
    sweeper1.Update(currentMillis);
    if (sweeper1._Continue == 2)
    {
      QA = Q_CALIB2;
      sweeper1._Continue = 0;
      sweeper1.SetPosition(SERVSTARTPOS);
    }
  }
  if (QA == Q_CALIB2)
  {
    sweeper2.Update(currentMillis);
    if (sweeper2._Continue == 2)
    {
      QA = Q_START;
      sweeper2._Continue = 0;
      sweeper2.SetPosition(SERVSTARTPOS);
    }
  }
}

//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------

void loop()
{ 
  unsigned char n, val;
  unsigned char from = 0;
  char c;
  unsigned char addr, CMD;

  if(rcReadPackage(&pckg)==DATA_READY)
  {
    addr = rcGetAddr(&pckg);
    if(addr!=MY_ADDR) return;
    from = rcGetFrom(&pckg);
    CMD = rcGetCmd(&pckg);

    switch(CMD)
    {
      case CMD_PING:
        break;
      case CMD_STOP:
        QA = Q_START;
        break;
      case CMD_FWD:
        QA = Q_GO_WORK;       
        break;
      case CMD_FWD2:
        break;
      case CMD_BACK:
        QA = Q_GO_HOME;
        break;
      case CMD_BACK2:
        break;
      case CMD_LEFT:
        QA = Q_CALIB1;
        break;
      case CMD_RIGHT:
        break;
      case CMD_FAST_LEFT:
        break;
      case CMD_FAST_LEFT2:
        break;
      case CMD_FAST_RIGHT:
        break;
      case CMD_FAST_RIGHT2:
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
        break;
      case CMD_GET_REG:
        rcSend1B(&pckg, 0, CMD_ANS_GET_REG, QA);
        break;
      case CMD_GET_ALL_REG:
        GetAllRegs(from);
        break;
      case CMD_I2C: // Команда I2C-устройству. Формат: <адрес устройства> <команда> <аргумент1> <аргумент2>
        break;
      default:
        rcError(ERR_ILLEGAL_CMD);
    }
    if((!IS_NOT_ACK_CMD(CMD)) || CMD==CMD_PING)
      rcSendACK(&pckg, 0);
  }
  ExecuteAutomaton();  
}

