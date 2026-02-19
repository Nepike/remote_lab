/*
 * KDevices
 * 
 * Version 1.03
 * 
 * 22.08.2018, 24.10.2018
 * LP 19.06.2023
 */

#include "kwdev.h"

char kwdev::sline[50]; // Вспомогательный буфер для формирования строки
// Диапазон скоростей
int kwdev::speed_min = 0;
int kwdev::speed_max = 100;

// Значение, символизирующее случайный выбор
const int kwdev::MAGIC_VALUE = -999;

const int kwdev::ANGLE_MIN = 0;
const int kwdev::ANGLE_MAX = 90;

const int kwdev::SRV_ID_DEV = 12; // Идентификатор устройства Pololu Maestro. По умолчанию = 12

// Порты
Stream *kwdev::MotorStream;
Stream *kwdev::ServoStream;

// Точность позиционирования
int kwdev::eps = 5;
bool kwdev::debug = false;

//------------------------------------------------------------------------------
/*
 * Драйверы двигателей актуаторов
 */
void smcCWrite(byte c) { kwdev::MotorStream->write(c); }

Tsmc kwdev::SMC(smcCWrite);

//------------------------------------------------------------------------------
/*
 * Сервомашинки
 */

inline void srv_putchar(unsigned char c) { kwdev::ServoStream->write(c); }

unsigned char srv_getchar(void)
{
  unsigned char c;
  c = kwdev::ServoStream->read();
  return c;
}

TMServo kwdev::Servo(srv_putchar, srv_getchar, kwdev::ANGLE_MIN, kwdev::ANGLE_MAX);

//------------------------------------------------------------------------------

/*
 * Таймер
 */
unsigned long kwdev::T_WCNT = 0; /**< Глобальный счетчик времени */

//------------------------------------------------------------------------------

/*
 * Актуатор
 */
int TActuator::Eval(void)
{
  int sp = 0;
  byte stat = 0;
  int cr = 0;

  if(timeDelayRegime) // Без датчика о.с. Работаем со временем
  {
    cr = currpos = (kwdev::T_WCNT-ct);
    if(abs(goalpos)<=currpos)
    {
      goalpos = 0;
      ct = kwdev::T_WCNT;
      sp = 0;
      stat = 1;
    }
    else
      // currpos<goalpos => выдвигаем шток
      sp = (goalpos<0)?-kwdev::speed_max:kwdev::speed_max;
  }
  else
  {
    cr = analogRead(adcnum)>>2;
    int cr100 = map(cr, r0, r1, min_bound, max_bound);

    rb.Add(cr100);
    currpos = rb.GetVal();

    if((abs(goalpos-currpos)<kwdev::eps) || (goalpos<0))
    {
      sp = 0;
      stat = 1;
    }
    else
      // currpos<goalpos => выдвигаем шток
      sp = (currpos<goalpos)?-kwdev::speed_max:kwdev::speed_max;
  }

  if(sp!=currspeed)
  {
    currspeed = sp;
    kwdev::SMC.Go(devaddr, currspeed);
    delay(20);
  }
  if(kwdev::debug)
  {

    if(devaddr==1)
    {
      sprintf(kwdev::sline, "%d: %d %d %d", devaddr, goalpos, currpos, currspeed);
      Serial.println(kwdev::sline);
    }
  }
  return stat;
}


/*
 * Аналог сервомашинки
 */
int TServoL298::Eval(void)
{
  int sp = 0;
  byte stat = 0;
  int cr = 0;

/*
  if((abs(goalpos-currpos)<kwdev::eps) || (goalpos<0))
  {
    sp = 0;
    stat = 1;
    return stat;
  }
*/  
  if(timeDelayRegime) // Без датчика о.с. Работаем со временем
  {
    cr = currpos = (kwdev::T_WCNT-ct);
    if(abs(goalpos)<=currpos)
    {
      goalpos = 0;
      ct = kwdev::T_WCNT;
      sp = 0;
      stat = 1;
    }
    else
      // currpos<goalpos => выдвигаем шток
      sp = (goalpos<0)?-kwdev::speed_max:kwdev::speed_max;
  }
  else
  {
    cr = analogRead(adcnum)>>2;
    //int cr100 = map(cr, r0, r1, min_bound, max_bound);
    int cr100 = map(cr, r0, r1, 0, 100);

    rb.Add(cr100);
    currpos = rb.GetVal();

    if((abs(goalpos-currpos)<kwdev::eps) || (goalpos<0))
    {
      sp = 0;
      stat = 1;
    }
    else
      // currpos<goalpos => двигаем шток
      sp = (currpos<goalpos)?-kwdev::speed_max:kwdev::speed_max;
  }

  if(sp!=currspeed)
  {
    currspeed = sp;
    _dctl(currspeed);
    //delay(20);
  }
  if(kwdev::debug)
  {
    sprintf(kwdev::sline, "EVAL: %d %d %d", goalpos, currpos, currspeed);
    Serial.println(kwdev::sline);
  }
  return stat;
}

void TServoL298::_dctl(int mspeed)
{
  if(mspeed==kwdev::MAGIC_VALUE) mspeed = random(min_bound, max_bound);
  else
  {
    if(mspeed>max_bound) mspeed = max_bound;
    if(mspeed<min_bound) mspeed = min_bound;
  }
  cspeed = mspeed;
  int pwms = map(mspeed, min_bound, max_bound, -255, 255);

  if(pwms==0)
  {
    digitalWrite(pin_dir, HIGH);
    digitalWrite(pin_pwm, HIGH);
  }
  else
  if(pwms>0)
  {
    digitalWrite(pin_dir, LOW);
    analogWrite(pin_pwm, pwms);
  }
  else
  {
    digitalWrite(pin_dir, HIGH);
    analogWrite(pin_pwm, (int)255+pwms);
  }
}

/*
 * Драйвер двигателя
 */
void TL298::_dctl(int mspeed)
{
  if(mspeed==kwdev::MAGIC_VALUE) mspeed = random(min_bound, max_bound);
  else
  {
    if(mspeed>max_bound) mspeed = max_bound;
    if(mspeed<min_bound) mspeed = min_bound;
  }
  cspeed = mspeed;
  int pwms = map(mspeed, min_bound, max_bound, -255, 255);

  if(pwms==0)
  {
    digitalWrite(pin_dir, HIGH);
    digitalWrite(pin_pwm, HIGH);
  }
  else
  if(pwms>0)
  {
    digitalWrite(pin_dir, LOW);
    analogWrite(pin_pwm, pwms);
  }
  else
  {
    digitalWrite(pin_dir, HIGH);
    analogWrite(pin_pwm, (int)255+pwms);
  }
}

