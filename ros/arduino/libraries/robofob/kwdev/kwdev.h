/*
 * KwDevices
 * 
 * Version 1.03
 * 
 * 22.08.2018, 24.05.2022
 * LP 19.06.2023
 */
 
#ifndef _KWDEVICES_H_
#define _KWDEVICES_H_

#include <Arduino.h>
#include "rbuff.h"
#include "maestro2.h"
#include "smc.h"
#include "DFRobotDFPlayerMini.h"

#ifndef fmap
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
#endif

namespace kwdev
{
  extern Tsmc SMC;
  extern TMServo Servo;
  
  extern unsigned long T_WCNT; /**< Глобальный счетчик времени */
  extern char sline[]; // Вспомогательный буфер для формирования строки

  // Диапазон скоростей
  extern int speed_min;
  extern int speed_max;

  // Значение, символизирующее случайный выбор
  extern const int MAGIC_VALUE;

  extern const int ANGLE_MIN;
  extern const int ANGLE_MAX;

  extern const int SRV_ID_DEV; // Идентификатор устройства Pololu Maestro. По умолчанию = 12

  extern Stream *MotorStream;
  extern Stream *ServoStream;

  // Точность позиционирования
  extern int eps;
  extern bool debug;
}

/*
 *  Абстрактный класс исполнительных устройств
 * 
 */
struct TDevice
{
  TDevice(char *nm, int minb, int maxb)
  {
    min_bound = minb;
    max_bound = maxb;
    strcpy(name, nm);
  }
  virtual void Enable(void) = 0;
  virtual int Eval(void) = 0;
  virtual void SetGoal(int g, int arg = 0) = 0;
  virtual void Reset(void) = 0;  
  virtual void GetStatus(void) = 0;
  virtual int GetCurrPos(void) = 0;
  
  virtual void _dctl(int val) = 0; // Прямое управление
  virtual void getR0R1(int &vr0, int &vr1) { vr0 = vr1 = 0; }

  int min_bound;
  int max_bound;
  //const char *name;
  char name[12];
};

/*
 * Актуатор
 */
struct TActuator:TDevice
{
public:
  /*
   * addr - адрес устройства
   * adc - номер АЦП, к которому присоединен датчик положения
   * minr, maxr - минимальное и максимальное значение АЦП (при крайних положениях). От 0 до 255 (adc<<2)
   * Если minr, maxr < 0, то актуатор работает без обратной связи, по времени
   */
  TActuator(char *nm, byte addr, byte adc, int minr, int maxr, int minb, int maxb): TDevice(nm, minb, maxb)
  {
    devaddr = addr;
    adcnum = adc;
    r0 = minr;
    r1 = maxr;
    currspeed = currpos = 0;
    goalpos = -1;
    timeDelayRegime = ((r0<0 || r1<0)) || ((r0==0 && r1==0));
    ct = 0;
  }
  virtual void Enable(void) { kwdev::SMC.exitSafeStart(devaddr); };
  virtual int Eval(void);
  virtual void SetGoal(int g, int arg = 0) 
  {
    if(g == kwdev::MAGIC_VALUE) g = random(min_bound, max_bound);
    else
    {
      if(g>max_bound) g = max_bound;
      if(g<min_bound) g = min_bound;
    }
    if(!timeDelayRegime && g<0) g = 0;
    goalpos = g;
    ct = kwdev::T_WCNT; 
  };
  virtual void Reset(void) { Enable(); goalpos = currpos; }
  virtual void GetStatus(void) 
  { 
    int cr = analogRead(adcnum)>>2;
    sprintf(kwdev::sline, "(%s %d cspeed=%d cpos=%d R=%d)", name, devaddr, currspeed, currpos, cr); 
  }
  virtual int GetCurrPos(void) { return currpos; }
  virtual void _dctl(int val) { kwdev::SMC.Go(devaddr, val); }

private:  

  byte devaddr; // Адрес драйвера
  byte adcnum;  // Канал АЦП для датчика положения
  int r0, r1;
  int currspeed;
  int goalpos; // Целевая позиция
  int currpos; // Текущая позиция

  boolean timeDelayRegime;
  // Для работы с таймером
  unsigned long ct;
  TRBuffer<int, 2> rb;
};

/*
 * Аналог сервомашинки
 */
struct TServoL298:TDevice
{
public:
  /*
   * nm - имя
   * pdir - нога направления движения
   * ppwm - нога скорости движения
   * adc - номер АЦП, к которому присоединен датчик положения
   * minr, maxr - минимальное и максимальное значение АЦП (при крайних положениях). От 0 до 255 (adc<<2)
   * Если minr, maxr < 0, то актуатор работает без обратной связи, по времени
   */
  TServoL298(char *nm, byte pdir, byte ppwm, byte adc, int minr, int maxr, int minb, int maxb): TDevice(nm, minb, maxb)
  {

    adcnum = adc;
    pin_dir = pdir;
    pin_pwm = ppwm;
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);

    r0 = minr;
    r1 = maxr;
    currspeed = currpos = 0;
    goalpos = -1;
    timeDelayRegime = ((r0<0 || r1<0)) || ((r0==0 && r1==0));
    ct = 0;
    _dctl(0);
  }
  virtual void getR0R1(int &vr0, int &vr1) { vr0 = r0; vr1 = r1;}
  virtual void Enable(void) {};
  virtual int Eval(void);
  virtual void SetGoal(int g, int arg = 0) 
  {
    if(g == kwdev::MAGIC_VALUE) g = random(min_bound, max_bound);
    else
    {
      if(g>max_bound) g = max_bound;
      if(g<min_bound) g = min_bound;
    }
    goalpos = g;
  };
  virtual void Reset(void) { Enable(); goalpos = currpos; }
  virtual void GetStatus(void) 
  { 
    int cr = analogRead(adcnum)>>2;
    sprintf(kwdev::sline, "(%s cspeed=%d cpos=%d R=%d)", name, currspeed, currpos, cr); 
  }
  virtual int GetCurrPos(void) { return currpos; }
  virtual void _dctl(int val);

private:  
  int pin_dir, pin_pwm;
  int cspeed;
  byte adcnum;  // Канал АЦП для датчика положения

  int r0, r1;
  int currspeed;
  int goalpos; // Целевая позиция
  int currpos; // Текущая позиция

  boolean timeDelayRegime;
  // Для работы с таймером
  unsigned long ct;
  TRBuffer<int, 2> rb;
};

//------------------------------------------------------------------------------
/*
 * Драйвер двигателя
 */
struct TL298:TDevice
{
public:  
  /*
   * nm - имя
   * pdir - нога направления движения
   * ppwm - нога скорости движения
   */
  TL298(char *nm, int pdir, int ppwm, int minb, int maxb): TDevice(nm, minb, maxb)
  {
    pin_dir = pdir;
    pin_pwm = ppwm;
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
    _dctl(0);
  }

  virtual void Enable(void) {};
  virtual int Eval(void) {};
  virtual void SetGoal(int g, int arg = 0) { _dctl(g); };
  virtual void Reset(void) { _dctl(0); };
  virtual void GetStatus(void) { sprintf(kwdev::sline, "(%s %d %d 0)", name, pin_pwm, cspeed); }
  virtual int GetCurrPos(void) { return cspeed; }
  virtual void _dctl(int mspeed);
  
private:
  int pin_dir, pin_pwm;
  int cspeed;
};


//------------------------------------------------------------------------------
/*
 * Устройство PWM (например, твердотельное реле)
 */
struct TPWMDevice:TDevice
{
public:  
  TPWMDevice(char *nm, int pin, int minb, int maxb): TDevice(nm, minb, maxb)
  {
    pin_pwm = pin;
    pinMode(pin_pwm, OUTPUT);
    Go(0, 0);
  }

  virtual void Enable(void) {};
  virtual int Eval(void) {};
  virtual void SetGoal(int g, int arg = 0) { Go(g, arg); };
  virtual void Reset(void) { Go(0); };
  virtual void GetStatus(void) { sprintf(kwdev::sline, "(%s %d %d 0)", name, pin_pwm, cspeed); }
  virtual int GetCurrPos(void) { return cspeed; }
  virtual void _dctl(int val) { Go(val); }
  
private:
  void Go(int val, int arg = 0)
  // val – значение от 0 до 100
  {
    if(val == kwdev::MAGIC_VALUE) val = random(min_bound, max_bound);
    else
    {    
      if(val>max_bound) val = max_bound;
      if(val<min_bound) val = min_bound;  
    }
    val = map(val, min_bound, max_bound, 0, 255);
    cspeed = val;
    analogWrite(pin_pwm, val);
  }  
  int pin_pwm;
  int cspeed;
};

/*
 * Серво
 */

struct TServo:TDevice
{
public:  
  TServo(char *nm, byte cn, int minb, int maxb): TDevice(nm, minb, maxb)
  {
    id_dev = kwdev::SRV_ID_DEV;
    channel = cn;
    Go(0);
  }

  virtual void Enable(void) {};
  virtual int Eval(void) {};
  virtual void SetGoal(int g, int arg = 0) { Go(g, arg); };
  virtual void Reset(void) { Go(0, 0); };
  virtual void GetStatus(void) { sprintf(kwdev::sline, "(%s %d %d 0)", name, channel, cspeed); }
  virtual int GetCurrPos(void) { return cspeed; }
  virtual void _dctl(int val) {}
  
private:
  byte id_dev;
  byte channel;
  void Go(int angle, int arg = 0)
  // angle – значение от -100 до 100
  {
    if(angle == kwdev::MAGIC_VALUE) angle = random(min_bound, max_bound);
    else
    {        
      if(angle>max_bound) angle = max_bound;
      if(angle<min_bound) angle = min_bound;
    }
    cspeed = angle;
    byte a = map(angle, min_bound, max_bound, kwdev::ANGLE_MIN, kwdev::ANGLE_MAX);
    kwdev::Servo.SetAng(id_dev, channel, a);
    delay(10);
  }  
  int cspeed;
};


struct TMP3Player:TDevice
{
public:  
  TMP3Player(char *nm, Stream &stream, int minb, int maxb): TDevice(nm, minb, maxb)
  {
    songnum = 0;
    myDFPlayer.begin(stream);
    myDFPlayer.volume(15); // Set volume value. From 0 to 30
  }

  virtual void Enable(void) {};
  virtual int Eval(void) {};
  virtual void SetGoal(int g, int vol = 0) { Go(g, vol); };
  virtual void Reset(void) { songnum = 0; };
  virtual void GetStatus(void) { sprintf(kwdev::sline, "(%s %d 0 0)", name, songnum); }
  virtual int GetCurrPos(void) { return songnum; }
  virtual void _dctl(int val) {}
  
private:
  DFRobotDFPlayerMini myDFPlayer;
  int songnum; // Номер фрагмента
  void Go(int sn, int vol = 0)
  {
    songnum = sn;
    if(sn == 0) // Выключаем
    {
      myDFPlayer.reset();
      return;
    }

    if(songnum == kwdev::MAGIC_VALUE) songnum = random(min_bound, max_bound);
    else
    {        
      if(songnum>max_bound) songnum = max_bound;
      if(songnum<min_bound) songnum = min_bound;
    }
    myDFPlayer.play(songnum);
    if(vol==0) vol = 15;
    if(vol<0) vol = 0;
    if(vol>30) vol = 30;
    myDFPlayer.volume(vol); // Set volume value. From 0 to 30
  }  
};

#endif

