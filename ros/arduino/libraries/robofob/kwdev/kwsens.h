/*
 * KwSensors
 * Version 1.03
 * 22.08.2018
 * LP 02.12.2018
 */
 
#ifndef _KWSENSORS_H_
#define _KWSENSORS_H_

#include <Wire.h>
#include <BH1750.h>
#include "rbuff.h"

/* ОПРЕДЕЛЕНИЕ ПОЛОЖЕНИЯ МОДУЛЯ ТОЛЬКО ПО АКСЕЛЕРОМЕТРУ 
   (строки можно и не писать, но их наличие освобождает память программ за счет неиспользуемого датчика)
*/
// Не использовать гироскоп
#define BMX055_DISABLE_BMG

// Не использовать магнитометр
#define BMX055_DISABLE_BMM

// Подключаем библиотеку iarduino_Position_BMX055 для работы с Trema-модулем IMU 9 DOF.
#include <iarduino_Position_BMX055.h>

//iarduino_Position_BMX055 sensor(BMA);
/***
#ifndef fmap
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
#endif
***/

class TSensor
{
private:  
  TRBuffer<float, 4> buff;
  float s_min, s_max;
  const char *name;
  float pred_x;

public:
  TSensor(char *nm, float min_sens_val, float max_sens_val):name(nm)
  {
    s_min = min_sens_val;
    s_max = max_sens_val;    
    pred_x = 0;
  }
  void Read(void)
  {
    float r = _read();
    SaveData(r);
  }
  char *Name(void) { return name; }
  float GetPh(void) { return buff.GetVal(); }
  float GetNorm(void)
  { 
    float r = buff.GetVal();
    float normval = fmap(r, s_min, s_max, 0, 1);
    return normval;
  }
  float GetVal(float &ph, float &x, float &dx)
  { 
    ph = buff.GetVal();
    x = fmap(ph, s_min, s_max, 0, 1);
    dx = x - pred_x;
    pred_x = x;
  }

protected:
  virtual float _read(void) = 0;
  void SaveData(float r)
  {
    if(r<s_min) r = s_min;
    if(r>s_max) r = s_max;
    buff.Add(r);    
  }
};

class TSensAnalog:public TSensor
{
protected:  
  int sPin;
  virtual float _read(void)
  {
    int r = analogRead(sPin);
    return r;
  }
public:  
  TSensAnalog(char *nm, int sensorPin, float min_sens_val, float max_sens_val):
    TSensor(nm, min_sens_val, max_sens_val)
  {
    sPin = sensorPin;
    //pinMode(sPin, INPUT);    
  }
};

class TSensTMP36:public TSensAnalog
{
public:  
  TSensTMP36(char *nm, int sensorPin, float min_sens_val, float max_sens_val): 
    TSensAnalog(nm, sensorPin, min_sens_val, max_sens_val) { }
protected:
  virtual float _read(void)
  {
    int r = analogRead(sPin);
    float v = (float)r * 5.0/1024.0; // Преобразуем показания в напряжение
    float tC = (v - 0.5) * 100; // Определяем температуру, исходя из 10 мВ на 1 градус со смещением 500 мВ
    //tC = tC - 19; // Странная поправка
    return tC;
  }
};

class TSensLuxBH750:public TSensor
{
private:  
  BH1750 lightMeter;
  
protected:  
  virtual float _read(void)
  {
    uint16_t lux = lightMeter.readLightLevel();
    return (float)lux;
  }

public:  
  TSensLuxBH750(char *nm, float min_sens_val, float max_sens_val): TSensor(nm, min_sens_val, max_sens_val)
  {
    lightMeter.begin();
  }
};


iarduino_Position_BMX055 Position_BMX055_sensor(BMA);

class TSensAccYBMX055:public TSensor
{
private:  
  /*
    Создаём объект sensor указывая что ему требуется работать только с акселерометром.
    Если указать параметр BMA, то объект будет работать только с акселерометром.
    Если указать параметр BMG, то объект будет работать только с гироскопом.
    Если указать параметр BMM, то объект будет работать только с магнитометром.
    Если указать параметр BMX, то объект будет работать со всеми датчиками сразу.
  */
 
protected:  
  virtual float _read(void)
  {
    /*
      Функция read() читает данные того датчика, для которого был создан объект.
      Для объекта работающего с акселерометром, функция read() может принять
      один из четырёх параметров: BMA_M_S, BMA_G, BMA_DEG, или BMA_RAD.
      Если параметра нет, то используется параметр по умолчанию
      sensor.read(BMA_M_S); читать угловое ускорение в м/с² (по умолчанию).
      sensor.read(BMA_G);   читать угловое ускорение в g.
      sensor.read(BMA_DEG); читать углы «крен» и «тангаж» в градусах.
      sensor.read(BMA_RAD); читать углы «крен» и «тангаж» в радианах.
      Данные прочитанные функцией read() сохраняются в переменных axisX, axisY, axisZ и temp.
    */
    Position_BMX055_sensor.read(BMA_DEG);                                  
    return (float)Position_BMX055_sensor.axisY; // Крен
  }

public:  
  TSensAccYBMX055(char *nm, float min_sens_val, float max_sens_val): TSensor(nm, min_sens_val, max_sens_val)
  {
    Position_BMX055_sensor.begin();
  }
};

#endif
