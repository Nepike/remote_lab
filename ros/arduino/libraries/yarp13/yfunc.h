/*
 *
 * V 1.21
 *
 * 06.03.2021, 11.11.2022
 *
 * LP 16.03.2023
 */

#ifndef _YFUNC_H_
#define _YFUNC_H_

#include <Servo.h>
#include <Encoder.h>
#include <Wire.h>

#include <pidlib.h>

//-------------------------------------------------------------------
//  Hardware
//-------------------------------------------------------------------
#define EN_L  10
#define IN1_L  9
#define IN2_L  8

#define EN_R  13
#define IN1_R 12
#define IN2_R 11

#define PIN_ENC_LEFT_1  2 //17
#define PIN_ENC_LEFT_2  4 //18

#define PIN_ENC_RIGHT_1 3 //19
#define PIN_ENC_RIGHT_2 5 //20

// Рабочая индикация
#define PIN_LED1 31 // LED_BUILTIN; //13
#define PIN_LED2 33
#define PIN_LED3 33

// Индикаторы рефлекса питания
#define IND_REFL_POWER  30
// Индикаторы препятствий
#define IND_OBST_LEFT   32
#define IND_OBST_RIGHT  34
#define IND_OBST_CENTER 36

#define PIN_BEEP 35
#define PIN_GUN  37

// Бамперы
#define PIN_BUMP_L 51
#define PIN_BUMP_C 52
#define PIN_BUMP_R 53

// Режимы (обычно - рефлексы)
#define SWITCHES_NUM 8
extern byte PIN_SWITCHES[];

//-------------------------------------------------------------------

// Собственно ПИД-регуляторы
extern TPID2 PID_LEFT;
extern TPID2 PID_RIGHT;

// Конечные масштабные множители для согласования характеристик двигателей
extern float RATIO_LEFT;
extern float RATIO_RIGHT;

// Encoders
extern Encoder encoder_left;
extern Encoder encoder_right;

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

//-------------------------------------------------------------------
// Датчик Sharp
//-------------------------------------------------------------------
class TSharp
{
  public:
    TSharp(float _k = 0.5)
    {
      K = _k;
      A = 0;
      distance = 0;
    }
    int GetDist(int adc)
    {
      // переводим adc в напряжение
      float volts = adc*0.0048828125;
      // переводим в см
      if(volts!=0)
         distance = 32*pow(volts, -1.10);
         A = (1.0-K)*A + K*distance;
      return (int)A;
    }
    private:
      float K;
      float A;
      float distance;
};

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
void Motors_init();
void MotorL(int upwm, float actual_enc_speed);
void MotorR(int upwm, float actual_enc_speed);

//
// Тест моторов. Возвращает энкодерную скорость
// sp - значение ШИМ, tq - квант времени (50 мс)
// Энкодерная скорсть рассчитывается как количество импульсов энкодера за время tq
//
int TestMotors(int sp, long tq);
void Beep(int n);

void _motorL(int Pulse_Width);
void _motorR(int Pulse_Width);

#endif
