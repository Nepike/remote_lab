/*
 * Arduino Mega
 * 06.03.2021
 * LP 11.11.2022
 */

//#define _DEBUG_PID_

#include <Servo.h>
#include <Encoder.h>
#include <yfunc.h>
#include <pidlib.h>

//-------------------------------------------------------------------

#define SWITCHES_NUM 8
byte PIN_SWITCHES[SWITCHES_NUM] = {40, 41, 42, 43, 44, 45, 46, 47};

// Encoders
Encoder encoder_left(PIN_ENC_LEFT_1, PIN_ENC_LEFT_2);
Encoder encoder_right(PIN_ENC_RIGHT_1, PIN_ENC_RIGHT_2);

// Собственно ПИД-регуляторы
TPID2 PID_LEFT;
TPID2 PID_RIGHT;

// Конечные масштабные множители для согласования характеристик двигателей
float RATIO_LEFT = 1.0;
float RATIO_RIGHT = 1.0;

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
void Motors_init()
{
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}

void _motorL(int Pulse_Width)
{
  static int pred_val = 0;

  Pulse_Width = Pulse_Width*RATIO_LEFT;
  if(Pulse_Width>PID_LEFT.maxU) Pulse_Width = PID_LEFT.maxU;
  if(Pulse_Width<PID_LEFT.minU) Pulse_Width = PID_LEFT.minU;

  //if(Pulse_Width == pred_val) return;
  pred_val = Pulse_Width;

  if (Pulse_Width > 0)
  {
    analogWrite(EN_L, Pulse_Width);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }

  if (Pulse_Width < 0)
  {
    Pulse_Width=abs(Pulse_Width);
    analogWrite(EN_L, Pulse_Width);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }

  if (Pulse_Width == 0)
  {
    analogWrite(EN_L, Pulse_Width);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, HIGH);
  }
}

void _motorR(int Pulse_Width)
{
  static int pred_val = 0;

  Pulse_Width = Pulse_Width*RATIO_RIGHT;
  if(Pulse_Width>PID_RIGHT.maxU) Pulse_Width = PID_RIGHT.maxU;
  if(Pulse_Width<PID_RIGHT.minU) Pulse_Width = PID_RIGHT.minU;

  //if(Pulse_Width == pred_val) return;
  pred_val = Pulse_Width;

  if (Pulse_Width > 0)
  {
    analogWrite(EN_R, Pulse_Width);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }
  if (Pulse_Width < 0)
  {
    Pulse_Width=abs(Pulse_Width);
    analogWrite(EN_R, Pulse_Width);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }

  if (Pulse_Width == 0)
  {
    analogWrite(EN_R, Pulse_Width);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, HIGH);
  }
}

//
//
//
int TestMotors(int sp, long tq)
{
  long Tdelay = 2000;
  int n = Tdelay/tq;
  encoder_left.write(0);
  encoder_right.write(0);
  long ct = millis();
  
  _motorL(sp);
  _motorR(sp);

  while(millis()-ct < Tdelay);
  int encL = encoder_left.read();
  int encR = encoder_right.read();
  int enc_speed = (encL>encR)?encL:encR;
  enc_speed = enc_speed/n;

  _motorL(-sp);
  _motorR(-sp);
  delay(Tdelay);

  _motorL(-sp);
  _motorR(sp);
  delay(Tdelay);

  _motorL(sp);
  _motorR(-sp);
  delay(Tdelay);

  _motorL(0);
  _motorR(0);
  delay(Tdelay);
  return enc_speed;
}

void Beep(int n)
{
  for(byte i=0;i<n;i++)
  {
    digitalWrite(PIN_BEEP, HIGH);
    delay(500);
    digitalWrite(PIN_BEEP, LOW);
    delay(500);
  }
}

float upwm2enc_speed(int upwm)
{
  float sp = fmap(upwm, pidlib::minU, pidlib::maxU, -pidlib::MAX_ENC_SPEED, pidlib::MAX_ENC_SPEED);
  return sp;
}


#ifdef _DEBUG_PID_
char ssss[80];
char str_temp[8];
#endif

void MotorL(int upwm, float actual_enc_speed)
{
  if(upwm==0)
  {
    _motorL(0);
    return;
  }
  float goal_speed = upwm2enc_speed(upwm);
  float err = (goal_speed - actual_enc_speed)/pidlib::MAX_ENC_SPEED;
  float y = PID_LEFT.Eval(err);
  float u = PID_LEFT.y2u(y);

#ifdef _DEBUG_PID_
  ssss[0] = 0;
  dtostrf(upwm, 7, 1, str_temp); strcat(ssss, str_temp);
  dtostrf(goal_speed, 7, 2, str_temp); strcat(ssss, str_temp);
  dtostrf(err, 7, 2, str_temp); strcat(ssss, str_temp);
  dtostrf(y, 7, 2, str_temp); strcat(ssss, str_temp);
  dtostrf(u, 7, 1, str_temp); strcat(ssss, str_temp);
  Serial.println(ssss);
#endif
  _motorL(u);
}

void MotorR(int upwm, float actual_enc_speed)
{
  if(upwm==0)
  {
    _motorR(0);
    return;
  }
  float goal_speed = upwm2enc_speed(upwm);
  float err = (goal_speed - actual_enc_speed)/pidlib::MAX_ENC_SPEED;
  float y = PID_RIGHT.Eval(err);
  float u = PID_RIGHT.y2u(y);
  _motorR(u);
}
