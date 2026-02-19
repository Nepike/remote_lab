/**
 * \file tmu4func1.inc
 * Progect: TMUx
 * Двигательные функции
 *
 *  Chip ATmega328/Mega 2560
 *  \author Robofob
 *  \version 1.25
 *  \date 03.12.2016 / 29.11.2017
 *  \date LP 15.06.2020
 */

#ifndef _TMU4FUNC1_INC_
#define _TMU4FUNC1_INC_


// Идентификатор робота
// Для изменения ROBOT_ID использется внешняя утилита, изменяющая значение регистра REG_EEP[EEP_ID]
#define ROBOT_ID REG_EEP[EEP_ID]

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

//------------------------------------------------------------------

void Beep(byte n);
void SetServoAng(int ang);
void TestServo(void);
byte DetectObstacle(void);

ServoTimer2 myservo;  // create servo object to control a servo

void dcMotorLeftGo(int pwms)
{
  if(pwms==0)
  {
    digitalWrite(PIN_MOTOR_L1, HIGH);
    digitalWrite(PIN_MOTOR_L2, HIGH);
    stdrobot::dir_left = 0;
  }
  else
  if(pwms>0)
  {
    digitalWrite(PIN_MOTOR_L1, LOW);
    analogWrite(PIN_MOTOR_L2, pwms);
    stdrobot::dir_left = 1;
  }
  else
  {
    digitalWrite(PIN_MOTOR_L1, HIGH);
    analogWrite(PIN_MOTOR_L2, (int)255+pwms);
    stdrobot::dir_left = -1;
  }
}

void dcMotorRightGo(int pwms)
{
  if(pwms==0)
  {
    digitalWrite(PIN_MOTOR_R1, HIGH);
    digitalWrite(PIN_MOTOR_R2, HIGH);
    stdrobot::dir_right = 0;
  }
  else
  if(pwms>0)
  {
    analogWrite(PIN_MOTOR_R1, (int)255-pwms);
    digitalWrite(PIN_MOTOR_R2, HIGH);
    stdrobot::dir_right = 1;
  }
  else
  {
    analogWrite(PIN_MOTOR_R1, -pwms);
    digitalWrite(PIN_MOTOR_R2, LOW);
    stdrobot::dir_right = -1;
  }
}

void dcMotorsInit(void)
{
  // Настройка портов
  pinMode(PIN_MOTOR_L1, OUTPUT);
  pinMode(PIN_MOTOR_L2, OUTPUT);
  pinMode(PIN_MOTOR_R1, OUTPUT);
  pinMode(PIN_MOTOR_R2, OUTPUT);

  pinMode(PIN_BEEP, OUTPUT);

  pinMode(PIN_REFLEX_INDICATOR, OUTPUT);

  // Определяем входы двоичных бамперов
  // Подтягиваем к "1"
  pinMode(BUMPER_LEFT, INPUT);
  pinMode(BUMPER_RIGHT, INPUT);
  digitalWrite(BUMPER_LEFT, HIGH);
  digitalWrite(BUMPER_RIGHT, HIGH);
}

#endif
