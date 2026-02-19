
#ifndef _RCPMOTORFUNC_INC_
#define _RCPMOTORFUNC_INC_

/**
 * Двигательные функции
 * Только для mpctl, lpctl
 *
 *  \author Robofob
 *  \version 1.24
 *  \date 03.12.2016
 *  \date LP 24.08.2017
 */

//-------------------------------------------

#ifdef DEBUG_OUTPUT
  unsigned char stdrobot::debug_output = 1;
#else
  unsigned char stdrobot::debug_output = 0;
#endif

//----------------------------------------------------------
// LCD
//----------------------------------------------------------

#ifdef USE_ADA_FRUIT
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#else
// Set the LCD address to 0x20 or 0x27 for a 16 chars and 2 line display
// Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
// Ноги A4 (SDA) и A5 (SCL) заняты
LiquidCrystal_I2C lcd(0x20,16,2); // Обратите внимание на адрес устройства - 0x20 или 0x27
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

/// УЗД/Sharp (рефлексы)
#define USF_LEFT      A6
#define USF_RIGHT     A7
/// Соответствующие номера в массиве Sens
#define NUM_USF_LEFT   6
#define NUM_USF_RIGHT  7

/// Подключение энкодеров
#define PIN_encoderLeft   3 /**< Левый энкодер */
#define PIN_encoderRight  2 /**< Правый энкодер */

/// Пищалка
#define PIN_BEEP         10

//----------------------------------------------------------
// Базовые двигательные функции
//----------------------------------------------------------

/// Подключение двигателей
#define PIN_MOTOR_RA      4  // M2INA
#define PIN_MOTOR_RPWM    5  // M2PWM (PWM)
#define PIN_MOTOR_LPWM    6  // M1PWM (PWM)
#define PIN_MOTOR_LA      7  // M1INA

#define PIN_MOTOR_LB      8  // M1INB
#define PIN_MOTOR_RB      9  // M2INB

void MotorLeftGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR_LA, HIGH);
    digitalWrite(PIN_MOTOR_LB, HIGH);
    stdrobot::dir_left = 0;
  }
  else
  if(pwmspeed>0)
  {
    digitalWrite(PIN_MOTOR_LA, LOW);
    digitalWrite(PIN_MOTOR_LB, HIGH);
    analogWrite(PIN_MOTOR_LPWM, pwmspeed);
    stdrobot::dir_left = 1;
  }
  else
  {
    digitalWrite(PIN_MOTOR_LA, HIGH);
    digitalWrite(PIN_MOTOR_LB, LOW);
    analogWrite(PIN_MOTOR_LPWM, -pwmspeed);
    stdrobot::dir_left = -1;
  }
}

void MotorRightGo(int pwmspeed)
{
  if(pwmspeed==0)
  {
    digitalWrite(PIN_MOTOR_RA, HIGH);
    digitalWrite(PIN_MOTOR_RB, HIGH);
    stdrobot::dir_right = 0;
  }
  else
  if(pwmspeed>0)
  {
    digitalWrite(PIN_MOTOR_RA, LOW);
    digitalWrite(PIN_MOTOR_RB, HIGH);
    analogWrite(PIN_MOTOR_RPWM, pwmspeed);
    stdrobot::dir_right = 1;
  }
  else
  {
    digitalWrite(PIN_MOTOR_RA, HIGH);
    digitalWrite(PIN_MOTOR_RB, LOW);
    analogWrite(PIN_MOTOR_RPWM, -pwmspeed);
    stdrobot::dir_right = -1;
  }
}

void MotorsInit(void)
{
  pinMode(PIN_MOTOR_LA, OUTPUT);
  pinMode(PIN_MOTOR_LB, OUTPUT);
  pinMode(PIN_MOTOR_LPWM, OUTPUT);
  pinMode(PIN_MOTOR_RA, OUTPUT);
  pinMode(PIN_MOTOR_RB, OUTPUT);
  pinMode(PIN_MOTOR_RPWM, OUTPUT);

  // Настройка портов.
  // Подтягиваем входы бампера к "1"
  digitalWrite(BUMPER_LEFT, HIGH);
  digitalWrite(BUMPER_RIGHT, HIGH);

  digitalWrite(USF_LEFT, HIGH);
  digitalWrite(USF_RIGHT, HIGH);

  pinMode(PIN_BEEP, OUTPUT);
  pinMode(PIN_REFLEX_INDICATOR, OUTPUT);
}

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

#endif
