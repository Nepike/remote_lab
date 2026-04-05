
//----------------------------------------------------------
// Базовые двигательные функции
// Тип драйвера двигателей: DRV_VNH5019, DRV_ROBOCRAFT_L298N, DRV_DFROBOT_L298N, DRV_USR
//----------------------------------------------------------
#ifndef _MDRIVER_H_
#define _MDRIVER_H_

// Проверка того, что был установлен только один макрос выбора типа драйвера двигателей
#if (not defined DRV_VNH5019) && (not defined DRV_ROBOCRAFT_L298N) && (not defined DRV_DFROBOT_L298N) && (not defined DRV_USR)
#error "ERROR: define DRV_VNH5019, DRV_ROBOCRAFT_L298N, DRV_DFROBOT_L298N, DRV_USR"
#endif

#if (defined DRV_VNH5019) && (defined DRV_ROBOCRAFT_L298N) && (defined DRV_DFROBOT_L298N) && (defined DRV_USR)
#error "ERROR: define only one type: DRV_VNH5019, DRV_ROBOCRAFT_L298N, DRV_DFROBOT_L298N, DRV_USR"
#endif

//**********************************************************
//
// DFRpobot Shield
//
//**********************************************************
#ifdef DRV_DFROBOT_L298N

// Подключение двигателей
#define PIN_MOTOR_M1   7  // DIR1
#define PIN_MOTOR_E1   6  // PWM1

#define PIN_MOTOR_M2   4  // DIR2
#define PIN_MOTOR_E2   5  // PWM2

void MotorLeftGo(int pwm)
{
  if(pwm==0)
  {
    digitalWrite(PIN_MOTOR_E1, LOW);
  }
  else
  if(pwm>0)
  {
    digitalWrite(PIN_MOTOR_M1, HIGH);
    analogWrite(PIN_MOTOR_E1, pwm);
  }
  else
  {
    digitalWrite(PIN_MOTOR_M1, LOW);
    analogWrite(PIN_MOTOR_E1, (int)255+pwm);
  }  
}

void MotorRightGo(int pwm)
{
  if(pwm==0)
  {
    digitalWrite(PIN_MOTOR_E2, LOW);
  }
  else
  if(pwm>0)
  {
    digitalWrite(PIN_MOTOR_M2, HIGH);
    analogWrite(PIN_MOTOR_E2, pwm);
  }
  else
  {
    digitalWrite(PIN_MOTOR_M2, LOW);
    analogWrite(PIN_MOTOR_E2, (int)255+pwm);
  }  
}

void MotorsInit(void)
{
  pinMode(PIN_MOTOR_E1, OUTPUT);
  pinMode(PIN_MOTOR_M1, OUTPUT);
  pinMode(PIN_MOTOR_E2, OUTPUT);
  pinMode(PIN_MOTOR_M2, OUTPUT);
}

#endif

//**********************************************************
//
// ROBOCRAFT_L298N
//
//**********************************************************
#ifdef DRV_ROBOCRAFT_L298N

// Подключение двигателей
#define PIN_MOTOR_L1      7  // in1
#define PIN_MOTOR_L2      6  // in2 PWM

#define PIN_MOTOR_R1      5  // in3 PWM
#define PIN_MOTOR_R2      4  // in4

void MotorLeftGo(int pwm)
{
  if(pwm==0)
  {
    digitalWrite(PIN_MOTOR_L1, HIGH);
    digitalWrite(PIN_MOTOR_L2, HIGH);
  }
  else
  if(pwm>0)
  {
    digitalWrite(PIN_MOTOR_L1, LOW);
    analogWrite(PIN_MOTOR_L2, pwm);
  }
  else
  {
    digitalWrite(PIN_MOTOR_L1, HIGH);
    analogWrite(PIN_MOTOR_L2, (int)255+pwm);
  }  
}

void MotorRightGo(int pwm)
{
  if(pwm==0)
  {
    digitalWrite(PIN_MOTOR_R1, HIGH);
    digitalWrite(PIN_MOTOR_R2, HIGH);
  }
  else
  if(pwm>0)
  {
    analogWrite(PIN_MOTOR_R1, (int)255-pwm);
    digitalWrite(PIN_MOTOR_R2, HIGH);
  }
  else
  {
    analogWrite(PIN_MOTOR_R1, -pwm);
    digitalWrite(PIN_MOTOR_R2, LOW);
  }  
}

void MotorsInit(void)
{
  pinMode(PIN_MOTOR_L1, OUTPUT);
  pinMode(PIN_MOTOR_L2, OUTPUT);
  pinMode(PIN_MOTOR_R1, OUTPUT);
  pinMode(PIN_MOTOR_R2, OUTPUT);
}

#endif

//**********************************************************
//
// VNH5019
//
//**********************************************************
#ifdef DRV_VNH5019

/// Подключение двигателей
#define PIN_MOTOR_RA      4  // M2INA
#define PIN_MOTOR_RPWM    5  // M2PWM (PWM)
#define PIN_MOTOR_LPWM    6  // M1PWM (PWM)
#define PIN_MOTOR_LA      7  // M1INA

#define PIN_MOTOR_LB      8  // M1INB
#define PIN_MOTOR_RB      9  // M2INB

void MotorLeftGo(int pwm)
{
  if(pwm==0)
  {
    digitalWrite(PIN_MOTOR_LA, HIGH);
    digitalWrite(PIN_MOTOR_LB, HIGH);
  }
  else
  if(pwm>0)
  {
    digitalWrite(PIN_MOTOR_LA, LOW);
    digitalWrite(PIN_MOTOR_LB, HIGH);
    analogWrite(PIN_MOTOR_LPWM, pwm);
  }
  else
  {
    digitalWrite(PIN_MOTOR_LA, HIGH);
    digitalWrite(PIN_MOTOR_LB, LOW);
    analogWrite(PIN_MOTOR_LPWM, -pwm);
  }
}

void MotorRightGo(int pwm)
{
  if(pwm==0)
  {
    digitalWrite(PIN_MOTOR_RA, HIGH);
    digitalWrite(PIN_MOTOR_RB, HIGH);
  }
  else
  if(pwm>0)
  {
    digitalWrite(PIN_MOTOR_RA, LOW);
    digitalWrite(PIN_MOTOR_RB, HIGH);
    analogWrite(PIN_MOTOR_RPWM, pwm);
  }
  else
  {
    digitalWrite(PIN_MOTOR_RA, HIGH);
    digitalWrite(PIN_MOTOR_RB, LOW);
    analogWrite(PIN_MOTOR_RPWM, -pwm);
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
}

#endif

#endif
