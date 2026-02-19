/*
  Joystick Ctl Motor Driver
  Управление двигателями с помощью джойстика
  Version 1.02
  13.06.2016
  LP 27.06.2016
*/

#include "joyslib.h"

const char *Title = "\r\nJS Motor Driver 1.02";

// Подключение джойстика
#define Speed1Pin 0  // Координата x - скорость вращения (ROT)
#define Speed2Pin 1  // Координата y - скорость поступательного движения (TAN)

// Подключение моторов
#define M1APin   8
#define M1BPin   7
#define M1PWMPin 6

#define M2APin   3
#define M2BPin   4
#define M2PWMPin 5

const int MinSpeed = -255;
const int MaxSpeed = 255;

TJoystick JST(0, 255, 0, 255, MinSpeed, MaxSpeed);

void Motor1(int speed)
{
  int dp = (speed>0) ? 1:0;
  if(speed==0)
  {
    digitalWrite(M1APin, 1);
    digitalWrite(M1BPin, 1);    
  }
  else
  {
    digitalWrite(M1APin, dp);
    digitalWrite(M1BPin, !dp);    
  }
  analogWrite(M1PWMPin, abs(speed));
}

void Motor2(int speed)
{
  int dp = (speed>0) ? 1:0;
  if(speed==0)
  {
    digitalWrite(M2APin, 1);
    digitalWrite(M2BPin, 1);    
  }
  else
  {
    digitalWrite(M2APin, dp);
    digitalWrite(M2BPin, !dp);    
  }
  analogWrite(M2PWMPin, abs(speed));
}

void TestMotors(void)
{
  Serial.print("Test motors... ");
  #define TD 1000
  #define SP (MaxSpeed/4)
  
  Motor1(SP);  delay(TD);
  Motor1(-SP); delay(TD);
  Motor1(0);

  Motor2(SP);  delay(TD);
  Motor2(-SP); delay(TD);
  Motor2(0);
  Serial.println("Done");
}

void setup()
{
  Serial.begin(9600);
  Serial.println(Title);

  pinMode(M1APin, OUTPUT);
  pinMode(M1BPin, OUTPUT);
  pinMode(M1PWMPin, OUTPUT);

  pinMode(M2APin, OUTPUT);
  pinMode(M2BPin, OUTPUT);
  pinMode(M2PWMPin, OUTPUT);

  Motor1(0);
  Motor2(0);
  //TestMotors();
  delay(1000);
}


void loop()
{
  static int pred1 = 0;
  static int pred2 = 0;

  int a1 = analogRead(Speed1Pin)>>2;
  int a2 = analogRead(Speed2Pin)>>2;
  if(a1 != pred1 || a2 != pred2)
  {
    pred1 = a1;
    pred2 = a2;

    int U1, U2;
    int res = JST.j2u(a1, a2, &U1, &U2);
    Serial.print(a1); Serial.print(" ");  Serial.print(a2);
    Serial.print(" -> ");
    if(res==0)
    {
      Serial.print(U1); Serial.print(" ");  Serial.print(U2);
      Motor1(U1);
      Motor2(U2);
    }
    else
      Serial.print("***");
    Serial.println();
  }
}
