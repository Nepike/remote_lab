/**
 * Сервер бампера
 * 
 *  \author Robofob
 *  \version 2.0
 *  \date 13.12.2019
 *  \date LP 29.06.2020
 */

#include <Servo.h>
#include <Wire.h>
#include <TroykaCurrent.h>
#include "i2cwctl.h"

#define RL A0 //4
#define RR A1 //5
#define FS A2
#define CS A3
#define FLIR A6
#define FRIR A7
#define OUT 12
#define BLL 2
#define BFL 3
#define BFR 4
#define BRR 5
#define ServL 6
#define ServR 7
#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define TOP 70
#define DOWN 0
#define TRESH 430

Servo servoleft;
Servo servoright;

ACS712 dataI(CS);

const char* Title = "Bumper server v1.00";
byte Color[I2CColorServer::DATALEN];
byte CMD[5];
byte LastCMD = 0;
int TargetAngle = 0;
bool ServoDetach = true;
//Bumper status: 0 - on bottom, 1 - on top, 2 - move down, 3 - move up, 4 - error while moving down, 5 - error while moving up
byte Status = 0; 

int BTime = 5000;
long Time = 5000;

byte fl,fr,b1,b2,b3,b4;

void Readsensors(void)
{
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  Color[0] = pulseIn(OUT,LOW); //Red 
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  Color[1] = pulseIn(OUT,LOW); //Green
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  Color[2] = pulseIn(OUT,LOW); //Blue
  Color[3] = map(analogRead(RL),0,1023,0,255);   //Left line sensor
  Color[4] = map(analogRead(RR),0,1023,0,255);   //Forward line sensor 
  Color[5] = Status;
  if (analogRead(FS) >= TRESH)
    Color[6] = 1;
  else
    Color[6] = 0;
  Color[7] = map(analogRead(FLIR),0,1023,0,255);
  Color[8] = map(analogRead(FRIR),0,1023,0,255);
  if (digitalRead(BLL) == 1 or digitalRead(BFL) == 1 or digitalRead(BFR) == 1 or digitalRead(BRR) == 1)
    Color[9] = 1;
  else
    Color[9] = 0;
}

void Outputresults(void)
{
  char buf[50];
  sprintf(buf,"%d %d %d %d %d %d %d %d %d %d", Color[0],Color[1],Color[2],Color[3],Color[4],Color[5],Color[6],Color[7],Color[8],Color[9]);
  Serial.println(buf);
}

void requestI2CEvent()
{
  Wire.write(Color, I2CColorServer::DATALEN);
}

void MoveBumper()
{
  if ((TargetAngle - servoright.read()) > 0) //Up
  {
     if (ServoDetach)
    {
      //servoleft.attach(ServL);
      //servoright.attach(ServR); 
      ServoDetach = false;
    }
    servoleft.write(servoleft.read()-1);
    servoright.write(servoright.read()+1);  
    delay(20);
  }
  else if ((TargetAngle - servoright.read()) < 0) //Down
  {
    if (ServoDetach)
    {
      //servoleft.attach(ServL);
      //servoright.attach(ServR); 
      ServoDetach = false;
    }
    servoleft.write(servoleft.read()+1);
    servoright.write(servoright.read()-1);  
    delay(20);
  }
  else
  {
    if (!ServoDetach)
    {
      //servoleft.detach();
      //servoright.detach(); 
      ServoDetach = true;
    }
    if (Status == 2)
      Status = 0;
    if (Status == 3)
      Status = 1;
  }
}

void receiveI2CEvent(int howMany)
{ 
  while (Wire.available())
    for(byte i=0; i<5; i++)
      CMD[i] = Wire.read();
  if (CMD[0] != LastCMD)
  {
    if (CMD[0] == 1)
    {
      TargetAngle = TOP;
      Status = 3;
    }
    else
    {
      TargetAngle = DOWN;
      Status = 2;
    }
    LastCMD = CMD[0];
  }
}

void Test(void)
{
  if (abs(millis() - Time) >= BTime)
  {
    if (CMD[0] == 0)
    {
      CMD[0] = 1;
      TargetAngle = TOP;
      Status = 3;
    }
    else
    {
      CMD[0] = 0;
      TargetAngle = DOWN;
      Status = 2;
    }
    Time = millis();
  }
}

void CheckCS(void)
{
  if (abs(dataI.readCurrentDC()) > 0.3)
  {
    if (TargetAngle == DOWN)
    {
      servoleft.write(179 - TOP);
      servoright.write(TOP);  
      TargetAngle = DOWN;
      LastCMD = 1;
      Status = 4;
    }
    else
    {
      servoleft.write(179 - DOWN);
      servoright.write(DOWN);  
      TargetAngle = TOP;
      LastCMD = 0;
      Status = 5;
    }
    delay(250);
    //servoleft.detach();
    //servoright.detach();
    ServoDetach = false;
  }
}

void setup()
{  
  CMD[0] = 0;
  Status = 0;
  
  Serial.begin(9600);
  Wire.begin(I2CColorServer::ADDR);

  Wire.onRequest(requestI2CEvent);
  Wire.onReceive(receiveI2CEvent);

  pinMode(RR,INPUT);
  pinMode(RL,INPUT);

  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);
  pinMode(OUT,INPUT);

  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  memset(Color,0,sizeof(Color));

  servoleft.attach(ServL);
  servoright.attach(ServR); 
 
  Serial.println(Title);
}

void loop()
{
  //Test();
  MoveBumper();
  CheckCS();
  Readsensors();
  Outputresults();
}
