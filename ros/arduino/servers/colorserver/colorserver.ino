/*
 * Color server
 * 
 * V 1.00
 * 22.08.2018
 * LP 22.08.2018
 *
 */

#include <Wire.h>
#include "i2cwctl.h"

#define RR A1 //5
#define RL A0 //4
#define OUT 7
#define S0 8
#define S1 9
#define S2 10
#define S3 11

const char* Title = "Color server v1.00";
byte Color[I2CColorServer::DATALEN];

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
}

void Outputresults(void)
{
  char buf[50];
  sprintf(buf,"%d %d %d %d %d", Color[0],Color[1],Color[2],Color[3],Color[4]);
  Serial.println(buf);
}

void requestI2CEvent()
{
  Wire.write(Color, I2CColorServer::DATALEN);
}

void setup()
{  
  Serial.begin(9600);
  Wire.begin(I2CColorServer::ADDR);

  Wire.onRequest(requestI2CEvent);

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
 
  Serial.println(Title);
}

void loop()
{
  Readsensors();
  Outputresults();
}

