/*
 * I2C Servo Server
 * V 1.01
 * 22.01.2020
 * LP 06.02.2020
 * Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h> 

#include "maestro2.h"
#include <SoftwareSerial.h>

#include "i2cwctl.h"

char *Title = "I2C Srv Server 1.01";

#define ID_DEV 12 // Идентификатор устройства Pololu Maestro. По умолчанию = 12

#define _DEBUG_

SoftwareSerial ServoSerial(2, 3); // RX, TX
//----------------------------------------------------------------

const int ANGLE_MIN = 0;
const int ANGLE_MAX = 110;

inline void srv_putchar(unsigned char c) { ServoSerial.write(c); }

unsigned char srv_getchar(void)
{
  unsigned char c;
  c = ServoSerial.read();
  return c;
}

TMServo mServo(srv_putchar, srv_getchar, ANGLE_MIN, ANGLE_MAX);

//----------------------------------------------------------------

// create servo objects to control a servo  
// a maximum of eight servo objects can be created 
#define DR_SERVO_NUM 5
Servo dservo[DR_SERVO_NUM];

byte PIN_SERVO[DR_SERVO_NUM] = {6, 7, 8, 9, 10};

//----------------------------------------------------------------
// События: прием запросов от мастера
//----------------------------------------------------------------
byte I2Cdata[I2CServoServer::DATALEN];  // данные I2C
byte I2Cnum = 0;                        // количество байт I2C

void receiveI2CEvent(int howMany)
{
  byte I2CCMD;
  // Чтение передаваемого пакета
  I2CCMD = I2CDCS::ReadI2CpacketData(&I2Cnum, I2Cdata);

  if(I2CCMD == I2CServoServer::CMD_SET_DR)
  {
    int num = I2Cdata[0];
    int angle = I2Cdata[1];
#ifdef _DEBUG_
    Serial.print(num);
    Serial.print(" ");
    Serial.println(angle);
#endif
    dservo[num].write(angle);
  }
  if(I2CCMD == I2CServoServer::CMD_SET_MS) // Pololu maestro
  {
    int num = I2Cdata[0];
    int angle = I2Cdata[1];
#ifdef _DEBUG_
    Serial.print(num);
    Serial.print(" ");
    Serial.println(angle);
#endif
    mServo.SetAng(ID_DEV, num, angle);

 }
}

void requestI2CEvent()
{
}

void setup() 
{
  // join i2c bus with address #I2CServoServer::ADDR
  Wire.begin(I2CServoServer::ADDR);

  // register events
  Wire.onRequest(requestI2CEvent);
  Wire.onReceive(receiveI2CEvent);

  // attaches the servo on pin to the servo object 
  for(byte n=0;n<DR_SERVO_NUM;n++)
    dservo[n].attach(PIN_SERVO[n]);
    
  // Настройка com-порта на скорость 9600
  Serial.begin(9600);
  Serial.println(Title);

  ServoSerial.begin(9600);
}

void loop() 
{
  
}
