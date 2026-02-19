/*
 * I2C Ultrasonic HR-SC04 sensors Server
 * V 1.05
 * 28.08.2015
 * LP 29.04.2019
 *
 * Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
 *
 *
 */

#include "Ultrasonic.h"
#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include "i2cwctl.h"

char *Title = "I2C USonic Server 1.05";

// Основные УЗД (рефлексы)
#define US_TIMEOUT 40000L

// sensor connected to:
// Ultrasonic(Trig, Echo, timeout)
Ultrasonic U1(2, 3, US_TIMEOUT);
Ultrasonic U2(4, 5, US_TIMEOUT);
Ultrasonic U3(6, 7, US_TIMEOUT);
Ultrasonic U4(8, 9, US_TIMEOUT);
Ultrasonic U5(10, 11, US_TIMEOUT);
Ultrasonic U6(12, 13, US_TIMEOUT);

I2CUSonicServer::USonicData data;

void requestI2CEvent()
{
  // Отправляем считанные с портов данные
  Wire.write(data.rawdata, sizeof(data.rawdata));
}

void Read_Sensors(void)
// get distance
{
  static byte n = 0;
  switch(n)
  {
    case 0: data.dist[0] = U1.Ranging(CM); break;
    case 1: data.dist[1] = U2.Ranging(CM); break;
    case 2: data.dist[2] = U3.Ranging(CM); break;
    case 3: data.dist[3] = U4.Ranging(CM); break;
    case 4: data.dist[4] = U5.Ranging(CM); break;
    case 5: data.dist[5] = U6.Ranging(CM); break;
  }
  n++;
  if(n>=I2CUSonicServer::USONIC_NUM) n = 0;
  delay(5);
}

void PrintSensors(void)
{
  for(int i=0; i<I2CUSonicServer::USONIC_NUM; i++)
  {
    Serial.print(data.dist[i], DEC);
    Serial.print(" ");
  }
  Serial.println();
}

void setup()
{
  Wire.begin(I2CUSonicServer::ADDR); // join i2c bus with address I2CUSonicServer::ADDR
  Wire.onRequest(requestI2CEvent);   // register event

  Serial.begin(9600);
  Serial.println(Title);

  memset(data.dist, 0, sizeof(data.dist));
}

void loop()
{
  Read_Sensors();
  PrintSensors();
  //delay(100);   // arbitary wait time.
}
