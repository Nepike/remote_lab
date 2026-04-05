/*
 * I2C Data Server
 * V 1.02
 * 28.08.2015
 * LP 12.01.2016
 * Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include "i2cwctl.h"

char *Title = "I2C Data Server 1.02";

unsigned char SENSDATA[I2CDataServer::DATALEN];

//----------------------------------------------------------------
// События: прием запросов от клиента
//----------------------------------------------------------------

void requestI2CEvent()
{
  // Отправляем считанные с портов данные
  Wire.write(SENSDATA, I2CDataServer::DATALEN);
}

//------------------------------------------------------------

void setup()
{
  Wire.begin(I2CDataServer::ADDR); // join i2c bus with address I2CDATASERVERADDR
  Wire.onRequest(requestI2CEvent); // register event

  Serial.begin(9600);
  Serial.println(Title);

  memset(SENSDATA, 0, I2CDataServer::DATALEN);
}

//------------------------------------------------------------

void ReadSensors(void)
{
  static byte n = 0;
  SENSDATA[n] = analogRead(n)>>2;

  n++;
  if(n>=I2CDataServer::ADC_NUM) n=0;

  for(int i=0;i<I2CDataServer::PORT_NUM;i++)
    SENSDATA[i+I2CDataServer::ADC_NUM] = digitalRead(i);  
}

void loop() 
{
  ReadSensors();
}
