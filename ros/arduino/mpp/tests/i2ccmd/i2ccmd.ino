 /**
 * \file i2ccmd.ino
 * Progect:
 *  Тестовая программа. Прием i2c-команд
 *  Chip ATmega328
 *  \author Robofob
 *  \version 1.06
 *  \date 05.05.2015
 *  \date Last Change: 05.02.2016
 */
// Общий формат принимаемой i2c-команды: <cmd> <количество байт> <байты данных>

#include <Wire.h>
#include <i2cwctl.h>

#define Title "I2C cmd reciever 1.01"


// Буфер для вывода сообщений
char str[60];

//----------------------------------------------------------------
// События: прием запросов от мастера
//----------------------------------------------------------------
byte I2C_COMMAND_READY = 0;
byte I2CCMD, I2CDATA1, I2CDATA2;
int I2CINTDATA;

void receiveI2CEvent(int howMany)
// Принимается сначала младший байт, а зетем - старший
{
  byte len;
  byte buff[8];
  memset(buff,0,sizeof(buff));
  I2CCMD = I2CDCS::ReadI2CpacketData(&len, buff);

  I2CDATA1 = buff[0];
  I2CDATA2 = buff[1];
  I2CINTDATA = ((int)I2CDATA2<<8) | I2CDATA1;
  I2C_COMMAND_READY = 1;
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void setup()
{
  Wire.begin(I2CLPW::ADDR);   // join i2c bus with address I2C_LPW_CTL_ADDR
  Wire.onReceive(receiveI2CEvent);

  Serial.begin(9600);
  Serial.println(Title);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void loop()
{
  if(I2C_COMMAND_READY)
  {
    I2C_COMMAND_READY = 0;
    snprintf(str, sizeof(str), "> cmd = %2x arg = %d (%2x %2x)", I2CCMD, I2CINTDATA, I2CDATA1, I2CDATA2);
    Serial.println(str);
  }
}

