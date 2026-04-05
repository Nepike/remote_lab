/*
 * I2C Master. Simple Transmitter
 *
 * V 1.02
 * 11.03.2015
 *
 */

#include <Wire.h>
#include "i2cwctl.h"

byte buff[RC5Server::DATALEN];

int q = 0;
#define T_READ 10

void ShowBuff(void)
{
  for(int n=0;n<RC5Server::DATALEN;n++)
  {
    if(n%RC5Server::NUMRECV == 0)
      Serial.print("-- ");
    Serial.print(buff[n], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

unsigned int data0 = 0x0A12; // ---- 1ddd dddd dddd -> 0000 1:010 0001 0010
unsigned int data1 = 0x0812; // ---- 1ddd dddd dddd -> 0000 1:000 0001 0010
unsigned int data2 = 0x0734; // ---- 0ddd dddd dddd -> 0000 0:111 0011 0100

void SendCmd(byte cmd)
{
  I2CDCS::SendCommand(RC5Server::ADDR, cmd, 0, buff);
}

void SendCmd2(byte cmd, unsigned int d0, unsigned int d1)
{
  byte datalen = RC5Server::FormDataBuff2(d0, d1, buff);
  I2CDCS::SendCommand(RC5Server::ADDR, cmd, datalen, buff);
}

void SendRaw1(byte cmd, unsigned int d)
{
  byte datalen = RC5Server::FormRawDataBuff(d, buff);
  I2CDCS::SendCommand(RC5Server::ADDR, cmd, datalen, buff);
}

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("RC5 Master Demo 1.02");
  delay(1000);
  SendRaw1(RC5Server::DCMD_GENERATE, data0);
}

void loop()
{
 /* 
  SendCmd(RC5Server::DCMD_CLEAR_DATA);
  delay(500);  
  SendRaw1(RC5Server::DCMD_GENERATE, data0);
  delay(5000);
  SendCmd(RC5Server::DCMD_STOP_GENERATE);
  delay(5000);


  SendCmd(RC5Server::DCMD_CLEAR_DATA);
  delay(500);  
  SendCmd2(RC5Server::DCMD_GENERATE, data1, data2);
  delay(5000);
  SendCmd(RC5Server::DCMD_STOP_GENERATE);
  delay(5000);

*/
  if(q<=T_READ)
  {
    // Получение данных от приемо-передатчика
    I2CDCS::ReadData(RC5Server::ADDR, RC5Server::DATALEN, buff);
    ShowBuff();
    delay(100);
  }
  if(q==T_READ+1)
  {
    Serial.println("Send CMD_CLEAR_DATA...");
    SendCmd(RC5Server::DCMD_CLEAR_DATA);
    delay(100);
  }
  q++;
  if(q>T_READ+1) q = 0;
 
}
