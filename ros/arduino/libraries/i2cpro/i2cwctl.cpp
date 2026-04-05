/*
 * I2C Servers
 * V 1.10
 *
 * I2CDCS: Общие функции I2C
 * I2CLPW: Протокол системы управления рулевым колесом RWPx1
 * I2CDataServer: Работа с дополнительным сервером данных
 * I2CUSonicServer: Работа с сервером УЗД
 * I2CRC5: Работа с сервером RC5
 *
 * 28.08.2015/05.02.2016/27.02.2018
 * LP 06.02.2023
*/


#include <Arduino.h>
#include <Wire.h>
#include <i2cwctl.h>

void I2CDCS::SendCommand1I(byte addr, byte cmd, int data)
{
  byte buff[4];
  buff[0] = cmd;
  buff[1] = 2;
  buff[2] = (byte)(data);
  buff[3] = (byte)(data>>8);

  Wire.beginTransmission(addr);
  Wire.write(buff,4);
  Wire.endTransmission();
}

void I2CDCS::SendCommand2B(byte addr, byte cmd, byte data1, byte data2)
{
  byte buff[4];
  buff[0] = cmd;
  buff[1] = 2;
  buff[2] = data1;
  buff[3] = data2;

  Wire.beginTransmission(addr);
  Wire.write(buff,4);
  Wire.endTransmission();
}

void I2CDCS::SendCommand(byte addr, byte cmd, byte datalen, byte databuff[])
{
  byte buff[4];
  buff[0] = cmd;
  buff[1] = datalen;

  Wire.beginTransmission(addr);
  Wire.write(buff,2);
  Wire.write(databuff, datalen);
  Wire.endTransmission();
}

/// Получение данных от сервера
int I2CDCS::ReadData(byte addr, unsigned int size, byte buff[])
{
  Wire.requestFrom(addr, size);  // request size bytes from slave device #addr
  memset(buff, 0, size);
  byte n = 0;
  while(Wire.available())        // slave may send less than requested
  {
    buff[n]= Wire.read();
    if(n<size)
      n++;
  }
  return n;
}

/// Чтение передаваемого пакета
/// Возвращает код команды и формирует параметры len и buff
byte I2CDCS::ReadI2CpacketData(byte *len, byte buff[])
{
  byte cmd;
  cmd = Wire.read();  // Команда
  *len = Wire.read(); // Количество байт
  for(byte i=0;i<*len;i++)
    buff[i] = Wire.read();
  return cmd;
}

I2CVisionSensor::ParsedData I2CVisionSensor::ParsePacket(byte *packet, int len)
{
  ParsedData data;
  if( len >= 32 )
  {
    data.status = packet[1];
    data.servoHorisontal = packet[2];
    data.servoVertical = packet[3];
    data.USrange = packet[4];
    for(int i = 0 ; i < 9; i++ )
    {
      data.sampleData[i].ObjectID = packet[5 + i*3];
      data.sampleData[i].TrackID = packet[5 + i*3 + 1];
      data.sampleData[i].Square = packet[5 + i*3 + 2] >> 4;
      data.sampleData[i].Size = packet[5 + i*3 + 2] & B00001111;
    }
  }
  return data;
}

/*
 * RC5DSRV
 */

void RC5DSRV::SetBeacon(byte cmd, unsigned long data, byte numdigits, byte freq)
{
  byte c[12];
  c[0] = cmd;
  c[1] = 6;
  c[2] = (byte)(data >> 24);
  c[3] = (byte)(data >> 16);
  c[4] = (byte)(data >>  8);
  c[5] = (byte)(data);
  c[6] = numdigits;
  c[7] = freq;
  Wire.beginTransmission(RC5DSRV::ADDR_TRN);
  Wire.write(c, 8);
  Wire.endTransmission();
}

/// Получение данных от сервера
void RC5DSRV::RcvGetData(unsigned long rc5data[])
{
  const unsigned int sz = RC5DSRV::DATALEN;
  byte buff[sz];
  Wire.requestFrom(RC5DSRV::ADDR_RCV, sz);  // request size bytes from slave device #addr
  memset(buff, 0, sizeof(buff));
  byte n = 0;
  while(Wire.available())        // slave may send less than requested
  {
    buff[n]= Wire.read();
    if(n<sz)
      n++;
  }
  // Раскидываем buff
  n = 0;  
  for(int i=0;i<RC5DSRV::NUMRECV;i++)
  {
    rc5data[i] = ((unsigned long)buff[n] << 24) | ((unsigned long)buff[n+1] << 16) | 
                 ((unsigned long)buff[n+2] << 8) | ((unsigned long)buff[n+3]);
    n+=4;                 
  }  
}

void RC5DSRV::RcvSendCmd(byte cmd)
{
  byte c[2];
  c[0] = cmd;
  c[1] = 0;
  Wire.beginTransmission(RC5DSRV::ADDR_RCV);
  Wire.write(c, 2);
  Wire.endTransmission();
}
