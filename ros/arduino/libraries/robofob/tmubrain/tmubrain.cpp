/*
  Бортовая СУ
  Протокол rcX

  Version 1.17
  02.05.2014 - 23.12.2016
  LP 08.11.2019
*/

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "rcproto2.h"
#include "i2cwctl.h"
#include "tmucmd.h"

#include "tmubrain.h"

//----------------------------------------------------------
using namespace tmb;

unsigned char ADDR = 255;

#define rxPin 11 // pin connects to smcSerial RX. For Arduino Mega 11, for Nano 3
#define txPin 12 // pin connects to smcSerial TX. For Arduino Mega 12, for Nano 4

SoftwareSerial smcSerial = SoftwareSerial(rxPin, txPin);

TPckg tmb::tbpckg;

byte tmb::Sensors[MAX_LEN];
byte tmb::Locator[MAX_LEN];
byte tmb::DataServer[MAX_LEN];
byte tmb::RC5Data[MAX_LEN];
byte tmb::VisionSensor[MAX_LEN];
byte tmb::Color[MAX_LEN];         //3 color channels and 3 line channels
byte tmb::Registers[MAX_LEN];

byte tmb::sensnum = 0;
byte tmb::locnum = 0;
byte tmb::dsnum = 0;
byte tmb::rc5num = 0;
byte tmb::vsnum = 0;
byte tmb::csnum = 0;
byte tmb::rgnum = 0;

//----------------------------------------------------------
// Функции для rcproto
// Связь с mctl по программному порту №2
//----------------------------------------------------------

static void rcError(unsigned char n)
{
  const char *msg = "";
  char fstr[40];
  switch(n)
  {
    case DATA_READY: msg = "DATA_READY"; break;
    case DATA_WAIT: msg = "DATA_WAIT"; break;
    case ERR_HDR1: msg = "ERR_HDR1"; break;
    case ERR_HDR2: msg = "ERR_HDR2"; break;
    case ERR_ILLEGAL_CMD: msg = "ERR_ILLEGAL_CMD"; break;
    case ERR_FORMAT: msg = "ERR_FORMAT"; break;
    case ERR_LEN_ERROR: msg = "ERR_LEN_ERROR"; break;
    case ERR_BUFF_LEN: msg = "ERR_BUFF_LEN"; break;
    case ERR_CS: msg = "ERR_CS"; break;          // Ошибка контрольной суммы
    default:
      msg = "?????";
  }
  snprintf(fstr, sizeof(fstr), "rcError %d: %s", n, msg);
  Serial.println(fstr);
}

static unsigned char rcReadByte(void) { return smcSerial.read(); }
static void rcWriteByte(unsigned char c) { smcSerial.write(c); }
static unsigned char rcWasByte(void) { return smcSerial.available(); }
// Проверка таймаута. Возвращает 0, если истекло время ожидания очередного символа пакета
static unsigned char rcTimeoutEvent(void) { return 0; }
// Сброс счетчика таймаута
static void rcResetTimeoutCnt(void) {}

//----------------------------------------------------------

int GetAck(TPckg *p)
// Получаем подтверждение
{
  byte addr;
  byte AckRegime = 1;
  int need_ack = (p->LAST_CMD == CMD_GET_SENS ||
                  p->LAST_CMD == CMD_GET_USR_DATA ||
                  p->LAST_CMD == CMD_GET_REG ||
                  p->LAST_CMD == CMD_PING ||
                  AckRegime);
  if(!need_ack) return 0;
  unsigned char res;
  do
  {
    res = rcReadPackage(p);
    if(res != DATA_WAIT && res != DATA_READY)
    {
      Serial.print("*** GetAck: ");
      Serial.println(res);
      return 0;
    }
  } while (res != DATA_READY);
  return 1;
}

int Pckg2Array(TPckg *p, byte array[])
{
  byte len = p->rcBUFF[POS_LEN];
  if(len>=RC_MAX_BUFF-POS_LEN)
  {
    error("packet len error: ", len);
    return 0;
  }
  byte n = 0;
  for(byte i=POS_DATA;i<POS_DATA+len;i++)
  {
    array[n++] = p->rcBUFF[i];
  }
  return n;
}

void tmb::Init(long bdr)
{
  smcSerial.begin(bdr);
  TPckgInit(&tbpckg, ADDR, rcError, rcReadByte, rcWriteByte,
          rcWasByte, rcTimeoutEvent, rcResetTimeoutCnt);
}

void tmb::error(const char *msg, int n)
{
  Serial.print("Error:");
  Serial.print(msg);
  Serial.println(n);
}

void tmb::ShowArray(const char *s, byte array[], byte dim)
{
  Serial.print(s);
  Serial.print(' ');
  for(int i=0;i<dim;i++)
  {
    Serial.print(array[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void tmb::DrawLocator(byte array[], byte dim, byte lim)
{
  if(dim==0) return;
  Serial.print(dim);
  Serial.print(": ");
  for(int i=0;i<dim;i++)
  {
    int val = array[(dim-1)-i]; // Массив "перевернут": 0 - крайнее правое положение
    char c = (val>lim)?'*':' ';
    Serial.print(c);
  }
  Serial.println();
}

//--------------------------------------------------------------------------------------
// Пользовательские функции верхнего уровня
//--------------------------------------------------------------------------------------

void tmb::ReadMainSensors(void)
// Read sensors. Ответ - CMD_ANS_GET_SENS
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_GET_SENS);
  while(!GetAck(&tbpckg));
  sensnum = Pckg2Array(&tbpckg, Sensors);
}

void tmb::ReadLocator(void)
// Read Locator info. Ответ - CMD_ANS_GET_USR_DATA
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_GET_USR_DATA);
  while(!GetAck(&tbpckg));
  locnum = Pckg2Array(&tbpckg, Locator);
}

void tmb::ReadI2CDataServer(void)
// Read I2CDataServer. Ответ - CMD_ANS_GET_I2C_DATA
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, I2CDataServer::ADDR, I2CDataServer::DATALEN);
  while(!GetAck(&tbpckg));
  dsnum = Pckg2Array(&tbpckg, DataServer);
}

void tmb::ReadRC5Server(void)
// Read RC5Server. Ответ - CMD_ANS_GET_I2C_DATA
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, RC5Server::ADDR, RC5Server::DATALEN);
  while(!GetAck(&tbpckg));
  rc5num = Pckg2Array(&tbpckg, RC5Data);
}

/// Опрос всех датчиков
void tmb::ReadAllSensors(void)
{
  static byte cnt = 0;
  switch(cnt)
  {
    case 0:
      // Read sensors
      // Ответ - CMD_ANS_GET_SENS
      rcSendCMDNoArg(&tbpckg, ADDR, CMD_GET_SENS);
      break;
    case 1:
      // Read Locator info
      // Ответ - CMD_ANS_GET_USR_DATA
      rcSendCMDNoArg(&tbpckg, ADDR, CMD_GET_USR_DATA);
      break;
    case 2:
      // Read I2CDataServer
      // Ответ - CMD_ANS_GET_I2C_DATA
      rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, I2CDataServer::ADDR, I2CDataServer::DATALEN);
      break;
    case 3:
      // Read RC5Server
      // Ответ - CMD_ANS_GET_I2C_DATA
      rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, RC5Server::ADDR, RC5Server::DATALEN);
      break;
  }
  cnt++;
  if(cnt>3) cnt = 0;
  while(!GetAck(&tbpckg));

  switch(tbpckg.rcBUFF[POS_CMD])
  {
    case CMD_ANS_GET_SENS:
      sensnum = Pckg2Array(&tbpckg, Sensors);
      break;
    case CMD_ANS_GET_USR_DATA:
      locnum = Pckg2Array(&tbpckg, Locator);
      break;
    case CMD_ANS_GET_I2C_DATA:
      // Пытаемся определить, от кого пришел пакет
      // Сначала в пакете идет адрес i2c-устройства
      unsigned char i2caddr = tbpckg.rcBUFF[POS_DATA];
      if (i2caddr == I2CDataServer::ADDR)
        dsnum = Pckg2Array(&tbpckg, DataServer);
      else
        if (i2caddr == RC5Server::ADDR)
          rc5num = Pckg2Array(&tbpckg, RC5Data);
        else
        {
          Serial.print("*** Unknown: ");
          Serial.println(i2caddr, HEX);
        }
      break;
  }
}

void tmb::RobotStop(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_STOP);
  GetAck(&tbpckg);
}

void tmb::RobotBeep(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_BEEP_ON);
  GetAck(&tbpckg);
  delay(300);
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_BEEP_OFF);
  GetAck(&tbpckg);
}

void tmb::RobotBeepOn(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_BEEP_ON);
  GetAck(&tbpckg);
}

void tmb::RobotBeepOff(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_BEEP_OFF);
  GetAck(&tbpckg);
}


void tmb::RobotGoFwd(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_FWD);
  GetAck(&tbpckg);
}

void tmb::RobotGoBack(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_BACK);
  GetAck(&tbpckg);
}

void tmb::RobotGoFastLeft(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_FAST_LEFT);
  GetAck(&tbpckg);
}

void tmb::RobotGoFastRight(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_FAST_RIGHT); 
  GetAck(&tbpckg);
}

void tmb::RobotGoLeft(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_LEFT);
  GetAck(&tbpckg);
}

void tmb::RobotGoRight(void)
{
  rcSendCMDNoArg(&tbpckg, ADDR, CMD_RIGHT); 
  GetAck(&tbpckg);
}

void tmb::RobotSetReg(byte reg, byte val)
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_SET_REG,  reg,  val);
  GetAck(&tbpckg);
}

void tmb::RobotSetLocator(int enable_locator)
{
  tmb::RobotSetReg(REG_LOC_ENABLE, enable_locator);
}

void tmb::RobotStartRC5(unsigned long data)
{
  // Команда I2C-устройству. 
  // Формат: <i2c-адрес устройства> <команда> <количество аргументов> <аргумент1> <аргумент2> ...
  byte i2cbuff[RC5Server::DATALEN];
  byte i2clen = RC5Server::FormRawDataBuff(data, i2cbuff);
  rcSendI2CDataArray(&tbpckg, ADDR, CMD_I2C, RC5Server::ADDR, RC5Server::DCMD_GENERATE, i2clen, i2cbuff);
  GetAck(&tbpckg);
}

void tmb::RobotStopRC5(void)
// Остановить генерацию
{
  rcSendI2CDataArray(&tbpckg, ADDR, CMD_I2C, RC5Server::ADDR, RC5Server::DCMD_STOP_GENERATE, 0, NULL);
  GetAck(&tbpckg);
}

void tmb::RobotClearRC5(void)
{
  rcSendI2CDataArray(&tbpckg, ADDR, CMD_I2C, RC5Server::ADDR, RC5Server::DCMD_CLEAR_DATA, 0, NULL);
  GetAck(&tbpckg);
}

/// Сбросить значения счетчиков энкодеров
void tmb::RobotResetEncoders(void)
{
  tmb::RobotSetReg(REG_D1, 0);
  delay(200);
  tmb::RobotSetReg(REG_D2, 0);
}

void tmb::RobotGoFwd2(int dist)
{
  rcSendCMD1Arg(&tbpckg, ADDR, CMD_FWD2, dist);
  GetAck(&tbpckg);
}

void tmb::RobotGoBack2(int dist)
{
  rcSendCMD1Arg(&tbpckg, ADDR, CMD_BACK2, dist);
  GetAck(&tbpckg);
}

void tmb::RobotGoLeft2(int ang)
{
  rcSendCMD1Arg(&tbpckg, ADDR, CMD_FAST_LEFT2, ang);
  GetAck(&tbpckg);
}

void tmb::RobotGoRight2(int ang)
{
  rcSendCMD1Arg(&tbpckg, ADDR, CMD_FAST_RIGHT2, ang);
  GetAck(&tbpckg);
}

void tmb::RobotGo(int vleft, int vright)
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_SET_SPEED,  vleft,  vright);
  GetAck(&tbpckg);
}

void tmb::SetPwmSpeed(byte speed)
{
  RobotSetReg(REG_PWMSPEED, speed);
}

void tmb::SetServo(int ang)
{
  rcSendCMD1Arg(&tbpckg, ADDR, CMD_SET_SERVO_POS, ang);
  GetAck(&tbpckg);
}

/// Отправить I2C команду
void tmb::SendI2CCmd(byte i2c_dev_addr, byte i2c_dev_cmd, byte datalen, byte* databuf)
{
  rcSendI2CDataArray(&tbpckg, ADDR, CMD_I2C, i2c_dev_addr, i2c_dev_cmd, datalen, databuf);
  GetAck(&tbpckg);
}

/// Отправить I2C команду VisionSensor
void tmb::SendI2CCmdVisionSensor(byte CMD, byte ARG)//byte datalen, byte* databuf)
{
  rcSendI2CDataArray(&tbpckg, ADDR, CMD_I2C, I2CVisionSensor::ADDR, CMD, 1, &ARG);
  GetAck(&tbpckg);
}

/// Запросить I2C данные с VisionSensor
bool tmb::ReadI2CDataVisionSensor(I2CVisionSensor::ParsedData *visionData)
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, I2CVisionSensor::ADDR, I2CVisionSensor::DATALEN);
  
  while(!GetAck(&tbpckg));
  
  if( tbpckg.rcBUFF[POS_CMD] == CMD_ANS_GET_I2C_DATA){		  
	  unsigned char i2caddr = tbpckg.rcBUFF[POS_DATA];
	  if (i2caddr == I2CVisionSensor::ADDR){
		vsnum = Pckg2Array(&tbpckg, VisionSensor);
		*visionData = I2CVisionSensor::ParsePacket(VisionSensor,vsnum);
		return true;
		
	  }
  }
  return false;
}

/// Запросить I2C данные с контроллера датчиков линии и цвета
void tmb::ReadColorServer(void)
// Read color server. Ответ - CMD_ANS_GET_I2C_DATA
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, I2CColorServer::ADDR, I2CColorServer::DATALEN);
  while(!GetAck(&tbpckg));
  csnum = Pckg2Array(&tbpckg, Color);
}

/// Отправить I2C команду SignalServer
void tmb::SendI2CCmdSignalServer(byte CMD, byte datalen, byte* ARG)
{
  rcSendI2CDataArray(&tbpckg, ADDR, CMD_I2C, I2CSignalServer::ADDR, CMD, datalen, ARG);
  GetAck(&tbpckg);
}

/// Запросить I2C данные с контроллера сигнальной связи
void tmb::ReadSignalServer(void)
// Read signal server. Ответ - CMD_ANS_GET_I2C_DATA
{
  rcSendCMD2Arg(&tbpckg, ADDR, CMD_GET_I2C_DATA, I2CSignalServer::ADDR2, I2CSignalServer::DATALEN);
  while(!GetAck(&tbpckg));
  rc5num = Pckg2Array(&tbpckg, RC5Data);
}

void tmb::GetAllReg(void)
{
   rcSendCMDNoArg(&tbpckg, ADDR, CMD_GET_ALL_REG);
   while(!GetAck(&tbpckg));
   rgnum = Pckg2Array(&tbpckg, Registers);
}
