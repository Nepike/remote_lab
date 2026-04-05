/*
  Бортовая СУ
  Протокол rcX

  Version 1.17
  02.05.2014 - 23.12.2016
  LP 27.02.2018
*/

#ifndef _TMU_BRAIN_H_
#define _TMU_BRAIN_H_

#include <Wire.h>
#include <SoftwareSerial.h>
#include "rcproto2.h"
#include "i2cwctl.h"

//----------------------------------------------------------

namespace tmb
{

extern TPckg tbpckg;

void Init(long bdr);

void error(const char *msg, int n);

void ShowArray(const char *s, byte array[], byte dim);

void DrawLocator(byte array[], byte dim, byte lim);

//----------------------------------------------------------
//
//----------------------------------------------------------

#define MAX_LEN 60
extern byte Sensors[];
extern byte Locator[];
extern byte DataServer[];
extern byte RC5Data[];
extern byte VisionSensor[];
extern byte Color[];
extern byte Registers[];

extern byte sensnum;
extern byte locnum;
extern byte dsnum;
extern byte rc5num;
extern byte vsnum;
extern byte csnum;
extern byte rgnum;

//--------------------------------------------------------------------------------------
// Пользовательские функции верхнего уровня
//--------------------------------------------------------------------------------------

void RobotStop(void);
void RobotGoFwd(void);
void RobotGoBack(void);
void RobotGoFastLeft(void);
void RobotGoFastRight(void);
void RobotGoLeft(void);
void RobotGoRight(void);

void RobotGoFwd2(int dist);
void RobotGoBack2(int dist);
void RobotGoLeft2(int ang);
void RobotGoRight2(int ang);
void RobotGo(int vleft, int vright);

void RobotBeep(void);
void RobotBeepOn(void);
void RobotBeepOff(void);

/**
 * Опрос датчиков. Читаются:
 * - Sensors[sensnum]
 * - Locator[locnum]
 * - DataServer[dsnum]
 * - RC5Data[rc5num]
 */
void ReadAllSensors(void);

// Read sensors. Ответ - CMD_ANS_GET_SENS
void ReadMainSensors(void);

// Read Locator info. Ответ - CMD_ANS_GET_USR_DATA
void ReadLocator(void);

// Read I2CDataServer. Ответ - CMD_ANS_GET_I2C_DATA
void ReadI2CDataServer(void);

// Read RC5Server. Ответ - CMD_ANS_GET_I2C_DATA
void ReadRC5Server(void);

void RobotSetLocator(int enable_locator);

void RobotStartRC5(unsigned long data);

/// Остановить генерацию
void RobotStopRC5(void);
void RobotClearRC5(void);

/// Сбросить значения счетчиков энкодеров
void RobotResetEncoders(void);

/// Установить значение регистра
void RobotSetReg(byte reg, byte val);

void SetPwmSpeed(byte speed);

/// Установить угол сервомашинки
void SetServo(int ang);

/// Отправить I2C команду
void SendI2CCmd(byte i2c_dev_addr, byte i2c_dev_cmd, byte datalen, byte* databuf);

// VisionSensor отправить команду
void SendI2CCmdVisionSensor(byte CMD, byte ARG);

// VisionSensor считать данные
bool ReadI2CDataVisionSensor( I2CVisionSensor::ParsedData *visionData);

/// Запросить данные с контроллера датчиков линии и цвета
void ReadColorServer(void);

/// Отправить I2C команду SignalServer
void SendI2CCmdSignalServer(byte CMD, byte datalen, byte* ARG);

/// Запросить I2C данные с контроллера сигнальной связи
void ReadSignalServer(void);

void GetAllReg(void);
}

#endif
