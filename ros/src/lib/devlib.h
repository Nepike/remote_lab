/**
 * DEV library (Linux)
 * \version 1.06
 * \date 28.02.2014
 * \date LP 14.08.2015
*/

#ifndef _DEVLIB_H_
#define _DEVLIB_H_

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>

#include "rcproto/rcproto2.h"
#include "rcproto/tmucmd.h"
#include "dev/comportlib.h"
#include "dev/xbeelib.h"

/// Дескриптор порта
struct TPortDescriptor
{
  int id;          /// id порта
  char *name;      /// Имя порта
  int bdrn;        /// скорость (0,1)
  int fd;          /// Файловый дескриптор
  TPortDescriptor() { id = -1; name = NULL; };
  void Read(FILE *f);
  void Open(int ntr); // ntr - количество итераций
};

struct TRobotDescriptor
{
  int id;    /// id робота
  int pdn;   /// номер дескриптора порта
  TPckg pckg;
  TPortDescriptor *pds;
  void Read(FILE *f);
  TRobotDescriptor() { pds = NULL; }
  void ConnectPDS(void);
};

//----------------------------------------------------------
// XBee robots
//----------------------------------------------------------

/// Адресация робота XBee
struct TXBRobotDescriptor
{
  unsigned char id; /// Robot's id
  TXBAddr addr;     /// XBee address
  TPckg pckg;
  void Show(void);
  TXBRobotDescriptor() {  }
};

/// Получить ссылку на пакет по адресу (id робота)
TPckg *GetPckg(unsigned char adr);

/// Получить ссылку на пакет по адресу (id робота) для XBee-роботов
TPckg *XBGetPckg(unsigned char adr);

/// Получить дескриптор файла по адресу (id робота)
int GetFd(unsigned char adr);

void rsReadPortsRobots(FILE *f);

/// Чтение конфигурации роботов XBee sic
void rsXBReadRobots(FILE *f1, FILE *f2);

/// Получить ссылку на пакет по id робота для XBee-роботов
TPckg *XBGetPckg(unsigned char adr, TXBAddr **addr);

/// Получить XBee-адрес по id робота для XBee-роботов
TXBAddr *XBGetAddr(int id);

/// \param ntr - количество попыток открытия порта
void rsOpenPorts(int ntr);

void rsClosePorts(void);

#endif
