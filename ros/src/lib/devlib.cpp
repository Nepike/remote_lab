/**
 * DEV library (Linux)
 * \version 1.07
 * \date 28.02.2014
 * \date LP 19.08.2015
*/

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "servlib.h"
#include "devlib.h"

//----------------------------------------------------------
//
//----------------------------------------------------------

void TPortDescriptor::Read(FILE *f)
{
  char s[512], v[512];
  if(!ReadString(f, s))
    error("TPortDescriptor::Read error: no info");
  if(sscanf(s, "%d %s %d", &id, v, &bdrn)!=3)
    error("TPortDescriptor::Read error: format error 2");

  if(id<=0 || id>255)
    error("TPortDescriptor::Read: Illegal id (%d)", id);

  name = newstr(v);

  if(bdrn<0 || bdrn>5)
    error("TPortDescriptor::Read: Illegal bdrn (%d)", bdrn);
}

void TPortDescriptor::Open(int ntr)
{
  // Открываем порт
  printf("Try open port '%s' at speed %d... ", name, bdrn);
  fflush(stdout);
  fd = OpenPort(name, bdrn, ntr);
  if (fd < 0)
    error("[%d] error %d opening %s: %s", fd, errno, name, strerror (errno));
  printf("ok\n");
  fflush(stdout);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

#define MAX_PORT_DESCR 20
TPortDescriptor PortDescr[MAX_PORT_DESCR];
int PortDescrNum = 0;

void ReadPortDescr(FILE *f)
{
  printf("Read ports descr... ");
  if(!ReadInt(f, &PortDescrNum))
    error("read PortDescrNum error"); 

  if(PortDescrNum<=0 || PortDescrNum>=MAX_PORT_DESCR)
    error("Illegal PortDescrNum (%d)'", PortDescrNum);
  for(int i=0;i<PortDescrNum;i++)
    PortDescr[i].Read(f);
  printf("ok\n");
}

void rsOpenPorts(int ntr)
// ntr - iterations
{
  printf("Open ports\n");
  for(int i=0;i<PortDescrNum;i++)
    PortDescr[i].Open(ntr);
}

void rsClosePorts(void)
{
  for(int i=0;i<PortDescrNum;i++)
    close(PortDescr[i].fd);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void TRobotDescriptor::Read(FILE *f)
{
  char s[512];
  if(!ReadString(f, s))
    error("TRobotDescriptor::Read error: no info");

  if(sscanf(s, "%d %d", &id, &pdn)!=2)
    error("TRobotDescriptor::Read format error 2");
  if(id<=0 || id>255)
    error("Illegal id (%d)", id);
  if(id<=0 || id>255)
    error("Illegal pdn (%d)", id);

   extern uchar MY_ADDR;
   void rcError(uchar n);
   uchar rcReadByte(void);
   void rcWriteByte(uchar c);
   uchar rcWasByte(void);
   uchar rcTimeoutEvent(void);
   void rcResetTimeoutCnt(void);

  TPckgInit(&pckg, MY_ADDR, rcError, rcReadByte, rcWriteByte, rcWasByte, rcTimeoutEvent,rcResetTimeoutCnt);
}

void TRobotDescriptor::ConnectPDS(void)
{
  for(int i=0;i<PortDescrNum;i++)
    if(PortDescr[i].id == pdn)
    {
      pds = &PortDescr[i];
      return;
    }
  error("Robot %d: can't find PortDescr %d", id, pdn);
}

//----------------------------------------------------------

void TXBRobotDescriptor::Show(void)
{
  printf("id = %2d MAC = ", id);
  for(int i=0;i<8;i++) printf("%2X ", addr.MacAddr[i]);
  printf(" NetAddr = %2X %2X\n", addr.NetAddr[0], addr.NetAddr[1]);
}

//----------------------------------------------------------

#define MAX_ROBOT 20
TRobotDescriptor Robot[MAX_ROBOT];
TXBRobotDescriptor XBRobot[MAX_ROBOT];
int RobotNum = 0;

void ReadRobotDescr(FILE *f)
{
  printf("Read robots descr... ");
  if(!ReadInt(f,&RobotNum))
    error("read RobotNum error");
  if(RobotNum<=0 || RobotNum>=MAX_ROBOT)
    error("Illegal RobotNum (%d)'", RobotNum);
  for(int i=0;i<RobotNum;i++)
  {
    Robot[i].Read(f); 
    Robot[i].ConnectPDS(); 
  }
  printf("ok\n");
}

void rsReadPortsRobots(FILE *f)
{
  ReadPortDescr(f);
  ReadRobotDescr(f);
}

/*
# <N>
# <Robot ID> <MAC (HEX)> <Network Address (HEX)>
# ...
# <Robot ID> <MAC (HEX)> <Network Address (HEX)>
#
*/
void rsXBReadRobots(FILE *f1, FILE *f2)
// Чтение конфигурации роботов XBee
{
  l_string s;
  printf("Read robots descr... ");
  if(!ReadString(f1, s))
    error("rsXBReadPortsRobotsr: format error 1");
  RobotNum = atoi(s);
  if(RobotNum<=0 || RobotNum>=MAX_ROBOT)
    error("rsXBReadPortsRobots: illegal NRobots (%d)", RobotNum);
  for(int i=0;i<RobotNum;i++)
  {
    if (!ReadString(f1,s))
      error("rsXBReadPortsRobots: format error 2");
    int n = sscanf(s, "%hhd", &XBRobot[i].id);
    if(!ReadString(f2, s))
      error("rsXBReadPortsRobots: format error 2");

    n += sscanf(s,"%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
      // MAC (HEX)
      &XBRobot[i].addr.MacAddr[0],
      &XBRobot[i].addr.MacAddr[1],
      &XBRobot[i].addr.MacAddr[2],
      &XBRobot[i].addr.MacAddr[3],
      &XBRobot[i].addr.MacAddr[4],
      &XBRobot[i].addr.MacAddr[5],
      &XBRobot[i].addr.MacAddr[6],
      &XBRobot[i].addr.MacAddr[7],
      // Network Address (HEX)
      &XBRobot[i].addr.NetAddr[0],
      &XBRobot[i].addr.NetAddr[1]);
    if(n!=11)
      error("rsXBReadPortsRobots: format error 3");
    XBRobot[i].Show();
  }
}

int rsGetXBAddr(int id, TXBAddr **addr)
{
  for(int i=0;i<RobotNum;i++)
  {
    if(XBRobot[i].id==id)
    {
      *addr = &XBRobot[i].addr;
      return 1;
    }
  }
  error("rsGetXBAddr: Robot %d not found", id);
  return 0;
}

//----------------------------------------------------------
//
//----------------------------------------------------------

TPckg *GetPckg(unsigned char adr)
// Получить ссылку на пакет по адресу (id робота)
{
  for(int i=0;i<RobotNum;i++)
    if(Robot[i].id==adr)
      return &Robot[i].pckg;
  error("GetPckg error (ADDR = %d)", adr);
  return NULL;
}

TPckg *XBGetPckg(unsigned char adr, TXBAddr **addr)
// Получить ссылку на пакет по id робота для XBee-роботов
{
  for(int i=0;i<RobotNum;i++)
    if(XBRobot[i].id==adr)
    {
      *addr = &XBRobot[i].addr;
      return &XBRobot[i].pckg;
    }
  error("XBGetPckg error (ADDR = %d)", adr);
  return NULL;
}

TXBAddr *XBGetAddr(int id)
// Получить XBee-адрес по id робота для XBee-роботов
{
  for(int i=0;i<RobotNum;i++)
  {
    if(XBRobot[i].id==id)
      return &XBRobot[i].addr;
  }
  error("XBGetAddr: Robot %d not found", id);
  return NULL;
}

int GetFd(unsigned char adr)
// Получить дескриптор файла по адресу (id робота)
{
  for(int i=0;i<RobotNum;i++)
    if(Robot[i].id==adr)
      return Robot[i].pds->fd;
  error("GetFd error (ADDR = %d)", adr);
  return 0;
}
