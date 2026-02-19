/**
  Протокол - rcX2
  \version 1.23
  \author Robofob

  Формат пакета: 
  0xFF 0xFF id from cmd n B1 B2 ...Bn Cs
    id - адресат
    from - получатель
    cmd - команда
    n - количество аргументов
    B1...Bn - аргументы
    Cs - контрольная сумма

  \date 18.01.2014
  \date LP 17.01.2017
*/

#include <stdio.h>
#include "rcproto.h"

void TPckgInit(TPckg *p)
{
  p->pckg_pos = 0;
  p->LAST_CMD = 0;
}

//----------------------------------------------------------
//
//----------------------------------------------------------


/**
 * Основная функция чтения пакета
 */
uchar rcReadPackage(TPckg *p)
{
  uchar  c;
  static int data_len = 0; 

  if(!rcWasByte()) return DATA_WAIT;

  // Проверка таймаута
  // Долго ждать можно лишь первый символ
  if(rcTimeoutEvent() && p->pckg_pos!=0)
  {
    p->pckg_pos=0;
    return ERR_TIMEOUT;
  }
  else rcResetTimeoutCnt();

  p->rcBUFF[p->pckg_pos] = c = rcReadByte();

  if(p->pckg_pos==0)
  {
    if(c!=HDR_BYTE) { p->pckg_pos=0; return ERR_HDR1; }
    p->pckg_pos++;
    return DATA_WAIT;
  }
  if(p->pckg_pos==1)
  {
    if(c!=HDR_BYTE) { p->pckg_pos=0; return ERR_HDR2; }
    p->pckg_pos++;
    return DATA_WAIT;
  }

  if(p->pckg_pos==POS_LEN)
  {
    if(c>=RC_MAX_BUFF-POS_LEN) { p->pckg_pos=0; return ERR_LEN_ERROR; }
    data_len = c;
  }

  if(p->pckg_pos>=RC_MAX_BUFF) { p->pckg_pos=0; return ERR_BUFF_LEN; }

  if(p->pckg_pos==data_len+POS_DATA) // Дочитали до конца
  {
    // Проверяем контрольную сумму
    // Пока формально (!!!)
    p->pckg_pos = 0;
    if(c!=CS_VALUE) return ERR_CS;
    return DATA_READY;
  }

  p->pckg_pos++;

  return DATA_WAIT;
}

//----------------------------------------------------------

uchar rcReadCommand(TPckg *p, uchar *addr)
{
  uchar  i;
  uchar cmd, len, cs;

  p->rcBUFF[0] = rcReadByte();
  if(p->rcBUFF[0]!=HDR_BYTE) rcError(ERR_HDR1);

  p->rcBUFF[1] = rcReadByte();
  if(p->rcBUFF[1]!=HDR_BYTE) rcError(ERR_HDR2);

  *addr = p->rcBUFF[2] = rcReadByte(); // addr

  p->rcBUFF[3] = rcReadByte();         // from

  cmd = p->rcBUFF[4] = rcReadByte();   // cmd
  len = p->rcBUFF[POS_LEN] = rcReadByte();  // len
  if(len>RC_MAX_BUFF) rcError(ERR_BUFF_LEN);

  for(i=0;i<len;i++)
    p->rcBUFF[i+POS_DATA] = rcReadByte();

  cs = p->rcBUFF[(uchar)(len+POS_DATA)] = rcReadByte(); // Контрольная сумма
  if(cs!=CS_VALUE) rcError(ERR_CS);

  return cmd;
}

int rcReadPackageFromBuff(TPckg *p, uchar *buff, int offs)
{
  uchar  i;
  uchar len, cs;

  p->rcBUFF[0] = buff[offs++];
  if(p->rcBUFF[0]!=HDR_BYTE) return 0;

  p->rcBUFF[1] = buff[offs++];
  if(p->rcBUFF[1]!=HDR_BYTE) return 0;

  p->rcBUFF[2] = buff[offs++];   // addr
  p->rcBUFF[3] = buff[offs++];   // from
  p->rcBUFF[4] = buff[offs++];   // cmd
  len = p->rcBUFF[POS_LEN] = buff[offs++];  // len
  if(len>RC_MAX_BUFF) return 0;
  for(i=0;i<len;i++)
    p->rcBUFF[i+POS_DATA] = buff[offs++];

  cs = p->rcBUFF[(uchar)(len+POS_DATA)] = buff[offs]; // Контрольная сумма
  if(cs!=CS_VALUE) return 0;

  return 1;
}

//----------------------------------------------------------
// Чтение
//----------------------------------------------------------

void rcGetBI(TPckg *p, uchar *n, int *intval)
{
  uchar len;
  uchar iLOW, iHIGHT;
  len=p->rcBUFF[POS_LEN];
  if(len!=3) rcError(ERR_FORMAT);
  *n = p->rcBUFF[POS_DATA];
  iLOW = p->rcBUFF[POS_DATA+1];
  iHIGHT = p->rcBUFF[POS_DATA+2];
  *intval = ((int)iHIGHT<<8)| iLOW;
}

void rcGet1B(TPckg *p, uchar *b)
{
  uchar len = p->rcBUFF[POS_LEN];
  if(len!=1) rcError(ERR_FORMAT);
  *b = p->rcBUFF[POS_DATA];
}

void rcGet2B(TPckg *p, uchar *b1, uchar *b2)
{
  uchar len;
  len=p->rcBUFF[POS_LEN];
  if(len!=2) rcError(ERR_FORMAT);
  *b1 = p->rcBUFF[POS_DATA];
  *b2 = p->rcBUFF[POS_DATA+1];
}

void rcGet4B(TPckg *p, uchar *b1, uchar *b2, uchar *b3, uchar *b4)
{
  uchar len;
  len=p->rcBUFF[POS_LEN];
  if(len!=4) rcError(ERR_FORMAT);
  *b1 = p->rcBUFF[POS_DATA];
  *b2 = p->rcBUFF[POS_DATA+1];
  *b3 = p->rcBUFF[POS_DATA+2];
  *b4 = p->rcBUFF[POS_DATA+3];
}

/// Get i2c command
void rcGetI2CDataBuff(TPckg *p, uchar *i2caddr, uchar *i2ccmd, uchar *datanum, uchar databuff[])
{
  uchar i, len;
  len=p->rcBUFF[POS_LEN];
  if(len<3) rcError(ERR_FORMAT);
  *i2caddr = p->rcBUFF[POS_DATA];
  *i2ccmd = p->rcBUFF[POS_DATA+1];
  *datanum = p->rcBUFF[POS_DATA+2];
  for(i=0;i<*datanum;i++)
    databuff[i] = p->rcBUFF[POS_DATA+3+i];
}

//----------------------------------------------------------
// Работа с пакетом
//----------------------------------------------------------

uchar rcGetAddr(TPckg *p)
{ return p->rcBUFF[POS_ADDR]; }

uchar rcGetCmd(TPckg *p)
{ return p->rcBUFF[POS_CMD]; }

uchar rcGetFrom(TPckg *p)
{ return p->rcBUFF[POS_FROM]; }

//----------------------------------------------------------
// Запись
//----------------------------------------------------------

int rcFormPackageCMDNoArg(TPckg *p, uchar addr, uchar cmd)
{
  p->LAST_CMD = cmd;
  p->rcBUFF[0] = p->rcBUFF[1] = HDR_BYTE;
  p->rcBUFF[2] = addr;
  p->rcBUFF[3] = MY_ADDR;
  p->rcBUFF[4] = cmd;
  p->rcBUFF[5] = 0;
  p->rcBUFF[6] = CS_VALUE;
  p->datalen = 7;
  return p->datalen;
}

int rcFormPackage1B(TPckg *p, uchar addr, uchar cmd, uchar b)
{
  p->LAST_CMD = cmd;
  p->rcBUFF[0] = p->rcBUFF[1] = HDR_BYTE;
  p->rcBUFF[2] = addr;
  p->rcBUFF[3] = MY_ADDR;
  p->rcBUFF[4] = cmd;
  p->rcBUFF[5] = 1;
  p->rcBUFF[6] = b;
  p->rcBUFF[7] = CS_VALUE;
  p->datalen = 8;
  return p->datalen;
}

int rcFormPackageCMD2Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2)
{
  p->LAST_CMD = cmd;
  p->rcBUFF[0] = p->rcBUFF[1] = HDR_BYTE;
  p->rcBUFF[2] = addr;
  p->rcBUFF[3] = MY_ADDR;
  p->rcBUFF[4] = cmd;
  p->rcBUFF[5] = 2;
  p->rcBUFF[6] = arg1;
  p->rcBUFF[7] = arg2;
  p->rcBUFF[8] = CS_VALUE;
  p->datalen = 9;
  return p->datalen;
}

int rcFormPackageCMD3Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3)
{
  p->LAST_CMD = cmd;
  p->rcBUFF[0] = p->rcBUFF[1] = HDR_BYTE;
  p->rcBUFF[2] = addr;
  p->rcBUFF[3] = MY_ADDR;
  p->rcBUFF[4] = cmd;
  p->rcBUFF[5] = 3;
  p->rcBUFF[6] = arg1;
  p->rcBUFF[7] = arg2;
  p->rcBUFF[8] = arg3;
  p->rcBUFF[9] = CS_VALUE;
  p->datalen = 10;
  return p->datalen;
}

int rcFormPackageCMD4Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4)
{
  p->LAST_CMD = cmd;
  p->rcBUFF[0] = p->rcBUFF[1] = HDR_BYTE;
  p->rcBUFF[2] = addr;
  p->rcBUFF[3] = MY_ADDR;
  p->rcBUFF[4] = cmd;
  p->rcBUFF[5] = 4;
  p->rcBUFF[6] = arg1;
  p->rcBUFF[7] = arg2;
  p->rcBUFF[8] = arg3;
  p->rcBUFF[9] = arg4;
  p->rcBUFF[10] = CS_VALUE;
  p->datalen = 11;
  return p->datalen;
}

int rcFormPackageI2CDataArray(TPckg *p, uchar addr, uchar cmd, uchar i2caddr, uchar i2ccmd, uchar i2clen, uchar i2cbuff[])
{
  uchar len = 3+i2clen;
  p->LAST_CMD = cmd;
  p->rcBUFF[0] = p->rcBUFF[1] = HDR_BYTE;
  p->rcBUFF[2] = addr;
  p->rcBUFF[3] = MY_ADDR;
  p->rcBUFF[4] = cmd;
  p->rcBUFF[5] = len;
  p->rcBUFF[6] = i2caddr;
  p->rcBUFF[7] = i2ccmd;
  p->rcBUFF[8] = i2clen;
  uchar n = 9;
  for(uchar i=0;i<i2clen;i++)
    p->rcBUFF[n++] = i2cbuff[i];
  p->rcBUFF[n++] = CS_VALUE;  
  p->datalen = n;
  return p->datalen;
}

void rcSendCMDNoArg(TPckg *p, uchar addr, uchar cmd)
{
  rcFormPackageCMDNoArg(p, addr, cmd);
  rcWritePackage(p);
}

void rcSend1B(TPckg *p, uchar addr, uchar cmd, uchar b)
{
  rcFormPackage1B(p, addr, cmd, b);
  rcWritePackage(p);
}

void rcSendCMD1Arg(TPckg *p, uchar addr, uchar cmd, uchar arg)
{ rcSend1B(p, addr, cmd, arg); }

void rcSendCMD2Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2)
{
  rcFormPackageCMD2Arg(p, addr, cmd, arg1, arg2);
  rcWritePackage(p);
}

void rcSendCMD3Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3)
{
  rcFormPackageCMD3Arg(p, addr, cmd, arg1, arg2, arg3);
  rcWritePackage(p);
}

void rcSendCMD4Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4)
{
  rcFormPackageCMD4Arg(p, addr, cmd, arg1, arg2, arg3, arg4);
  rcWritePackage(p);
}

void rcSendI2CDataArray(TPckg *p, uchar addr, uchar cmd, uchar i2caddr, uchar i2ccmd, uchar i2clen, uchar i2cbuff[])
{
  rcFormPackageI2CDataArray(p, addr, cmd, i2caddr, i2ccmd, i2clen, i2cbuff);
  rcWritePackage(p);
}

void rcSendACK(TPckg *p, uchar addr) { rcSendCMDNoArg(p, addr, CMD_ACK); }

void rcWritePackage(TPckg *p)
{
  int i;
  for(i=0;i<p->datalen;i++)
    rcWriteByte(p->rcBUFF[i]);
}
