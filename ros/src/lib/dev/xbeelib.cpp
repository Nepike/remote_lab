/**
 * Протокол - XBee/rcX2
 * \author Robofob
 * \version 1.02
 * \date  07.06.2014
 * \date  LP 09.06.2014
*/

#include <stdio.h>
#include <string.h>
#include "../rcproto/rcproto2.h"
#include "../rcproto/tmucmd.h"
#include "xbeelib.h"

//----------------------------------------------------------

unsigned char XBFrame[XBFRAME_SIZE];
TUFrameHeader XBFrameHeader;

//----------------------------------------------------------
void rcError(uchar n);
uchar rcReadByte(void);
void rcWriteByte(uchar c);
uchar rcWasByte(void);
uchar rcTimeoutEvent(void);
void rcResetTimeoutCnt(void);

int xbReadFrame(void)
{
  int i, k, datalen;
  XBFrame[1] = rcReadByte(); 
  XBFrame[2] = rcReadByte(); 
  datalen = (((unsigned int)XBFrame[1]<<8) | (unsigned int)XBFrame[2]);
  
  if(datalen+4>=XBFRAME_SIZE) return 0;
  k = 3;
  for(i=0;i<datalen;i++)
    XBFrame[k++] = rcReadByte();

  if((XBFrame[15]!=HDR_BYTE) || (XBFrame[16]!=HDR_BYTE))
    return 0;

  return k;
}

int rcReadXBeePackage(TPckg *p)
{
  int n;
  if(!rcWasByte()) return 0;
  memset(XBFrame,0,sizeof(XBFrame));
  XBFrame[0] = rcReadByte();
  if(XBFrame[0]!=XB_FRAME_HDR) return 0;  
  
  // Чтение фрейма в буфер
  if(!xbReadFrame()) return 0;  
  
  // Чтение прикладной части пакета из буфера (1 - все хорошо, формат правильный, 0 - иначе)
  // В принимаемом пакете (фрейме) количество байт на 2 меньше
  n = rcReadPackageFromBuff(p, XBFrame, XB_HDR_LEN-2);
  return n;
}

void xbSetFrameAddr(TUFrameHeader *frh, TXBAddr *addr)
{
  int i;
  frh->str.StartByte = 0x7e;
  frh->str.Len[0] = 0;
  frh->str.Len[1] = 0;

  frh->str.Type = 0x10;
  frh->str.Id =  0x01;
  if(addr)
  {
    for(i=0;i<8;i++)
      frh->str.MacAddr[i] = addr->MacAddr[i];
    frh->str.NetAddr[0] = addr->NetAddr[0];
    frh->str.NetAddr[1] = addr->NetAddr[1];
  }
  else
  {
    for(i=0;i<8;i++)
      frh->str.MacAddr[i] = 0x00;
    frh->str.NetAddr[0] = frh->str.NetAddr[1] = 0x00;
  }

  frh->str.Opt[0] = 0x00;
  frh->str.Opt[1] = 0x00;
}

int rcFormXBPackage(TPckg *p)
{
  unsigned char cs = 0xff; // контрольная сумма
  unsigned char i, k;

  // Задаем стандартную часть фрейма для информативной части длиной datalen байт
  // Берем младший байт
  XBFrameHeader.str.Len[0] = 0;
  XBFrameHeader.str.Len[1] = (unsigned char)((int)p->datalen+(int)XB_HDR_LEN-3);
  // Вычисляем контрольную сумму:
  // вычитаем из 0xff значения из стандартной и информативной частей
  for(i=3;i<XB_HDR_LEN; i++) cs -= XBFrameHeader.arr[i];
  for(i=0;i<p->datalen; i++) cs -= p->rcBUFF[i];

  // Записываем
  k = 0;
  for(i=0;i<XB_HDR_LEN;i++)
    XBFrame[k++] = XBFrameHeader.arr[i];
  for(i=0;i<p->datalen;i++)
    XBFrame[k++] = p->rcBUFF[i];
  XBFrame[k++] = cs;
  return k;
}

void xbWritePackage(int framelen)
// Записываем фрейм
{
  int i;
  for(i=0;i<framelen;i++)
    rcWriteByte(XBFrame[i]);
  for(i=0;i<framelen;i++)
    printf("%2d: %2X (%d)\n", i, XBFrame[i], XBFrame[i]);
}

void xbSend1B(TPckg *p, unsigned char addr, unsigned char cmd, unsigned char b)
{
  int frlen;
  rcFormPackage1B(p, addr, cmd, b);
  frlen = rcFormXBPackage(p);
  xbWritePackage(frlen);
}

void xbSendCMD1Arg(TPckg *p, unsigned char addr, unsigned char cmd, unsigned char arg)
{ xbSend1B(p, addr, cmd, arg); }

void xbSendCMDNoArg(TPckg *p, unsigned char addr, unsigned char cmd)
{
  int frlen;
  rcFormPackageCMDNoArg(p, addr, cmd);
  frlen = rcFormXBPackage(p);
  xbWritePackage(frlen);
}

void xbSendCMD2Arg(TPckg *p, unsigned char addr, unsigned char cmd, unsigned char arg1, unsigned char arg2)
{
  int frlen;
  rcFormPackageCMD2Arg(p, addr, cmd, arg1, arg2);
  frlen = rcFormXBPackage(p);
  xbWritePackage(frlen);
}

void xbSendACK(TPckg *p, unsigned char addr)
{ xbSendCMDNoArg(p, addr, CMD_ACK); }

