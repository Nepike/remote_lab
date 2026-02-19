/**
 * Протокол - XBee/rcX2
 * \author Robofob
 * \version 1.02
 * \date  07.06.2014
 * \date  LP 09.06.2014
*/

#ifndef _XBEELIB_H_
#define _XBEELIB_H_

#include "../rcproto/rcproto2.h"
#include "../rcproto/tmucmd.h"

#pragma used+

//----------------------------------------------------------
/// Коды ошибок
#define ERR_XBFR_HDR 21

#define XB_FRAME_HDR 0x7E // Заголовок фрейма
#define XB_HDR_LEN   17   // Длина заголовка фрейма
#define XBFRAME_SIZE 64

/**
 * Заголовок фрейма для отправки сообщения
 * При приеме фрейм имеет длину на 2 байта меньше (пропадает id и один из байтов Opt)
 */
typedef struct
{
  unsigned char StartByte;   /// const: 0x7e
  unsigned char Len[2];
  unsigned char Type;        /// const: 0x10
  unsigned char Id;          /// const: 0x01 (при приеме отсутствует)
  unsigned char MacAddr[8];
  unsigned char NetAddr[2];
  unsigned char Opt[2];      /// const: {0x00,0x00}; (при приеме остается 1 байт)
} TFrameHeader;

typedef union
{
  TFrameHeader str;
  unsigned char arr[XB_HDR_LEN];
} TUFrameHeader;

// Адрес:
typedef struct
{
  unsigned char MacAddr[8];
  unsigned char NetAddr[2];
} TXBAddr;

extern unsigned char XBFrame[XBFRAME_SIZE];
extern TUFrameHeader XBFrameHeader;

/**
 * Чтение фрейма XBee в буфер XBFrame
 * \return Возвращает длину фрейма
 */
int xbReadFrame(void);

/** 
 * Чтение фрейма XBee и выделение пакета протокола rcX
 * \return Возвращает: 0 - пакета нет, 1 - пакет принят
 */
int rcReadXBeePackage(TPckg *p);
 
/**
 * Формирование фрейма
 * \return Возвращает длину фрейма 
 */
int rcFormXBPackage(TPckg *p);

/// Запись фрейма
void xbWritePackage(int framelen);

void xbSendCMDNoArg(TPckg *p, unsigned char addr, unsigned char cmd);
void xbSendCMD2Arg(TPckg *p, unsigned char addr, unsigned char cmd, unsigned char arg1, unsigned char arg2);
void xbSendCMD1Arg(TPckg *p, unsigned char addr, unsigned char cmd, unsigned char arg);
void xbSend1B(TPckg *p, unsigned char addr, unsigned char cmd, unsigned char b);
void xbSendACK(TPckg *p, unsigned char addr);

/** Установка параметров фрейма
 *  FrameHeader[XB_HDR_LEN] =
 *    {0x7e,0x0,0x00,0x10,0x01,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xff,0xfe,0x0,0x0}
 */
void xbSetFrameAddr(TUFrameHeader *frh, TXBAddr *addr);

#pragma used-

#endif
