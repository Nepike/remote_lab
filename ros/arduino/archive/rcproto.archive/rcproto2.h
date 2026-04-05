/**
  \file rcproto.h
  Протокол - rcX2
  \version 2.26
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
  \date LP 01.02.2018
*/

#ifndef _RC_PROTO2X_H_
#define _RC_PROTO2X_H_

#pragma used+

typedef unsigned char uchar;

#define HDR_BYTE 0xFF

// Контрольная сумма
#define CS_VALUE 0xAA


/*
 * Этот макрос может быть определен в rcproto2/tmucmd
*/
#ifndef CMD_ACK
  #define CMD_ACK  0x0B  /// Ответ: подтверждение
#endif

//----------------------------------------------------------
/// Коды ошибок
#define DATA_READY      0
#define DATA_WAIT       1

#define ERR_HDR1        2
#define ERR_HDR2        3
#define ERR_ILLEGAL_CMD 4
#define ERR_FORMAT      5
#define ERR_LEN_ERROR   6
#define ERR_BUFF_LEN    7
#define ERR_CS          8   /// Ошибка контрольной суммы
#define ERR_TIMEOUT     9   /// Истечение времени ожидания

//----------------------------------------------------------
//
//----------------------------------------------------------

#define RC_MAX_BUFF    80 //64
/// Пакет
typedef struct
{
  uchar rcBUFF[RC_MAX_BUFF];
  uchar pckg_pos;
  uchar LAST_CMD;
  uchar datalen;

  uchar MY_ADDR;

  void (*rcErrorPtr)(uchar n);
  uchar (*rcReadBytePtr)(void);
  void (*rcWriteBytePtr)(uchar c);
  uchar (*rcWasBytePtr)(void);         // Проверка наличия пришедшего символа

  uchar (*rcTimeoutEventPtr)(void);    // Проверка таймаута. Возвращает 0, если истекло время ожидания очередного символа пакета
  void (*rcResetTimeoutCntPtr)(void);  // Сброс счетчика таймаута

} TPckg;


void TPckgInit(TPckg *p,
  uchar myaddr,
  void (*fErrorPtr)(uchar),
  uchar (*fReadBytePtr)(void),
  void (*fWriteByte)(uchar),
  uchar (*fWasBytePtr)(void),          // Проверка наличия пришедшего символа
  uchar (*fTimeoutEventPtr)(void),     // Проверка таймаута. Возвращает 0, если истекло время ожидания очередного символа пакета
  void (*fResetTimeoutCntPtr)(void));  // Сброс счетчика таймаута

/** Смещения в пакете
 * 0    1    2  3    4   5 6 ...
 * 0xFF 0xFF id from cmd n B1 B2 ...Bn Cs
 */
#define POS_ADDR 2
#define POS_FROM 3
#define POS_CMD  4
#define POS_LEN  5
#define POS_DATA 6

//----------------------------------------------------------
// Чтение
//----------------------------------------------------------
/**
 * Основная функция чтения пакета
 * \return Возвращает 0, если все хорошо и код ошибки в противном случае
 */
uchar rcReadPackage(TPckg *p);

/**
 * Чтение пакета из буфера
 * \return Возвращает 1, если все хорошо (формат правильный) и 0 в противном случае
 */
int rcReadPackageFromBuff(TPckg *p, uchar *buff, int offs);

/**
 * Чтение пакета в ждущем режиме
 * \return Возвращает код команды и адрес (id получателя)
 */
uchar rcReadCommand(TPckg *p, uchar *addr);

/// Read BYTE, int
void rcGetBI(TPckg *p, uchar *n, int *intval);

/// Get BYTE
void rcGet1B(TPckg *p, uchar *b);

/// Get BYTE, BYTE
void rcGet2B(TPckg *p, uchar *b1, uchar *b2);

/// Get 4 BYTEs
void rcGet4B(TPckg *p, uchar *b1, uchar *b2, uchar *b3, uchar *b4);

/// Get i2c command
void rcGetI2CDataBuff(TPckg *p, uchar *i2caddr, uchar *i2ccmd, uchar *datanum, uchar databuff[]);

//----------------------------------------------------------
// Работа с пакетом
//----------------------------------------------------------
uchar rcGetAddr(TPckg *p);
uchar rcGetCmd(TPckg *p);
uchar rcGetFrom(TPckg *p);

//----------------------------------------------------------
// Запись
//----------------------------------------------------------

/// Запись пакета
void rcWritePackage(TPckg *p);
void rcWriteArray(TPckg *p, uchar addr, uchar from, uchar cmd, uchar array[], uchar len);

void rcSendACK(TPckg *p, uchar addr);
void rcSend1B(TPckg *p, uchar addr, uchar cmd, uchar b);

void rcSendCMDNoArg(TPckg *p, uchar addr, uchar cmd);
void rcSendCMD1Arg(TPckg *p, uchar addr, uchar cmd, uchar arg);
void rcSendCMD2Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2);
void rcSendCMD3Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3);
void rcSendCMD4Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4);

void rcSendI2CDataArray(TPckg *p, uchar addr, uchar cmd, uchar i2caddr, uchar i2ccmd, uchar i2clen, uchar i2cbuff[]);

int rcFormPackageCMDNoArg(TPckg *p, uchar addr, uchar cmd);
int rcFormPackage1B(TPckg *p, uchar addr, uchar cmd, uchar b);
int rcFormPackageCMD2Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2);
int rcFormPackageCMD3Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3);
int rcFormPackageCMD4Arg(TPckg *p, uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4);

int rcFormPackageI2CDataArray(TPckg *p, uchar addr, uchar cmd, uchar i2caddr, uchar i2ccmd, uchar i2clen, uchar i2cbuff[]);

//----------------------------------------------------------
// Объектная форма
//----------------------------------------------------------

class TPckgObj
{
public:
  TPckgObj(uchar myaddr,
    void (*fErrorPtr)(uchar),
    uchar (*fReadBytePtr)(void),
    void (*fWriteByte)(uchar),
    uchar (*fWasBytePtr)(void),        // Проверка наличия пришедшего символа
    uchar (*fTimeoutEventPtr)(void),   // Проверка таймаута. Возвращает 0, 
                                       // если истекло время ожидания очередного символа пакета
    void (*fResetTimeoutCntPtr)(void)) // Сброс счетчика таймаута
  { 
     TPckgInit(&pckg, myaddr, fErrorPtr, fReadBytePtr, fWriteByte,
          fWasBytePtr, fTimeoutEventPtr, fResetTimeoutCntPtr);
  }

/**
 * Основная функция чтения пакета
 * \return Возвращает 0, если все хорошо и код ошибки в противном случае
 */
  uchar ReadPackage(void) { return rcReadPackage(&pckg); }

/**
 * Чтение пакета в ждущем режиме
 * \return Возвращает код команды и адрес (id получателя)
 */
  uchar ReadCommand(uchar *addr) { return rcReadCommand(&pckg, addr); }

  uchar GetAddr(void) { return rcGetAddr(&pckg); }
  uchar GetCmd(void) { return rcGetCmd(&pckg); }
  uchar GetFrom(void) { return rcGetFrom(&pckg); }

  /// Запись пакета
  void WritePackage(void) { rcWritePackage(&pckg); }

  void SendACK(uchar addr) { rcSendACK(&pckg, addr); }
  void Send1B(uchar addr, uchar cmd, uchar b) { rcSend1B(&pckg, addr, cmd, b); }

  void SendCMDNoArg(uchar addr, uchar cmd) { rcSendCMDNoArg(&pckg, addr, cmd); }

  void SendCMD1Arg(uchar addr, uchar cmd, uchar arg) { rcSendCMD1Arg(&pckg, addr, cmd, arg); }
  void SendCMD2Arg(uchar addr, uchar cmd, uchar arg1, uchar arg2) { rcSendCMD2Arg(&pckg, addr, cmd, arg1, arg2); }

  void SendCMD3Arg(uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3) 
  { rcSendCMD3Arg(&pckg, addr, cmd, arg1, arg2, arg3); }

  void SendCMD4Arg(uchar addr, uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4) 
  { rcSendCMD4Arg(&pckg, addr, cmd, arg1, arg2, arg3, arg4); }

  void SendI2CDataArray(uchar addr, uchar cmd, uchar i2caddr, uchar i2ccmd, uchar i2clen, uchar i2cbuff[]) 
  { rcSendI2CDataArray(&pckg, addr, cmd, i2caddr, i2ccmd, i2clen, i2cbuff); }

  uchar GetPckgInfo(uchar *from, uchar *cmd)
  {    
    *from = GetFrom();
    *cmd = GetCmd();
    return GetAddr();
  }

  void WriteArray(uchar addr, uchar from, uchar cmd, uchar array[], uchar len)
  { rcWriteArray(&pckg, addr, from, cmd, array, len); }

private:
  TPckg pckg;
};

#pragma used-

#endif
