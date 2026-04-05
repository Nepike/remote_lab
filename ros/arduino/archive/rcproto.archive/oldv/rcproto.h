/**
  \file rcproto.h
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

#ifndef _RC_PROTO2X_H_
#define _RC_PROTO2X_H_

#pragma used+

typedef unsigned char uchar;

#define HDR_BYTE 0xFF

// Контрольная сумма
#define CS_VALUE 0xAA

//----------------------------------------------------------
// --- Команды ---
//----------------------------------------------------------
/// Движение
#define CMD_STOP             0x01  /// Останов
#define CMD_FWD              0x02  /// Вперед
#define CMD_BACK             0x03  /// Назад
#define CMD_LEFT             0x04  /// Налево (одним колесом)
#define CMD_RIGHT            0x05  /// Направо (одним колесом)
#define CMD_FAST_LEFT        0x06  /// Налево (танковый разворот с реверсом)
#define CMD_FAST_RIGHT       0x07  /// Направо (танковый разворот с реверсом

#define CMD_BEEP             0x08  /// Звуковой сигнал
#define CMD_BEEP_ON          0x09  /// Включить пищалку
#define CMD_BEEP_OFF         0x0A  /// Выключить пищалку

#define CMD_ACK              0x0B  /// Ответ: подтверждение
#define CMD_GET_SENS         0x0C  /// Запрос: Получить значения всех сенсоров (АЦП)
#define CMD_ANS_GET_SENS     0x0D  /// Ответ на CMD_GET_SENS
#define CMD_SET_ANG          0x0E  /// Установить угол сервопривода (град)

/// Работа с регистрами
#define CMD_SET_REG          0x0F  /// Установить значение регистра
#define CMD_GET_REG          0x10  /// Запрос: Получить значение регистра
#define CMD_ANS_GET_REG      0x11  /// Ответ на CMD_GET_REG: значение регистра
#define CMD_GET_ALL_REG      0x12  /// Запрос: Получить значения всех регистров
#define CMD_ANS_GET_ALL_REG  0x13  /// Ответ на CMD_GET_ALL_REG: значения всех регистров

#define CMD_PING             0xFF  /// Команда опроса готовности контроллера

/// Движение с дополнительным аргументом
#define CMD_STOP2              0x21
#define CMD_FWD2               0x22
#define CMD_BACK2              0x23
#define CMD_LEFT2              0x24
#define CMD_RIGHT2             0x25
#define CMD_FAST_LEFT2         0x26
#define CMD_FAST_RIGHT2        0x27
#define CMD_SET_SPEED          0x28  /// Установить скорости вращения колес. Формат: LeftSpeed, RightSpeed

/// Прочее
#define CMD_DEBUG              0x31  /// Запуск процедуры отладки
#define CMD_GET_USR_DATA       0x32  /// Запрос: Получить массив пользовательских данных
#define CMD_ANS_GET_USR_DATA   0x36  /// Ответ на CMD_GET_USR_DATA
#define CMD_I2C                0x33  /// Команда I2C-устройству. Формат: <адрес I2C-устройства> <команда> <аргумент1> <аргумент2>
#define CMD_GET_I2C_DATA       0x34  /// Запрос: Получить массив данных от I2C-устройства.
                                     /// Формат: <адрес I2C-устройства> <размер запрашиваемых данных>
#define CMD_ANS_GET_I2C_DATA   0x35  /// Ответ на CMD_GET_I2C_DATA

//----------------------------------------------------------

/**
 * Регистры
 */
#define REG_ID          0   /// ID робота
#define REG_VERSION     1   /// Версия
#define REG_ANG_STEP    2   /// Шаг угла поворота
#define REG_STOP_SPEED  3   /// Скорость останова (коррекционное значение)
#define REG_SPEED       4   /// Текущая скорость
#define REG_LOC_ENABLE  5   /// Разрешение поворота локатора
#define REG_ACK         6   /// Флаг режима подтверждения
#define REG_USR         7   /// REG_USR

#define REG_D1          8   /// REG_ENC_LEFT. GlobalEncoderLeftCnt, см.
#define REG_D2          9   /// REG_ENC_RIGHT. GlobalEncoderRightCnt, см.
#define REG_ENC_LEFT    REG_D1
#define REG_ENC_RIGHT   REG_D2

#define REG_BUMP_DIST   10  /// Дистанция срабатывания цифровых (контактных) бамперов
#define REG_USS_DIST    11  /// Дистанция срабатывания аналоговых (УЗД) бамперов

#define REG_MINU        12  /// minU
#define REG_MAXU        13  /// maxU
#define REG_MINSPEED    14  /// minSpeed Скорость трогания
#define REG_MAXSPEED    15  /// maxSpeed
#define REG_PWMSPEED    16  /// pwmspeed Скорость движения без ПИД-управления
#define REG_STATUS      17  /// Регистр статуса

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
} TPckg;

extern uchar MY_ADDR;

void TPckgInit(TPckg *p);

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
// Эти функции должны быть реализованы в основной программе
//----------------------------------------------------------
extern void rcError(uchar n);
extern uchar rcReadByte(void);
extern void rcWriteByte(uchar c);
extern uchar rcWasByte(void);         // Проверка наличия пришедшего символа

extern uchar rcTimeoutEvent(void);    // Проверка таймаута. Возвращает 0, если истекло время ожидания очередного символа пакета
extern void rcResetTimeoutCnt(void);  // Сброс счетчика таймаута

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
// Вспомогательные функции
//----------------------------------------------------------

inline uchar IS_MOVE_CMD(uchar cmd)
{
  return  (cmd == CMD_STOP ||
           cmd == CMD_FWD || cmd == CMD_FWD2 || cmd == CMD_BACK || cmd == CMD_BACK2 ||
           cmd == CMD_LEFT || cmd == CMD_LEFT2 || cmd == CMD_RIGHT || cmd == CMD_RIGHT2 ||
           cmd == CMD_FAST_LEFT || cmd == CMD_FAST_LEFT2 || cmd == CMD_FAST_RIGHT || cmd == CMD_FAST_RIGHT2 || cmd==CMD_SET_SPEED);
}

inline uchar IS_NOT_ACK_CMD(uchar  cmd)
{ return (cmd==CMD_GET_SENS || cmd==CMD_GET_ALL_REG ||
          cmd==CMD_GET_REG  || cmd==CMD_GET_USR_DATA || cmd==CMD_GET_I2C_DATA); }

inline uchar IS_REENTERABLE_MOVING(uchar  cmd)
{ return (cmd==CMD_FWD2 || cmd==CMD_BACK2 || cmd==CMD_FAST_LEFT2 || cmd==CMD_FAST_RIGHT2 || cmd==CMD_SET_SPEED); }


#pragma used-

#endif
