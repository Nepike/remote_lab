/**
  \file tmucmd.h
  Протокол - rcX2
  \version 2.26
  \author Robofob

  Коды команд и регистры для архитектуры TMU

  \date 18.01.2014
  \date LP 09.02.2018
*/

#ifndef _TMU_CMD_H_
#define _TMU_CMD_H_

#pragma used+

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

/*
 * Этот макрос может быть определен в rcproto2/tmucmd
*/
#ifndef CMD_ACK
  #define CMD_ACK            0x0B  /// Ответ: подтверждение
#endif

#define CMD_GET_SENS         0x0C  /// Запрос: Получить значения всех сенсоров (АЦП)
#define CMD_ANS_GET_SENS     0x0D  /// Ответ на CMD_GET_SENS

/// Работа с сервомашинкой
#define CMD_SET_SERVO_POS    0x0E  /// Установить сервомашинку в позицию ang (угол в градусах от 0 до 180)

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
                                     /// Значения скоростей от -100 до +100 (в процентах от pidlib::maxSpeed,
                                     /// т.е. того, что определено в роботе)

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

#define REG_SENS_ENABLE 18 /// Определение номенклатуры датчиков: 0x01 - разрешить Sharp, 0x02 - разрешить УЗД */

// Регистры энкодеров 16 разрядов, разделенные на 2 байта D0 и D1
#define REG_ENC_LEFT_D0  19
#define REG_ENC_LEFT_D1  20
#define REG_ENC_RIGHT_D0 21
#define REG_ENC_RIGHT_D1 22

//----------------------------------------------------------
// Вспомогательные функции
//----------------------------------------------------------

inline unsigned char IS_MOVE_CMD(unsigned char cmd)
{
  return  (cmd == CMD_STOP ||
           cmd == CMD_FWD || cmd == CMD_FWD2 || cmd == CMD_BACK || cmd == CMD_BACK2 ||
           cmd == CMD_LEFT || cmd == CMD_LEFT2 || cmd == CMD_RIGHT || cmd == CMD_RIGHT2 ||
           cmd == CMD_FAST_LEFT || cmd == CMD_FAST_LEFT2 || cmd == CMD_FAST_RIGHT || cmd == CMD_FAST_RIGHT2 || cmd==CMD_SET_SPEED);
}

inline unsigned char IS_NOT_ACK_CMD(unsigned char cmd)
{ return (cmd==CMD_GET_SENS || cmd==CMD_GET_ALL_REG ||
          cmd==CMD_GET_REG  || cmd==CMD_GET_USR_DATA || cmd==CMD_GET_I2C_DATA); }

inline unsigned char IS_REENTERABLE_MOVING(unsigned char cmd)
{ return (cmd==CMD_FWD2 || cmd==CMD_BACK2 || cmd==CMD_FAST_LEFT2 || cmd==CMD_FAST_RIGHT2 || cmd==CMD_SET_SPEED); }

#pragma used-

#endif
