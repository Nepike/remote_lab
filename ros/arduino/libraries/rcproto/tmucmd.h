/**
  \file tmucmd.h
  Протокол - rcX2
  \version 2.31
  \author Robofob

  Коды команд и регистры для архитектуры TMU

  \date 18.01.2014, 13.08.2020
  \date LP 16.01.2024
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
#define CMD_SET_REG          0x0F  /// Установить значение регистра ОЗУ
#define CMD_SET_EEP_REG      0x1F  /// Установить значение регистра ПЗУ (EEPROM)

#define CMD_GET_REG          0x10  /// Запрос: Получить значение регистра ОЗУ
#define CMD_GET_EEP_REG      0x14  /// Запрос: Получить значение регистра ПЗУ (EEPROM)

#define CMD_ANS_GET_REG      0x11  /// Ответ на CMD_GET_REG: значение регистра ОЗУ
#define CMD_ANS_GET_EEP_REG  0x15  /// Ответ на CMD_GET_REG: значение регистра ПЗУ (EEPROM)

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

#define CMD_SET_VSPEED         0x29  /// Установка линейной и угловой скоростей vlin vang (в градусах)

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
 * Регистры ОЗУ
 */
#define NREG_RAM 15 // Количество регистров ОЗУ

#define REG_VERSION      0   // Версия
#define REG_ANG_STEP     1   // ANG_STEP Шаг угла поворота
#define REG_STOP_SPEED   2   // StopSpeed Скорость останова (коррекционное значение)
#define REG_SPEED        3   // Speed Текущая скорость
#define REG_LOC_ENABLE   4   // Разрешение поворота локатора
#define REG_ACK          5   // Флаг режима подтверждения
#define REG_USR          6   // REG_USR

// Здесь хранятся не тики энкодеров, а пройденное расстояние в см.
#define REG_D1           7   // GlobalEncoderLeftCnt -> sm
#define REG_D2           8   // GlobalEncoderRightCnt -> sm
#define REG_STATUS       9   // Регистр статуса

// Регистры энкодеров 16 разрядов, разделенные на 2 байта D0 и D1
#define REG_ENC_LEFT_D0  10  // GlobalEncoderLeftCnt
#define REG_ENC_LEFT_D1  11  //
#define REG_ENC_RIGHT_D0 12  // GlobalEncoderRightCnt
#define REG_ENC_RIGHT_D1 13  //
#define REG_CURR_ANG     14  // Текущий угол поворота локатора

/**
 * Регистры ПЗУ
 */
#define NREG_EEP 24 // Количество регистров EEPROM

#define EEP_ID           0 // ID робота
#define EEP_TM_INTERRUPT 1 // Период прерываний таймера, мс
#define EEP_MINU         2 // Минимальное значение управления minU (от 0 до 255)
#define EEP_MAXU         3 // Максимльное значение управления maxU (от 0 до 255)
#define EEP_MINSPEED     4 // minSpeed - Скорость трогания 
#define EEP_MAXSPEED     5 // maxSpeed - Определяется процедурой DefineMaxSpeed(pidlib::maxU)
#define EEP_CHECK_U      6 // Проверяемое напряжения питания
#define EEP_USS_DIST     7 // Дистанция срабатывания рефлекторных датчиков: для tmu-2 - датчики Sharp, для tmu-4 - УЗД датчики, см. 
#define EEP_BUMP_DIST    8 // Дистанция срабатывания бамперов типа контакт/Sharp

#define EEP_KDB_MOV      9 // Коэффициент для скорости движения вперед при отладке, % (0..100)
#define EEP_KDB_ROT     10 // Коэффициент для скорости разворота при отладке, % (0..100)

// Геометрия робота. Диаметр колеса, мм.
#define EEP_GMT_RWD_LO  11
#define EEP_GMT_RWD_HI  12

// Геометрия робота. Количество импульсов энкодеров на 1 оборот колеса
// (т.к. мы регистрируем изменения сигнала, а не сам сигнал, то будем писать в 2 раза больше)
#define EEP_GMT_RECNT_LO 13
#define EEP_GMT_RECNT_HI 14

// Геометрия робота. Расстояние между колесами, мм.
#define EEP_GMT_RLW_LO   15
#define EEP_GMT_RLW_HI   16

// ПИД-регулятор
#define EEP_PID_KP       17 // Пропорциональное звено, %, (0..255)
#define EEP_PID_KI       18 // Интегрирующее звено, %, (0..255)
#define EEP_PID_KD       19 // Дифференциальное звено, %, (0..255)

#define EEP_SENS_ENABLE  20  // Определение номенклатуры датчиков: 0x01 - разрешить Sharp, 0x02 - разрешить УЗД
#define EEP_REFLEX       21  // 0x03 Регистр рефлексов: 0x01 - рефлекс питания, 0x02 - рефлекс препятствия
#define EEP_PWMSPEED     22  // CSPEED - скорость движения без ПИД-управления (базовая скорость движения) = 250
#define EEP_TM_STOP      23  // Время, в течение которого которое робот отрабатывает последнюю команду после потери связи с ВУУ, децисек.

struct TPair
{
  const char *name;
  int code;
};

extern TPair RAM_REG_NAMES[];
extern TPair EEPROM_REG_NAMES[];

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
