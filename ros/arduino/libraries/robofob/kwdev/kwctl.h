/*
 * 
 * 22.08.2018
 * LP 08.11.2022
 */
 
#ifndef _KWCTL_H_
#define _KWCTL_H_

#include <EEPROM.h>
#include <TimerOne.h>
#include <LiquidCrystal_I2C.h>
#include "Ultrasonic.h"

#include "rbuff.h"
#include "kwdev.h"
#include "devdef.h"

//------------------------------------------------------------------------------
//
// Сенсоры
//
//------------------------------------------------------------------------------

#define PROG_LEN 10 // Длина программы
#define STR_LEN  10

namespace kwc
{

//------------------------------------------------------------------------------
//
// Эффекторы
//
//------------------------------------------------------------------------------
// Актуаторы. Драйверы Pololu Simple Motor  подключены по UART: Serial1, 19200

// Двигатели L298
extern const int PIN_M_DIR[];
extern const int PIN_M_PWM[];

// Сервомашинки. Драйверы Pololu Maestro, подключены по UART
// Serial2, 9600

// Управление двигателями (реле PWM)
extern const int PIN_PWM_DEV[];

//------------------------------------------------------------------------------
//
// Сенсоры
//
//------------------------------------------------------------------------------
// УЗД
extern Ultrasonic US_SENSORS[];
extern const int MAX_US_DISTANCE;


// Массив значений сенсоров
extern int sData[];

// Актуаторы. Драйверы Pololu Simple Motor  подключены по UART: Serial1, 19200
// АЦП актуаторов (A0, A1,...)
extern const int PIN_ACT_ADC[];

// Аналоговые сенсоры
extern const int PIN_ANALOG_SENS[];

//------------------------------------------------------------------------------
extern byte debug_regime;
extern boolean was_cmd_debug_regime;
extern int GLOBAL_PWM_LIMIT;

//------------------------------------------------------------------------------
//
// Программа
// Определяется как статическая структура, т.к. далее она целиком записывается/считывается
// в/из EEPROM
//------------------------------------------------------------------------------
struct TProgram
{
  char pname[STR_LEN+1]; // Имя программы
  int data[_kwc_NUM_DEV][PROG_LEN];    // Собственно программы
  char description[_kwc_NUM_DEV][STR_LEN+1];// Описания программ
  byte ratio[_kwc_NUM_DEV];            // Масштабные множители
  byte enabled[_kwc_NUM_DEV];          // Признаки активности программ
  byte tcnt[_kwc_NUM_DEV];             // Счетчик позиции (индекс) в циклограмме
};

// Масштабный множитель для всех значений амплитуд двигательных программ, [1..5]
extern float progr_scale_A;
// Масштабный множитель для частот двигательных программ, [1..5]
extern float progr_scale_F;

extern TProgram Program;

extern LiquidCrystal_I2C lcd;
extern boolean FirstRun;
extern int eeAddress;

extern long GlobalProgCounter;

extern TDevice *dev[];

//
// Управление режимами и индикация
//
// Выбор режима (отладка/работа)
extern int PIN_REGIME_SWITCH;

// Индикатор режима отладки
extern int PIN_DEBUG_REG_INDICATOR;

// Пищалка
extern int PIN_BEEP;

void LCDInit(char *title);
void LCDprint(const char *s);
void LCDprint(byte b);
void LCDprint(byte line, const char *s);
void LCDprint(byte line, byte row, char *s);
void LCDprint(byte line, byte row, byte b);
void LCDprint(byte line, byte b);


void Beep(byte n);

extern Stream *PCStream;
extern Stream *MotorStream;
extern Stream *Servo_Stream;
/**
 *  _pc_stream - Computer (9600)
 * _motor_stream - Actuators (Pololu motor drivers) (19200)
 * _servo_stream - Servo (Pololu Maestro) (9600)
 *
 */
void InitSystem(char *title, Stream &_pc_stream, Stream &_motor_stream, Stream &_servo_stream);

void ProgramInit(void);

/*
 * Таймер
 */
void timerIsr();


/*
 * Инициализация всех устройств
 *   70, 170 - China long
 *    9, 250 - China extra long
 * Китайские короткие:
 *   80(82)-170(179)
 * SKF короткие:
 *   14(15)-150(150)
 */
int InitDevices(void);

// Сброс всех устройств (драйверов)
void ResetAll(void);

// Отработка заданий
void ExecuteProgram(void);

/*
 * Инициализация программы и запись ее в EEPROM
 * Должна вызываться один раз при прошивке контроллера
 * См. boolean kwc::FirstRun
 */
void ResetDeviceProgram(void);

void PrintSensors(void);

// Показать статус устройств (скорости и проч)
void ShowStatus(void);

void ReadSensors(void);

void ProgramWrite(void);
void ProgramRead(void);

String MReadStr(char c);
void MReadStr(char *res, char c);
int MReadInt(void);

/*
 * Чтение и отработка внешней команды
 */
void EvalExtCmd(void);
void MakeProgramStep(void);
void Read_and_Display_Regimes(void);

};

#endif
