/**
 *
 *  LP 14.11.2022
 */
#ifndef _DEVICES_INC_
#define _DEVICES_INC_

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
//******************************************************************************
//
// Mega
//
//******************************************************************************

//
// ** Сенсоры
//
// УЗД
#define _kwc_NUM_US_SENS  2

// Аналоговые сенсоры
#define _kwc_NUM_ANALOG_SENS 8

// ** Эффекторы

// Актуаторы. Драйверы Pololu Simple Motor  подключены по UART: Serial1, 19200
// АЦП актуаторов (A0, A1,...)
#define _kwc_NUM_ACTUATORS  6

// Двигатели L298
#define _kwc_NUM_298MOTORS 4

// Сервомашинки. Драйверы Pololu Maestro, подключены по UART
#define _kwc_NUM_SERVO  12

// Управление двигателями (реле PWM)
#define _kwc_NUM_PWM_DEV 6

#else

//******************************************************************************
//
// Nano
//
//******************************************************************************

//
// ** Сенсоры
//
// УЗД
#define _kwc_NUM_US_SENS  2
// Аналоговые сенсоры
#define _kwc_NUM_ANALOG_SENS 4

// ** Эффекторы

// Актуаторы. Драйверы Pololu Simple Motor  подключены по UART: Serial1, 19200
// АЦП актуаторов (A0, A1,...)
#define _kwc_NUM_ACTUATORS  0

// Двигатели L298
#define _kwc_NUM_298MOTORS 0

// Сервомашинки. Драйверы Pololu Maestro, подключены по UART
#define _kwc_NUM_SERVO  6

// Управление двигателями (реле PWM)
#define _kwc_NUM_PWM_DEV 0

#endif

//******************************************************************************
//
//******************************************************************************

// Общее количество устройств
#define _kwc_NUM_DEV (_kwc_NUM_ACTUATORS + _kwc_NUM_298MOTORS + _kwc_NUM_SERVO + _kwc_NUM_PWM_DEV)

// Общее число сенсоров
#define _kwc_NUM_SENS  (_kwc_NUM_US_SENS + _kwc_NUM_ANALOG_SENS)

#endif
