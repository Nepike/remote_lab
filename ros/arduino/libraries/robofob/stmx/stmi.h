/*
 *  Интерфейсные функции к stmdrv
 *  
 *  20.06.2020
 *  LP 04.04.2021
 * 
 */

#ifndef _STMI_H_

#define _STMI_H_

#include <Wire.h> 

#define ADDR_STM_SERVER  0x85

extern void stmMotorsInit(void);
extern void stmMotorLeftGo(int speed);
extern void stmMotorRightGo(int speed);

#endif
