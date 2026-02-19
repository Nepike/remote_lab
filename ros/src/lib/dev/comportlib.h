/**
 * COM port library (Linux)
 * \author Robofob
 * \version 1.04
 * \date 28.02.2014
 * \date LP 19.08.2015
*/

#ifndef _COMPORTLIB_H_
#define _COMPORTLIB_H_

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

/// BaudRateNum: 0: 115200, 1: 9600, 2: 19200, 3: 57600
int OpenPort(char *comportname, int BaudRateNum);

/// BaudRateNum: 0: 115200, 1: 9600, 2: 19200, 3: 57600
/// ntr - iterations
int OpenPort(char *comportname, int BaudRateNum, int ntr);

#endif
