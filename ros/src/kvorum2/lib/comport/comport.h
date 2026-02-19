/**
 * COM port library (Linux)
 * \author Robofob
 * \version 1.03
 * \date 28.02.2014
 * \date LP 23.08.2014
*/

#ifndef _COMPORTLIB_H_
#define _COMPORTLIB_H_

#define _LINUX_

#ifdef _LINUX_

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#else
#include <windows.h>
#endif

namespace Comport
{

#ifdef _LINUX_
/// BaudRateNum: 0: 115200, 1: 9600, 2: 19200
int OpenPort(char *comportname, int BaudRateNum);

/// BaudRateNum: 0: 115200, 1: 9600, 2: 19200
/// ntr - iterations
int OpenPort(char *comportname, int BaudRateNum, int ntr);

#else

/// BaudRateNum: 0: 115200, 1: 9600, 2: 19200
HANDLE OpenPort(char *comportname, int BaudRateNum);

#endif

int ReadByte(unsigned char *c);
int WriteByte(unsigned char c);
void Close(void);

}

#endif
