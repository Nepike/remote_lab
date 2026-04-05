/**
 * COM port library (Linux)
 * \author Robofob
 * \version 1.03
 * \date 28.02.2014
 * \date LP 23.08.2014
*/

#ifdef _LINUX_
#include <termios.h>
#endif

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "comport.h"

#ifdef _LINUX_

int Gfda = -1;

int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    return -1; // error("error %d from tcgetattr", errno);

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // ignore break signal
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // 0: read doesn't block
  tty.c_cc[VTIME] = 5;            // 2 5 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    return -2; // error ("error %d from tcsetattr", errno);
  return 0;
}

int set_blocking(int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    return -3; //error ("error %d from tggetattr", errno);

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;      // 2 5 (0.5 seconds read timeout)

  if(tcsetattr (fd, TCSANOW, &tty) != 0)
    return -4; // error("error %d setting term attributes", errno);
  return 0;
}

int Comport::OpenPort(char *comportname, int BaudRateNum)
// BaudRateNum: 0: 115200, 1: 9600, 2: 19200
{
  int fd;
  int BDR;
  switch(BaudRateNum)
  {
    case 0: BDR = B115200; break;
    case 1: BDR = B9600;   break;
    case 2: BDR = B19200;   break;
    default: return -5;
  }

  // Открываем порт
  fd = open(comportname, O_RDWR | O_NOCTTY | O_SYNC);
  if(fd < 0)
    return -6;

  if(set_interface_attribs(fd, BDR, 0)!=0)  // set speed to BDR bps, 8n1 (no parity)
    return -7;
  if(set_blocking(fd, 0))                   // 0 set no blocking
    return -8;

  Gfda = fd;

  return fd;
}

int Comport::OpenPort(char *comportname, int BaudRateNum, int ntr)
// BaudRateNum: 0: 115200, 1: 9600, 2: 19200
// ntr - iterations
{
  int n = 0, fd;  
  do
  {
    fd = OpenPort(comportname, BaudRateNum);
    if(fd<0)
    {
      printf("\nOpenPort [%s] error %d (%d): %s", comportname,  fd, errno, strerror(errno));
    }
    n++;
  } while(fd<0 && n<=ntr);
  return fd;
}

#else

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

HANDLE GhSerial = NULL;

HANDLE Comport::OpenPort(char *comportname, int BaudRateNum)
// BaudRateNum: 0: 115200, 1: 9600, 2: 19200
{
  HANDLE hSerial;
  int BDR = CBR_9600;
  switch(BaudRateNum)
  {
    case 0: BDR = CBR_115200; break;
    case 1: BDR = CBR_9600;   break;
    case 2: BDR = CBR_19200;  break;
  }

  hSerial = CreateFile(comportname, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  // Do some basic settings
  DCB serialParams = { 0 };
  serialParams.DCBlength = sizeof(serialParams);

  GetCommState(hSerial, &serialParams);
  serialParams.BaudRate = BDR;
  serialParams.ByteSize = 8;
  serialParams.StopBits = ONESTOPBIT;
  serialParams.Parity = NOPARITY;
  SetCommState(hSerial, &serialParams);

  // Set timeouts
  COMMTIMEOUTS timeout = { 0 };
  timeout.ReadIntervalTimeout = 550;
  timeout.ReadTotalTimeoutConstant = 550;
  timeout.ReadTotalTimeoutMultiplier = 550;
  timeout.WriteTotalTimeoutConstant = 550;
  timeout.WriteTotalTimeoutMultiplier = 510;

  SetCommTimeouts(hSerial, &timeout);

  if (hSerial == INVALID_HANDLE_VALUE) 
    // "Error opening port"
    hSerial = NULL;

  GhSerial = hSerial;
  return hSerial;
}

#endif

int Comport::ReadByte(unsigned char *c)
{
#ifdef _LINUX_
  int n = read(Gfda, c, 1);
#else
  DWORD n;
  ReadFile(GhSerial, c, 1, &n, NULL);
#endif
  return (n==1);
}

int Comport::WriteByte(unsigned char c)
{
#ifdef _LINUX_
  int n = write(Gfda, &c, 1);
#else
  DWORD n;
  WriteFile(GhSerial, &c, 1, &n, NULL);
#endif
  return (n==1);
}

void Comport::Close(void)
{
#ifdef _LINUX_
  close(Gfda);
#else
  CloseHandle(GhSerial);
#endif
}
