/**
 * Pololu Maestro Library
 * \version 1.02
 * Author: Malyshev A.
 * \date: 26 August 2013
 * Date of last modification: 03.06.2014
*/

#include "maestrolib.h"


/// Check condition a_min <= a <= a_max in main cycle
int a2t(int angle)
{
  int t;
  t = (angle - ANG_MIN) * (T_MAX - T_MIN) / (ANG_MAX - ANG_MIN) + T_MIN;
  return t;
}

/// Gets the position of a Maestro channel.
/// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }

  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
  return response[0] + 256*response[1];
}


/// Sets the target of a Maestro channel.
/// See the "Serial Servo Commands" section of the user's guide.
/// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}

int maestroSetSpeed(int fd, unsigned char channel, unsigned short speed)
{
  unsigned char command[] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F};

  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}

int maestroSetAcceleration(int fd, unsigned char channel, unsigned short acceleration)
{
  unsigned char command[] = {0x89, channel, acceleration & 0x7F, acceleration >> 7 & 0x7F};

  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}

int maestroSetPWM(int fd, unsigned char time, unsigned char period)
{
  unsigned char command[] = {0x8A, time & 0x7F, time >> 7 & 0x7F, period & 0x7F, period >> 7 & 0x7F};

  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}


int maestroGetMovingState(int fd)
{
  unsigned char command[] = {0x93};

  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }

  unsigned char response[1];

  if(read(fd,response,1) != 1)
  {
    perror("error reading");
    return -1;
  }

  return response[0];
}

int maestroGetErrors(int fd) 
// response
{
  unsigned char command[] = {0xA1};

  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }

  unsigned char response[2];

  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
  return response[0] + 256*response[1];
}

int maestroGoHome(int fd)
{
  unsigned char command[] = {0xA2};

  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }

  return 0;
}
