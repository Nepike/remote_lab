/**
 * Pololu Maestro Library
 * \version 1.02
 * Author: Malyshev A.
 * \date: 26 August 2013
 * Date of last modification: 03.06.2014
*/

#ifndef MESTRO_LIB_H
#define MESTRO_LIB_H

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#define T_MIN 4000
#define T_MAX 8000
#define ANG_MIN -60
#define ANG_MAX 60

int a2t(int angle); /// Check condition a_min <= a <= a_max in main cycle

/// Gets the position of a Maestro channel.
/// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel);

/// Sets the target of a Maestro channel.
/// See the "Serial Servo Commands" section of the user's guide.
/// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target);

int maestroSetSpeed(int fd, unsigned char channel, unsigned short speed);

int maestroSetAcceleration(int fd, unsigned char channel, unsigned short acceleration);

int maestroSetPWM(int fd, unsigned char time, unsigned char period);

int maestroGetMovingState(int fd);

int maestroGetErrors(int fd); /// response

int maestroGoHome(int fd);

#endif // MESTRO_LIB_H
