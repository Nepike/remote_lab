/**
 * RosCamera Library
 * Сервисная библиотека
 * \version 1.01
 * \date 24.07.2014
 * LP 24.07.2014
*/

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <curses.h>
#include <ros/ros.h>

#include "erlib.h"

#include <msg_ans/ans.h>
#include <msg_rsaction/action.h>
#include <msg_pllcmd/pllcmd.h>

#include "servlib.h"
#include "roscamlib.h"
