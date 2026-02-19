/*
 * This file is part of exec_wrapper project. It is subject to the license terms in 
 * the LICENSE file found in the top-level directory of this distribution. No part of 
 * exec_wrapper project, including this file, may be copied, modified, propagated, or 
 * distributed except according to the terms contained in the LICENSE file.
 */

#include "exec_wrapper.h"

#include <unistd.h>
#include <iostream>

#define COMMAND_STRING "bin/test_prog arg1"

int main(int argc, char *argv[])
{
  int ret = 0;
  ExecutionNode en(COMMAND_STRING);

  ret = en.startExecution();
  if (ret < 0)
  {
    std::cerr << "startExecution failed" << std::endl;
    return 1;
  }

  long t = 0;
  while(!en.hasStarted())
  {
    std::cout << "starting node" << std::endl;
    usleep(500000);
  }

  while((!en.hasFinished()) && (t < 2))
  {
    std::cout << "job processing" << std::endl;
    usleep(1000000);
    t += 1;
  }

  if (!en.hasFinished())
  {
    std::cout << "trying to cancel execution" << std::endl;
    en.cancelExecution();
  }

  timespec exec_time;
  ret = en.getTime(&exec_time);
  if (ret < 0)
  {
    std::cerr << "getTime failed" << std::endl;
    return 1;
  }

  std::cout << "finished > time " << exec_time.tv_sec << " s "
    << exec_time.tv_nsec << " ns" << std::endl;

  return 0;
}
