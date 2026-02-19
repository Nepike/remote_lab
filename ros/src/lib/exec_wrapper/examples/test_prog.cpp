/*
 * This file is part of exec_wrapper project. It is subject to the license terms in 
 * the LICENSE file found in the top-level directory of this distribution. No part of 
 * exec_wrapper project, including this file, may be copied, modified, propagated, or 
 * distributed except according to the terms contained in the LICENSE file.
 */

#include <iostream>
#include <string>
#include <unistd.h>

int 
main(int argc, char* argv[])
{
  for(int i = 0; i < 5; i++)
  {
    std::cout << "job processing";
    if (i < argc)
    {
      std::cout << " " << std::string( argv[i]);
    }
    std::cout << std::endl;
    sleep(1);
  }

  return 0;
}
