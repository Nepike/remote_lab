/**
 * exec_wrapper library
 * \author Maxim Rovbo
 * \version 1.02
 * \date 28.09.2014
 * \date LP 15.10.2014
*/

/*
 * This file is part of exec_wrapper project. It is subject to the license terms in 
 * the LICENSE file found in the top-level directory of this distribution. No part of 
 * exec_wrapper project, including this file, may be copied, modified, propagated, or 
 * distributed except according to the terms contained in the LICENSE file.
 */

#include "exec_wrapper.h"

#include <stdlib.h>
#include <exception>
#include <stdexcept>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <string>

#include <stdio.h> // for __environ


ExecutionNode::ExecutionNode(std::string command_string) :
  is_started_(false), is_finished_(false),
  command_string_(command_string),
  child_pid_(0)
{
}

ExecutionNode::ExecutionNode(void) :
  is_started_(false), is_finished_(false),
  command_string_(""),
  child_pid_(0)
{
}

void ExecutionNode::setCommand(std::string command_string)
{
  is_started_ = false;
  is_finished_= false;
  command_string_ = command_string;
  child_pid_ = 0;
}

ExecutionNode::~ExecutionNode()
{
  cancelExecution();
}

bool ExecutionNode::hasStarted()
{
  if (is_started_)
    return true;
  else
    return false;
}

bool ExecutionNode::hasFinished()
{
  int ret;
  int return_status;

  if (child_pid_ == 0)
  {
    std::cerr << "hasFinished: no child to watch (child_pid_ == 0)" << std::endl;
  }

  if (is_finished_)
    return true;
  else
  {
    // maybe it has finished but we didn't notice
    ret = waitpid(child_pid_, &return_status, WNOHANG);
    if (ret == 0)
    {
      return false;
    }
    else if (ret < 0)
    {
      // an error occured
      perror("hasFinshed parent");
      throw std::logic_error("hasFinished() failed");
    }
    else
    {
      // a child finished
      // std::cout << "Child pid finished > pid = " <<  ret << std::endl;
      // result status was already set, let's add time
      is_finished_ = true; 
      ret = clock_gettime(CLOCK_MONOTONIC, &stop_time_);
      if (ret < 0)
      {
        perror("hasFinished");
        return -1;
      }
      return true;
    }
  }
}

int ExecutionNode::getTime(timespec *execution_time)
{
  int ret;

  if (!is_started_)
  {
    std::cerr << "Process didn't start yet" << std::endl;
    return -1; 
  }

  if (is_finished_)
  {
    timespec buf = stop_time_;
    buf.tv_nsec -= start_time_.tv_nsec;
    buf.tv_sec -= start_time_.tv_sec;
    *execution_time = buf;
    return 0;
  }

  timespec now;

  // can use CLOCK_MONOTONIC_RAW for even more reliable results
  // but it's non-POSIX.
  //
  // NOTE: if clock_gettime isn't implemented, gettimeofday 
  // should be used instead. I don't know why it wouldn't
  // be implemented, though, and too lazy to write the extra code.
  ret = clock_gettime(CLOCK_MONOTONIC, &now);

  if (ret < 0)
  {
    perror("getTime");
    return -1;
  }
  timespec buf = now;
  buf.tv_nsec -= start_time_.tv_nsec;
  buf.tv_sec -= start_time_.tv_sec;
  *execution_time = buf;
  return 0;
}

int ExecutionNode::cancelExecution()
{
  int ret;
  if (child_pid_ == 0)
    return -1;

  // a child has not been waited upon yet and may be still
  // working. Need to clean that up.
  kill(-child_pid_, 9); // kill signal. can't be caught, blocked or ignored
  // NOTE: kill may fail (presumably, because the child has already 
  // terminated on its own and thus, successfully completed execution.
  // This version just ignores that and reports a preemptive termination.

  child_pid_ = waitpid(child_pid_, &ret, 0);

  //TODO: maybe analyze ret
  is_finished_ = true;

  ret = clock_gettime(CLOCK_MONOTONIC, &stop_time_);
  if (ret < 0)
  {
    perror("cancelExecution");
    return -1;
  }

  return 0;
}

int ExecutionNode::startExecution(bool debug)
{
  int ret;
  is_started_ = false;
  is_finished_ = false;

  ret = clock_gettime(CLOCK_MONOTONIC, &start_time_);
  if (ret < 0)
  {
    perror("startExecution");
    return -1;
  }

  pid_t child_pid = 0;

  child_pid = fork(); 

  if(child_pid < 0)
  {
    perror("startExecution");
    return -1;
  }
  else if(child_pid == 0)
  {
    // child
    if(debug)
      std::cout << "Fork successful > child pid = " << getpid() << std::endl;

    // set child's pgid to its pid so that all processes it spawns
    // will have this pgid. Usefull for killing them together.
    // ret = setpgid(getpid(), getgid());
    ret = setpgid(0, 0);
    if (ret < 0)
    {
      perror("startExecution child");
      exit(1);
    }

    char *exec_argv[4];
    char *command_string = new char[command_string_.length() + 1];
    strncpy(command_string,
        command_string_.c_str(),
        command_string_.length()
        );
    command_string[command_string_.length()] = '\0';

    const char *SHELL_PATH = "/bin/sh";
    exec_argv[0] = new char[strlen(SHELL_PATH) + 1];
    strncpy(exec_argv[0], SHELL_PATH, strlen(SHELL_PATH) + 1);
    exec_argv[1] = new char[3];
    strncpy(exec_argv[1], "-c", strlen("-c") + 1);
    exec_argv[2] = command_string;
    exec_argv[3] = (char *)NULL;
    if(debug)
      std::cout << "Trying to execve ' " <<
        std::string(exec_argv[0]) << " " <<
        std::string(exec_argv[1]) << " " <<
        std::string(exec_argv[2]) << "'" << std::endl;
    ret = execve(exec_argv[0], exec_argv, __environ);
    if (ret < 0)
    {
      // memory cleanup
      delete [] exec_argv[0];
      delete [] exec_argv[1];
      delete [] command_string;
      perror("startExecution child");
      exit(1);
    }
  }

  // parent
  if(debug)
    std::cout << "Child pid = " << child_pid << ", my pid = " << getpid() << std::endl;
  child_pid_ = child_pid;
  is_started_ = true;

  return 0;
}
