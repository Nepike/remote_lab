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
#ifndef _EXEC_WRAPPER_H_
#define _EXEC_WRAPPER_H_

#include <string>

/*! \brief Execution unit for a single command.
 *  Encapsulates necessary data and provides an interface. Execution is non-blocking.
 *
 *  ExecutionNode is a small class that almost wraps fork / execve to execute a shell 
 *  command via C / C++ code in a non-blocking way. It also keeps some information
 *  about the command, its execution status, time of start, etc. It provides some 
 *  small set of methods to start the command, cancel it and get some additional
 *  information about its execution. It mostly invokes different standard Linux 
 *  functions at the right time with the right arguments, with an attempt to 
 *  comply with the POSIX standard.
 */
class ExecutionNode
{
  public:
    /*! A constructor.
     *
     * \param command_string is a string that contains the shell command required to 
     * run in a separate process.
     */
    ExecutionNode(std::string command_string);
    ExecutionNode(void);

    void setCommand(std::string command_string);
    /*! A destructor.
     */
    ~ExecutionNode();

    /*! \brief Start task execution.
     *
     * Start execution of the command in the command_string_ in a separate
     * process (via fork / exec). A non-blocking operation.
     *
     * \return 0 if successful, less than 0 if an error occured
     */
    int startExecution(bool debug = true);

    /*! \brief Cancel ongoing execution.
     *
     * Cancel execution if it has started and hasn't finished yet. If it
     * has finished, than this method marks the task as finished and 
     * marks the time of it.
     *
     * \return 0 if successful, less than 0 if an error occured.
     */
    int cancelExecution();

    /*! Check if the task was started.
     *
     * \return True if the task was started successfully, false otherwise.
     */
    bool hasStarted();

    /*! Check if the task has finished.
     *
     * \return True if the task was finished or cancelled.
     */
    bool hasFinished();

    /*! \brief Get time the task ran.
     *
     * Get the time the process has been running if it is still running.
     * If it isn't get the time it was running. Fails if it wasn't started at all.
     *
     * \param execution_time is a pointer to a timespec structure that the method
     * will write the result to.
     * \return 0 if successful, less than 0 if an error occured.
     */
    int getTime(timespec *execution_time);

  private:
    /*! \brief True if the task was started.
     */
    bool is_started_;

    /*! \brief True if the task was finished
     * 
     * This boolean variable is true if the task was finished normally or canceled.
     */
    bool is_finished_;

    /*! \brief Command to execute.
     *
     * This string contains the command to execute in the task. Any shell command is 
     * supported.
     */
    std::string command_string_;

    /*! \brief Time of start.
     *
     * Time struct (see man clock_gettime) that contains the time of the start of 
     * execution of the task. It contains members of seconds and nanoseconds
     * precision.
     */
    timespec start_time_;

    /*! \brief Time of finish.
     *
     * Time struct (see man clock_gettime) that contains the time when the execution
     * of the task stopped. It contains members of seconds and nanoseconds
     * precision.
     */
    timespec stop_time_;

    /*! \brief PID of the child.
     *
     * PID (process id) of the child process that executes a shell (/bin/sh) which in 
     * turn executes the given command. Its group id is also set to its PID so that
     * it can be killed on demand with all the processes it may have spawned.
     */
    pid_t child_pid_;
};

#endif
