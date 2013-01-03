/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * Scheduler.h --- AP_HAL_SMACCM scheduler.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 20 December 2012
 */

#ifndef __AP_HAL_SMACCM_SCHEDULER_H__
#define __AP_HAL_SMACCM_SCHEDULER_H__

#include <AP_HAL_SMACCM.h>

#define SMACCM_SCHEDULER_MAX_TIMER_PROCS 4

class SMACCM::SMACCMScheduler : public AP_HAL::Scheduler
{
public:
  SMACCMScheduler();

  /**
   * Initialize the scheduler.  This initializes the HWF4 timer
   * driver.
   *
   * @param arg reserved, pass NULL
   */
  void init(void *arg);

  /** Delay for "ms" milliseconds. */
  void delay(uint16_t ms);

  /** Delay for "us" microseconds. */
  void delay_microseconds(uint16_t us);

  /** Return the time since init in milliseconds. */
  uint32_t millis();

  /** Return the time since init in microseconds. */
  uint32_t micros();

  /**
   * Register a callback to run every millisecond during a call to
   * "delay" as long as the delay remaining is at least "min_time_ms".
   *
   * The callback is executed in the thread that calls "delay".
   */
  void register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

  /** Register a callback to run every 1ms (1kHz). */
  void register_timer_process(AP_HAL::TimedProc);

  /**
   * Register a callback to run if a timer process takes too long to
   * execute.  The second argument is ignored.
   */
  void register_timer_failsafe(AP_HAL::TimedProc, uint32_t);

  /**
   * Suspend execution of timed procedures.  Calls to this function do
   * not nest.
   */
  void suspend_timer_procs();

  /**
   * Resume execution of timed procedures.  Calls to this function do
   * not nest.
   */
  void resume_timer_procs();

  /**
   * Enter a critical section where timed processes will not be run.
   * Calls to this function nest.
   */
  void begin_atomic();

  /**
   * Leave a critical section, re-enabling timed processes.  Calls to
   * this function nest.
   */
  void end_atomic();

  /** Print an error message and halt execution. */
  void panic(const prog_char_t *errormsg);

  /** Reboot the firmware.  Not implemented. */
  void reboot();

  /**
   * Run timed and deferred processes.  This should not be called from
   * client code.
   */
  void run_callbacks();

  /**
   * Run the failsafe callback.  This should not be called from client
   * code.
   */
  void run_failsafe_cb();

  /**
   * Run the delay callback.  This should not be called from client
   * code.
   */
  void run_delay_cb();

private:
  AP_HAL::Proc m_delay_cb;      /* delay callback */
  void *m_task;                 /* opaque scheduler task handle */
  void *m_delay_cb_task;        /* opaque delay cb task handle */
  AP_HAL::TimedProc m_procs[SMACCM_SCHEDULER_MAX_TIMER_PROCS];
  AP_HAL::TimedProc m_failsafe_cb;
  uint8_t m_num_procs;          /* number of entries in "m_procs" */
};

#endif // __AP_HAL_SMACCM_SCHEDULER_H__
