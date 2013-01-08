/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * Scheduler.cpp --- AP_HAL_SMACCM scheduler.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 20 December 2012
 */

#include <hwf4/gpio.h>
#include <hwf4/timer.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "Scheduler.h"

using namespace SMACCM;

extern const AP_HAL::HAL& hal;

/** Rate in milliseconds of timed process execution. (1kHz) */
#define SCHEDULER_TICKS  (1 / (portTICK_RATE_MS))

/** Stack size of the scheduler thread. */
#define SCHEDULER_STACK_SIZE 1536

/** Priority of the scheduler timer process task. */
#define SCHEDULER_PRIORITY (configMAX_PRIORITIES - 1)

/** Rate in milliseconds of the delay callback task. */
#define DELAY_CB_TICKS (1 / (portTICK_RATE_MS))

/** Stack size of the delay callback task. */
#define DELAY_CB_STACK_SIZE 512

/** Priority of the delay callback task. */
#define DELAY_CB_PRIORITY 0

/**
 * Recursive mutex used to block "scheduler_task" during atomic
 * sections.
 */
static xSemaphoreHandle g_atomic;

/** High-priority thread managing timer procedures. */
static void scheduler_task(void *arg)
{
  SMACCMScheduler *sched = (SMACCMScheduler *)arg;
  portTickType last_wake_time;
  portTickType now;

  vTaskSetApplicationTaskTag(NULL, (pdTASK_HOOK_CODE)3);
  last_wake_time = xTaskGetTickCount();

  for (;;) {
    /* If "vTaskDelayUntil" would return immediately without blocking,
     * call the failsafe callback to notify the client that we've
     * missed our deadline, and reset the wakeup time to the current
     * time. */
    now = xTaskGetTickCount();
    if (last_wake_time + SCHEDULER_TICKS <= now) {
      sched->run_failsafe_cb();
      last_wake_time = now;
    } else {
      vTaskDelayUntil(&last_wake_time, SCHEDULER_TICKS);

      xSemaphoreTakeRecursive(g_atomic, portMAX_DELAY);
      sched->run_callbacks();
      xSemaphoreGiveRecursive(g_atomic);
    }
  }
}

/** Count of the number of threads in "delay". */
static uint32_t g_delay_count;

/** Binary semaphore given when a thread enters "delay". */
static xSemaphoreHandle g_delay_event;

/**
 * Low-priority thread that calls the delay callback every 1ms.
 *
 * Whenever any thread enters a call to "delay", it unblocks this
 * task.
 *
 * We use a count of the number of threads currently in "delay"
 * because there could be more than one.
 */
static void delay_cb_task(void *arg)
{
  SMACCMScheduler *sched = (SMACCMScheduler *)arg;
  portTickType last_wake_time;
  portTickType now;

  vTaskSetApplicationTaskTag(NULL, (pdTASK_HOOK_CODE)4);
  last_wake_time = xTaskGetTickCount();

  for (;;) {
    portENTER_CRITICAL();
    uint32_t delay_count = g_delay_count;
    portEXIT_CRITICAL();

    if (delay_count > 0) {
      /* If some thread is in "delay", call the delay callback. */
      sched->run_delay_cb();

      /* If "vTaskDelayUntil" would return immediately without
       * blocking, that means we've missed our deadline.  However,
       * this thread is best-effort, so we'll just readjust our
       * schedule accordingly and run 1ms from now.
       *
       * Without this, the thread runs many times to "catch up" if
       * something takes too long in a higher priority thread. */
      now = xTaskGetTickCount();
      if (last_wake_time + DELAY_CB_TICKS <= now) {
        last_wake_time = now;
      } else {
        vTaskDelayUntil(&last_wake_time, DELAY_CB_TICKS);
      }
    } else {
      /* Wait for a thread to enter a delay. */
      xSemaphoreTake(g_delay_event, portMAX_DELAY);
      last_wake_time = xTaskGetTickCount();
    }
  }
}

SMACCMScheduler::SMACCMScheduler()
  : m_delay_cb(NULL), m_task(NULL), m_delay_cb_task(NULL),
    m_failsafe_cb(NULL), m_num_procs(0)
{
}

void SMACCMScheduler::init(void *arg)
{
  timer_init();

  g_atomic = xSemaphoreCreateRecursiveMutex();

  vSemaphoreCreateBinary(g_delay_event);
  xSemaphoreTake(g_delay_event, portMAX_DELAY);

  xTaskCreate(scheduler_task, (signed char *)"scheduler",
              SCHEDULER_STACK_SIZE, this, SCHEDULER_PRIORITY,
              &m_task);

  xTaskCreate(delay_cb_task, (signed char *)"delay_cb",
              DELAY_CB_STACK_SIZE, this, DELAY_CB_PRIORITY,
              &m_delay_cb_task);
}

void SMACCMScheduler::delay(uint16_t ms)
{
  /* Wake up the delay callback thread. */
  portENTER_CRITICAL();
  ++g_delay_count;
  portEXIT_CRITICAL();
  xSemaphoreGive(g_delay_event);

  timer_msleep(ms);

  /* Put the delay callback thread back to sleep. */
  portENTER_CRITICAL();
  --g_delay_count;
  portEXIT_CRITICAL();
}

uint32_t SMACCMScheduler::millis()
{
  return (uint32_t)(timer_get_ticks() / 1000ULL);
}

// XXX this is going to wrap every 1.1 hours
uint32_t SMACCMScheduler::micros()
{
  return (uint32_t)timer_get_ticks();
}

void SMACCMScheduler::delay_microseconds(uint16_t us)
{
  timer_usleep(us);
}

void SMACCMScheduler::register_delay_callback(AP_HAL::Proc k, uint16_t)
{
  m_delay_cb = k;
}

void SMACCMScheduler::register_timer_process(AP_HAL::TimedProc k)
{
  for (int i = 0; i < m_num_procs; ++i) {
    if (m_procs[i] == k)
      return;
  }

  if (m_num_procs < SMACCM_SCHEDULER_MAX_TIMER_PROCS) {
    portENTER_CRITICAL();
    m_procs[m_num_procs] = k;
    ++m_num_procs;
    portEXIT_CRITICAL();
  }
}

void SMACCMScheduler::register_timer_failsafe(AP_HAL::TimedProc k, uint32_t)
{
  m_failsafe_cb = k;
}

void SMACCMScheduler::suspend_timer_procs()
{
  xSemaphoreTakeRecursive(g_atomic, portMAX_DELAY);
}

void SMACCMScheduler::resume_timer_procs()
{
  xSemaphoreGiveRecursive(g_atomic);
}

void SMACCMScheduler::begin_atomic()
{
}

void SMACCMScheduler::end_atomic()
{
}

void SMACCMScheduler::panic(const prog_char_t *errormsg)
{
  hal.console->println_P(errormsg);

  // Try to grab "g_atomic" to suspend timer processes, but with a
  // timeout in case a timer proc is locked up.
  xSemaphoreTakeRecursive(g_atomic, 10);

  for(;;)
    ;
}

void SMACCMScheduler::reboot()
{
  for(;;)
    ;
}

void SMACCMScheduler::run_callbacks()
{
  uint32_t now = micros();

  // Run timer processes if not suspended.
  portENTER_CRITICAL();
  uint8_t num_procs = m_num_procs;
  portEXIT_CRITICAL();

  for (int i = 0; i < num_procs; ++i) {
    if (m_procs[i] != NULL) {
      m_procs[i](now);
    }
  }
}

void SMACCMScheduler::run_failsafe_cb()
{
  if (m_failsafe_cb)
    m_failsafe_cb(micros());
}

void SMACCMScheduler::run_delay_cb()
{
  if (m_delay_cb)
    m_delay_cb();
}
