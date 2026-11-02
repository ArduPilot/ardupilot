/*
    Aerospace Decoder - Copyright (C) 2018..2022 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    sysperf.c
 * @brief   performance measurement.
 *
 * @addtogroup monitor
 * @{
 */

#include "sysperf.h"

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

#if HAL_USE_LOAD_MEASURE == TRUE

/* Control object.*/
static sys_load_data_t  _load = {
  .state = SYS_MEASURE_STOP
};

/**
 * @brief Per-reader load statistics window
 */
typedef struct {
  uint64_t        run_snapshot;     /**< @brief _total_run at window start. */
  uint64_t        idle_snapshot;    /**< @brief _total_idle at window start.*/
  sys_cpu_load_t  peak;             /**< @brief peak load in this window.   */
} sys_load_window_t;

/* Per-reader windows, updated from the idle hooks, read and restarted by
   each reader independently via sysReadResetCPULoad().*/
static sys_load_window_t _windows[SYS_LOAD_READER_COUNT];

/* Monotonic totals of measured run and idle cycles, updated from the idle
   hooks. 64-bit so they never wrap in practice.*/
static uint64_t _total_run;
static uint64_t _total_idle;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/*===========================================================================*/
/* Module external functions.                                                */
/*===========================================================================*/

/**
 * @brief Initialise CPU load measuring
 *
 * @api
 */
void sysInitLoadMeasure(void) {

  _load.state = SYS_MEASURE_STOP;
}

/**
 * @brief Start CPU load measuring
 *
 * @return True on success else false
 *
 * @api
 */
bool sysStartLoadMeasure(void) {

  if (_load.state != SYS_MEASURE_STOP) {
    return false;
  }

  _load.stop = false;
  _load.state = SYS_MEASURE_INIT;
  return true;
}

/**
 * @brief Request stop of CPU load measuring
 *
 * @return True on success else false
 *
 * @api
 */
bool sysStopLoadMeasure(void) {

  if (_load.state == SYS_MEASURE_STOP || _load.stop) {
    return false;
  }

  _load.stop = true;
  return true;
}

/**
 * @brief Get the peak CPU load measured.
 *
 * @return Peak CPU load  reached as percentage * 100
 *
 * @api
 */
sys_cpu_load_t sysGetCPUPeakLoad(void) {

  return _load.peak;
}

/**
 * @brief Clear the peak CPU load so it restarts from the current load.
 * @note  Called from thread context. No locking is needed: the idle hooks
 *        that update the peak only run when no other thread is runnable.
 *
 * @api
 */
void sysClearCPUPeakLoad(void) {

  _load.peak = (sys_cpu_load_t)0;
}

/**
 * @brief Read a reader's windowed average/peak load and restart its window.
 * @note  Each reader observes the load accumulated since its own previous
 *        call, without disturbing the other readers or the moving average.
 * @note  Called from thread context. No locking is needed: the idle hooks
 *        that update the shared totals only run when no other thread is
 *        runnable, so they can never interleave with this function.
 *
 * @param[in]  reader  the window to read and restart
 * @param[out] avg     average load over the window (percentage * 100), or NULL
 * @param[out] peak    peak load over the window (percentage * 100), or NULL
 *
 * @return True if the window contained measured cycles. False means no
 *         data: measurement is stopped or not yet started, or the CPU had
 *         no idle time at all during the window (the idle hooks never ran).
 *
 * @api
 */
bool sysReadResetCPULoad(sys_load_reader_t reader,
                         sys_cpu_load_t *avg, sys_cpu_load_t *peak) {

  sys_load_window_t *w = &_windows[reader];
  const uint64_t run  = _total_run - w->run_snapshot;
  const uint64_t idle = _total_idle - w->idle_snapshot;

  if (avg != NULL) {
    *avg = (run + idle > 0) ?
           (sys_cpu_load_t)((run * SYS_CPU_MAX_LOAD) / (run + idle)) : 0;
  }
  if (peak != NULL) {
    *peak = w->peak;
  }
  w->run_snapshot = _total_run;
  w->idle_snapshot = _total_idle;
  w->peak = 0;
  return (run + idle) > 0;
}

/**
 * @brief Get the moving average of CPU load.
 *
 * @return CPU average load as percentage * 100
 *
 * @api
 */
sys_cpu_load_t sysGetCPUAverageLoad(void) {

  return _load.average;
}

/**
 * @brief Get the CPU load statistics
 *
 * @return Load statistics
 * @retval MSG_OK on statistics set
 *         MSG_TIMEOUT if measurement not enabled
 *
 * @api
 */
msg_t sysGetCPULoadStatistics(sys_load_stats_t *stats) {

  chDbgCheck(stats != NULL);

  if (_load.state != SYS_MEASURE_ACTIVE) {
    return MSG_TIMEOUT;
  }
  chSysLock();
  stats->last     = _load.run.last * SYS_CPU_MAX_LOAD / (_load.run.last +
                                                         _load.idle.last);
  stats->current  = _load.average;
  stats->peak     = _load.peak;
  stats->idle_max = _load.idle.worst / (SystemCoreClock / 1000);
  stats->run_max  = _load.run.worst / (SystemCoreClock / 1000);
  chSysUnlock();
  return MSG_OK;
}

/**
 * @brief Called from the RTOS idle enter hook
 * @note  The idle measurement management is not included in the counts.
 *        If active the run timing will be captured (stopped) here.
 *        The kernel is locked when this function is called.
 *        The stack is that of the thread about to be switched out.
 *
 * @special
 */
void sysIdleEnterMeasure(void) {
  switch (_load.state) {
    case SYS_MEASURE_STOP:

      /* Measurement is not active.*/
      return;

    case SYS_MEASURE_INIT:

      /* Measurement start requested.*/
      chTMObjectInit(&_load.idle);
      chTMObjectInit(&_load.run);
      _load.peak = (sys_cpu_load_t)0;
      _load.average = (sys_cpu_load_t)0;
      chTMStartMeasurementX(&_load.idle);
      _load.state = SYS_MEASURE_ACTIVE;

      /* Return to scheduler for switch to idle context.*/
      return;

    case SYS_MEASURE_ACTIVE: {

      /* Stop the run measurement.*/
      chTMStopMeasurementX(&_load.run);
      _total_run += _load.run.last;

      /* Calculate current load from idle to run.*/
      rtcnt_t idle = _load.idle.cumulative / _load.idle.n;
      rtcnt_t run  = _load.run.cumulative / _load.run.n;
      sys_cpu_load_t current = 0;
      if (idle + run > (rtcnt_t)0) {
        current = (run * SYS_CPU_MAX_LOAD) / (idle + run);
      }

      /* Update the average and peak.*/
      _load.average = current;
      if (current > _load.peak) {
        _load.peak = current;
      }

      /* Track the peak for each reader's window.*/
      for (int i = 0; i < SYS_LOAD_READER_COUNT; i++) {
        if (current > _windows[i].peak) {
          _windows[i].peak = current;
        }
      }

      /* Scale TM accumulator every second.*/
      if (_load.run.cumulative > SystemCoreClock) {
        _load.run.n /= 2;
        _load.run.cumulative /= 2;
      }

      if (_load.stop) {

        /* Run + idle cycle is calculated. Stop will have valid results.*/
        _load.state = SYS_MEASURE_STOP;
        _load.stop = false;
        return;
      }

      /* Start measurement of idle time.*/
      chTMStartMeasurementX(&_load.idle);
      return;
    } /* End case SYS_MEASURE_ACTIVE */

    default:
      chDbgAssert(false, "invalid state entering idle");
  } /* end switch on state.*/
}

/**
 * @brief Called from the RTOS idle leave hook
 * @note  The measurement handling code is excluded from measurement
 *        If active the run timing will be started here.
 *        The kernel is locked when this function is called.
 *
 * @special
 */
void sysIdleLeaveMeasure(void) {
  switch (_load.state) {
    case SYS_MEASURE_STOP:
      return;

    case SYS_MEASURE_INIT:

      /* Init is handled in idle thread enter.*/
      return;

    case SYS_MEASURE_ACTIVE: {

      /* RTOS has exited idle so capture time now.*/
      chTMStopMeasurementX(&_load.idle);
      _total_idle += _load.idle.last;

      /* Scale TM accumulator every second.*/
      if (_load.idle.cumulative > SystemCoreClock) {
        _load.idle.n /=  2;
        _load.idle.cumulative /= 2;
      }

      /* Start measurement of run time.*/
      chTMStartMeasurementX(&_load.run);
      return;
    }
  }
}
#else
  void sysIdleEnterMeasure(void) { };
  void sysIdleLeaveMeasure(void) { };
#endif /* HAL_USE_LOAD_MEASURE == TRUE*/

/** @} */
