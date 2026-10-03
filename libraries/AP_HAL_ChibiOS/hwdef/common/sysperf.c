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

/* Control objects, one per core: each core's idle hooks time against its own
   cycle counter, so a measurement must start and stop on the same core.*/
#if CH_CFG_SMP_MODE == TRUE
#define SYS_LOAD_CORE()   port_get_core_id()
#else
#define SYS_LOAD_CORE()   0U
#endif
static sys_load_data_t  _loads[PORT_CORES_NUMBER];

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

  for (unsigned i = 0; i < PORT_CORES_NUMBER; i++) {
    _loads[i].state = SYS_MEASURE_STOP;
  }
}

/**
 * @brief Start CPU load measuring
 *
 * @return True on success else false
 *
 * @api
 */
bool sysStartLoadMeasure(void) {

  bool started = false;
  for (unsigned i = 0; i < PORT_CORES_NUMBER; i++) {
    sys_load_data_t *load = &_loads[i];
    if (load->state != SYS_MEASURE_STOP) {
      continue;
    }
    load->stop = false;
    load->state = SYS_MEASURE_INIT;
    started = true;
  }
  return started;
}

/**
 * @brief Request stop of CPU load measuring
 *
 * @return True on success else false
 *
 * @api
 */
bool sysStopLoadMeasure(void) {

  bool stopped = false;
  for (unsigned i = 0; i < PORT_CORES_NUMBER; i++) {
    sys_load_data_t *load = &_loads[i];
    if (load->state == SYS_MEASURE_STOP || load->stop) {
      continue;
    }
    load->stop = true;
    stopped = true;
  }
  return stopped;
}

/**
 * @brief Get the peak CPU load measured.
 *
 * @return Peak CPU load  reached as percentage * 100
 *
 * @api
 */
sys_cpu_load_t sysGetCPUPeakLoad(void) {

  return _loads[0].peak;
}

/**
 * @brief Get the peak CPU load measured on one core.
 *
 * @return Peak CPU load reached as percentage * 100
 *
 * @api
 */
sys_cpu_load_t sysGetCoreCPUPeakLoad(unsigned core) {

  return core < PORT_CORES_NUMBER ? _loads[core].peak : 0U;
}

/**
 * @brief Get the moving average of CPU load.
 *
 * @return CPU average load as percentage * 100
 *
 * @api
 */
sys_cpu_load_t sysGetCPUAverageLoad(void) {

  return _loads[0].average;
}

/**
 * @brief Get the moving average of CPU load on one core.
 *
 * @return CPU average load as percentage * 100
 *
 * @api
 */
sys_cpu_load_t sysGetCoreCPUAverageLoad(unsigned core) {

  return core < PORT_CORES_NUMBER ? _loads[core].average : 0U;
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

  sys_load_data_t *load = &_loads[0];
  if (load->state != SYS_MEASURE_ACTIVE) {
    return MSG_TIMEOUT;
  }
  chSysLock();
  stats->last     = load->run.last * SYS_CPU_MAX_LOAD / (load->run.last +
                                                         load->idle.last);
  stats->current  = load->average;
  stats->peak     = load->peak;
  stats->idle_max = load->idle.worst / (SystemCoreClock / 1000);
  stats->run_max  = load->run.worst / (SystemCoreClock / 1000);
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
  sys_load_data_t *load = &_loads[SYS_LOAD_CORE()];
  switch (load->state) {
    case SYS_MEASURE_STOP:

      /* Measurement is not active.*/
      return;

    case SYS_MEASURE_INIT:

      /* Measurement start requested.*/
      chTMObjectInit(&load->idle);
      chTMObjectInit(&load->run);
      load->peak = (sys_cpu_load_t)0;
      load->average = (sys_cpu_load_t)0;
      chTMStartMeasurementX(&load->idle);
      load->state = SYS_MEASURE_ACTIVE;

      /* Return to scheduler for switch to idle context.*/
      return;

    case SYS_MEASURE_ACTIVE: {

      /* Stop the run measurement.*/
      chTMStopMeasurementX(&load->run);

      /* Calculate current load from idle to run.*/
      rtcnt_t idle = load->idle.cumulative / load->idle.n;
      rtcnt_t run  = load->run.cumulative / load->run.n;
      sys_cpu_load_t current = 0;
      if (idle + run > (rtcnt_t)0) {
        current = (run * SYS_CPU_MAX_LOAD) / (idle + run);
      }

      /* Update the average and peak.*/
      load->average = current;
      if (current > load->peak) {
        load->peak = current;
      }

      /* Scale TM accumulator every second.*/
      if (load->run.cumulative > SystemCoreClock) {
        load->run.n /= 2;
        load->run.cumulative /= 2;
      }

      if (load->stop) {

        /* Run + idle cycle is calculated. Stop will have valid results.*/
        load->state = SYS_MEASURE_STOP;
        load->stop = false;
        return;
      }

      /* Start measurement of idle time.*/
      chTMStartMeasurementX(&load->idle);
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
  sys_load_data_t *load = &_loads[SYS_LOAD_CORE()];
  switch (load->state) {
    case SYS_MEASURE_STOP:
      return;

    case SYS_MEASURE_INIT:

      /* Init is handled in idle thread enter.*/
      return;

    case SYS_MEASURE_ACTIVE: {

      /* RTOS has exited idle so capture time now.*/
      chTMStopMeasurementX(&load->idle);

      /* Scale TM accumulator every second.*/
      if (load->idle.cumulative > SystemCoreClock) {
        load->idle.n /=  2;
        load->idle.cumulative /= 2;
      }

      /* Start measurement of run time.*/
      chTMStartMeasurementX(&load->run);
      return;
    }
  }
}
#else
  void sysIdleEnterMeasure(void) { };
  void sysIdleLeaveMeasure(void) { };
#endif /* HAL_USE_LOAD_MEASURE == TRUE*/

/** @} */
