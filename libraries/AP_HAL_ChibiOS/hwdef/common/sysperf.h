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

#pragma once
#include "ch.h"
#include "hal.h"
#include "hwdef.h"

/**
 * @file    sysperf.h
 * @brief   performance measurement.
 *
 * @addtogroup monitor
 * @{
 */

/*===========================================================================*/
/* Module definitions.                                                       */
/*===========================================================================*/

#define SYS_CPU_MAX_LOAD  10000   /**< @brief maximum CPU load as %.xx. */

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

#if HAL_USE_LOAD_MEASURE == TRUE

typedef uint16_t  sys_cpu_load_t;   /**< @brief CPU load in percent * 100.  */

/**
 * @brief Load measurement control structure
 */
typedef struct {
  enum {
    SYS_MEASURE_STOP = 0,
    SYS_MEASURE_INIT,
    SYS_MEASURE_ACTIVE
  } state;
  time_measurement_t  idle;         /**< @brief idle time measurement.     */
  time_measurement_t  run;          /**< @brief run time measurement.      */
  sys_cpu_load_t      average;      /**< @brief moving average of load.    */
  sys_cpu_load_t      peak;         /**< @brief peak of load.              */
  bool                stop;
} sys_load_data_t;

/**
 * @brief Instantaneous load reading request structure
 */
typedef struct {
  sys_cpu_load_t      last;         /**< @brief load from last idle/run.    */
  sys_cpu_load_t      current;      /**< @brief current average load.       */
  sys_cpu_load_t      peak;         /**< @brief peak load captured.         */
  time_msecs_t        idle_max;     /**< @brief longest idle time in ms.    */
  time_msecs_t        run_max;      /**< @brief longest run time in ms.     */
} sys_load_stats_t;

#endif /* SYS_USE_LOAD_MEASURE == TRUE */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern thread_t *sysstat_thd;

#ifdef __cplusplus
extern "C" {
#endif
#if HAL_USE_LOAD_MEASURE == TRUE
  void            sysInitLoadMeasure(void);
  bool            sysStartLoadMeasure(void);
  bool            sysStopLoadMeasure(void);
  sys_cpu_load_t  sysGetCPUPeakLoad(void);
  sys_cpu_load_t  sysGetCPUAverageLoad(void);
  msg_t           sysGetCPULoadStatistics(sys_load_stats_t *stats);
#endif /* HAL_USE_LOAD_MEASURE == TRUE */
  void            sysIdleEnterMeasure(void);
  void            sysIdleLeaveMeasure(void);
#ifdef __cplusplus
}
#endif
/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/** @} */
