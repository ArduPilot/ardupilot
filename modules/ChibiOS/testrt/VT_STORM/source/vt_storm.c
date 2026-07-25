/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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
 * @file    irq_storm.c
 * @brief   IRQ Storm stress test code.
 *
 * @addtogroup IRQ_STORM
 * @{
 */

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "vt_storm.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static const vt_storm_config_t *config;

static virtual_timer_t watchdog;
static virtual_timer_t wrapper;
static virtual_timer_t sweeper0, sweeperm1, sweeperp1, sweeperm3, sweeperp3;
static virtual_timer_t guard0, guard1, guard2, guard3;
static virtual_timer_t continuous;
static volatile sysinterval_t delay;
static volatile bool saturated;
static uint32_t vtcus;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void rnddly(void) {
#if VT_STORM_CFG_RANDOMIZE != FALSE
   /* Pseudo-random delay.*/
   {
     static volatile unsigned x = 0;
     unsigned r;

     r = rand() & 15;
     while (r--) {
       x++;
     }
   }
#endif
}

static void watchdog_cb(virtual_timer_t *vtp, void *p) {

  (void)vtp;
  (void)p;

  chSysLockFromISR();
  chVTResetI(&sweeper0);
  chVTResetI(&sweeperm1);
  chVTResetI(&sweeperp1);
  chVTResetI(&sweeperm3);
  chVTResetI(&sweeperp3);
  chVTResetI(&wrapper);
  chSysUnlockFromISR();

  saturated = true;
}

static void wrapper_cb(virtual_timer_t *vtp, void *p) {

  (void)vtp;
  (void)p;
}

static void sweeper0_cb(virtual_timer_t *vtp, void *p) {

  (void)p;

  chSysLockFromISR();
  rnddly();
  chVTSetI(&wrapper, (sysinterval_t)-1, wrapper_cb, NULL);
  chVTSetI(vtp, delay, sweeper0_cb, NULL);
  chSysUnlockFromISR();
}

static void sweeperm1_cb(virtual_timer_t *vtp, void *p) {

  (void)p;

  chSysLockFromISR();
  rnddly();
  chVTSetI(vtp, delay - 1, sweeperm1_cb, NULL);
  chSysUnlockFromISR();
}

static void sweeperp1_cb(virtual_timer_t *vtp, void *p) {

  (void)p;

  chSysLockFromISR();
  rnddly();
  chVTSetI(vtp, delay + 1, sweeperp1_cb, NULL);
  chSysUnlockFromISR();
}

static void sweeperm3_cb(virtual_timer_t *vtp, void *p) {

  (void)p;

  chSysLockFromISR();
  rnddly();
  chVTSetI(vtp, delay - 3, sweeperm3_cb, NULL);
  chSysUnlockFromISR();
}

static void sweeperp3_cb(virtual_timer_t *vtp, void *p) {

  (void)p;

  chSysLockFromISR();
  rnddly();
  chVTSetI(vtp, delay + 3, sweeperp3_cb, NULL);
  chSysUnlockFromISR();
}

static void continuous_cb(virtual_timer_t *vtp, void *p) {

  (void)vtp;
  (void)p;

  vtcus++;
  rnddly();
}

static void guard_cb(virtual_timer_t *vtp, void *p) {

  (void)vtp;
  (void)p;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   VT storm execution.
 *
 * @param[in] cfg       pointer to the test configuration structure
 *
 * @api
 */
void vt_storm_execute(const vt_storm_config_t *cfg) {
  unsigned i;
  sysinterval_t periodic;

  config = cfg;

  /* Printing environment information.*/
  chprintf(cfg->out, "");
  chprintf(cfg->out, "\r\n*** ChibiOS/RT VT-STORM long duration test\r\n***\r\n");
  chprintf(cfg->out, "*** Kernel:       %s\r\n", CH_KERNEL_VERSION);
  chprintf(cfg->out, "*** Compiled:     %s\r\n", __DATE__ " - " __TIME__);
#ifdef PORT_COMPILER_NAME
  chprintf(cfg->out, "*** Compiler:     %s\r\n", PORT_COMPILER_NAME);
#endif
  chprintf(cfg->out, "*** Architecture: %s\r\n", PORT_ARCHITECTURE_NAME);
#ifdef PORT_CORE_VARIANT_NAME
  chprintf(cfg->out, "*** Core Variant: %s\r\n", PORT_CORE_VARIANT_NAME);
#endif
  chprintf(cfg->out, "*** System Clock: %d\r\n", cfg->sysclk);
#ifdef PORT_INFO
  chprintf(cfg->out, "*** Port Info:    %s\r\n", PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  chprintf(cfg->out, "*** Platform:     %s\r\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  chprintf(cfg->out, "*** Test Board:   %s\r\n", BOARD_NAME);
#endif
  chprintf(cfg->out, "***\r\n");
  chprintf(cfg->out, "*** Randomize:        %d\r\n", VT_STORM_CFG_RANDOMIZE);
  chprintf(cfg->out, "*** Hammers:          %d\r\n", VT_STORM_CFG_HAMMERS);
  chprintf(cfg->out, "*** Minimum Delay:    %d ticks\r\n", VT_STORM_CFG_MIN_DELAY);
  chprintf(cfg->out, "*** System Time size: %d bits\r\n", CH_CFG_ST_RESOLUTION);
  chprintf(cfg->out, "*** Intervals size:   %d bits\r\n", CH_CFG_INTERVALS_SIZE);
  chprintf(cfg->out, "*** SysTick:          %d Hz\r\n", CH_CFG_ST_FREQUENCY);
  chprintf(cfg->out, "*** Delta:            %d ticks\r\n", CH_CFG_ST_TIMEDELTA);
  chprintf(cfg->out, "\r\n");

#if VT_STORM_CFG_HAMMERS
  /* Starting hammer timers.*/
  gptStart(cfg->gpt1p, cfg->gptcfg1p);
  gptStart(cfg->gpt2p, cfg->gptcfg2p);
  gptStartContinuous(cfg->gpt1p, 99);
  gptStartContinuous(cfg->gpt2p, 101);
#endif

  /* Interval for the periodic timer, note that slow systicks would
     result in a period of 1, which is not acceptable, increasing it
     to two.*/
  periodic = TIME_US2I(50);
  if (periodic < (sysinterval_t)2) {
    periodic = (sysinterval_t)2;
  }

  for (i = 1; i <= VT_STORM_CFG_ITERATIONS; i++) {
    bool warning;

    chprintf(cfg->out, "Iteration %d\r\n", i);
    chThdSleep(TIME_MS2I(10));

    /* Starting continuous timer.*/
    vtcus = 0;

    delay = TIME_MS2I(5);
    if (delay < (sysinterval_t)VT_STORM_CFG_MIN_DELAY) {
      delay = (sysinterval_t)VT_STORM_CFG_MIN_DELAY;
    }
    saturated = false;
    warning   = false;
    do {
      rfcu_mask_t mask;
      sysinterval_t decrease;

      /* Starting sweepers.*/
      chSysLock();
      chVTSetI(&watchdog, TIME_MS2I(501), watchdog_cb, NULL);
      chVTSetI(&sweeper0, delay, sweeper0_cb, NULL);
      chVTSetI(&sweeperm1, delay - 1, sweeperm1_cb, NULL);
      chVTSetI(&sweeperp1, delay + 1, sweeperp1_cb, NULL);
      chVTSetI(&sweeperm3, delay - 3, sweeperm3_cb, NULL);
      chVTSetI(&sweeperp3, delay + 3, sweeperp3_cb, NULL);
      chVTSetI(&wrapper, (sysinterval_t)-1, wrapper_cb, NULL);
      chVTSetContinuousI(&continuous, periodic, continuous_cb, NULL);
      chVTSetI(&guard0, TIME_MS2I(250) + (CH_CFG_TIME_QUANTUM / 2), guard_cb, NULL);
      chVTSetI(&guard1, TIME_MS2I(250) + (CH_CFG_TIME_QUANTUM - 1), guard_cb, NULL);
      chVTSetI(&guard2, TIME_MS2I(250) + (CH_CFG_TIME_QUANTUM + 1), guard_cb, NULL);
      chVTSetI(&guard3, TIME_MS2I(250) + (CH_CFG_TIME_QUANTUM * 2), guard_cb, NULL);

      /* Letting them run for a while.*/
      chThdSleepS(TIME_MS2I(100));

      /* Stopping everything.*/
      chVTResetI(&watchdog);
      chVTResetI(&sweeper0);
      chVTResetI(&sweeperm1);
      chVTResetI(&sweeperp1);
      chVTResetI(&sweeperm3);
      chVTResetI(&sweeperp3);
      chVTResetI(&wrapper);
      chVTResetI(&continuous);
      chVTResetI(&guard0);
      chVTResetI(&guard1);
      chVTResetI(&guard2);
      chVTResetI(&guard3);

      /* Check for relevant RFCU events.*/
      mask = chRFCUGetAndClearFaultsI(CH_RFCU_VT_INSUFFICIENT_DELTA |
                                      CH_RFCU_VT_SKIPPED_DEADLINE);
      chSysUnlock();

      if (saturated) {
        chprintf(cfg->out, "#");
        break;
      }
      else if (mask == CH_RFCU_VT_INSUFFICIENT_DELTA) {
        palToggleLine(config->line);
        chprintf(cfg->out, "x");
        warning = true;
      }
      else if (mask == CH_RFCU_VT_SKIPPED_DEADLINE) {
        palToggleLine(config->line);
        chprintf(cfg->out, "+");
        warning = true;
      }
      else if (mask == (CH_RFCU_VT_INSUFFICIENT_DELTA | CH_RFCU_VT_SKIPPED_DEADLINE)) {
        palToggleLine(config->line);
        chprintf(cfg->out, "*");
        warning = true;
      }
      else {
        palToggleLine(config->line);
        chprintf(cfg->out, ".");
      }
//      if (delay >= TIME_US2I(1)) {
//        delay -= TIME_US2I(1);
//      }
      decrease = delay / (sysinterval_t)32;
      if (decrease == (sysinterval_t)0) {
        decrease = (sysinterval_t)1;
      }
      delay = delay - decrease;
    } while (delay >= (sysinterval_t)VT_STORM_CFG_MIN_DELAY);

    if (warning) {
      chprintf(cfg->out, "\r\nRFCU warning detected at %u uS %u ticks",
               TIME_I2US(delay), delay);
    }
    else {
      chprintf(cfg->out, "\r\nNo warnings");
    }
    if (saturated) {
      chprintf(cfg->out, "\r\nSaturated at %u uS %u ticks", TIME_I2US(delay), delay);
    }
    else {
      chprintf(cfg->out, "\r\nNon saturated");
    }
    chprintf(cfg->out, "\r\nContinuous ticks %u\r\n\r\n", vtcus);
  }
}

#if VT_STORM_CFG_HAMMERS || defined(__DOXYGEN__)
/**
 * @brief   GPT callback 1.
 */
void vt_storm_gpt1_cb(GPTDriver *gptp) {

  (void)gptp;

  rnddly();
}

/**
 * @brief   GPT callback 2.
 */
void vt_storm_gpt2_cb(GPTDriver *gptp) {

  (void)gptp;

  rnddly();
}
#endif

/** @} */
