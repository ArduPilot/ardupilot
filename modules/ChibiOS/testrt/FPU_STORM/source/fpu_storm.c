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
 * @file    fpu_storm.c
 * @brief   FPU Storm stress test code.
 *
 * @addtogroup FPU_STORM
 * @{
 */

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "fpu_storm.h"

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

static const fpu_storm_config_t *config;

static uint32_t wdgcnt;
static bool saturated;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

CC_NO_INLINE static float ff1(float par) {

  asm volatile ("nop" : : : "s1", "s2", "s3", "s4", "s5", "s6", "s7",
                            "s8", "s9", "s10", "s11", "s12", "s13",
                            "s14", "s15", "s16", "s17", "s18", "s19",
                            "s20", "s21", "s22", "s23", "s24", "s25",
                            "s26", "s27", "s28", "s29", "s30", "s31");

  return par;
}

CC_NO_INLINE static float ff2(float par1, float par2,
                              float par3, float par4) {

  asm volatile ("nop" : : : "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
                            "s12", "s13", "s14", "s15", "s16", "s17", "s18",
                            "s19", "s20", "s21", "s22", "s23", "s24", "s25",
                            "s26", "s27", "s28", "s29", "s30", "s31");

  return (par1 + par2) * (par3 + par4);
}

/*
 * Test worker thread.
 */
static THD_WORKING_AREA(waWorkerThread, 256);
static THD_FUNCTION(WorkerThread, arg) {

  (void)arg;

  while (1) {
    float f1, f2, f3, f4, f5, f6;

    /* Loading values in FPU registers for monitoring.*/
    f1 = ff1(3.0f);
    f2 = ff1(4.0f);
    f3 = ff1(5.0f);
    f5 = f1 + f2 + f3;
    f4 = ff1(6.0f);
    f5 = ff2(f5, f4, f5, f4);
    f6 = ff1(f5) * ff1(f2);
    if (f5 != 324.0f) {
      chSysHalt("float corrupion #1");
    }
    if (f6 != 1296.0f) {
      chSysHalt("float corrupion #2");
    }

    wdgcnt++;
  }
}

/*
 * Test periodic thread.
 */
static THD_WORKING_AREA(waPeriodicThread, 256);
static THD_FUNCTION(PeriodicThread, arg) {

  (void)arg;

  while (1) {
    float f1, f2, f3, f4, f5, f6;

    /* Loading values in FPU registers for monitoring.*/
    f1 = ff1(4.0f);
    f2 = ff1(5.0f);
    f3 = ff1(6.0f);
    f5 = f1 + f2 + f3;
    f4 = ff1(7.0f);
    f5 = ff2(f5, f4, f5, f4);
    f6 = ff1(f5) * ff1(f2);
    if (f5 != 484.0f) {
      chSysHalt("float corrupion #3");
    }
    if (f6 != 2420.0f) {
      chSysHalt("float corrupion #4");
    }

    chThdSleepSeconds(1);
  }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   GPT1 callback.
 */
void fpu_storm_gpt1_cb(GPTDriver *gptp) {
  static uint32_t lastcnt;
  float f1, f2, f3, f4, f5, f6;

  (void)gptp;

  /* Loading values in FPU registers for monitoring.*/
  f1 = ff1(2.0f);
  f2 = ff1(3.0f);
  f3 = ff1(4.0f);
  f5 = f1 + f2 + f3;
  f4 = ff1(5.0f);
  f5 = ff2(f5, f4, f5, f4);
  f6 = ff1(f5) * ff1(f2);
  if (f5 != 196.0f) {
    chSysHalt("float corrupion #5");
  }
  if (f6 != 588.0f) {
    chSysHalt("float corrupion #6");
  }

  /* Checking saturation condition.*/
  if (lastcnt == wdgcnt) {
    saturated = true;
  }
  lastcnt = wdgcnt;
}

/**
 * @brief   GPT2 callback.
 */
void fpu_storm_gpt2_cb(GPTDriver *gptp) {
  static uint32_t lastcnt;
  float f1, f2, f3, f4, f5, f6;

  (void)gptp;

  /* Loading values in FPU registers for monitoring.*/
  f1 = ff1(1.0f);
  f2 = ff1(2.0f);
  f3 = ff1(3.0f);
  f5 = f1 + f2 + f3;
  f4 = ff1(4.0f);
  f5 = ff2(f5, f4, f5, f4);
  f6 = ff1(f5) * ff1(f2);
  if (f5 != 100.0f) {
    chSysHalt("float corrupion #7");
  }
  if (f6 != 200.0f) {
    chSysHalt("float corrupion #8");
  }

  /* Checking saturation condition.*/
  if (lastcnt == wdgcnt) {
    saturated = true;
  }
  lastcnt = wdgcnt;
}

/**
 * @brief   IRQ storm execution.
 *
 * @param[in] cfg       pointer to the test configuration structure
 *
 * @api
 */
void fpu_storm_execute(const fpu_storm_config_t *cfg) {
  thread_t *worker, *periodic;
  unsigned i;
  gptcnt_t interval, threshold, worst;

  /* Global configuration pointer.*/
  config = cfg;

  /* Starting timers using the stored configurations.*/
  gptStart(cfg->gpt1p, cfg->gptcfg1p);
  gptStart(cfg->gpt2p, cfg->gptcfg2p);

  /*
   * Starting the worker threads.
   */
  worker   = chThdCreateStatic(waWorkerThread, sizeof waWorkerThread,
                             NORMALPRIO - 20, WorkerThread, NULL);
  periodic = chThdCreateStatic(waPeriodicThread, sizeof waPeriodicThread,
                             NORMALPRIO - 10, PeriodicThread, NULL);

  /* Printing environment information.*/
  chprintf(cfg->out, "");
  chprintf(cfg->out, "\r\n*** ChibiOS/RT FPU-STORM long duration test\r\n***\r\n");
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
  chprintf(cfg->out, "*** Iterations:   %d\r\n", FPU_STORM_CFG_ITERATIONS);
  chprintf(cfg->out, "*** Randomize:    %d\r\n", FPU_STORM_CFG_RANDOMIZE);
  chprintf(cfg->out, "\r\n");

  /* Test loop.*/
  worst = 0;
  for (i = 1; i <= FPU_STORM_CFG_ITERATIONS; i++){

    chprintf(cfg->out, "Iteration %d\r\n", i);
    saturated = false;
    threshold = 0;

    /* Timer intervals starting at 2mS then decreased by 10% after each
       cycle.*/
    for (interval = 2000; interval >= 2; interval -= (interval + 49) / 50) {

      /* Timers programmed slightly out of phase each other.*/
      gptStartContinuous(cfg->gpt1p, interval - 1); /* Slightly out of phase.*/
      gptStartContinuous(cfg->gpt2p, interval + 1); /* Slightly out of phase.*/

      /* Storming for one second.*/
      chThdSleepMilliseconds(250);

      /* Timers stopped.*/
      gptStopTimer(cfg->gpt1p);
      gptStopTimer(cfg->gpt2p);

      /* Did the storm saturate the threads chain?*/
      if (!saturated)
        chprintf(cfg->out, ".");
      else {
        chprintf(cfg->out, "#");
        if (threshold == 0)
          threshold = interval;
        break;
      }
    }
    /* Gives threads a chance to empty the mailboxes before next cycle.*/
    chThdSleepMilliseconds(20);
    chprintf(cfg->out, "\r\nSaturated at %d uS\r\n\r\n", threshold);
    if (threshold > worst)
      worst = threshold;
  }
  gptStopTimer(cfg->gpt1p);
  gptStopTimer(cfg->gpt2p);

  chprintf(cfg->out, "Worst case at %d uS\r\n", worst);
  chprintf(cfg->out, "\r\nTest Complete\r\n");

  /* Terminating threads and cleaning up.*/
  chThdTerminate(worker);
  chThdTerminate(periodic);
  chThdWait(worker);
  chThdWait(periodic);
}

/** @} */
