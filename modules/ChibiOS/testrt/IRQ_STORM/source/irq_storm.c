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
#include "irq_storm.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define MSG_SEND_LEFT                   (msg_t)0
#define MSG_SEND_RIGHT                  (msg_t)1

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static const irq_storm_config_t *config;

static bool saturated;

/*
 * Mailboxes and buffers.
 */
static mailbox_t mb[IRQ_STORM_CFG_NUM_THREADS];
static msg_t b[IRQ_STORM_CFG_NUM_THREADS][IRQ_STORM_CFG_MAILBOX_SIZE];

/*
 * Threads working areas.
 */
static THD_WORKING_AREA(irq_storm_thread_wa[IRQ_STORM_CFG_NUM_THREADS],
                        IRQ_STORM_CFG_STACK_SIZE);

/*
 * Pointers to threads.
 */
static thread_t *threads[IRQ_STORM_CFG_NUM_THREADS];

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*
 * Test worker threads.
 */
static THD_FUNCTION(irq_storm_thread, arg) {
  static unsigned cnt = 0;
  unsigned me = (unsigned)arg;
  unsigned target;
  msg_t msg;

  chRegSetThreadName("irq_storm");

  /* Thread loop, until terminated.*/
  while (chThdShouldTerminateX() == false) {

    /* Waiting for a message.*/
   chMBFetchTimeout(&mb[me], &msg, TIME_INFINITE);

#if IRQ_STORM_CFG_RANDOMIZE != FALSE
   /* Pseudo-random delay.*/
   {
     static volatile unsigned x = 0;
     unsigned r;

     chSysLock();
     r = rand() & 15;
     chSysUnlock();
     while (r--)
       x++;
   }
#else /* IRQ_STORM_CFG_RANDOMIZE == FALSE */
   /* No delay.*/
#endif /* IRQ_STORM_CFG_RANDOMIZE == FALSE */

    /* Deciding in which direction to re-send the message.*/
    if (msg == MSG_SEND_LEFT)
      target = me - 1;
    else
      target = me + 1;

    if (target < IRQ_STORM_CFG_NUM_THREADS) {
      /* If this thread is not at the end of a chain re-sending the message,
         note this check works because the variable target is unsigned.*/
      msg = chMBPostTimeout(&mb[target], msg, TIME_IMMEDIATE);
      if (msg != MSG_OK)
        saturated = TRUE;
    }
    else {
      /* Provides a visual feedback about the system.*/
      if (++cnt >= 500) {
        cnt = 0;
        palToggleLine(config->line);
      }
    }
  }
}

/**
 * @brief   GPT1 callback.
 */
void irq_storm_gpt1_cb(GPTDriver *gptp) {
  msg_t msg;

  (void)gptp;
  chSysLockFromISR();
  msg = chMBPostI(&mb[0], MSG_SEND_RIGHT);
  if (msg != MSG_OK)
    saturated = true;
  chSysUnlockFromISR();
}

/**
 * @brief   GPT2 callback.
 */
void irq_storm_gpt2_cb(GPTDriver *gptp) {
  msg_t msg;

  (void)gptp;
  chSysLockFromISR();
  msg = chMBPostI(&mb[IRQ_STORM_CFG_NUM_THREADS - 1], MSG_SEND_LEFT);
  if (msg != MSG_OK)
    saturated = true;
  chSysUnlockFromISR();
}

/**
 * @brief   IRQ storm execution.
 *
 * @param[in] cfg       pointer to the test configuration structure
 *
 * @api
 */
void irq_storm_execute(const irq_storm_config_t *cfg) {
  unsigned i;
  gptcnt_t interval, threshold, worst;

  /* Global configuration pointer.*/
  config = cfg;

  /* Starting timers using the stored configurations.*/
  gptStart(cfg->gpt1p, cfg->gptcfg1p);
  gptStart(cfg->gpt2p, cfg->gptcfg2p);

  /*
   * Initializes the mailboxes and creates the worker threads.
   */
  for (i = 0; i < IRQ_STORM_CFG_NUM_THREADS; i++) {
    chMBObjectInit(&mb[i], b[i], IRQ_STORM_CFG_MAILBOX_SIZE);
    threads[i] = chThdCreateStatic(irq_storm_thread_wa[i],
                                   sizeof irq_storm_thread_wa[i],
                                   IRQ_STORM_CFG_THREADS_PRIORITY,
                                   irq_storm_thread,
                                   (void *)i);
  }

  /* Printing environment information.*/
  chprintf(cfg->out, "\r\n*** ChibiOS/RT IRQ-STORM long duration test\r\n***\r\n");
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
  chprintf(cfg->out, "*** Iterations:   %d\r\n", IRQ_STORM_CFG_ITERATIONS);
  chprintf(cfg->out, "*** Randomize:    %d\r\n", IRQ_STORM_CFG_RANDOMIZE);
  chprintf(cfg->out, "*** Threads:      %d\r\n", IRQ_STORM_CFG_NUM_THREADS);
  chprintf(cfg->out, "*** Mailbox size: %d\r\n\r\n", IRQ_STORM_CFG_MAILBOX_SIZE);

  /* Test loop.*/
  worst = 0;
  for (i = 1; i <= IRQ_STORM_CFG_ITERATIONS; i++){

    chprintf(cfg->out, "Iteration %d\r\n", i);
    saturated = false;
    threshold = 0;

    /* Timer intervals starting at 2mS then decreased by 10% after each
       cycle.*/
    for (interval = 2000; interval >= 2; interval -= (interval + 9) / 10) {

      /* Timers programmed slightly out of phase each other.*/
      gptStartContinuous(cfg->gpt1p, interval - 1); /* Slightly out of phase.*/
      gptStartContinuous(cfg->gpt2p, interval + 1); /* Slightly out of phase.*/

      /* Storming for one second.*/
      chThdSleepMilliseconds(1000);

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
  for (i = 0; i < IRQ_STORM_CFG_NUM_THREADS; i++) {
    chThdTerminate(threads[i]);
    chThdWait(threads[i]);
    threads[i] = NULL;
  }
}

/** @} */
