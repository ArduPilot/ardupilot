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

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "rt_test_root.h"
#include "oslib_test_root.h"

#include "chprintf.h"
#include "shell.h"

#include "portab.h"

#if defined(STM32_SDC_SDIO_UNALIGNED_SUPPORT)
#define PORTAB_UNALIGNED_SUPPORT        STM32_SDC_SDIO_UNALIGNED_SUPPORT

#elif defined(STM32_SDC_SDMMC_UNALIGNED_SUPPORT)
#define PORTAB_UNALIGNED_SUPPORT        STM32_SDC_SDMMC_UNALIGNED_SUPPORT

#else
#error "unexpected definitions"
#endif

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE       THD_WORKING_AREA_SIZE(2048)

#define SDC_BURST_SIZE      16

/* Buffer for block read/write operations, note that extra bytes are
   allocated in order to support unaligned operations.*/
static uint8_t buf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 4];

/* Additional buffer for sdcErase() test */
static uint8_t buf2[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE];

void cmd_sdc(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *mode[] = {"SDV11", "SDV20", "MMC", NULL};
  systime_t start, end;
  uint32_t n, startblk;

  if (argc != 1) {
    chprintf(chp, "Usage: sdiotest read|write|erase|all\r\n");
    return;
  }

  /* Card presence check.*/
  if (!blkIsInserted(&PORTAB_SDCD1)) {
    chprintf(chp, "Card not inserted, aborting.\r\n");
    return;
  }

  /* Connection to the card.*/
  chprintf(chp, "Connecting... ");
  if (sdcConnect(&PORTAB_SDCD1)) {
    chprintf(chp, "failed\r\n");
    return;
  }

  chprintf(chp, "OK\r\n\r\nCard Info\r\n");
  chprintf(chp, "CSD      : %08X %8X %08X %08X \r\n",
           PORTAB_SDCD1.csd[3], PORTAB_SDCD1.csd[2], PORTAB_SDCD1.csd[1], PORTAB_SDCD1.csd[0]);
  chprintf(chp, "CID      : %08X %8X %08X %08X \r\n",
           PORTAB_SDCD1.cid[3], PORTAB_SDCD1.cid[2], PORTAB_SDCD1.cid[1], PORTAB_SDCD1.cid[0]);
  chprintf(chp, "Mode     : %s\r\n", mode[PORTAB_SDCD1.cardmode & 3U]);
  chprintf(chp, "Capacity : %DMB\r\n", PORTAB_SDCD1.capacity / 2048);

  /* The test is performed in the middle of the flash area.*/
  startblk = (PORTAB_SDCD1.capacity / MMCSD_BLOCK_SIZE) / 2;

  if ((strcmp(argv[0], "read") == 0) ||
      (strcmp(argv[0], "all") == 0)) {

    /* Single block read performance, aligned.*/
    chprintf(chp, "Single block aligned read performance:           ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&PORTAB_SDCD1, startblk, buf, 1)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n++;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

    /* Multiple sequential blocks read performance, aligned.*/
    chprintf(chp, "16 sequential blocks aligned read performance:   ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&PORTAB_SDCD1, startblk, buf, SDC_BURST_SIZE)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n += SDC_BURST_SIZE;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

#if PORTAB_UNALIGNED_SUPPORT
    /* Single block read performance, unaligned.*/
    chprintf(chp, "Single block unaligned read performance:         ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&PORTAB_SDCD1, startblk, buf + 1, 1)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n++;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

    /* Multiple sequential blocks read performance, unaligned.*/
    chprintf(chp, "16 sequential blocks unaligned read performance: ");
    start = chVTGetSystemTime();
    end = chTimeAddX(start, TIME_MS2I(1000));
    n = 0;
    do {
      if (blkRead(&PORTAB_SDCD1, startblk, buf + 1, SDC_BURST_SIZE)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n += SDC_BURST_SIZE;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);
#endif /* PORTAB_UNALIGNED_SUPPORT */
  }

  if ((strcmp(argv[0], "write") == 0) ||
      (strcmp(argv[0], "all") == 0)) {
    unsigned i;

    memset(buf, 0xAA, MMCSD_BLOCK_SIZE * 2);
    chprintf(chp, "Writing...");
    if(sdcWrite(&PORTAB_SDCD1, startblk, buf, 2)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");

    memset(buf, 0x55, MMCSD_BLOCK_SIZE * 2);
    chprintf(chp, "Reading...");
    if (blkRead(&PORTAB_SDCD1, startblk, buf, 1)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");

    for (i = 0; i < MMCSD_BLOCK_SIZE * 2; i++)
      buf[i] = i + 8;
    chprintf(chp, "Writing...");
    if(sdcWrite(&PORTAB_SDCD1, startblk, buf, 2)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");

    memset(buf, 0, MMCSD_BLOCK_SIZE * 2);
    chprintf(chp, "Reading...");
    if (blkRead(&PORTAB_SDCD1, startblk, buf, 1)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");
  }

  if ((strcmp(argv[0], "erase") == 0) ||
      (strcmp(argv[0], "all") == 0)) {
    /**
     * Test sdcErase()
     * Strategy:
     *   1. Fill two blocks with non-constant data
     *   2. Write two blocks starting at startblk
     *   3. Erase the second of the two blocks
     *      3.1. First block should be equal to the data written
     *      3.2. Second block should NOT be equal too the data written (i.e. erased).
     *   4. Erase both first and second block
     *      4.1 Both blocks should not be equal to the data initially written
     * Precondition: SDC_BURST_SIZE >= 2
     */
    memset(buf, 0, MMCSD_BLOCK_SIZE * 2);
    memset(buf2, 0, MMCSD_BLOCK_SIZE * 2);
    /* 1. */
    unsigned int i = 0;
    for (; i < MMCSD_BLOCK_SIZE * 2; ++i) {
      buf[i] = (i + 7) % 'T'; //Ensure block 1/2 are not equal
    }
    /* 2. */
    if(sdcWrite(&PORTAB_SDCD1, startblk, buf, 2)) {
      chprintf(chp, "sdcErase() test write failed\r\n");
      goto exittest;
    }
    /* 3. (erase) */
    if(sdcErase(&PORTAB_SDCD1, startblk + 1, startblk + 2)) {
      chprintf(chp, "sdcErase() failed\r\n");
      goto exittest;
    }
    sdcflags_t errflags = sdcGetAndClearErrors(&PORTAB_SDCD1);
    if(errflags) {
      chprintf(chp, "sdcErase() yielded error flags: %d\r\n", errflags);
      goto exittest;
    }
    if(sdcRead(&PORTAB_SDCD1, startblk, buf2, 2)) {
      chprintf(chp, "single-block sdcErase() failed\r\n");
      goto exittest;
    }
    /* 3.1. */
    if(memcmp(buf, buf2, MMCSD_BLOCK_SIZE) != 0) {
      chprintf(chp, "sdcErase() non-erased block compare failed\r\n");
      goto exittest;
    }
    /* 3.2. */
    if(memcmp(buf + MMCSD_BLOCK_SIZE,
              buf2 + MMCSD_BLOCK_SIZE, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(chp, "sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    /* 4. */
    if(sdcErase(&PORTAB_SDCD1, startblk, startblk + 2)) {
      chprintf(chp, "multi-block sdcErase() failed\r\n");
      goto exittest;
    }
    if(sdcRead(&PORTAB_SDCD1, startblk, buf2, 2)) {
      chprintf(chp, "single-block sdcErase() failed\r\n");
      goto exittest;
    }
    /* 4.1 */
    if(memcmp(buf, buf2, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(chp, "multi-block sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    if(memcmp(buf + MMCSD_BLOCK_SIZE,
              buf2 + MMCSD_BLOCK_SIZE, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(chp, "multi-block sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    /* END of sdcErase() test */
  }

  /* Card disconnect and command end.*/
exittest:
  sdcDisconnect(&PORTAB_SDCD1);
}

static const ShellCommand commands[] = {
  {"sdc", cmd_sdc},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&PORTAB_SD1,
  commands
};

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Board-dependent GPIO setup code.*/
  portab_setup();

  /* Starting a serial port for test report output.*/
  sdStart(&PORTAB_SD1, NULL);

  /* Shell manager initialization.*/
  shellInit();

  /* Activates the  SDC driver using default configuration.*/
  sdcStart(&PORTAB_SDCD1, NULL);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Normal main() thread activity, spawning shells.*/
  while (true) {
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                            "shell", NORMALPRIO + 1,
                                            shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp);               /* Waiting termination.             */
    chThdSleepMilliseconds(1000);
  }
}
