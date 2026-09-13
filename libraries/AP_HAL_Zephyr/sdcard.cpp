/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */

/* Deliberately mirrors AP_HAL_ChibiOS/sdcard.cpp - same function order and
 * semantics, so the two can be diffed against each other. */

/* AP_HAL.h FIRST: HAL_OS_FATFS_IO is defined in AP_HAL/board/zephyr.h, and
   sdcard.h pulls in nothing but <stdbool.h>. Without this the guard below
   evaluates FALSE, the whole file compiles to NOTHING, and the failure surfaces
   only at link time as "undefined reference to sdcard_stop()/sdcard_retry()"
   from AP_Filesystem_FATFS.cpp. */
#include <AP_HAL/AP_HAL.h>
#include "sdcard.h"

#if defined(HAL_OS_FATFS_IO) && HAL_OS_FATFS_IO

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include "Semaphores.h"

#include <ff.h>
#include <zephyr/storage/disk_access.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS.h>
#endif

/* Report a microSD bring-up failure over MAVLink, not just to the console.
 *
 * The console on this HAL is USB CDC on every board, and under Renode the CDC
 * is not exposed at all - so printk() and the Zephyr SD driver's own LOG_ERR
 * go nowhere an emulated flight can see. A card that fails to come up then
 * shows only as a downstream symptom: AP_Logger reports
 * "Failed to create log directory /APM/logs : ENOSPC", which is FR_NOT_ENABLED
 * (no work area) and says nothing about WHY the volume was never mounted.
 *
 * That cost real time - the failing step had to be inferred from an errno.
 * Say it plainly instead, on a link the test harness records. */
#if HAL_GCS_ENABLED
#define AP_SDCARD_FAIL(fmt, ...) GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SD: " fmt, ##__VA_ARGS__)
#else
#define AP_SDCARD_FAIL(fmt, ...) ::printf("SD: " fmt "\n", ##__VA_ARGS__)
#endif

extern const AP_HAL::HAL& hal;

static FATFS SDC_FS; // FATFS object
static HAL_Semaphore sem;
static bool sdcard_running;

/* Must match `disk-name` on the sdmmc node in mr_vmu_rt1176_mimxrt1176_cm7.dts.
   The bare name is what disk_access_*() takes; the trailing ':' form is what
   FatFs wants as a volume prefix. */
#define ZEPHYR_DISK_NAME "SD"
#define SDCARD_VOLUME    ZEPHYR_DISK_NAME ":"

/*
  initialise microSD card if available. This is called during
  AP_BoardConfig initialisation. The parameter BRD_SD_SLOWDOWN
  controls a scaling factor on the microSD clock
 */
bool sdcard_init()
{
    WITH_SEMAPHORE(sem);

    /* BRD_SD_SLOWDOWN is NOT available here: AP_BoardConfig::get_sdcard_slowdown()
     * is ChibiOS-only. */

    if (sdcard_running) {
        sdcard_stop();
    }

    /*
      try up to 3 times to init microSD interface
     */
    const uint8_t tries = 3;
    int last_disk_rc = 0;
    FRESULT last_fr = FR_OK;
    for (uint8_t i=0; i<tries; i++) {
        /* Explicit, though f_mount would trigger it via zfs_diskio.c's
           disk_initialize(). Done separately so a card-absent failure is
           distinguishable from an unformatted-card failure. Mirrors ChibiOS
           calling sdcConnect() before f_mount(). */
        last_disk_rc = disk_access_init(ZEPHYR_DISK_NAME);
        if (last_disk_rc != 0) {
            continue;
        }
        /* trailing 1 = mount now rather than lazily on first access, so a
           missing or unformatted card is reported here instead of at the first
           log write */
        last_fr = f_mount(&SDC_FS, SDCARD_VOLUME, 1);
        if (last_fr != FR_OK) {
            continue;
        }
        printf("Successfully mounted SDCard\n");

        sdcard_running = true;
        return true;
    }

    /* Name the step that failed. disk_access_init() failing means the card
       never completed identification - HAL_SD_Init() and, on this driver,
       HAL_SD_ConfigWideBusOperation(), which reads the SCR as a FIFO data
       transfer. f_mount() failing after a good disk_access_init() means the
       card answered but the volume did not parse. The two want completely
       different investigations, and from the outside they look identical. */
    /* Report only when the outcome CHANGES. AP_Logger retries start_new_log()
       for as long as logging is wanted and not running, so an unfixable
       failure would otherwise repeat every ~30 s for the whole flight - 11
       copies of the same line in the first five minutes when this was first
       switched on. A diagnostic that floods the link is one that gets turned
       off, and it would also push real messages out of a full TX buffer. */
    static int reported_disk_rc = 0;
    static FRESULT reported_fr = FR_OK;
    static bool reported_any;

    if (!reported_any || last_disk_rc != reported_disk_rc || last_fr != reported_fr) {
        reported_any = true;
        reported_disk_rc = last_disk_rc;
        reported_fr = last_fr;
        if (last_disk_rc != 0) {
            AP_SDCARD_FAIL("disk_access_init(%s) failed rc=%d after %u tries",
                           ZEPHYR_DISK_NAME, last_disk_rc, (unsigned)tries);
        } else {
            AP_SDCARD_FAIL("f_mount(%s) failed FRESULT=%u after %u tries",
                           SDCARD_VOLUME, (unsigned)last_fr, (unsigned)tries);
        }
    }

    sdcard_running = false;
    return false;
}

/*
  stop sdcard interface (for reboot)
 */
void sdcard_stop(void)
{
    // unmount
    f_mount(nullptr, SDCARD_VOLUME, 0);
    if (sdcard_running) {
        /* Zephyr has no disk_access_deinit(); the driver re-acquires on the
           next disk_access_init(). Clearing the flag is what makes a later
           sdcard_retry() re-run the whole init sequence. */
        sdcard_running = false;
    }
}

bool sdcard_retry(void)
{
    if (!sdcard_running) {
        if (sdcard_init()) {
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
            // create APM directory
            AP::FS().mkdir("/APM");
#endif
        }
    }
    return sdcard_running;
}

#endif  /* defined(HAL_OS_FATFS_IO) && HAL_OS_FATFS_IO */
