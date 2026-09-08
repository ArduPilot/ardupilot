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
    for (uint8_t i=0; i<tries; i++) {
        /* Explicit, though f_mount would trigger it via zfs_diskio.c's
           disk_initialize(). Done separately so a card-absent failure is
           distinguishable from an unformatted-card failure. Mirrors ChibiOS
           calling sdcConnect() before f_mount(). */
        if (disk_access_init(ZEPHYR_DISK_NAME) != 0) {
            continue;
        }
        /* trailing 1 = mount now rather than lazily on first access, so a
           missing or unformatted card is reported here instead of at the first
           log write */
        if (f_mount(&SDC_FS, SDCARD_VOLUME, 1) != FR_OK) {
            continue;
        }
        printf("Successfully mounted SDCard\n");

        sdcard_running = true;
        return true;
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
