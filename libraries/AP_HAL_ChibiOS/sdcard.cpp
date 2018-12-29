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
 */

#include "SPIDevice.h"
#include "sdcard.h"
#include "hwdef/common/spi_hook.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

#ifdef USE_POSIX
static FATFS SDC_FS; // FATFS object
static bool sdcard_running;
static ChibiOS::Semaphore sem;
#endif

#if HAL_USE_SDC
static SDCConfig sdcconfig = {
  NULL,
  SDC_MODE_4BIT,
  0
};
#elif HAL_USE_MMC_SPI
MMCDriver MMCD1;
static AP_HAL::OwnPtr<AP_HAL::SPIDevice> device;
static MMCConfig mmcconfig;
static SPIConfig lowspeed;
static SPIConfig highspeed;
#endif

/*
  initialise microSD card if avaialble. This is called during
  AP_BoardConfig initialisation. The parameter BRD_SD_SLOWDOWN
  controls a scaling factor on the microSD clock
 */
bool sdcard_init()
{
#ifdef USE_POSIX
    sem.take(HAL_SEMAPHORE_BLOCK_FOREVER);

    uint8_t sd_slowdown = AP_BoardConfig::get_sdcard_slowdown();
#if HAL_USE_SDC

    if (SDCD1.bouncebuffer == nullptr) {
        bouncebuffer_init(&SDCD1.bouncebuffer, 512);
    }

    if (sdcard_running) {
        sdcard_stop();
    }

    const uint8_t tries = 3;
    for (uint8_t i=0; i<tries; i++) {
        sdcconfig.slowdown = sd_slowdown;
        sdcStart(&SDCD1, &sdcconfig);
        if(sdcConnect(&SDCD1) == HAL_FAILED) {
            sdcStop(&SDCD1);
            continue;
        }
        if (f_mount(&SDC_FS, "/", 1) != FR_OK) {
            sdcDisconnect(&SDCD1);
            sdcStop(&SDCD1);
            continue;
        }
        printf("Successfully mounted SDCard (slowdown=%u)\n", (unsigned)sd_slowdown);

        // Create APM Directory if needed
        mkdir("/APM", 0777);
        mkdir("/APM/LOGS", 0777);
        sdcard_running = true;
        sem.give();
        return true;
    }
#elif HAL_USE_MMC_SPI
    if (sdcard_running) {
        sdcard_stop();
    }

    sdcard_running = true;

    device = AP_HAL::get_HAL().spi->get_device("sdcard");
    if (!device) {
        printf("No sdcard SPI device found\n");
        sem.give();
        return false;
    }
    device->set_slowdown(sd_slowdown);
    
    mmcObjectInit(&MMCD1);

    mmcconfig.spip =
            static_cast<ChibiOS::SPIDevice*>(device.get())->get_driver();
    mmcconfig.hscfg = &highspeed;
    mmcconfig.lscfg = &lowspeed;

    /*
      try up to 3 times to init microSD interface
     */
    const uint8_t tries = 3;
    for (uint8_t i=0; i<tries; i++) {
        mmcStart(&MMCD1, &mmcconfig);

        if (mmcConnect(&MMCD1) == HAL_FAILED) {
            mmcStop(&MMCD1);
            continue;
        }
        if (f_mount(&SDC_FS, "/", 1) != FR_OK) {
            mmcDisconnect(&MMCD1);
            mmcStop(&MMCD1);
            continue;
        }
        printf("Successfully mounted SDCard (slowdown=%u)\n", (unsigned)sd_slowdown);

        // Create APM Directory if needed
        mkdir("/APM", 0777);
        mkdir("/APM/LOGS", 0777);
        sem.give();
        return true;
    }
    sdcard_running = false;
#endif
    sem.give();
#endif
    return false;
}

/*
  stop sdcard interface (for reboot)
 */
void sdcard_stop(void)
{
#ifdef USE_POSIX
    // unmount
    f_mount(nullptr, "/", 1);
#endif
#if HAL_USE_SDC
    if (sdcard_running) {
        sdcDisconnect(&SDCD1);
        sdcStop(&SDCD1);
        sdcard_running = false;
    }
#elif HAL_USE_MMC_SPI
    if (sdcard_running) {
        mmcDisconnect(&MMCD1);
        mmcStop(&MMCD1);
        sdcard_running = false;
    }
#endif
}

void sdcard_retry(void)
{
    if (!sdcard_running) {
        sdcard_init();
    }
}

#if HAL_USE_MMC_SPI

/*
  hooks to allow hal_mmc_spi.c to work with HAL_ChibiOS SPI
  layer. This provides bounce buffers for DMA, DMA channel sharing and
  bus locking
 */

void spiStartHook(SPIDriver *spip, const SPIConfig *config)
{
    device->set_speed(config == &lowspeed ?
        AP_HAL::Device::SPEED_LOW : AP_HAL::Device::SPEED_HIGH);
}

void spiStopHook(SPIDriver *spip)
{
}

void spiSelectHook(SPIDriver *spip)
{
    if (sdcard_running) {
        device->get_semaphore()->take_blocking();
        device->set_chip_select(true);
    }
}

void spiUnselectHook(SPIDriver *spip)
{
    if (sdcard_running) {
        device->set_chip_select(false);
        device->get_semaphore()->give();
    }
}

void spiIgnoreHook(SPIDriver *spip, size_t n)
{
    if (sdcard_running) {
        device->clock_pulse(n);
    }
}

void spiSendHook(SPIDriver *spip, size_t n, const void *txbuf)
{
    if (sdcard_running) {
        device->transfer((const uint8_t *)txbuf, n, nullptr, 0);
    }
}

void spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf)
{
    if (sdcard_running) {
        device->transfer(nullptr, 0, (uint8_t *)rxbuf, n);
    }
}

#endif

