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

extern const AP_HAL::HAL& hal;

#ifdef USE_POSIX
static FATFS SDC_FS; // FATFS object
static bool sdcard_init_done;
#endif

#if HAL_USE_MMC_SPI
MMCDriver MMCD1;
static AP_HAL::OwnPtr<AP_HAL::SPIDevice> device;
static MMCConfig mmcconfig;
static SPIConfig lowspeed;
static SPIConfig highspeed;
static bool sdcard_running;
#endif

/*
  initialise microSD card if avaialble
 */
void sdcard_init()
{
#ifdef USE_POSIX
    if (sdcard_init_done) {
        return;
    }
    sdcard_init_done = true;
#if HAL_USE_SDC

    bouncebuffer_init(&SDCD1.bouncebuffer, 512);
    
    sdcStart(&SDCD1, NULL);

    if(sdcConnect(&SDCD1) == HAL_FAILED) {
        printf("Err: Failed to initialize SDIO!\n");
    } else {
        if (f_mount(&SDC_FS, "/", 1) != FR_OK) {
            printf("Err: Failed to mount SD Card!\n");
            sdcDisconnect(&SDCD1);
        } else {
            printf("Successfully mounted SDCard..\n");
        }
        //Create APM Directory
        mkdir("/APM", 0777);
    }
#elif HAL_USE_MMC_SPI
    device = AP_HAL::get_HAL().spi->get_device("sdcard");
    if (!device) {
        printf("No sdcard SPI device found\n");
        return;
    }
    
    sdcard_running = true;

    mmcObjectInit(&MMCD1);

    mmcconfig.spip =
            static_cast<ChibiOS::SPIDevice*>(device.get())->get_driver();
    mmcconfig.hscfg = &highspeed;
    mmcconfig.lscfg = &lowspeed;

    /*
      try up to 3 times to init microSD interface
     */
    const uint8_t tries = 3;
    bool start_ok = false;
    for (uint8_t i=0; i<tries; i++) {
        mmcStart(&MMCD1, &mmcconfig);

        if (mmcConnect(&MMCD1) != HAL_FAILED) {
            start_ok = true;
            break;
        }
        mmcStop(&MMCD1);
        hal.scheduler->delay(100);
    }

    if (!start_ok) {
        printf("Err: Failed to initialize SDCARD_SPI!\n");
        sdcard_running = false;
    } else {
        if (f_mount(&SDC_FS, "/", 1) != FR_OK) {
            printf("Err: Failed to mount SD Card!\n");
            mmcDisconnect(&MMCD1);
        } else {
            printf("Successfully mounted SDCard..\n");
        }
        //Create APM Directory
        mkdir("/APM", 0777);
    }
#endif
#endif
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
#if HAL_USE_MMC_SPI
    if (sdcard_running) {
        mmcDisconnect(&MMCD1);
        mmcStop(&MMCD1);
        sdcard_running = false;
    }
#endif
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

