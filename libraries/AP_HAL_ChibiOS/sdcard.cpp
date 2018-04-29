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

#ifdef USE_POSIX
static FATFS SDC_FS; // FATFS object
#endif

#if HAL_USE_MMC_SPI
MMCDriver MMCD1;
static AP_HAL::OwnPtr<AP_HAL::SPIDevice> device;
static MMCConfig mmcconfig;
static SPIConfig lowspeed;
static SPIConfig highspeed;
#endif

void sdcard_init() {

#ifdef USE_POSIX
#if HAL_USE_SDC
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
#endif
#if HAL_USE_MMC_SPI
    device = AP_HAL::get_HAL().spi->get_device("sdcard");

    mmcObjectInit(&MMCD1);

    mmcconfig.spip =
            static_cast<ChibiOS::SPIDevice*>(device.get())->get_driver();
    mmcconfig.hscfg = &highspeed;
    mmcconfig.lscfg = &lowspeed;

    mmcStart(&MMCD1, &mmcconfig);

    if (mmcConnect(&MMCD1) == HAL_FAILED) {
        printf("Err: Failed to initialize SDCARD_SPI!\n");
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

#if HAL_USE_MMC_SPI

void spiStartHook(SPIDriver *spip, const SPIConfig *config) {
    device->set_speed(
            config == &lowspeed ?
                    AP_HAL::Device::SPEED_LOW : AP_HAL::Device::SPEED_HIGH);
}

void spiStopHook(SPIDriver *spip) {
}

void spiSelectHook(SPIDriver *spip) {
    static_cast<ChibiOS::SPIDevice*>(device.get())->spi_select();
}

void spiUnselectHook(SPIDriver *spip) {
    static_cast<ChibiOS::SPIDevice*>(device.get())->spi_unselect();
}

void spiIgnoreHook(SPIDriver *spip, size_t n) {
    static_cast<ChibiOS::SPIDevice*>(device.get())->spi_ignore(n);
}

void spiSendHook(SPIDriver *spip, size_t n, const void *txbuf) {
    static_cast<ChibiOS::SPIDevice*>(device.get())->spi_send(n, txbuf);
}

void spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf) {
    static_cast<ChibiOS::SPIDevice*>(device.get())->spi_receive(n, rxbuf);
}

#endif

