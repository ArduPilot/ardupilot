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

#include <hal.h>
#include "SPIDevice.h"
#include "sdcard.h"
#include "bouncebuffer.h"
#include "CrashDump.h"
#include "hwdef/common/spi_hook.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include "stm32_util.h"

extern const AP_HAL::HAL& hal;

#if HAL_USE_FATFS
static FATFS SDC_FS; // FATFS object
#ifndef HAL_BOOTLOADER_BUILD
static HAL_Semaphore sem;
#endif
static bool sdcard_running;
static uint32_t sdcard_last_fail_ms;
#ifndef HAL_SDCARD_RETRY_INTERVAL_MS
#define HAL_SDCARD_RETRY_INTERVAL_MS 2000U
#endif
#endif

#if HAL_USE_SDC
static SDCConfig sdcconfig = {
  SDC_MODE_4BIT,
  0
};
#elif HAL_USE_MMC_SPI
MMCDriver MMCD1;
static AP_HAL::SPIDevice *device;
static MMCConfig mmcconfig;
static SPIConfig lowspeed;
static SPIConfig highspeed;
#ifndef HAL_SDCARD_SPI_INIT_TRIES
#define HAL_SDCARD_SPI_INIT_TRIES 3U
#endif
#endif

// initialise the microSD block device without mounting its filesystem
bool sdcard_init_raw(uint8_t sd_slowdown, uint8_t tries)
{
#if HAL_USE_FATFS
#if HAL_USE_SDC

#if STM32_SDC_USE_SDMMC2 == TRUE
    auto &sdcd = SDCD2;
#else
    auto &sdcd = SDCD1;
#endif

    // local bounce buffer pointer (SDCDriver no longer carries this field)
    static struct bouncebuffer_t *sdc_bouncebuffer;

    if (sdc_bouncebuffer == nullptr) {
        // allocate 4k-32k bouncebuffer for microSD to match size in
        // AP_Logger
#if defined(STM32H7)
        bouncebuffer_init(&sdc_bouncebuffer, AP_FATFS_MAX_IO_SIZE, true);
        // allocation failure, pick a smaller size
        if (sdc_bouncebuffer->dma_buf == nullptr) {
            bouncebuffer_init(&sdc_bouncebuffer, AP_FATFS_MIN_IO_SIZE, true);
#if AP_FILESYSTEM_FATFS_ENABLED
            AP_Filesystem_FATFS::set_io_size(AP_FATFS_MIN_IO_SIZE);
#endif
        } else {
#if AP_FILESYSTEM_FATFS_ENABLED
            AP_Filesystem_FATFS::set_io_size(AP_FATFS_MAX_IO_SIZE);
#endif
        }
#else
        bouncebuffer_init(&sdc_bouncebuffer, AP_FATFS_MAX_IO_SIZE, false);
#if AP_FILESYSTEM_FATFS_ENABLED
        AP_Filesystem_FATFS::set_io_size(AP_FATFS_MAX_IO_SIZE);
#endif
#endif
        if (sdc_bouncebuffer->dma_buf == nullptr) {    // we are never going to be able to log
            sdcard_running = false;
            return false;
        }
    }

    if (sdcard_running) {
        sdcard_stop();
    }

    for (uint8_t i=0; i<tries; i++) {
        sdcconfig.slowdown = sd_slowdown;
        sdcStart(&sdcd, &sdcconfig);
        if(sdcConnect(&sdcd) == HAL_FAILED) {
            sdcStop(&sdcd);
            continue;
        }
        sdcard_running = true;
        return true;
    }
#elif HAL_USE_MMC_SPI
    if (MMCD1.buffer == nullptr) {
        // allocate 16 byte non-cacheable buffer for microSD
        MMCD1.buffer = (uint8_t*)malloc_axi_sram(MMC_BUFFER_SIZE);
    }

    if (sdcard_running) {
        sdcard_stop();
    }

    sdcard_running = true;

    if (device == nullptr) {
        device = AP_HAL::get_HAL().spi->get_device_ptr("sdcard");
        if (!device) {
            hal.console->printf("No sdcard SPI device found\n");
            sdcard_running = false;
            return false;
        }
    }
    device->set_slowdown(sd_slowdown);

    mmcObjectInit(&MMCD1, MMCD1.buffer);

    mmcconfig.spip = (static_cast<ChibiOS::SPIDevice*>(device))->get_driver();
    mmcconfig.hscfg = &highspeed;
    mmcconfig.lscfg = &lowspeed;

#if defined(RP2350) && defined(HAL_GPIO_PIN_SDCARD_CS)
    /*
     * The ChibiOS MMC-SPI driver calls spiStart/spiSelect/spiSend/spiReceive
     * directly through the ChibiOS SPI HAL, bypassing the ArduPilot SPI hooks.
     * lowspeed/highspeed must therefore be fully initialised for SPI_SELECT_MODE_PAD
     * and the RP2350 PL022 hardware (SSPCR0/SSPCPSR).
     *
     * SPI clock = CLK_PERI = CLK_SYS = 375 MHz (Laurel PLL config).
     * f_SPI = CLK_PERI / (SSPCPSR * (1 + SCR)).
     * SCR is 8-bit [15:8] in SSPCR0 (max 255); SSPCPSR must be even in [2,254].
     *
     * lowspeed  ~399 kHz: SSPCPSR=4, SCR=234 → 375e6/(4*235) = 398.9 kHz
     * highspeed ~ 25 MHz: SSPCPSR=2, SCR=7   → 375e6/(2*8)   = 23.4  MHz
     *
     * SSPCR0 layout: SCR[15:8] | CPHA[7] | CPOL[6] | FRF[5:4]=00 | DSS[3:0]=7
     * MODE0 => CPOL=0, CPHA=0 => no extra bits.
     */
    lowspeed.ssport  = PAL_PORT(HAL_GPIO_PIN_SDCARD_CS);
    lowspeed.sspad   = (uint16_t)PAL_PAD(HAL_GPIO_PIN_SDCARD_CS);
    lowspeed.SSPCR0  = (234U << 8U) | 0x07U;
    lowspeed.SSPCPSR = 4U;
    highspeed.ssport  = PAL_PORT(HAL_GPIO_PIN_SDCARD_CS);
    highspeed.sspad   = (uint16_t)PAL_PAD(HAL_GPIO_PIN_SDCARD_CS);
    highspeed.SSPCR0  = (7U << 8U) | 0x07U;
    highspeed.SSPCPSR = 2U;
#endif

    /*
      try up to 3 times to init microSD interface
     */
    const uint8_t tries = (uint8_t)HAL_SDCARD_SPI_INIT_TRIES;

#if defined(RP2350) && CH_CFG_SMP_MODE == TRUE
    /*
     * HAL_CORE_SPI1 controls which core the SPI1 bus thread runs on.
     * sdcard_init() always runs on core0. If the SPI1 bus thread (on core1)
     * has already called spiStart(SPID1) — allocating DMA on core1 — then
     * mmcConnect's spiStart would be a no-op (SPID1 already SPI_READY) but
     * the DMA IRQs would fire on core1 while the waiting thread is on core0.
     * Fix: stop SPID1 here so mmcConnect re-starts it from core0, routing
     * DMA IRQs to core0 where sdcard_init blocks.
     * Requires dmaChannelFreeI to safely free channels from the non-owning
     * core (see rp_dma.c fix).
     */
    {
        SPIDriver *spip = mmcconfig.spip;
        if (spip->state == SPI_READY) {
            spiStop(spip);
        }
    }
#endif

    for (uint8_t i=0; i<tries; i++) {
        mmcStart(&MMCD1, &mmcconfig);
        if (mmcConnect(&MMCD1) == HAL_FAILED) {
            mmcStop(&MMCD1);
            continue;
        }
        sdcard_running = true;
        return true;
    }
#endif
    sdcard_running = false;
#endif  // HAL_USE_FATFS
    return false;
}

BaseBlockDevice *sdcard_get_block_device()
{
#if HAL_USE_SDC
#if STM32_SDC_USE_SDMMC2 == TRUE
    return reinterpret_cast<BaseBlockDevice *>(&SDCD2);
#else
    return reinterpret_cast<BaseBlockDevice *>(&SDCD1);
#endif
#elif HAL_USE_MMC_SPI
    return reinterpret_cast<BaseBlockDevice *>(&MMCD1);
#else
    return nullptr;
#endif
}

bool sdcard_init()
{
#if HAL_USE_FATFS
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(sem);
    const uint8_t sd_slowdown = AP_BoardConfig::get_sdcard_slowdown();
#else
    const uint8_t sd_slowdown = 0;
#endif

    for (uint8_t i = 0; i < 3; i++) {
        if (!sdcard_init_raw(sd_slowdown, 1)) {
            continue;
        }
        if (f_mount(&SDC_FS, "/", 1) == FR_OK) {
            printf("Successfully mounted SDCard (slowdown=%u)\n", (unsigned)sd_slowdown);
            return true;
        }
        sdcard_stop();
    }
#endif
    return false;
}

/*
  stop sdcard interface (for reboot)
 */
void sdcard_stop(void)
{
#if AP_CRASHDUMP_FATFS_ENABLED && (HAL_USE_SDC || \
    (HAL_USE_MMC_SPI && CRASHDUMP_SD_SPI_SUPPORTED_MCU))
    // Do this before unmounting or disabling the peripheral clock. A fault
    // after this point must not try to use the cached sector map.
    crashdump_sd_invalidate();
#endif
#if HAL_USE_FATFS
    // unmount
    f_mount(nullptr, "/", 1);
#endif
#if HAL_USE_SDC
#if STM32_SDC_USE_SDMMC2 == TRUE
    auto &sdcd = SDCD2;
#else
    auto &sdcd = SDCD1;
#endif
    if (sdcard_running) {
        sdcDisconnect(&sdcd);
        sdcStop(&sdcd);
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

bool sdcard_retry(void)
{
#if HAL_USE_FATFS
#if AP_CRASHDUMP_FATFS_ENABLED && (HAL_USE_SDC || \
    (HAL_USE_MMC_SPI && CRASHDUMP_SD_SPI_SUPPORTED_MCU))
    const bool sdcard_was_running = sdcard_running;
#endif
    if (!sdcard_running) {
        // Avoid repeated long probe sequences when no card is present.
        // Boot paths can call retry_mount() many times in a tight loop.
        const uint32_t now_ms = AP_HAL::millis();
        if ((now_ms - sdcard_last_fail_ms) < HAL_SDCARD_RETRY_INTERVAL_MS) {
            return false;
        }
        if (sdcard_init()) {
            sdcard_last_fail_ms = 0;
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
// create APM directory without re-entering AP::FS()
// callers may already hold the FATFS backend mutex on targets where mutexes are non-recursive.
            const FRESULT res = f_mkdir("/APM");
            (void)res;
#endif
        } else {
            sdcard_last_fail_ms = now_ms;
        }
    }
#if AP_CRASHDUMP_FATFS_ENABLED && (HAL_USE_SDC || \
    (HAL_USE_MMC_SPI && CRASHDUMP_SD_SPI_SUPPORTED_MCU))
    if (sdcard_running &&
        (!sdcard_was_running || !crashdump_sd_ready())) {
        crashdump_sd_init();
    }
#endif
    return sdcard_running;
#endif
    return false;
}

#if HAL_USE_MMC_SPI

AP_HAL::SPIDevice *sdcard_get_spi_device()
{
    return device;
}

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

__RAMFUNC__ void spiAcquireBusHook(SPIDriver *spip)
{
    if (sdcard_running) {
        ChibiOS::SPIDevice *devptr = static_cast<ChibiOS::SPIDevice*>(device);
        devptr->acquire_bus(true, true);
    }
}

__RAMFUNC__ void spiReleaseBusHook(SPIDriver *spip)
{
    if (sdcard_running) {
        ChibiOS::SPIDevice *devptr = static_cast<ChibiOS::SPIDevice*>(device);
        devptr->acquire_bus(false, true);
    }
}

__RAMFUNC__ void spiSelectHook(SPIDriver *spip)
{
    if (sdcard_running) {
        device->get_semaphore()->take_blocking();
        device->set_chip_select(true);
    }
}

__RAMFUNC__ void spiUnselectHook(SPIDriver *spip)
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

__RAMFUNC__ void spiSendHook(SPIDriver *spip, size_t n, const void *txbuf)
{
    if (sdcard_running) {
        device->transfer((const uint8_t *)txbuf, n, nullptr, 0);
    }
}

__RAMFUNC__ void spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf)
{
    if (sdcard_running) {
        device->transfer(nullptr, 0, (uint8_t *)rxbuf, n);
    }
}

#endif
