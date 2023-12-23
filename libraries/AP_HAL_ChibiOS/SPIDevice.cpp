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
 */

#include <hal.h>
#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_InternalError/AP_InternalError.h>
#include "Util.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>
#include "hwdef/common/stm32_util.h"

#if HAL_USE_SPI == TRUE

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

// SPI mode numbers
#if defined(STM32H7)
#define SPIDEV_MODE0    0
#define SPIDEV_MODE1    SPI_CFG2_CPHA
#define SPIDEV_MODE2    SPI_CFG2_CPOL
#define SPIDEV_MODE3    SPI_CFG2_CPOL | SPI_CFG2_CPHA

#define SPI1_CLOCK  STM32_SPI1CLK
#define SPI2_CLOCK  STM32_SPI2CLK
#define SPI3_CLOCK  STM32_SPI3CLK
#define SPI4_CLOCK  STM32_SPI4CLK
#define SPI5_CLOCK  STM32_SPI5CLK
#define SPI6_CLOCK  STM32_SPI6CLK

#else // F4 and F7
#define SPIDEV_MODE0    0
#define SPIDEV_MODE1    SPI_CR1_CPHA
#define SPIDEV_MODE2    SPI_CR1_CPOL
#define SPIDEV_MODE3    SPI_CR1_CPOL | SPI_CR1_CPHA

#define SPI1_CLOCK  STM32_PCLK2
#define SPI2_CLOCK  STM32_PCLK1
#define SPI3_CLOCK  STM32_PCLK1
#define SPI4_CLOCK  STM32_PCLK2
#define SPI5_CLOCK  STM32_PCLK2
#define SPI6_CLOCK  STM32_PCLK2
#endif

// expected bus clock speeds
static const uint32_t bus_clocks[6] = {
    SPI1_CLOCK, SPI2_CLOCK, SPI3_CLOCK, SPI4_CLOCK, SPI5_CLOCK, SPI6_CLOCK
};

static const struct SPIDriverInfo {
    SPIDriver *driver;
    uint8_t busid; // used for device IDs in parameters
    uint8_t dma_channel_rx;
    uint8_t dma_channel_tx;
    ioline_t sck_line;
} spi_devices[] = { HAL_SPI_BUS_LIST };

// device list comes from hwdef.dat
ChibiOS::SPIDesc SPIDeviceManager::device_table[] = { HAL_SPI_DEVICE_LIST };

SPIBus::SPIBus(uint8_t _bus) :
    DeviceBus(APM_SPI_PRIORITY),
    bus(_bus)
{
    chMtxObjectInit(&dma_lock);

    // allow for sharing of DMA channels with other peripherals
    dma_handle = new Shared_DMA(spi_devices[bus].dma_channel_rx,
                                spi_devices[bus].dma_channel_tx,
                                FUNCTOR_BIND_MEMBER(&SPIBus::dma_allocate, void, Shared_DMA *),
                                FUNCTOR_BIND_MEMBER(&SPIBus::dma_deallocate, void, Shared_DMA *));

    // remember the SCK line for stop_peripheral()/start_peripheral()
#if HAL_SPI_SCK_SAVE_RESTORE
    sck_mode = palReadLineMode(spi_devices[bus].sck_line);
#endif
}

/*
  allocate DMA channel
 */
void SPIBus::dma_allocate(Shared_DMA *ctx)
{
    // nothing to do as we call spiStart() on each transaction
}

/*
  deallocate DMA channel
 */
void SPIBus::dma_deallocate(Shared_DMA *ctx)
{
    chMtxLock(&dma_lock);
    // another non-SPI peripheral wants one of our DMA channels
    stop_peripheral();
    chMtxUnlock(&dma_lock);
}


SPIDevice::SPIDevice(SPIBus &_bus, SPIDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{
    set_device_bus(spi_devices[_bus.bus].busid);
    set_device_address(_device_desc.device);
    freq_flag_low = derive_freq_flag(device_desc.lowspeed);
    freq_flag_high = derive_freq_flag(device_desc.highspeed);

    set_speed(AP_HAL::Device::SPEED_LOW);

    asprintf(&pname, "SPI:%s:%u:%u",
             device_desc.name,
             (unsigned)bus.bus, (unsigned)device_desc.device);
    AP_HAL::SPIDevice::setup_bankselect_callback(device_desc.bank_select_cb);
    AP_HAL::SPIDevice::set_register_rw_callback(device_desc.register_rw_cb);
    //printf("SPI device %s on %u:%u at speed %u mode %u\n",
    //       device_desc.name,
    //       (unsigned)bus.bus, (unsigned)device_desc.device,
    //       (unsigned)frequency, (unsigned)device_desc.mode);
}

SPIDevice::~SPIDevice()
{
    //printf("SPI device %s on %u:%u closed\n", device_desc.name,
    //       (unsigned)bus.bus, (unsigned)device_desc.device);
    free(pname);
}

SPIDriver * SPIDevice::get_driver() {
	return spi_devices[device_desc.bus].driver;
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        freq_flag = freq_flag_high;
        break;
    case AP_HAL::Device::SPEED_LOW:
        freq_flag = freq_flag_low;
        break;
    }
    return true;
}

/*
  setup a bus slowdown factor for high speed mode
 */
void SPIDevice::set_slowdown(uint8_t slowdown)
{
    slowdown = constrain_int16(slowdown+1, 1, 32);
    freq_flag_high = derive_freq_flag(device_desc.highspeed / slowdown);
}

/*
  low level transfer function
 */
bool SPIDevice::do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    bool old_cs_forced = cs_forced;

    if (!set_chip_select(true)) {
        return false;
    }

    bool ret = true;

#if defined(HAL_SPI_USE_POLLED)
    for (uint32_t i=0; i<len; i++) {
        const uint8_t b = spiPolledExchange(spi_devices[device_desc.bus].driver, send?send[i]:0);
        if (recv) {
            recv[i] = b;
        }
    }
#else
    if (!bus.bouncebuffer_setup(send, len, recv, len)) {
        set_chip_select(old_cs_forced);
        return false;
    }
    osalSysLock();
    hal.util->persistent_data.spi_count++;
    if (send == nullptr) {
        spiStartReceiveI(spi_devices[device_desc.bus].driver, len, recv);
    } else if (recv == nullptr) {
        spiStartSendI(spi_devices[device_desc.bus].driver, len, send);
    } else {
        spiStartExchangeI(spi_devices[device_desc.bus].driver, len, send, recv);
    }
    // we allow SPI transfers to take a maximum of 20ms plus 32us per
    // byte. This covers all use cases in ArduPilot. We don't ever
    // expect this timeout to trigger unless there is a severe MCU
    // error
    const uint32_t timeout_us = 20000U + len * 32U;
    msg_t msg = osalThreadSuspendTimeoutS(&spi_devices[device_desc.bus].driver->thread, TIME_US2I(timeout_us));
    osalSysUnlock();
    if (msg == MSG_TIMEOUT) {
        ret = false;
        if (!hal.scheduler->in_expected_delay()) {
            INTERNAL_ERROR(AP_InternalError::error_t::spi_fail);
        }
        spiAbort(spi_devices[device_desc.bus].driver);
    }
    bus.bouncebuffer_finish(send, recv, len);
#endif
    set_chip_select(old_cs_forced);
    return ret;
}

/*
  this pulses the clock for n bytes. The data is ignored.
 */
bool SPIDevice::clock_pulse(uint32_t n)
{
    msg_t msg;
    const uint32_t timeout_us = 20000U + n * 32U;
    if (!cs_forced) {
        //special mode to init sdcard without cs asserted
        bus.semaphore.take_blocking();
        acquire_bus(true, true);
        osalSysLock();
        spiStartIgnoreI(spi_devices[device_desc.bus].driver, n);
        msg = osalThreadSuspendTimeoutS(&spi_devices[device_desc.bus].driver->thread, TIME_US2I(timeout_us));
        osalSysUnlock();
        if (msg == MSG_TIMEOUT) {
            spiAbort(spi_devices[device_desc.bus].driver);
        }
        acquire_bus(false, true);
        bus.semaphore.give();
    } else {
        if (!bus.semaphore.check_owner()) {
            return false;
        }
        osalSysLock();
        spiStartIgnoreI(spi_devices[device_desc.bus].driver, n);
        msg = osalThreadSuspendTimeoutS(&spi_devices[device_desc.bus].driver->thread, TIME_US2I(timeout_us));
        osalSysUnlock();
        if (msg == MSG_TIMEOUT) {
            spiAbort(spi_devices[device_desc.bus].driver);
        }
    }
    return msg != MSG_TIMEOUT;
}

uint32_t SPIDevice::derive_freq_flag_bus(uint8_t busid, uint32_t _frequency)
{
    uint32_t spi_clock_freq = SPI1_CLOCK;
    if (busid > 0 && uint8_t(busid-1) < ARRAY_SIZE(bus_clocks)) {
        spi_clock_freq = bus_clocks[busid-1] / 2;
    }

    // find first divisor that brings us below the desired SPI clock
    uint32_t i = 0;
    while (spi_clock_freq > _frequency && i<7) {
        spi_clock_freq >>= 1;
        i++;
    }

    // assuming the bitrate bits are consecutive in the CR1 register,
    // we can just multiply by BR_0 to get the right bits for the desired
    // scaling
#if defined(STM32H7)
    return (i * SPI_CFG1_MBR_0) | SPI_CFG1_DSIZE_VALUE(7); // 8 bit transfers
#else
    return i * SPI_CR1_BR_0;
#endif
}

uint32_t SPIDevice::derive_freq_flag(uint32_t _frequency)
{
    uint8_t busid = spi_devices[device_desc.bus].busid;
    return derive_freq_flag_bus(busid, _frequency);
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!bus.semaphore.check_owner()) {
        return false;
    }
    if ((send_len == recv_len && send == recv) || !send || !recv) {
        // simplest cases, needed for DMA
        return do_transfer(send, recv, recv_len?recv_len:send_len);
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    bool ret = do_transfer(buf, buf, send_len+recv_len);
    if (ret && recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return ret;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    if (!bus.semaphore.check_owner()) {
        return false;
    }
    uint8_t buf[len];
    memcpy(buf, send, len);
    bool ret = do_transfer(buf, buf, len);
    if (ret) {
        memcpy(recv, buf, len);
    }
    return ret;
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.semaphore;
}


AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

/*
  stop the SPI peripheral and set the SCK line as a GPIO to prevent the clock
  line floating while we are waiting for the next spiStart()
 */
void SPIBus::stop_peripheral(void)
{
    if (!spi_started) {
        return;
    }
    const auto &sbus = spi_devices[bus];
#if HAL_SPI_SCK_SAVE_RESTORE
    if (spi_mode == SPIDEV_MODE0 || spi_mode == SPIDEV_MODE1) {
        // Clock polarity is 0, so we need to set the clock line low before spi reset
        palClearLine(sbus.sck_line);
    } else {
        // Clock polarity is 1, so we need to set the clock line high before spi reset
        palSetLine(sbus.sck_line);
    }
    palSetLineMode(sbus.sck_line, PAL_MODE_OUTPUT_PUSHPULL);
#endif
    spiStop(sbus.driver);
    spi_started = false;
}

/*
  start the SPI peripheral and restore the IO mode of the SCK line
 */
void SPIBus::start_peripheral(void)
{
    if (spi_started) {
        return;
    }

    /* start driver and setup transfer parameters */
    spiStart(spi_devices[bus].driver, &spicfg);
#if HAL_SPI_SCK_SAVE_RESTORE
    // restore sck pin mode from stop_peripheral()
    palSetLineMode(spi_devices[bus].sck_line, sck_mode);
#endif
    spi_started = true;
}

/*
 used to acquire bus and (optionally) assert cs
*/
bool SPIDevice::acquire_bus(bool set, bool skip_cs)
{
    if (!bus.semaphore.check_owner()) {
        return false;
    }
    if (set && cs_forced) {
        return true;
    }
    if (!set && !cs_forced) {
        return false;
    }
    if (!set && cs_forced) {
        if(!skip_cs) {
            spiUnselectI(spi_devices[device_desc.bus].driver);          /* Slave Select de-assertion.       */
        }
        spiReleaseBus(spi_devices[device_desc.bus].driver);              /* Ownership release.               */
        cs_forced = false;
        bus.dma_handle->unlock();
    } else {
        bus.dma_handle->lock();
        spiAcquireBus(spi_devices[device_desc.bus].driver);              /* Acquire ownership of the bus.    */
        bus.spicfg.ssport = PAL_PORT(device_desc.pal_line);
        bus.spicfg.sspad = PAL_PAD(device_desc.pal_line);
        bus.spicfg.end_cb = nullptr;
#if defined(STM32H7)
        bus.spicfg.cfg1 = freq_flag;
        bus.spicfg.cfg2 = device_desc.mode;
        if (bus.spicfg.dummytx == nullptr) {
            bus.spicfg.dummytx = (uint32_t *)malloc_dma(4);
            memset(bus.spicfg.dummytx, 0xFF, 4);
        }
        if (bus.spicfg.dummyrx == nullptr) {
            bus.spicfg.dummyrx = (uint32_t *)malloc_dma(4);
        }
#else
        bus.spicfg.cr1 = (uint16_t)(freq_flag | device_desc.mode);
        bus.spicfg.cr2 = 0;
#endif
        bus.spi_mode = device_desc.mode;
        bus.stop_peripheral();
        bus.start_peripheral();
        if(!skip_cs) {
            spiSelectI(spi_devices[device_desc.bus].driver);                /* Slave Select assertion.          */
        }
        cs_forced = true;
    }
    return true;
}

/*
  allow for control of SPI chip select pin
 */
bool SPIDevice::set_chip_select(bool set) {
    return acquire_bus(set, false);
}

/*
  return a SPIDevice given a string device name
 */
AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_table)) {
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }

    SPIDesc &desc = device_table[i];

    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }
    if (busp == nullptr) {
        // create a new one
        busp = new SPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;

        buses = busp;
    }

    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*busp, desc));
}

void SPIDeviceManager::set_register_rw_callback(const char* name, AP_HAL::Device::RegisterRWCb cb)
{
    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_table)) {
        return;
    }

    device_table[i].register_rw_cb = cb;

}


#ifdef HAL_SPI_CHECK_CLOCK_FREQ

/*
  test clock frequencies. This measures the actual SPI clock
  frequencies on all configured SPI buses. Used during board bringup
  to validate clock configuration
 */
void SPIDevice::test_clock_freq(void)
{
    // delay for USB to come up
    DEV_PRINTF("Waiting for USB\n");
    for (uint8_t i=0; i<3; i++) {
        hal.scheduler->delay(1000);
        DEV_PRINTF("Waiting %u\n", (unsigned)AP_HAL::millis());
    }
    DEV_PRINTF("CLOCKS=\n");
    for (uint8_t i=0; i<ARRAY_SIZE(bus_clocks); i++) {
        DEV_PRINTF("%u:%u ", unsigned(i+1), unsigned(bus_clocks[i]));
    }
    DEV_PRINTF("\n");

    // we will send 1024 bytes without any CS asserted and measure the
    // time it takes to do the transfer
    uint16_t len = 1024;
    uint8_t *buf1 = (uint8_t *)hal.util->malloc_type(len, AP_HAL::Util::MEM_DMA_SAFE);
    uint8_t *buf2 = (uint8_t *)hal.util->malloc_type(len, AP_HAL::Util::MEM_DMA_SAFE);
    for (uint8_t i=0; i<ARRAY_SIZE(spi_devices); i++) {
        SPIConfig spicfg {};
        const uint32_t target_freq = 2000000UL;
        // use a clock divisor of 256 for maximum resolution
#if defined(STM32H7)
        spicfg.cfg1 = derive_freq_flag_bus(spi_devices[i].busid, target_freq);
#else
        spicfg.cr1 = derive_freq_flag_bus(spi_devices[i].busid, target_freq);
#endif
        spiAcquireBus(spi_devices[i].driver);
        spiStart(spi_devices[i].driver, &spicfg);
        uint32_t t0 = AP_HAL::micros();
        spiStartExchange(spi_devices[i].driver, len, buf1, buf2);
        chSysLock();
        msg_t msg = osalThreadSuspendTimeoutS(&spi_devices[i].driver->thread, chTimeMS2I(100));
        chSysUnlock();
        if (msg == MSG_TIMEOUT) {
            spiAbort(spi_devices[i].driver);
            DEV_PRINTF("SPI[%u] FAIL %p %p\n", spi_devices[i].busid, buf1, buf2);
            spiStop(spi_devices[i].driver);
            spiReleaseBus(spi_devices[i].driver);
            continue;
        }
        uint32_t t1 = AP_HAL::micros();
        spiStop(spi_devices[i].driver);
        spiReleaseBus(spi_devices[i].driver);
        DEV_PRINTF("SPI[%u] clock=%u\n", unsigned(spi_devices[i].busid), unsigned(1000000ULL * len * 8ULL / uint64_t(t1 - t0)));
    }
    hal.util->free_type(buf1, len, AP_HAL::Util::MEM_DMA_SAFE);
    hal.util->free_type(buf2, len, AP_HAL::Util::MEM_DMA_SAFE);
}
#endif // HAL_SPI_CHECK_CLOCK_FREQ

#endif // HAL_USE_SPI
