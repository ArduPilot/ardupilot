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
#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>
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
} spi_devices[] = { HAL_SPI_BUS_LIST };

#define MHZ (1000U*1000U)
#define KHZ (1000U)
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
    if (spi_started) {
        spiStop(spi_devices[bus].driver);
        spi_started = false;
    }
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
void SPIDevice::do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    bool old_cs_forced = cs_forced;

    if (!set_chip_select(true)) {
        return;
    }

    bus.bouncebuffer_setup(send, len, recv, len);

    if (send == nullptr) {
        spiReceive(spi_devices[device_desc.bus].driver, len, recv);
    } else if (recv == nullptr) {
        spiSend(spi_devices[device_desc.bus].driver, len, send);
    } else {
        spiExchange(spi_devices[device_desc.bus].driver, len, send, recv);
    }

    bus.bouncebuffer_finish(send, recv, len);

    set_chip_select(old_cs_forced);
}

bool SPIDevice::clock_pulse(uint32_t n)
{
    if (!cs_forced) {
        //special mode to init sdcard without cs asserted
        bus.semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER);
        acquire_bus(true, true);
        spiIgnore(spi_devices[device_desc.bus].driver, n);
        acquire_bus(false, true);
        bus.semaphore.give();
    } else {
        bus.semaphore.assert_owner();
        spiIgnore(spi_devices[device_desc.bus].driver, n);
    }
    return true;
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
        hal.console->printf("SPI: not owner of 0x%x\n", unsigned(get_bus_id()));
        return false;
    }
    if ((send_len == recv_len && send == recv) || !send || !recv) {
        // simplest cases, needed for DMA
        do_transfer(send, recv, recv_len?recv_len:send_len);
        return true;
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    do_transfer(buf, buf, send_len+recv_len);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    bus.semaphore.assert_owner();
    uint8_t buf[len];
    memcpy(buf, send, len);
    do_transfer(buf, buf, len);
    memcpy(recv, buf, len);
    return true;
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
 used to acquire bus and (optionally) assert cs
*/
bool SPIDevice::acquire_bus(bool set, bool skip_cs)
{
    bus.semaphore.assert_owner();
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
        bus.spicfg.end_cb = nullptr;
        bus.spicfg.ssport = PAL_PORT(device_desc.pal_line);
        bus.spicfg.sspad = PAL_PAD(device_desc.pal_line);
#if defined(STM32H7)
        bus.spicfg.cfg1 = freq_flag;
        bus.spicfg.cfg2 = device_desc.mode;
#else
        bus.spicfg.cr1 = (uint16_t)(freq_flag | device_desc.mode);
        bus.spicfg.cr2 = 0;
#endif
        if (bus.spi_started) {
            spiStop(spi_devices[device_desc.bus].driver);
            bus.spi_started = false;
        }
        spiStart(spi_devices[device_desc.bus].driver, &bus.spicfg);        /* Setup transfer parameters.       */
        bus.spi_started = true;
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

#ifdef HAL_SPI_CHECK_CLOCK_FREQ

/*
  test clock frequencies. This measures the actual SPI clock
  frequencies on all configured SPI buses. Used during board bringup
  to validate clock configuration
 */
void SPIDevice::test_clock_freq(void)
{
    // delay for USB to come up
    hal.console->printf("Waiting for USB\n");
    for (uint8_t i=0; i<3; i++) {
        hal.scheduler->delay(1000);
        hal.console->printf("Waiting %u\n", AP_HAL::millis());
    }
    hal.console->printf("SPI1_CLOCK=%u SPI2_CLOCK=%u SPI3_CLOCK=%u SPI4_CLOCK=%u\n",
                        SPI1_CLOCK, SPI2_CLOCK, SPI3_CLOCK, SPI4_CLOCK);

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
        msg_t msg = osalThreadSuspendTimeoutS(&spi_devices[i].driver->thread, TIME_MS2I(100));
        chSysUnlock();
        if (msg == MSG_TIMEOUT) {
            spiAbort(spi_devices[i].driver);
            hal.console->printf("SPI[%u] FAIL %p %p\n", spi_devices[i].busid, buf1, buf2);
            spiStop(spi_devices[i].driver);
            spiReleaseBus(spi_devices[i].driver);
            continue;
        }
        uint32_t t1 = AP_HAL::micros();
        spiStop(spi_devices[i].driver);
        spiReleaseBus(spi_devices[i].driver);
        hal.console->printf("SPI[%u] clock=%u\n", spi_devices[i].busid, unsigned(1000000ULL * len * 8ULL / uint64_t(t1 - t0)));
    }
    hal.util->free_type(buf1, len, AP_HAL::Util::MEM_DMA_SAFE);
    hal.util->free_type(buf2, len, AP_HAL::Util::MEM_DMA_SAFE);
}
#endif // HAL_SPI_CHECK_CLOCK_FREQ

#endif // HAL_USE_SPI
