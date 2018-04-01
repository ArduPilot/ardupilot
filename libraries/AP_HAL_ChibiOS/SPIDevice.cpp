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
#include <AP_HAL/utility/OwnPtr.h>
#include "Util.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

#if HAL_USE_SPI == TRUE

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

// SPI mode numbers
#define SPIDEV_MODE0    0
#define SPIDEV_MODE1    SPI_CR1_CPHA
#define SPIDEV_MODE2    SPI_CR1_CPOL
#define SPIDEV_MODE3    SPI_CR1_CPOL | SPI_CR1_CPHA

#define SPI1_CLOCK  STM32_PCLK2
#define SPI2_CLOCK  STM32_PCLK1
#define SPI3_CLOCK  STM32_PCLK1
#define SPI4_CLOCK  STM32_PCLK2

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
    // allow for sharing of DMA channels with other peripherals
    dma_handle = new Shared_DMA(spi_devices[bus].dma_channel_rx,
                                spi_devices[bus].dma_channel_tx,
                                FUNCTOR_BIND_MEMBER(&SPIBus::dma_allocate, void),
                                FUNCTOR_BIND_MEMBER(&SPIBus::dma_deallocate, void));
        
}

/*
  allocate DMA channel
 */
void SPIBus::dma_allocate(void)
{
    // nothing to do as we call spiStart() on each transaction
}

/*
  deallocate DMA channel
 */
void SPIBus::dma_deallocate(void)
{
    // another non-SPI peripheral wants one of our DMA channels
    if (spi_started) {
        spiStop(spi_devices[bus].driver);
        spi_started = false;
    }
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
  low level transfer function
 */
void SPIDevice::do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    bool old_cs_forced = cs_forced;

    if (!set_chip_select(true)) {
        return;
    }
    uint8_t *recv_buf = recv;
    const uint8_t *send_buf = send;

    bus.bouncebuffer_setup(send_buf, len, recv_buf, len);

    spiExchange(spi_devices[device_desc.bus].driver, len, send_buf, recv_buf);

    if (recv_buf != recv) {
        memcpy(recv, recv_buf, len);
    }
    
    set_chip_select(old_cs_forced);
}

uint16_t SPIDevice::derive_freq_flag(uint32_t _frequency)
{
    uint8_t i;
    uint32_t spi_clock_freq;
    switch(device_desc.bus) {
        case 0:
            spi_clock_freq = SPI1_CLOCK;
            break;
        case 1:
            spi_clock_freq = SPI2_CLOCK;
            break;
        case 2:
            spi_clock_freq = SPI4_CLOCK;
            break;
        case 3:
            spi_clock_freq = SPI4_CLOCK;
            break;
        default:
            spi_clock_freq = SPI1_CLOCK;
            break;
    }

    for(i = 0; i <= 7; i++) {
        spi_clock_freq /= 2;
        if (spi_clock_freq <= _frequency) {
            break;
        }
    }
    switch(i) {
        case 0: //PCLK DIV 2
            return 0;
        case 1: //PCLK DIV 4
            return SPI_CR1_BR_0;
        case 2: //PCLK DIV 8
            return SPI_CR1_BR_1;
        case 3: //PCLK DIV 16
            return SPI_CR1_BR_1 | SPI_CR1_BR_0;
        case 4: //PCLK DIV 32
            return SPI_CR1_BR_2;
        case 5: //PCLK DIV 64
            return SPI_CR1_BR_2 | SPI_CR1_BR_0;
        case 6: //PCLK DIV 128
            return SPI_CR1_BR_2 | SPI_CR1_BR_1;
        case 7: //PCLK DIV 256
        default:
            return SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    }
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!bus.semaphore.check_owner()) {
        hal.console->printf("SPI: not owner of 0x%x\n", unsigned(get_bus_id()));
        return false;
    }
    if (send_len == recv_len && send == recv) {
        // simplest cases, needed for DMA
        do_transfer(send, recv, recv_len);
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
  allow for control of SPI chip select pin
 */
bool SPIDevice::set_chip_select(bool set)
{
    bus.semaphore.assert_owner();
    if (set && cs_forced) {
        return true;
    }
    if (!set && !cs_forced) {
        return false;
    }
    if (!set && cs_forced) {
        spiUnselectI(spi_devices[device_desc.bus].driver);          /* Slave Select de-assertion.       */
        spiReleaseBus(spi_devices[device_desc.bus].driver);              /* Ownership release.               */
        cs_forced = false;
        bus.dma_handle->unlock();
    } else {
        bus.dma_handle->lock();
        spiAcquireBus(spi_devices[device_desc.bus].driver);              /* Acquire ownership of the bus.    */
        bus.spicfg.end_cb = nullptr;
        bus.spicfg.ssport = PAL_PORT(device_desc.pal_line);
        bus.spicfg.sspad = PAL_PAD(device_desc.pal_line);
        bus.spicfg.cr1 = (uint16_t)(freq_flag | device_desc.mode);
        bus.spicfg.cr2 = 0;
        if (bus.spi_started) {
            spiStop(spi_devices[device_desc.bus].driver);
            bus.spi_started = false;
        }
        spiStart(spi_devices[device_desc.bus].driver, &bus.spicfg);        /* Setup transfer parameters.       */
        bus.spi_started = true;
        spiSelectI(spi_devices[device_desc.bus].driver);                /* Slave Select assertion.          */
        cs_forced = true;
    }
    return true;
}

/*
  return a SPIDevice given a string device name
 */
AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE_SIMPLE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE_SIMPLE(device_table)) {
        printf("SPI: Invalid device name: %s\n", name);
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

#endif
