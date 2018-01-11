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

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
#define SPI_BUS_FLOW  1
#define SPI_BUS_RADIO 0

#define SPIDEV_CS_FLOW             GPIOB, 12
#define SPIDEV_CS_RADIO            GPIOA, 4

#define SPIDEV_RADIO           1
#define SPIDEV_FLOW            2

#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3
#define SPI_BUS_SENSORS 0
#define SPI_BUS_RAMTRON 1
#define SPI_BUS_RADIO 1
#define SPI_BUS_EXT 2

#define SPIDEV_CS_MS5611           GPIOD, 7
#define SPIDEV_CS_EXT_MS5611       GPIOC, 14
#define SPIDEV_CS_MPU              GPIOC, 2
#define SPIDEV_CS_HMC              GPIOC, 1
#define SPIDEV_CS_EXT_MPU          GPIOE, 4
#define SPIDEV_CS_LSM9DS0_G        GPIOC, 13 // same cs for both internal and external
#define SPIDEV_CS_LSM9DS0_AM       GPIOC, 15 // same cs for both internal and external
#define SPIDEV_CS_RAMTRON          GPIOD, 10
#define SPIDEV_CS_RADIO            GPIOD, 10
#define SPIDEV_CS_FLOW             GPIOE, 4
#define SPIDEV_CS_EXT0             GPIOE, 4

// these device numbers are chosen to match those used when running NuttX. That prevent
// users having to recal when updating to ChibiOS
#define SPIDEV_LSM9DS0_G        1
#define SPIDEV_LSM9DS0_AM       2
#define SPIDEV_BARO             3
#define SPIDEV_MPU              4
#define SPIDEV_HMC              5
#define SPIDEV_EXT_MPU          1
#define SPIDEV_EXT_BARO         2
#define SPIDEV_EXT_LSM9DS0_AM   3
#define SPIDEV_EXT_LSM9DS0_G    4
#define SPIDEV_EXT0             5

#define SPIDEV_RAMTROM          10
#define SPIDEV_CYRF             11


#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4
#define SPI_BUS_SENSORS 0
#define SPI_BUS_RAMTRON 1
#define SPI_BUS_BARO    1

#define SPIDEV_CS_MPU              GPIOC, 2
#define SPIDEV_CS_ICM              GPIOC, 15
#define SPIDEV_CS_BARO             GPIOD, 7
#define SPIDEV_CS_RAMTRON          GPIOD, 10
#define SPIDEV_CS_MAG              GPIOE, 15

// these device numbers are chosen to match those used when running NuttX. That prevent
// users having to recal when updating to ChibiOS
#define SPIDEV_BARO             3
#define SPIDEV_MPU              4
#define SPIDEV_MAG              5
#define SPIDEV_ICM              6
#define SPIDEV_RAMTROM          10

#endif // CONFIG_HAL_BOARD_SUBTYPE

// SPI mode numbers
#define SPIDEV_MODE0    0
#define SPIDEV_MODE1    SPI_CR1_CPHA
#define SPIDEV_MODE2    SPI_CR1_CPOL
#define SPIDEV_MODE3    SPI_CR1_CPOL | SPI_CR1_CPHA

#define SPI1_CLOCK  STM32_PCLK2
#define SPI2_CLOCK  STM32_PCLK1
#define SPI3_CLOCK  STM32_PCLK1
#define SPI4_CLOCK  STM32_PCLK2

static struct SPIDriverInfo {    
    SPIDriver *driver;
    uint8_t busid; // used for device IDs in parameters
    uint8_t dma_channel_rx;
    uint8_t dma_channel_tx;
} spi_devices[] = {
    { &SPID1, 1, STM32_SPI_SPI1_RX_DMA_STREAM, STM32_SPI_SPI1_TX_DMA_STREAM },
    { &SPID2, 2, STM32_SPI_SPI2_RX_DMA_STREAM, STM32_SPI_SPI2_TX_DMA_STREAM },
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3
    { &SPID4, 4, STM32_SPI_SPI4_RX_DMA_STREAM, STM32_SPI_SPI4_TX_DMA_STREAM },
#endif
};

#define MHZ (1000U*1000U)
#define KHZ (1000U)
SPIDesc SPIDeviceManager::device_table[] = {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
    SPIDesc("cypress",        SPI_BUS_RADIO,    SPIDEV_RADIO,         SPIDEV_CS_RADIO,      SPIDEV_MODE0, 2*MHZ, 2*MHZ),
    SPIDesc("pixartflow",     SPI_BUS_FLOW,     SPIDEV_FLOW,          SPIDEV_CS_FLOW,       SPIDEV_MODE3, 2*MHZ, 2*MHZ),
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3
    SPIDesc("ms5611",         SPI_BUS_SENSORS,  SPIDEV_BARO,          SPIDEV_CS_MS5611,     SPIDEV_MODE3, 20*MHZ, 20*MHZ),
    SPIDesc("ms5611_ext",     SPI_BUS_EXT,      SPIDEV_EXT_BARO,      SPIDEV_CS_EXT_MS5611, SPIDEV_MODE3, 20*MHZ, 20*MHZ),
    SPIDesc("mpu6000",        SPI_BUS_SENSORS,  SPIDEV_MPU,           SPIDEV_CS_MPU,        SPIDEV_MODE3, 1*MHZ, 8*MHZ ),
    SPIDesc("mpu9250",        SPI_BUS_SENSORS,  SPIDEV_MPU,           SPIDEV_CS_MPU,        SPIDEV_MODE3, 1*MHZ, 8*MHZ ),
    SPIDesc("mpu9250_ext",    SPI_BUS_EXT,      SPIDEV_EXT_MPU,       SPIDEV_CS_EXT_MPU,    SPIDEV_MODE3, 1*MHZ, 8*MHZ ),  
    SPIDesc("hmc5843",        SPI_BUS_SENSORS,  SPIDEV_HMC,           SPIDEV_CS_HMC,        SPIDEV_MODE3, 11*MHZ, 11*MHZ ),  
    SPIDesc("lsm9ds0_g",      SPI_BUS_SENSORS,  SPIDEV_LSM9DS0_G,     SPIDEV_CS_LSM9DS0_G,  SPIDEV_MODE3, 11*MHZ, 11*MHZ ),
    SPIDesc("lsm9ds0_am",     SPI_BUS_SENSORS,  SPIDEV_LSM9DS0_AM,    SPIDEV_CS_LSM9DS0_AM, SPIDEV_MODE3, 11*MHZ, 11*MHZ ),
    SPIDesc("lsm9ds0_ext_g",  SPI_BUS_EXT,      SPIDEV_EXT_LSM9DS0_G, SPIDEV_CS_LSM9DS0_G,  SPIDEV_MODE3, 11*MHZ, 11*MHZ ),
    SPIDesc("lsm9ds0_ext_am", SPI_BUS_EXT,      SPIDEV_EXT_LSM9DS0_AM,SPIDEV_CS_LSM9DS0_AM, SPIDEV_MODE3, 11*MHZ, 11*MHZ ),
    SPIDesc("ramtron",        SPI_BUS_RAMTRON,  SPIDEV_RAMTROM,       SPIDEV_CS_RAMTRON,    SPIDEV_MODE3, 8*MHZ, 8*MHZ ),
    SPIDesc("cypress",        SPI_BUS_RADIO,    SPIDEV_CYRF,          SPIDEV_CS_RADIO,      SPIDEV_MODE0, 2*MHZ, 2*MHZ),
    SPIDesc("external0m0",    SPI_BUS_EXT,      SPIDEV_EXT0,          SPIDEV_CS_EXT0,       SPIDEV_MODE0, 2*MHZ, 2*MHZ),
    SPIDesc("external0m1",    SPI_BUS_EXT,      SPIDEV_EXT0,          SPIDEV_CS_EXT0,       SPIDEV_MODE1, 2*MHZ, 2*MHZ),
    SPIDesc("external0m2",    SPI_BUS_EXT,      SPIDEV_EXT0,          SPIDEV_CS_EXT0,       SPIDEV_MODE2, 2*MHZ, 2*MHZ),
    SPIDesc("external0m3",    SPI_BUS_EXT,      SPIDEV_EXT0,          SPIDEV_CS_EXT0,       SPIDEV_MODE3, 2*MHZ, 2*MHZ),
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4
    SPIDesc("ms5611_int",     SPI_BUS_BARO,     SPIDEV_BARO,          SPIDEV_CS_BARO,       SPIDEV_MODE3, 20*MHZ, 20*MHZ),
    SPIDesc("mpu9250",        SPI_BUS_SENSORS,  SPIDEV_MPU,           SPIDEV_CS_MPU,        SPIDEV_MODE3, 1*MHZ, 8*MHZ ),
    SPIDesc("icm20608",       SPI_BUS_SENSORS,  SPIDEV_ICM,           SPIDEV_CS_ICM,        SPIDEV_MODE3, 1*MHZ, 8*MHZ ),
    SPIDesc("hmc5843",        SPI_BUS_SENSORS,  SPIDEV_MAG,           SPIDEV_CS_MAG,        SPIDEV_MODE3, 11*MHZ, 11*MHZ ),
    SPIDesc("ramtron",        SPI_BUS_RAMTRON,  SPIDEV_RAMTROM,       SPIDEV_CS_RAMTRON,    SPIDEV_MODE3, 8*MHZ, 8*MHZ ),
    SPIDesc("lis3mdl",        SPI_BUS_SENSORS,  SPIDEV_MAG,           SPIDEV_CS_MAG,        SPIDEV_MODE3, 500*KHZ, 500*KHZ),
#endif
};

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
    spiStop(spi_devices[bus].driver);
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
        bus.spicfg.ssport = device_desc.port;
        bus.spicfg.sspad = device_desc.pin;
        bus.spicfg.cr1 = (uint16_t)(freq_flag | device_desc.mode);
        bus.spicfg.cr2 = 0;
        spiStart(spi_devices[device_desc.bus].driver, &bus.spicfg);        /* Setup transfer parameters.       */
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
    for (i = 0; i<ARRAY_SIZE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_table)) {
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
