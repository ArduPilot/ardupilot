#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "SPIDriver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "GPIO.h"

#define SPI_DEBUGGING 0

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define MHZ (1000U*1000U)

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[LINUX_SPI_DEVICE_NUM_DEVICES] = {
    // different SPI tables per board subtype
    LinuxSPIDeviceDriver(1, 0, AP_HAL::SPIDevice_LSM9DS0_AM, SPI_MODE_3, 8, BBB_P9_17,  10*MHZ,10*MHZ),
    LinuxSPIDeviceDriver(1, 0, AP_HAL::SPIDevice_LSM9DS0_G,  SPI_MODE_3, 8, BBB_P8_9,   10*MHZ,10*MHZ),
    LinuxSPIDeviceDriver(2, 0, AP_HAL::SPIDevice_MS5611,     SPI_MODE_3, 8, BBB_P9_42,  10*MHZ,10*MHZ),
    LinuxSPIDeviceDriver(2, 0, AP_HAL::SPIDevice_MPU6000,    SPI_MODE_3, 8, BBB_P9_28,  500*1000, 20*MHZ),
    /* MPU9250 is restricted to 1MHz for non-data and interrupt registers */
    LinuxSPIDeviceDriver(2, 0, AP_HAL::SPIDevice_MPU9250,    SPI_MODE_3, 8, BBB_P9_23,  1*MHZ, 20*MHZ),
    LinuxSPIDeviceDriver(2, 0, AP_HAL::SPIDevice_Dataflash,  SPI_MODE_3, 8, BBB_P8_12,  6*MHZ, 6*MHZ),
};
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[LINUX_SPI_DEVICE_NUM_DEVICES] = {
    /* MPU9250 is restricted to 1MHz for non-data and interrupt registers */
    LinuxSPIDeviceDriver(0, 0, AP_HAL::SPIDevice_MPU9250, SPI_MODE_0, 8, RPI_GPIO_7,  1*MHZ, 16*MHZ),
    LinuxSPIDeviceDriver(0, 0, AP_HAL::SPIDevice_Ublox, SPI_MODE_0, 8, RPI_GPIO_8,  1*MHZ, 4*MHZ),
};
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[LINUX_SPI_DEVICE_NUM_DEVICES] = {
    LinuxSPIDeviceDriver(2, 0, AP_HAL::SPIDevice_MS5611,     SPI_MODE_3, 8, BBB_P9_42,  10*MHZ,10*MHZ),
    /* MPU9250 is restricted to 1MHz for non-data and interrupt registers */
    LinuxSPIDeviceDriver(2, 0, AP_HAL::SPIDevice_MPU9250,    SPI_MODE_3, 8, BBB_P9_23,  1*MHZ, 20*MHZ),
};
#else
// empty device table
LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[0];
#endif

// have a separate semaphore per bus
LinuxSemaphore LinuxSPIDeviceManager::_semaphore[LINUX_SPI_MAX_BUSES];

LinuxSPIDeviceDriver::LinuxSPIDeviceDriver(uint16_t bus, uint16_t subdev, enum AP_HAL::SPIDevice type, uint8_t mode, uint8_t bitsPerWord, int16_t cs_pin, uint32_t lowspeed, uint32_t highspeed):
    _bus(bus),
    _subdev(subdev),
    _type(type),
    _mode(mode),
    _bitsPerWord(bitsPerWord),
    _lowspeed(lowspeed),
    _highspeed(highspeed),
    _speed(highspeed),
    _cs_pin(cs_pin),
    _cs(NULL)
{
}

void LinuxSPIDeviceDriver::init()
{
    // Init the CS
    if(_cs_pin != SPI_CS_KERNEL) {
        _cs = hal.gpio->channel(_cs_pin);
        if (_cs == NULL) {
            hal.scheduler->panic("Unable to instantiate cs pin");
        }
        _cs->mode(HAL_GPIO_OUTPUT);
        _cs->write(1);       // do not hold the SPI bus initially
    } else {
        // FIXME Anything we need to do here for kernel-managed CS?
    }
}

AP_HAL::Semaphore* LinuxSPIDeviceDriver::get_semaphore()
{
    return LinuxSPIDeviceManager::get_semaphore(_bus);
}

void LinuxSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    LinuxSPIDeviceManager::transaction(*this, tx, rx, len);
}

void LinuxSPIDeviceDriver::set_bus_speed(enum bus_speed speed)
{
    if (speed == SPI_SPEED_LOW) {
        _speed = _lowspeed;
    } else {
        _speed = _highspeed;
    }
}

void LinuxSPIDeviceDriver::cs_assert()
{
    LinuxSPIDeviceManager::cs_assert(_type);
}

void LinuxSPIDeviceDriver::cs_release()
{
    LinuxSPIDeviceManager::cs_release(_type);
}

uint8_t LinuxSPIDeviceDriver::transfer(uint8_t data)
{
    uint8_t v = 0;
    transaction(&data, &v, 1);
    return v;
}

void LinuxSPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
    transaction(data, NULL, len);
}

void LinuxSPIDeviceManager::init(void *)
{
    for (uint8_t i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._bus >= LINUX_SPI_MAX_BUSES) {
            hal.scheduler->panic("SPIDriver: invalid bus number");
        }
        char path[255];
        snprintf(path, sizeof(path), "/dev/spidev%u.%u",
                 _device[i]._bus + LINUX_SPIDEV_BUS_OFFSET, _device[i]._subdev);
        _device[i]._fd = open(path, O_RDWR);
        if (_device[i]._fd == -1) {
            printf("Unable to open %s - %s\n", path, strerror(errno));
            hal.scheduler->panic("SPIDriver: unable to open SPI bus");
        }
#if SPI_DEBUGGING
        printf("Opened %s\n", path);
        fflush(stdout);
#endif
        _device[i].init();
    }
}

void LinuxSPIDeviceManager::cs_assert(enum AP_HAL::SPIDevice type)
{
    uint16_t bus = 0, i;
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == type) {
            bus = _device[i]._bus;
            break;
        }
    }
    if (i == LINUX_SPI_DEVICE_NUM_DEVICES) {
        hal.scheduler->panic("Bad device type");
    }

    // Kernel-mode CS handling
    if(_device[i]._cs_pin == SPI_CS_KERNEL)
        return;

    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._bus != bus) {
            // not the same bus
            continue;
        }
        if (_device[i]._type != type) {
            if (_device[i]._cs->read() != 1) {
                hal.console->printf("two CS enabled at once i=%u %u and %u\n",
                                    (unsigned)i, (unsigned)type, (unsigned)_device[i]._type);
            }
        }
    }
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == type) {
            _device[i]._cs->write(0);
        }
    }
}

void LinuxSPIDeviceManager::cs_release(enum AP_HAL::SPIDevice type)
{
    uint16_t bus = 0, i;
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == type) {
            bus = _device[i]._bus;
            break;
        }
    }
    if (i == LINUX_SPI_DEVICE_NUM_DEVICES) {
        hal.scheduler->panic("Bad device type");
    }

    // Kernel-mode CS handling
    if(_device[i]._cs_pin == SPI_CS_KERNEL)
        return;

    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._bus != bus) {
            // not the same bus
            continue;
        }
        _device[i]._cs->write(1);
    }
}

void LinuxSPIDeviceManager::transaction(LinuxSPIDeviceDriver &driver, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    // we set the mode before we assert the CS line so that the bus is
    // in the correct idle state before the chip is selected
    ioctl(driver._fd, SPI_IOC_WR_MODE, &driver._mode);

    cs_assert(driver._type);
    struct spi_ioc_transfer spi[1];
    memset(spi, 0, sizeof(spi));
    spi[0].tx_buf        = (uint64_t)tx;
    spi[0].rx_buf        = (uint64_t)rx;
    spi[0].len           = len;
    spi[0].delay_usecs   = 0;
    spi[0].speed_hz      = driver._speed;
    spi[0].bits_per_word = driver._bitsPerWord;
    spi[0].cs_change     = 0;

    if (rx != NULL) {
        // keep valgrind happy
        memset(rx, 0, len);
    }

    ioctl(driver._fd, SPI_IOC_MESSAGE(1), &spi);
    cs_release(driver._type);
}

/*
  return a SPIDeviceDriver for a particular device
 */
AP_HAL::SPIDeviceDriver *LinuxSPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    for (uint8_t i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == dev) {
            return &_device[i];
        }
    }
    return NULL;
}

/*
  return the bus specific semaphore
 */
AP_HAL::Semaphore *LinuxSPIDeviceManager::get_semaphore(uint16_t bus)
{
    return &_semaphore[bus];
}

#endif // CONFIG_HAL_BOARD
