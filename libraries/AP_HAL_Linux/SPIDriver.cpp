#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_ERLE
#include "SPIDriver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "GPIO.h"

#define SPI_DEBUGGING 1

using namespace Linux;

extern const AP_HAL::HAL& hal;

LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[LINUX_SPI_DEVICE_NUM_DEVICES] = {
    LinuxSPIDeviceDriver(1, LINUX_SPI_DEVICE_MS5611,  SPI_MODE_0, 8,   7,  6*1000*1000), /* SPIDevice_MS5611 */
    LinuxSPIDeviceDriver(1, LINUX_SPI_DEVICE_MPU6000, SPI_MODE_0, 8, 113, 20*1000*1000), /* SPIDevice_MPU6000 */
    LinuxSPIDeviceDriver(1, LINUX_SPI_DEVICE_MPU9250, SPI_MODE_0, 8,  49,  6*1000*1000), /* SPIDevice_MPU9250 */
    LinuxSPIDeviceDriver(0, LINUX_SPI_DEVICE_LSM9DS0, SPI_MODE_0, 8,   5,  6*1000*1000), /* SPIDevice_LSM9DS0 */
    LinuxSPIDeviceDriver(1, LINUX_SPI_DEVICE_FRAM,    SPI_MODE_0, 8,  44,  6*1000*1000)  /* SPIDevice_Dataflash */
};

// have a separate semaphore per bus
LinuxSemaphore LinuxSPIDeviceManager::_semaphore[LINUX_SPI_NUM_BUSES];

LinuxSPIDeviceDriver::LinuxSPIDeviceDriver(uint8_t bus, LinuxSPIDeviceType type, uint8_t mode, uint8_t bitsPerWord, uint8_t cs_pin, uint32_t speed):
    _bus(bus),
    _type(type),
    _fd(-1),
    _mode(mode),
    _bitsPerWord(bitsPerWord),    
    _speed(speed),
    _cs_pin(cs_pin)
{    
}

void LinuxSPIDeviceDriver::init()
{
    const char *path = _bus==0?"/dev/spidev1.0":"/dev/spidev2.0";
    _fd = open(path, O_RDWR);
    if (_fd == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver failed opening %s\n", path);
#endif        
        return;
    }
    int ret;
    ret = ioctl(_fd, SPI_IOC_WR_MODE, &_mode);
    if (ret == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver ioctl SPI_IOC_WR_MODE failed\n");
#endif                
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_RD_MODE, &_mode);
    if (ret == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver ioctl SPI_IOC_RD_MODE failed\n");
#endif                
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &_bitsPerWord);
    if (ret == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver ioctl SPI_IOC_WR_BITS_PER_WORD failed\n");
#endif                        
        goto failed;
    }
    
    ret = ioctl(_fd, SPI_IOC_RD_BITS_PER_WORD, &_bitsPerWord);
    if (ret == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver ioctl SPI_IOC_RD_BITS_PER_WORD failed\n");
#endif                        
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed);    
    if (ret == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver ioctl SPI_IOC_WR_MAX_SPEED_HZ failed\n");
#endif                                
        goto failed;
    }
 
    ret = ioctl(_fd, SPI_IOC_RD_MAX_SPEED_HZ, &_speed);    
    if (ret == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver ioctl SPI_IOC_RD_MAX_SPEED_HZ failed\n");
#endif                                        
        goto failed;
    }

    // Init the CS
    _cs = hal.gpio->channel(_cs_pin);
    _cs->mode(HAL_GPIO_OUTPUT);
    _cs->write(HIGH);       // do not hold the SPI bus initially
    return;

failed:
    close(_fd);
    _fd = -1;
    hal.scheduler->panic("SPI init failed");
}

AP_HAL::Semaphore* LinuxSPIDeviceDriver::get_semaphore()
{
    return LinuxSPIDeviceManager::get_semaphore(_bus);
}

void LinuxSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    cs_assert();
    struct spi_ioc_transfer spi[1];
    spi[0].tx_buf        = (uint64_t)tx;
    spi[0].rx_buf        = (uint64_t)rx;
    spi[0].len           = len;
    spi[0].delay_usecs   = 0;
    spi[0].speed_hz      = _speed;
    spi[0].bits_per_word = _bitsPerWord;
    spi[0].cs_change     = 0;
 
    ioctl(_fd, SPI_IOC_MESSAGE(1), &spi);
    cs_release();
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
        _device[i].init();
    }
}

void LinuxSPIDeviceManager::cs_assert(LinuxSPIDeviceType type)
{
    for (uint8_t i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i].get_bus() != _device[type].get_bus()) {
            // not the same bus
            continue;
        }
        if (i != type) {
            if (_device[i].get_cs()->read() != 1) {
                hal.scheduler->panic("two CS enabled at once");
            }
        }
    }
    _device[type].get_cs()->write(0);
}

void LinuxSPIDeviceManager::cs_release(LinuxSPIDeviceType type)
{
    for (uint8_t i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i].get_bus() != _device[type].get_bus()) {
            // not the same bus
            continue;
        }
        _device[i].get_cs()->write(1);
    }
}

/*
  return a SPIDeviceDriver for a particular device
 */
AP_HAL::SPIDeviceDriver* LinuxSPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    switch (dev) {
        case AP_HAL::SPIDevice_MPU6000:
            return &_device[LINUX_SPI_DEVICE_MPU6000];
        case AP_HAL::SPIDevice_MPU9250:
            return &_device[LINUX_SPI_DEVICE_MPU9250];
        case AP_HAL::SPIDevice_MS5611:
            return &_device[LINUX_SPI_DEVICE_MS5611];
        case AP_HAL::SPIDevice_LSM9DS0:
            return &_device[LINUX_SPI_DEVICE_LSM9DS0];       
        case AP_HAL::SPIDevice_Dataflash:
            return &_device[LINUX_SPI_DEVICE_FRAM];   

    }
    return NULL;
}

/*
  return the bus specific semaphore
 */
AP_HAL::Semaphore *LinuxSPIDeviceManager::get_semaphore(uint8_t bus)
{
    return &_semaphore[bus];
}

#endif // CONFIG_HAL_BOARD
