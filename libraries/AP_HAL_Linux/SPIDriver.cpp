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

LinuxSPIDeviceDriver::LinuxSPIDeviceDriver(const char *spipath, uint8_t mode, uint8_t bitsPerWord, uint8_t cs_pin, uint32_t speed):
    _spipath(spipath),
    _fd(-1),
    _mode(mode),
    _bitsPerWord(bitsPerWord),    
    _speed(speed),
    _cs_pin(cs_pin)
{    
}

void LinuxSPIDeviceDriver::init()
{
    _fd = open(_spipath, O_RDWR);
    if (_fd == -1) {
#if SPI_DEBUGGING
        hal.console->printf("LinuxSPIDeviceDriver failed opening _spipath\n");
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
    _cs->mode(GPIO_OUT);
    _cs->write(HIGH);       // do not hold the SPI bus initially
    return;

failed:
    close(_fd);
    _fd = -1;
    hal.scheduler->panic("SPI init failed");
}

AP_HAL::Semaphore* LinuxSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
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
    _cs->write(0);
}

void LinuxSPIDeviceDriver::cs_release()
{
    _cs->write(1);
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

LinuxSPIDeviceManager::LinuxSPIDeviceManager() :
    _device_ms5611("/dev/spidev2.0", SPI_MODE_0, 8, 7, 6*1000*1000), /* SPIDevice_MS5611 */
    _device_mpu6000("/dev/spidev2.0", SPI_MODE_0, 8, 113, 20*1000*1000), /* SPIDevice_MPU6000 */
    _device_mpu9250("/dev/spidev2.0", SPI_MODE_0, 8, 49, 6*1000*1000), /* SPIDevice_MPU9250 */
    _device_lsm9ds0("/dev/spidev1.0", SPI_MODE_0, 8, 5, 6*1000*1000), /* SPIDevice_LSM9DS0 */
    _device_fram("/dev/spidev2.0", SPI_MODE_0, 8, 5, 6*1000*1000) /* SPIDevice_Dataflash */
{}

void LinuxSPIDeviceManager::init(void *)
{
    _device_ms5611.init();
    _device_mpu6000.init();
    _device_mpu9250.init();
    _device_lsm9ds0.init();
}

/*
  return a SPIDeviceDriver for a particular device
 */
AP_HAL::SPIDeviceDriver* LinuxSPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    switch (dev) {
        case AP_HAL::SPIDevice_MPU6000:
            return &_device_mpu6000;            
        case AP_HAL::SPIDevice_MPU9250:
            return &_device_mpu9250;            
        case AP_HAL::SPIDevice_MS5611:
            return &_device_ms5611;
        case AP_HAL::SPIDevice_LSM9DS0:
            return &_device_lsm9ds0;                        
        case AP_HAL::SPIDevice_Dataflash:
            return &_device_fram;                        

    }
    return NULL;
}

#endif // CONFIG_HAL_BOARD
