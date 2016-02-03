// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// scan I2C and SPI buses for expected devices
//

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    hal.console->println("BusTest startup...");
}

static struct {
    const char *name;
    enum AP_HAL::SPIDeviceType dev;
    uint8_t whoami_reg;
} whoami_list[] = {
    { "MS5611",     AP_HAL::SPIDevice_MS5611,     0x00 | 0x80 },
    { "MPU9250",    AP_HAL::SPIDevice_MPU9250,    0x75 | 0x80 },
    { "MPU6000",    AP_HAL::SPIDevice_MPU6000,    0x75 | 0x80 },
    { "FRAM",       AP_HAL::SPIDevice_Dataflash,  0x00 | 0x80 },
    { "LSM9DS0_AM", AP_HAL::SPIDevice_LSM9DS0_AM, 0x0F | 0x80 },
    { "LSM9DS0_G",  AP_HAL::SPIDevice_LSM9DS0_G,  0x0F | 0x80 },
};

void loop(void)
{
    AP_HAL::SPIDeviceDriver *spi;
    AP_HAL::Semaphore *spi_sem;

    hal.console->printf("Scanning SPI bus devices\n");

    for (uint8_t i=0; i < ARRAY_SIZE(whoami_list); i++) {
        spi = hal.spi->device(whoami_list[i].dev);
        if (spi == NULL) {
            hal.console->printf("Failed to get SPI device for %s\n", whoami_list[i].name);
            continue;
        }
        spi_sem = spi->get_semaphore();
        if (!spi_sem->take(1000)) {
            hal.console->printf("Failed to get SPI semaphore for %s\n", whoami_list[i].name);
            continue;
        }
        uint8_t tx[2] = { whoami_list[i].whoami_reg, 0 };
        uint8_t rx[2];
        
        spi->transaction(tx, rx, 2);

        hal.console->printf("WHO_AM_I for %s: 0x%02x\n", whoami_list[i].name, (unsigned)rx[1]);
        spi_sem->give();
    }        
    hal.scheduler->delay(200);
}

AP_HAL_MAIN();
