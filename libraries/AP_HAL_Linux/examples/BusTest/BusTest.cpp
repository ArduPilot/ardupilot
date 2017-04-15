// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// scan I2C and SPI buses for expected devices
//

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    hal.console->println("BusTest startup...\n");
}

static struct {
    const char *name;
    uint8_t whoami_reg;
} whoami_list[] = {
    { "ms5611",     0x00 | 0x80 },
    { "mpu9250",    0x75 | 0x80 },
    { "mpu6000",    0x75 | 0x80 },
    { "lsm9ds0_am", 0x0F | 0x80 },
    { "lsm9ds0_g",  0x0F | 0x80 },
};

void loop(void)
{
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    AP_HAL::Semaphore *spi_sem;

    hal.console->printf("Scanning SPI bus devices\n");

    char *name = new char;
    hal.spi->get_device_name(0, name);
    /* Dummy is the default device at index 0 
       if there isn't registered device for the target board.
    */
    const char *dummy = {"**dummy**"};
    if (!strcmp(dummy, name)) {
        hal.console->printf("No devices, nothing to do.\n");
        while(1) { hal.scheduler->delay(500); };
    }

    /* We get possible registered device count on the target board */
    uint8_t spicount = hal.spi->get_count();
    for (uint8_t ref = 0; ref < spicount; ref++) {
        /* We get the name from the index and we compare
           with our possible devices list.
        */
        hal.spi->get_device_name(ref, name);
        for (uint8_t i = 0; i < ARRAY_SIZE(whoami_list); i++) {
            if (!strcmp(whoami_list[i].name, name)) {
                hal.console->printf("Device registered in table: %s\n", name);
                dev = hal.spi->get_device(name);
                spi_sem = dev->get_semaphore();
                hal.console->printf("Trying to send data...\n");

                if (!spi_sem->take(1000)) {
                    hal.console->printf("Failed to get SPI semaphore for %s\n", name);
                    break;
                }

                uint8_t tx[2] = { whoami_list[i].whoami_reg, 0 };
                uint8_t rx[2] = { 0, 0 };

                dev->transfer(tx, sizeof(tx), rx, sizeof(rx));

                if (rx[1] != 0) {
                    hal.console->printf("WHO_AM_I for %s: 0x%02x\n\n", whoami_list[i].name, (unsigned)rx[1]);
                } else {
                    hal.console->printf("No response %s!\n\n", name);
                    break;
                }

                spi_sem->give();

                break;
            }
        }
    }

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
