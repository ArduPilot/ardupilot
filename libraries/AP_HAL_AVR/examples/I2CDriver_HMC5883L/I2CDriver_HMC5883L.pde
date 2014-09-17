/*******************************************
*   Sample sketch that configures an HMC5883L 3 axis
*   magnetometer to continuous mode and reads back
*   the three axis of data.
*******************************************/

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

#define HMC5883L  0x1E

void setup() {
    hal.console->printf_P(PSTR("Initializing HMC5883L at address %x\r\n"),
                                HMC5883L);

    uint8_t stat = hal.i2c->writeRegister(HMC5883L,0x02,0x00);
    if (stat == 0) {
        hal.console->printf_P(PSTR("successful init\r\n"));
    } else {
        hal.console->printf_P(PSTR("failed init: return status %d\r\n"),
                (int)stat);
        for(;;);
    }
}

void loop() {
    uint8_t data[6];
    //read 6 bytes (x,y,z) from the device
    uint8_t stat = hal.i2c->readRegisters(HMC5883L,0x03,6, data);

    if (stat == 0){
        int x, y, z;
        x = data[0] << 8;
        x |= data[1];
        y = data[2] << 8;
        y |= data[3];
        z = data[4] << 8;
        z |= data[5];
        hal.console->printf_P(PSTR("x: %d y: %d z: %d \r\n"), x, y, z);
    } else { 
        hal.console->printf_P(PSTR("i2c error: status %d\r\n"), (int)stat);
    }
}

AP_HAL_MAIN();
