/*******************************************
*   Sample sketch that configures an HMC5883L 3 axis
*   magnetometer to continuous mode and reads back
*   the three axis of data.
*******************************************/

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library

#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>


const AP_HAL::HAL& hal = AP_HAL_VRBRAIN;

#define HMC5883L  0x3C
AP_Compass_HMC5843 compass;


bool read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)HMC5883L, address, value) != 0) {
        //healthy = false;
        return false;
    }
    return true;
}

// write_register - update a register value
bool write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)HMC5883L, address, value) != 0) {
        //healthy = false;
        return false;
    }
    return true;
}


void setup() {


    hal.console->printf_P(PSTR("Initializing HMC5883L at address %x\r\n"),
                                HMC5883L);
    /*
    uint8_t config[1];
    //uint8_t stat = write_register((uint8_t)0x02,(uint8_t)0x00);
    uint8_t stat = write_register((uint8_t)0x02,(uint8_t)0x00);
    uint8_t stat2 = read_register((uint8_t)0x02, config);
    if (stat) {
	if(stat2){
	    hal.console->printf_P(PSTR("successful init\r\n"));
	} else {
        hal.console->printf_P(PSTR("failed init: return status %d\r\n"),
                (int)stat2);
        for(;;);
	}
    }
    //hal.console->printf_P(PSTR("failed init: return status %d\r\n"),(int)stat);
    for(;;);
    */
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        hal.console->println_P(PSTR("COMPASS INIT ERROR"));
    }
}

void loop() {
    /*
    uint8_t data[6];
    //read 6 bytes (x,y,z) from the device
    //uint8_t stat = hal.i2c->readRegisters(HMC5883L,0x03,6, data);
    uint8_t stat = 1;
    if (stat == 0){
        int16_t x, y, z;
        x = (int16_t)data[0] << 8;
        x |= (int16_t)data[1];
        z = (int16_t)data[2] << 8;
        z |= (int16_t)data[3];
        y = (int16_t)data[4] << 8;
        y |= (int16_t)data[5];
        hal.console->printf_P(PSTR("x: %d y: %d z: %d \r\n"), -x, y, -z);
        hal.console->println();
    } else {
        hal.console->printf_P(PSTR("i2c error: status %d\r\n"), (int)stat);
    }
        hal.scheduler->delay(100);
        */

            hal.scheduler->delay(1000);
            if (compass.read()) {
                //float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
                hal.console->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d\n"),
                                0,
                                compass.mag_x,
                                compass.mag_y,
                                compass.mag_z);
            } else {
                hal.console->println_P(PSTR("not healthy"));
            }


}

AP_HAL_MAIN();
