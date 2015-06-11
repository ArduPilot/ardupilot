/*
  simple test of Storage API
 */
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <UARTDriver.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>
#include <AP_HAL_Boards.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::Storage *st;

void setup(void) 
{
    /*
      init Storage API
     */
    hal.console->printf_P(PSTR("Starting AP_HAL::Storage test\r\n"));
    st->init(NULL);

    /*
      Calculate XOR of the full conent of memory
      Do it by block of 8 bytes
    */
    unsigned int i, j;
    unsigned char buff[8], XOR_res = 0;

    for(i = 0; i < HAL_STORAGE_SIZE; i += 8)
    {
        st->read_block((void *) buff, i, 8);
        for(j = 0; j < 8; j++)
            XOR_res ^= buff[j];
    }

    /*
      print XORed result
     */
    hal.console->printf_P(PSTR("XORed ememory: %u\r\n"), (unsigned) XOR_res);
}

// In main loop do nothing
void loop(void) 
{	
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
