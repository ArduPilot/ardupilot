/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   Airspeed.pde - airspeed example sketch
 *
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <Filter.h>
#include <AP_Buffer.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_Notify.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Terrain.h>
#include <DataFlash.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_Vehicle::FixedWing aparm;

AP_Airspeed airspeed(aparm);

void setup()
{
    hal.console->println("ArduPilot Airspeed library test");

    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_PIN", 65);

    airspeed.init();
    airspeed.calibrate(false);
}

void loop(void)
{
    static uint32_t timer;
    if((hal.scheduler->millis() - timer) > 100) {
        timer = hal.scheduler->millis();
        airspeed.read();
        hal.console->printf("airspeed %.2f\n", airspeed.get_airspeed());
    }
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
