
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

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include "AP_Airspeed.h"

#include <AP_Vehicle/AP_Vehicle_Type.h>

// Dummy the AP_Airspeed class to allow building Airspeed only for plane, rover, sub, and copter & heli 2MB boards
// This could be removed once the build system allows for APM_BUILD_TYPE in header files
// note that this is re-definition of the one in AP_Airspeed.cpp, can't be shared as vehicle dependences cant go in header files
#ifndef AP_AIRSPEED_DUMMY_METHODS_ENABLED
#define AP_AIRSPEED_DUMMY_METHODS_ENABLED ((APM_BUILD_COPTER_OR_HELI && BOARD_FLASH_SIZE <= 1024) || \
                                            APM_BUILD_TYPE(APM_BUILD_AntennaTracker) || APM_BUILD_TYPE(APM_BUILD_Blimp))
#endif

#if !AP_AIRSPEED_DUMMY_METHODS_ENABLED

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#define PSI_RANGE_DEFAULT 0.05
#endif

#ifndef PSI_RANGE_DEFAULT
#define PSI_RANGE_DEFAULT 1.0f
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed_Params::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Airspeed type
    // @Description: Type of airspeed sensor
    // @Values: 0:None,1:I2C-MS4525D0,2:Analog,3:I2C-MS5525,4:I2C-MS5525 (0x76),5:I2C-MS5525 (0x77),6:I2C-SDP3X,7:I2C-DLVR-5in,8:DroneCAN,9:I2C-DLVR-10in,10:I2C-DLVR-20in,11:I2C-DLVR-30in,12:I2C-DLVR-60in,13:NMEA water speed,14:MSP,15:ASP5033,16:ExternalAHRS,17:NP210,100:SITL
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Airspeed_Params, type, 0, AP_PARAM_FLAG_ENABLE),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: Enables airspeed use for automatic throttle modes and replaces control from THR_TRIM. Continues to display and log airspeed if set to 0. Uses airspeed for control if set to 1. Only uses airspeed when throttle = 0 if set to 2 (useful for gliders with airspeed sensors behind propellers).
    // @Description{Copter, Blimp, Rover, Sub}: This parameter is not used by this vehicle. Always set to 0.
    // @Values: 0:DoNotUse,1:Use,2:UseWhenZeroThrottle
    // @User: Standard
    AP_GROUPINFO("USE", 2, AP_Airspeed_Params, use, 0),

    // @Param: OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OFFSET", 3, AP_Airspeed_Params, offset, 0),

    // @Param: RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("RATIO", 4, AP_Airspeed_Params, ratio, 2),

    // @Param: PIN
    // @DisplayName: Airspeed pin
    // @Description: The pin number that the airspeed sensor is connected to for analog sensors. Values for some autopilots are given as examples. Search wiki for "Analog pins".
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @User: Advanced
    AP_GROUPINFO("PIN", 5, AP_Airspeed_Params, pin, 0),
#endif // HAL_BUILD_AP_PERIPH

#if AP_AIRSPEED_AUTOCAL_ENABLE
    // @Param: AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @DisplayName{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Description: Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.
    // @User: Advanced
    AP_GROUPINFO("AUTOCAL", 6, AP_Airspeed_Params, autocal, 0),
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: TUBE_ORDR
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    // @Values: 0:Normal,1:Swapped,2:Auto Detect
    AP_GROUPINFO("TUBE_ORDR", 7, AP_Airspeed_Params, tube_order, 2),

    // @Param: SKIP_CAL
    // @DisplayName: Skip airspeed offset calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SKIP_CAL", 8, AP_Airspeed_Params, skip_cal, 0),
#endif // HAL_BUILD_AP_PERIPH

    // @Param: PSI_RANGE
    // @DisplayName: The PSI range of the device
    // @Description: This parameter allows you to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device
    // @User: Advanced
    AP_GROUPINFO("PSI_RANGE", 9, AP_Airspeed_Params, psi_range, PSI_RANGE_DEFAULT),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: BUS
    // @DisplayName: Airspeed I2C bus
    // @Description: Bus number of the I2C bus where the airspeed sensor is connected. May not correspond to board's I2C bus number labels. Retry another bus and reboot if airspeed sensor fails to initialize.
    // @Values: 0:Bus0,1:Bus1,2:Bus2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("BUS", 10, AP_Airspeed_Params, bus, 1),
#endif // HAL_BUILD_AP_PERIPH

    // @Param: DEVID
    // @DisplayName: Airspeed ID
    // @Description: Airspeed sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("DEVID", 11, AP_Airspeed_Params, bus_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    AP_GROUPEND
};

AP_Airspeed_Params::AP_Airspeed_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#else  // dummy implementation:

AP_Airspeed_Params::AP_Airspeed_Params(void) {};
const AP_Param::GroupInfo AP_Airspeed_Params::var_info[] = { AP_GROUPEND };

#endif

#endif  // AP_AIRSPEED_ENABLED
