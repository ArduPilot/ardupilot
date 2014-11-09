// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_PX4.h"

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] PROGMEM = {
    // @Param: _TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:APM2-MaxbotixI2C,3:APM2-PulsedLightI2C,4:PX4-I2C
    AP_GROUPINFO("_TYPE",    0, RangeFinder, _type[0], 0),

    // @Param: _PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    AP_GROUPINFO("_PIN",     1, RangeFinder, _pin[0], -1),

    // @Param: _SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: meters/Volt
    // @Increment: 0.001
    AP_GROUPINFO("_SCALING", 2, RangeFinder, _scaling[0], 3.0),

    // @Param: _OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: Volts
    // @Increment: 0.001
    AP_GROUPINFO("_OFFSET",  3, RangeFinder, _offset[0], 0.0),

    // @Param: _FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    AP_GROUPINFO("_FUNCTION", 4, RangeFinder, _function[0], 0),

    // @Param: _MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: centimeters
    // @Increment: 1
    AP_GROUPINFO("_MIN_CM",  5, RangeFinder, _min_distance_cm[0], 20),

    // @Param: _MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: centimeters
    // @Increment: 1
    AP_GROUPINFO("_MAX_CM",  6, RangeFinder, _max_distance_cm[0], 700),

    // @Param: _STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    AP_GROUPINFO("_STOP_PIN", 7, RangeFinder, _stop_pin[0], -1),

    // @Param: _SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: milliseconds
    // @Increment: 1
    AP_GROUPINFO("_SETTLE", 8, RangeFinder, _settle_time_ms[0], 0),

    // @Param: _RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    AP_GROUPINFO("_RMETRIC", 9, RangeFinder, _ratiometric[0], 1),

    // 10..12 left for future expansion

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:APM2-MaxbotixI2C,3:APM2-PulsedLightI2C,4:PX4-I2C
    AP_GROUPINFO("2_TYPE",    12, RangeFinder, _type[1], 0),

    // @Param: 2_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    AP_GROUPINFO("2_PIN",     13, RangeFinder, _pin[1], -1),

    // @Param: 2_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: meters/Volt
    // @Increment: 0.001
    AP_GROUPINFO("2_SCALING", 14, RangeFinder, _scaling[1], 3.0),

    // @Param: 2_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: Volts
    // @Increment: 0.001
    AP_GROUPINFO("2_OFFSET",  15, RangeFinder, _offset[1], 0.0),

    // @Param: 2_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    AP_GROUPINFO("2_FUNCTION",  16, RangeFinder, _function[1], 0),

    // @Param: 2_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: centimeters
    // @Increment: 1
    AP_GROUPINFO("2_MIN_CM",  17, RangeFinder, _min_distance_cm[1], 20),

    // @Param: 2_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: centimeters
    // @Increment: 1
    AP_GROUPINFO("2_MAX_CM",  18, RangeFinder, _max_distance_cm[1], 700),

    // @Param: 2_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    AP_GROUPINFO("2_STOP_PIN", 19, RangeFinder, _stop_pin[1], -1),

    // @Param: 2_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: milliseconds
    // @Increment: 1
    AP_GROUPINFO("2_SETTLE", 20, RangeFinder, _settle_time_ms[1], 0),

    // @Param: 2_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    AP_GROUPINFO("2_RMETRIC", 21, RangeFinder, _ratiometric[1], 1),
#endif

    AP_GROUPEND
};

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].healthy = false;
                continue;
            }
            drivers[i]->update();
        }
    }

    // work out primary instance - first healthy sensor
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && state[i].healthy) {
            primary_instance = i;
        }
    }
}
    
/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (type == RangeFinder_TYPE_PLI2C || 
        type == RangeFinder_TYPE_MBI2C) {
        // I2C sensor types are handled by the PX4Firmware code
        type = RangeFinder_TYPE_PX4;
    }
#endif
    if (type == RangeFinder_TYPE_PLI2C) {
        if (AP_RangeFinder_PulsedLightLRF::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_PulsedLightLRF(*this, instance, state[instance]);
            return;
        }
    } 
    if (type == RangeFinder_TYPE_MBI2C) {
        if (AP_RangeFinder_MaxsonarI2CXL::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_MaxsonarI2CXL(*this, instance, state[instance]);
            return;
        }
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (type == RangeFinder_TYPE_PX4) {
        if (AP_RangeFinder_PX4::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_PX4(*this, instance, state[instance]);
            return;
        }
    }
#endif
    if (type == RangeFinder_TYPE_ANALOG) {
        // note that analog must be the last to be checked, as it will
        // always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_analog(*this, instance, state[instance]);
            return;
        }
    }
}

