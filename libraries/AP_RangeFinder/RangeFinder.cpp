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
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) &&      \
    defined(HAVE_LIBIIO)
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_uLanding.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_VL53L1X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake.h"
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"
#include "AP_RangeFinder_PWM.h"
#include "AP_RangeFinder_BLPing.h"
#include "AP_RangeFinder_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {
<<<<<<< HEAD

	// @Group: 1_
	// @Path: AP_RangeFinder_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, RangeFinder, AP_RangeFinder_Params),

    // @Group: 1_
=======
    // @Param: _TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, RangeFinder, state[0].type, 0),

    // @Param: _PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Standard
    AP_GROUPINFO("_PIN",     1, RangeFinder, state[0].pin, -1),

    // @Param: _SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_SCALING", 2, RangeFinder, state[0].scaling, 3.0f),

    // @Param: _OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_OFFSET",  3, RangeFinder, state[0].offset, 0.0f),

    // @Param: _FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("_FUNCTION", 4, RangeFinder, state[0].function, 0),

    // @Param: _MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN_CM",  5, RangeFinder, state[0].min_distance_cm, 20),

    // @Param: _MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_CM",  6, RangeFinder, state[0].max_distance_cm, 700),

    // @Param: _STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Standard
    AP_GROUPINFO("_STOP_PIN", 7, RangeFinder, state[0].stop_pin, -1),

    // @Param: _SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_SETTLE", 8, RangeFinder, state[0].settle_time_ms, 0),

    // @Param: _RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Standard
    AP_GROUPINFO("_RMETRIC", 9, RangeFinder, state[0].ratiometric, 1),

    // @Param: _PWRRNG
    // @DisplayName: Powersave range
    // @Description: This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
    // @Units: m
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("_PWRRNG", 10, RangeFinder, _powersave_range, 0),

    // @Param: _GNDCLEAR
    // @DisplayName: Distance (in cm) from the range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 5 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_GNDCLEAR", 11, RangeFinder, state[0].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: _ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ADDR", 23, RangeFinder, state[0].address, 0),

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the first rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the first rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the first rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 49, RangeFinder, state[0].pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Rangefinder orientation
    // @Description: Orientation of rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 53, RangeFinder, state[0].orientation, ROTATION_PITCH_270),

    // @Group: _
>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
<<<<<<< HEAD
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),
=======
    // @Param: 2_TYPE
    // @DisplayName: Second Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Advanced
    AP_GROUPINFO("2_TYPE",    12, RangeFinder, state[1].type, 0),

    // @Param: 2_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("2_PIN",     13, RangeFinder, state[1].pin, -1),

    // @Param: 2_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_SCALING", 14, RangeFinder, state[1].scaling, 3.0f),

    // @Param: 2_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_OFFSET",  15, RangeFinder, state[1].offset, 0.0f),

    // @Param: 2_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("2_FUNCTION",  16, RangeFinder, state[1].function, 0),

    // @Param: 2_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MIN_CM",  17, RangeFinder, state[1].min_distance_cm, 20),

    // @Param: 2_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MAX_CM",  18, RangeFinder, state[1].max_distance_cm, 700),

    // @Param: 2_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("2_STOP_PIN", 19, RangeFinder, state[1].stop_pin, -1),

    // @Param: 2_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_SETTLE", 20, RangeFinder, state[1].settle_time_ms, 0),

    // @Param: 2_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("2_RMETRIC", 21, RangeFinder, state[1].ratiometric, 1),

    // @Param: 2_GNDCLEAR
    // @DisplayName: Distance (in cm) from the second range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the second range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_GNDCLEAR", 22, RangeFinder, state[1].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 2_ADDR
    // @DisplayName: Bus address of second rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_ADDR", 24, RangeFinder, state[1].address, 0),

    // @Param: 2_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the second rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 2_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the second rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 2_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the second rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("2_POS", 50, RangeFinder, state[1].pos_offset, 0.0f),

    // @Param: 2_ORIENT
    // @DisplayName: Rangefinder 2 orientation
    // @Description: Orientation of 2nd rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("2_ORIENT", 54, RangeFinder, state[1].orientation, ROTATION_PITCH_270),
>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
<<<<<<< HEAD
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),
=======

    // @Param: 3_TYPE
    // @DisplayName: Third Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Advanced
    AP_GROUPINFO("3_TYPE",    25, RangeFinder, state[2].type, 0),

    // @Param: 3_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("3_PIN",     26, RangeFinder, state[2].pin, -1),

    // @Param: 3_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("3_SCALING", 27, RangeFinder, state[2].scaling, 3.0f),

    // @Param: 3_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("3_OFFSET",  28, RangeFinder, state[2].offset, 0.0f),

    // @Param: 3_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("3_FUNCTION",  29, RangeFinder, state[2].function, 0),

    // @Param: 3_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MIN_CM",  30, RangeFinder, state[2].min_distance_cm, 20),

    // @Param: 3_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MAX_CM",  31, RangeFinder, state[2].max_distance_cm, 700),

    // @Param: 3_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("3_STOP_PIN", 32, RangeFinder, state[2].stop_pin, -1),

    // @Param: 3_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_SETTLE", 33, RangeFinder, state[2].settle_time_ms, 0),

    // @Param: 3_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("3_RMETRIC", 34, RangeFinder, state[2].ratiometric, 1),

    // @Param: 3_GNDCLEAR
    // @DisplayName: Distance (in cm) from the third range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the third range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_GNDCLEAR", 35, RangeFinder, state[2].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 3_ADDR
    // @DisplayName: Bus address of third rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_ADDR", 36, RangeFinder, state[2].address, 0),

    // @Param: 3_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the third rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 3_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the third rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 3_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the third rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("3_POS", 51, RangeFinder, state[2].pos_offset, 0.0f),

    // @Param: 3_ORIENT
    // @DisplayName: Rangefinder 3 orientation
    // @Description: Orientation of 3rd rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("3_ORIENT", 55, RangeFinder, state[2].orientation, ROTATION_PITCH_270),
>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
<<<<<<< HEAD
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),
=======

    // @Param: 4_TYPE
    // @DisplayName: Fourth Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Advanced
    AP_GROUPINFO("4_TYPE",    37, RangeFinder, state[3].type, 0),

    // @Param: 4_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("4_PIN",     38, RangeFinder, state[3].pin, -1),

    // @Param: 4_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("4_SCALING", 39, RangeFinder, state[3].scaling, 3.0f),

    // @Param: 4_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("4_OFFSET",  40, RangeFinder, state[3].offset, 0.0f),

    // @Param: 4_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("4_FUNCTION",  41, RangeFinder, state[3].function, 0),

    // @Param: 4_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MIN_CM",  42, RangeFinder, state[3].min_distance_cm, 20),

    // @Param: 4_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MAX_CM",  43, RangeFinder, state[3].max_distance_cm, 700),

    // @Param: 4_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("4_STOP_PIN", 44, RangeFinder, state[3].stop_pin, -1),

    // @Param: 4_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_SETTLE", 45, RangeFinder, state[3].settle_time_ms, 0),

    // @Param: 4_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("4_RMETRIC", 46, RangeFinder, state[3].ratiometric, 1),

    // @Param: 4_GNDCLEAR
    // @DisplayName: Distance (in cm) from the fourth range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the fourth range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_GNDCLEAR", 47, RangeFinder, state[3].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 4_ADDR
    // @DisplayName: Bus address of fourth rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_ADDR", 48, RangeFinder, state[3].address, 0),

    // @Param: 4_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 4_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 4_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("4_POS", 52, RangeFinder, state[3].pos_offset, 0.0f),

    // @Param: 4_ORIENT
    // @DisplayName: Rangefinder 4 orientation
    // @Description: Orientation of 4th range finder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("4_ORIENT", 56, RangeFinder, state[3].orientation, ROTATION_PITCH_270),
>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "4_",  60, RangeFinder, backend_var_info[3]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, RangeFinder, AP_RangeFinder_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  34, RangeFinder, backend_var_info[4]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, RangeFinder, AP_RangeFinder_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  36, RangeFinder, backend_var_info[5]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, RangeFinder, AP_RangeFinder_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  38, RangeFinder, backend_var_info[6]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, RangeFinder, AP_RangeFinder_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  40, RangeFinder, backend_var_info[7]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 41, RangeFinder, AP_RangeFinder_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_",  42, RangeFinder, backend_var_info[8]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 9
    // @Group: A_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[9], "A_", 43, RangeFinder, AP_RangeFinder_Params),

    // @Group: A_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[9], "A_",  44, RangeFinder, backend_var_info[9]),
#endif
    
    AP_GROUPEND
};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void RangeFinder::convert_params(void) {
    if (params[0].type.configured_in_storage()) {
        // _params[0]._type will always be configured in storage after conversion is done the first time
        return;
    }

    struct ConversionTable {
        uint8_t old_element;
        uint8_t new_index;
        uint8_t instance;
    };

    const struct ConversionTable conversionTable[] = {
            {0, 0, 0}, //0, TYPE 1
            {1, 1, 0}, //1, PIN 1
            {2, 2, 0}, //2, SCALING 1
            {3, 3, 0}, //3, OFFSET 1
            {4, 4, 0}, //4, FUNCTION 1
            {5, 5, 0}, //5, MIN_CM 1
            {6, 6, 0}, //6, MAX_CM 1
            {7, 7, 0}, //7, STOP_PIN 1
            {8, 8, 0}, //8, SETTLE 1
            {9, 9, 0}, //9, RMETRIC 1
            {10, 10, 0}, //10, PWRRNG 1 (previously existed only once for all sensors)
            {11, 11, 0}, //11, GNDCLEAR 1
            {23, 12, 0}, //23, ADDR 1
            {49, 13, 0}, //49, POS 1
            {53, 14, 0}, //53, ORIENT 1

            //{57, 1, 0}, //57, backend 1

            {12, 0, 1}, //12, TYPE 2
            {13, 1, 1}, //13, PIN 2
            {14, 2, 1}, //14, SCALING 2
            {15, 3, 1}, //15, OFFSET 2
            {16, 4, 1}, //16, FUNCTION 2
            {17, 5, 1}, //17, MIN_CM 2
            {18, 6, 1}, //18, MAX_CM 2
            {19, 7, 1}, //19, STOP_PIN 2
            {20, 8, 1}, //20, SETTLE 2
            {21, 9, 1}, //21, RMETRIC 2
            //{, 10, 1}, //PWRRNG 2 (previously existed only once for all sensors)
            {22, 11, 1}, //22, GNDCLEAR 2
            {24, 12, 1}, //24, ADDR 2
            {50, 13, 1}, //50, POS 2
            {54, 14, 1}, //54, ORIENT 2

            //{58, 3, 1}, //58, backend 2

            {25, 0, 2}, //25, TYPE 3
            {26, 1, 2}, //26, PIN 3
            {27, 2, 2}, //27, SCALING 3
            {28, 3, 2}, //28, OFFSET 3
            {29, 4, 2}, //29, FUNCTION 3
            {30, 5, 2}, //30, MIN_CM 3
            {31, 6, 2}, //31, MAX_CM 3
            {32, 7, 2}, //32, STOP_PIN 3
            {33, 8, 2}, //33, SETTLE 3
            {34, 9, 2}, //34, RMETRIC 3
            //{, 10, 2}, //PWRRNG 3 (previously existed only once for all sensors)
            {35, 11, 2}, //35, GNDCLEAR 3
            {36, 12, 2}, //36, ADDR 3
            {51, 13, 2}, //51, POS 3
            {55, 14, 2}, //55, ORIENT 3

            //{59, 5, 2}, //59, backend 3

            {37, 0, 3}, //37, TYPE 4
            {38, 1, 3}, //38, PIN 4
            {39, 2, 3}, //39, SCALING 4
            {40, 3, 3}, //40, OFFSET 4
            {41, 4, 3}, //41, FUNCTION 4
            {42, 5, 3}, //42, MIN_CM 4
            {43, 6, 3}, //43, MAX_CM 4
            {44, 7, 3}, //44, STOP_PIN 4
            {45, 8, 3}, //45, SETTLE 4
            {46, 9, 3}, //46, RMETRIC 4
            //{, 10, 3}, //PWRRNG 4 (previously existed only once for all sensors)
            {47, 11, 3}, //47, GNDCLEAR 4
            {48, 12, 3}, //48, ADDR 4
            {52, 13, 3}, //52, POS 4
            {56, 14, 3}, //56, ORIENT 4

            //{60, 7, 3}, //60, backend 4
    };

    char param_name[17] = {0};
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    info.old_key = 71;
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    info.old_key = 53;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    info.old_key = 35;
#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
    info.old_key = 197;
#else
    params[0].type.save(true);
    return; // no conversion is supported on this platform
#endif

    for (uint8_t i = 0; i < ARRAY_SIZE(conversionTable); i++) {
        uint8_t param_instance = conversionTable[i].instance + 1;
        uint8_t destination_index = conversionTable[i].new_index;

        info.old_group_element = conversionTable[i].old_element;
        info.type = (ap_var_type)AP_RangeFinder_Params::var_info[destination_index].type;

        hal.util->snprintf(param_name, sizeof(param_name), "RNGFND%X_%s", param_instance, AP_RangeFinder_Params::var_info[destination_index].name);
        param_name[sizeof(param_name)-1] = '\0';

        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }

    // force _params[0]._type into storage to flag that conversion has been done
    params[0].type.save(true);
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(enum Rotation orientation_default)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    convert_params();

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        params[i].orientation.set_default(orientation_default);
    }

    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (params[i].type == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
        }
    }

    Log_RFND();
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum RangeFinder_Type _type = (enum RangeFinder_Type)params[instance].type.get();
    switch (_type) {
    case RangeFinder_TYPE_PLI2C:
    case RangeFinder_TYPE_PLI2CV3:
    case RangeFinder_TYPE_PLI2CV3HP:
<<<<<<< HEAD
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type))) {
                break;
            }
=======
        if (!_add_backend(AP_RangeFinder_PulsedLightLRF::detect(1, state[instance], _type))) {
            _add_backend(AP_RangeFinder_PulsedLightLRF::detect(0, state[instance], _type));
>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0
        }
        break;
    case RangeFinder_TYPE_MBI2C:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(i, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)))) {
                break;
            }
        }
        break;
    case RangeFinder_TYPE_LWI2C:
        if (params[instance].address) {
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)));
#else
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                     hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
            }
#endif
        }
        break;
    case RangeFinder_TYPE_TRI2C:
        if (params[instance].address) {
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                      hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
            }
        }
        break;
    case RangeFinder_TYPE_VL53L0X:
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                                 hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
                if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
            }
        break;
    case RangeFinder_TYPE_BenewakeTFminiPlus:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_Benewake_TFMiniPlus::detect(state[instance], params[instance],
                                                                        hal.i2c_mgr->get_device(i, params[instance].address)))) {
                break;
            }
        }
        break;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    case RangeFinder_TYPE_PX4_PWM:
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height);
        }
        break;
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    case RangeFinder_TYPE_BBB_PRU:
        if (AP_RangeFinder_BBB_PRU::detect()) {
            drivers[instance] = new AP_RangeFinder_BBB_PRU(state[instance], params[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LightWareSerial(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LeddarOne(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ULANDING:
        if (AP_RangeFinder_uLanding::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_uLanding(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && defined(HAVE_LIBIIO)
    case RangeFinder_TYPE_BEBOP:
        if (AP_RangeFinder_Bebop::detect()) {
            drivers[instance] = new AP_RangeFinder_Bebop(state[instance], params[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_MAVLink:
        if (AP_RangeFinder_MAVLink::detect()) {
            drivers[instance] = new AP_RangeFinder_MAVLink(state[instance], params[instance]);
        }
        break;
    case RangeFinder_TYPE_MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_MaxsonarSerialLV(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ANALOG:
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance])) {
            drivers[instance] = new AP_RangeFinder_analog(state[instance], params[instance]);
        }
        break;
    case RangeFinder_TYPE_NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_NMEA(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_WASP:
        if (AP_RangeFinder_Wasp::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Wasp(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_BenewakeTF02:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], params[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TF02);
        }
        break;
    case RangeFinder_TYPE_BenewakeTFmini:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], params[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TFmini);
        }
        break;
    case RangeFinder_TYPE_PWM:
        if (AP_RangeFinder_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height);
        }
        break;
    case RangeFinder_TYPE_BLPing:
        if (AP_RangeFinder_BLPing::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_BLPing(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == RangeFinder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder_NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(mavlink_message_t *msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (params[i].type != RangeFinder_TYPE_NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        if (backend->orientation() != orientation) {
            continue;
        }
        return backend;
    }
    return nullptr;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

uint32_t RangeFinder::last_reading_ms(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

// Write an RFND (rangefinder) packet
void RangeFinder::Log_RFND()
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        const AP_RangeFinder_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_RFND pkt = {
                LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                dist         : s->distance_cm(),
                status       : (uint8_t)s->status(),
                orient       : s->orientation(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

RangeFinder *RangeFinder::_singleton;

namespace AP {

RangeFinder *rangefinder()
{
    return RangeFinder::get_singleton();
}

}
