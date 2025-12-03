#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include "AP_RangeFinder_Params.h"
#include "AP_RangeFinder.h"

#ifndef AP_RANGEFINDER_DEFAULT_ORIENTATION
#define AP_RANGEFINDER_DEFAULT_ORIENTATION ROTATION_PITCH_270
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_RangeFinder_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Rangefinder type
    // @Description: Type of connected rangefinder
    // @SortValues: AlphabeticalZeroAtTop
    // @Values: 0:None
    // @Values: 1:Analog
    // @Values: 2:MaxbotixI2C
    // @Values: 3:LidarLite-I2C
    // @Values: 5:PWM
    // @Values: 6:BBB-PRU
    // @Values: 7:LightWareI2C
    // @Values: 8:LightWareSerial
    // @Values: 9:Bebop
    // @Values: 10:MAVLink
    // @Values: 11:USD1_Serial
    // @Values: 12:LeddarOne
    // @Values: 13:MaxbotixSerial
    // @Values: 14:TeraRangerI2C
    // @Values: 15:LidarLiteV3-I2C
    // @Values: 16:VL53L0X or VL53L1X
    // @Values: 17:NMEA
    // @Values: 18:WASP-LRF
    // @Values: 19:BenewakeTF02
    // @Values: 20:BenewakeTFmini-Serial
    // @Values: 21:LidarLightV3HP
    // @Values: 22:PWM
    // @Values: 23:BlueRoboticsPing
    // @Values: 24:DroneCAN
    // @Values: 25:BenewakeTFmini-I2C
    // @Values: 26:LanbaoPSK-CM8JL65-CC5
    // @Values: 27:BenewakeTF03
    // @Values: 28:VL53L1X-ShortRange
    // @Values: 29:LeddarVu8-Serial
    // @Values: 30:HC-SR04
    // @Values: 31:GYUS42v2
    // @Values: 32:MSP
    // @Values: 33:USD1_CAN
    // @Values: 34:Benewake_CAN
    // @Values: 35:TeraRangerSerial
    // @Values: 36:Lua_Scripting
    // @Values: 37:NoopLoop_TOFSense
    // @Values: 38:NoopLoop_TOFSense_CAN
    // @Values: 39:NRA24_CAN
    // @Values: 40:NoopLoop_TOFSenseF_I2C
    // @Values: 41:JRE_Serial
    // @Values: 42:Ainstein_LR_D1
    // @Values: 43:RDS02UF
    // @Values: 44:HexsoonRadar
    // @Values: 45:LightWare-GRF
    // @Values: 46:BenewakeTFS20L
    // @Values: 100:SITL
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_RangeFinder_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog or PWM input pin that rangefinder is connected to. Analog RSSI or Airspeed ports can be used for Analog inputs (some autopilots provide others also), Non-IOMCU Servo/MotorOutputs can be used for PWM input when configured as "GPIOs". Values for some autopilots are given as examples. Search wiki for "Analog pins" for analog pin or "GPIOs", if PWM input type, to determine pin number.
    // @Values: -1:Not Used,11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @Range: -1 127
    // @User: Standard
    AP_GROUPINFO("PIN",     2, AP_RangeFinder_Params, pin, -1),

    // @Param: SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("SCALING", 3, AP_RangeFinder_Params, scaling, 3.0f),

    // @Param: OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("OFFSET",  4, AP_RangeFinder_Params, offset, 0.0f),

    // @Param: FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("FUNCTION", 5, AP_RangeFinder_Params, function, 0),

    // @Param: MIN
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in metres that rangefinder can reliably read
    // @Units: m
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MIN",  6, AP_RangeFinder_Params, min_distance, 0.20),

    // @Param: MAX
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in metres that rangefinder can reliably read
    // @Units: m
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MAX",  7, AP_RangeFinder_Params, max_distance, 7.00),

    // @Param: STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for the pwm rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This is used to enable powersaving when out of range. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @Values: -1:Not Used,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @Range: -1 127
    // @User: Standard
    AP_GROUPINFO("STOP_PIN", 8, AP_RangeFinder_Params, stop_pin, -1),

    // 9 was SETTLE

    // @Param: RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Standard
    AP_GROUPINFO("RMETRIC", 10, AP_RangeFinder_Params, ratiometric, 1),

    // @Param: PWRRNG
    // @DisplayName: Powersave range
    // @Description: This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
    // @Units: m
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("PWRRNG", 11, AP_RangeFinder_Params, powersave_range, 0),

    // @Param: GNDCLR
    // @DisplayName: Distance from the range finder to the ground
    // @Description: This parameter sets the expected range measurement that the range finder should return when the vehicle is on the ground.
    // @Units: m
    // @Range: 0.05 1.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("GNDCLR", 12, AP_RangeFinder_Params, ground_clearance, RANGEFINDER_GROUND_CLEARANCE_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the I2C and DroneCAN sensors to allow for multiple sensors on different addresses.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADDR", 23, AP_RangeFinder_Params, address, 0),

    // @Param: POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS", 49, AP_RangeFinder_Params, pos_offset, 0.0f),

    // @Param: ORIENT
    // @DisplayName: Rangefinder orientation
    // @Description: Orientation of rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 53, AP_RangeFinder_Params, orientation, AP_RANGEFINDER_DEFAULT_ORIENTATION),

    AP_GROUPEND
};


// PARAMETER_CONVERSION - Added: Dec-2024 for 4.7
void AP_RangeFinder_Params::convert_min_max_params(void)
{
    // ./Tools/autotest/test_param_upgrade.py --vehicle=arducopter --param "RNGFND1_MAX_CM=300->RNGFND1_MAX=3.00" --param "RNGFND2_MIN_CM=678->RNGFND2_MIN=6.78" --param "RNGFNDA_MIN_CM=1->RNGFNDA_MIN=0.01" --param "RNGFND5_GNDCLEAR=103->RNGFND5_GNDCLR=1.03"
    max_distance.convert_parameter_width(AP_PARAM_INT16, 0.01);
    min_distance.convert_parameter_width(AP_PARAM_INT16, 0.01);
    ground_clearance.convert_parameter_width(AP_PARAM_INT8, 0.01);
}

AP_RangeFinder_Params::AP_RangeFinder_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_RANGEFINDER_ENABLED
