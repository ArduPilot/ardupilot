#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_RangeFinder_Params.h"
#include "AP_RangeFinder.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_RangeFinder_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini
    // @User: Standard
    AP_GROUPINFO("TYPE",    1, AP_RangeFinder_Params, type, 0),

    // @Param: PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 11:PX4-airspeed port, 15:Pixhawk-airspeed port
    // @User: Standard
    AP_GROUPINFO("PIN",     2, AP_RangeFinder_Params, pin, -1),

    // @Param: SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("SCALING", 3, AP_RangeFinder_Params, scaling, 3.0f),

    // @Param: OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
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

    // @Param: MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
    // @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_CM",  6, AP_RangeFinder_Params, min_distance_cm, 20),

    // @Param: MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
    // @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_CM",  7, AP_RangeFinder_Params, max_distance_cm, 700),

    // @Param: STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Standard
    AP_GROUPINFO("STOP_PIN", 8, AP_RangeFinder_Params, stop_pin, -1),

    // @Param: SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SETTLE", 9, AP_RangeFinder_Params, settle_time_ms, 0),

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

    // @Param: GNDCLEAR
    // @DisplayName: Distance (in cm) from the range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 5 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GNDCLEAR", 12, AP_RangeFinder_Params, ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADDR", 23, AP_RangeFinder_Params, address, 0),

    // @Param: POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the first rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the first rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the first rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS", 49, AP_RangeFinder_Params, pos_offset, 0.0f),

    // @Param: ORIENT
    // @DisplayName: Rangefinder orientation
    // @Description: Orientation of rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 53, AP_RangeFinder_Params, orientation, ROTATION_PITCH_270),

    AP_GROUPEND
};

AP_RangeFinder_Params::AP_RangeFinder_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
