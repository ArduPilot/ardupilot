/*
 *       Example of AC_AttitudeControl library
 *       DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_Baro_Glitch.h>     // Baro glitch protection library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AC_PID.h>             // PID library
#include <AC_P.h>               // P library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>
#include <AC_AttitudeControl.h>
#include <AC_PosControl.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::MultiCopter aparm;

// INS and Baro declaration
AP_InertialSensor ins;
AP_Baro baro;
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

// GPS declaration
AP_GPS gps;
GPS_Glitch gps_glitch(gps);
Baro_Glitch baro_glitch(baro);

AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, baro, gps);

// Inertial Nav declaration
AP_InertialNav inertial_nav(ahrs, baro, gps_glitch, baro_glitch);

// fake PIDs
AC_P   p_angle_roll, p_angle_pitch, p_angle_yaw;
AC_PID pid_rate_roll, pid_rate_pitch, pid_rate_yaw;
AC_P   p_alt_pos, p_pos_xy;
AC_P   pid_alt_rate;
AC_PID pid_alt_accel;
AC_PID pid_rate_lat, pid_rate_lon;

// fake RC inputs
RC_Channel rc_roll(CH_1), rc_pitch(CH_2), rc_yaw(CH_4), rc_throttle(CH_3);

// fake motor and outputs
AP_MotorsQuad motors(rc_roll, rc_pitch, rc_throttle, rc_yaw);
int16_t motor_roll, motor_pitch, motor_yaw, motor_throttle;

// Attitude Control
AC_AttitudeControl ac_control(ahrs, aparm, motors, p_angle_roll, p_angle_pitch, p_angle_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw);

/// Position Control
AC_PosControl pos_control(ahrs, inertial_nav, motors, ac_control, p_alt_pos, pid_alt_rate, pid_alt_accel, p_pos_xy, pid_rate_lat, pid_rate_lon);

void setup()
{
    hal.console->println("AC_AttitudeControl library test");
}

void loop()
{
    // print message to user
    hal.console->printf_P(PSTR("this example tests compilation only"));
    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();
