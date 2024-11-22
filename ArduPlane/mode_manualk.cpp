/*#include "mode.h"
#include "Plane.h"

void ModeManualK::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);

    plane.nav_roll_cd = plane.AP::ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.AP::ahrs.pitch_sensor;
}
*/

#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if HAL_WITH_IO_MCU || CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>

#include "mode.h"
#include "Plane.h"
#include "zef_control.h"

using namespace AP;

//static AP_Baro barometer;
// Serial manager is needed for UART communications
static AP_SerialManager serial_manager;

//static uint16_t pwm = 1500;
//static int8_t delta = 1;


bool ModeManualK::_enter()
{
    rc().channel(CH_5)->set_range(1900);
    rc().channel(CH_6)->set_range(1900);
    
    //rc().channel(CH_7)->set_range(1900);
    
    //barometer.init();
    //barometer.calibrate();
    
    /*for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }*/
    
    // Initialize the UART for GPS system
    //serial_manager.init();
    //gps().init(serial_manager);
    
    zefiroControl.set_manual(true);
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, plane.get_throttle_input(true));
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, plane.get_throttle_input(true));
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, plane.get_throttle_input(true));
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, plane.get_throttle_input(true));
    
    return true;
}

void ModeManualK::_exit()
{
    /*for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->disable_ch(i);
    }*/
    //zefiroControl.stopInflators();
}

void ModeManualK::update()
{
    static uint32_t last_msg_ms;
    // Update GPS state based on possible bytes received from the module.
    gps().update();
    
    // If new GPS data is received, output its contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    if (last_msg_ms != gps().last_message_time_ms()) {
        // Reset the time of message
        last_msg_ms = gps().last_message_time_ms();

        /*// Acquire location
        const Location &loc = gps().location();
        float ground_speed = gps().ground_speed(); // Get ground speed in meters per second

        // Print the contents of message
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, " Lat: %.2fm \n"
                                        " Lon: %.2fm \n"
                                        " Alt: %.2fm \n"
                                        " Ground speed: %.2fm \n",
                                        (double)loc.lat,
                                        (double)loc.lng,
                                        (double)(loc.alt * 0.01f),
                                        (double)ground_speed);*/
    }
    
    //plane.barometer.accumulate();
    plane.barometer.update();
    
    zefiroControl.set_RHO(plane.barometer.get_pressure());
    
    /*if (barometer.healthy()) {
        //output barometer readings to console
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, " Pressure: %.2f Pa\n"
                            " Temperature: %.2f degC\n"
                            " Relative Altitude: %.2f m\n"
                            " climb=%.2f m/s\n"
                            "\n",
                            (double)barometer.get_pressure(),
                            (double)barometer.get_temperature(),
                            (double)barometer.get_altitude(),
                            (double)barometer.get_climb_rate());
    }*/
    
    //plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);
    
    AP::ahrs().update();
    float pitch = AP::ahrs().get_pitch(); // Get pitch angle in degrees
    float yaw = AP::ahrs().get_yaw(); // Get yaw angle in degrees
    float roll = AP::ahrs().get_roll(); // Get roll angle in degrees
    
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch: %f --- yaw: %f --- roll: %f", pitch, yaw, roll);
    
    // Read angular rates for roll, pitch, and yaw
    Vector3f angular_gyro = AP::ahrs().get_gyro();
    float angular_pitch = angular_gyro.x; // Get angular rate for pitch in degrees per second
    float angular_roll = angular_gyro.y; // Get angular rate for roll in degrees per second
    float angular_yaw = angular_gyro.z; // Get angular rate for yaw in degrees per second
    
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang_pitch: %f --- ang_yaw: %f --- ang_roll: %f", angular_pitch, angular_yaw, angular_roll);
    
    //zefiroControl.update(double U_longit_speed, double V_lateral_speed, double W_vertical_speed,
    //    double P_v_roll, double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw);
    
    
    Vector3f wind;
    //double wind_direction = AP::windvane()->get_apparent_wind_direction_rad();
    //double wind_speed = wind.xy().length(); // Magnitude da velocidade do vento
    double vx = -wind.x;
    double vy = -wind.y;

    
    double U_longit_speed = vx;
    double V_lateral_speed = vy;
    double W_vertical_speed = plane.barometer.get_climb_rate();
    double P_v_roll = angular_roll;
    double Q_v_pitch = angular_pitch;
    double R_v_yaw = angular_yaw;
    
    double control_aileron = rc().channel(CH_1)->get_control_in();//plane.roll_in_expo(false);
    double control_pitch = rc().channel(CH_2)->get_control_in();//plane.pitch_in_expo(false);
    double control_yaw = rc().channel(CH_4)->get_control_in();//plane.rudder_in_expo(false);
    double control_throttle = rc().channel(CH_3)->get_control_in();//plane.get_throttle_input(true);
    double control_right_switch = rc().channel(CH_5)->get_control_in();
    double control_left_switch = rc().channel(CH_6)->get_control_in();
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "rc5: %f || rc6: %f", control_right_switch, control_left_switch);
    
    int normalizador_inputs_apy = 4500;
    int normalizador_throttle = 50;
    int zero_throttle = 50;
    int normalizador_aux = 1900/2;
    int zero_aux = 1900/2;
    
    control_aileron /= normalizador_inputs_apy;
    control_pitch /= normalizador_inputs_apy;
    control_yaw /= normalizador_inputs_apy;
    control_throttle = (control_throttle - zero_throttle) / normalizador_throttle;
    control_left_switch = (control_left_switch - zero_aux) / normalizador_aux;
    control_right_switch = (control_right_switch - zero_aux) / normalizador_aux;
    
    double dead_zone = 0.05;
    control_aileron = zefiroControl.dead_zone(control_aileron, dead_zone);
    control_pitch = zefiroControl.dead_zone(control_pitch, dead_zone);
    control_yaw = zefiroControl.dead_zone(control_yaw, dead_zone);
    control_throttle = zefiroControl.dead_zone(control_throttle, dead_zone);
    control_left_switch = zefiroControl.dead_zone(control_left_switch, dead_zone);
    control_right_switch = zefiroControl.dead_zone(control_right_switch, dead_zone);
    
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "aileron: %f", control_aileron);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch: %f", control_pitch);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "yaw: %f", control_yaw);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "throttle: %f", control_throttle);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ch5: %f", control_left_switch);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ch6: %f", control_right_switch);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "======");*/

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang_flap: %f --- ang_airb: %f", control_right_switch, control_left_switch);

    zefiroControl.manual_inputs_update(control_aileron, control_pitch, control_yaw, control_throttle, control_right_switch, control_left_switch);
    
    zefiroControl.update( U_longit_speed, V_lateral_speed, W_vertical_speed,
        P_v_roll, Q_v_pitch, R_v_yaw,
        roll, pitch, yaw,
        0.0, 0.0, 0.0);
    
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "d1_angulo_motor: %f", zefiroControl.d1_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "d2_angulo_motor: %f", zefiroControl.d2_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "d3_angulo_motor: %f", zefiroControl.d3_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "d4_angulo_motor: %f", zefiroControl.d4_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=======");*/

    //double F1_forca_motor, F2_forca_motor, F3_forca_motor, F4_forca_motor;
    //double d1_angulo_motor, d2_angulo_motor, d3_angulo_motor, d4_angulo_motor;
    int16_t angle_min = 500;
    int16_t angle_max = 2500;
    
    //double d1_angulo_motor, d2_angulo_motor, d3_angulo_motor, d4_angulo_motor;
    //double dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo, dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo;
    int16_t output_d1_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d1_angulo_motor, angle_min, angle_max);
    int16_t output_d2_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d2_angulo_motor, angle_min, angle_max);
    int16_t output_d3_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d3_angulo_motor, angle_min, angle_max);
    int16_t output_d4_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d4_angulo_motor, angle_min, angle_max);
    
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang1: %d", output_d1_angulo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang2: %d", output_d2_angulo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang3: %d", output_d3_angulo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang4: %d", output_d4_angulo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "====");*/
    
    /*
        k_motor_tilt            = 41,            ///< tiltrotor motor tilt control
        k_tiltMotorRear         = 45,            ///<vectored thrust, rear tilt
        k_tiltMotorRearLeft     = 46,            ///<vectored thrust, rear left tilt
        k_tiltMotorRearRight    = 47,            ///<vectored thrust, rear right tilt
    */
    SRV_Channels::move_servo(SRV_Channel::k_motor_tilt, output_d1_angulo, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearRight, output_d2_angulo, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRear, output_d3_angulo, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearLeft, output_d4_angulo, angle_min, angle_max);
    
    /*SRV_Channels::move_servo(SRV_Channel::k_mount2_roll, output_d1_angulo, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_mount2_tilt, output_d2_angulo, angle_min, angle_max);
    //SRV_Channels::move_servo(SRV_Channel::k_mount2_roll, output_d3_angulo, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_mount2_pan, output_d4_angulo, angle_min, angle_max);*/
    
    //controles da empenagem
    int16_t stab_angle_min = 1100;
    int16_t stab_angle_max = 1900;
    int16_t output_vert_stabilizer = zefiroControl.get_value_to_pwm_servo(control_yaw, angle_min, angle_max);
    int16_t output_hor_stabilizer = zefiroControl.get_value_to_pwm_servo(control_pitch, angle_min, angle_max);
    //SRV_Channels::move_servo(SRV_Channel::k_rudder, output_vert_stabilizer, stab_angle_min, stab_angle_max);
    //SRV_Channels::move_servo(SRV_Channel::k_elevator, output_hor_stabilizer, stab_angle_min, stab_angle_max);
    //    k_mount_pan             = 6,            ///< mount yaw (pan)
    //    k_mount_tilt            = 7,            ///< mount pitch (tilt)
    SRV_Channels::move_servo(SRV_Channel::k_mount_pan, output_vert_stabilizer, stab_angle_min, stab_angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_mount_tilt, output_hor_stabilizer, stab_angle_min, stab_angle_max);
    
    
    //zefiroControl.operateInflators(&plane.barometer, plane.g2.min_balloon_pressure_diff, plane.g2.max_balloon_pressure_diff);
    
    int16_t min_motor = 1000;
    int16_t max_motor = 2000;
    if( AP::arming().is_armed() ) {
        /*int F1 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F1_forca_motor, min_motor, max_motor);
        int F2 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F2_forca_motor, min_motor, max_motor);
        int F3 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F3_forca_motor, min_motor, max_motor);
        int F4 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F4_forca_motor, min_motor, max_motor);
        */
        int F1 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M1, min_motor, max_motor);
        int F2 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M2, min_motor, max_motor);
        int F3 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M3, min_motor, max_motor);
        int F4 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M4, min_motor, max_motor);
        
        SRV_Channels::move_servo(SRV_Channel::k_motor1, F1, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor2, F2, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor3, F3, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor4, F4, min_motor, max_motor);
    } else {
        SRV_Channels::move_servo(SRV_Channel::k_motor1, 0, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor2, 0, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor3, 0, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor4, 0, min_motor, max_motor);
    }

    //plane.nav_roll_cd = plane.AP::ahrs.roll_sensor;
    //plane.nav_pitch_cd = plane.AP::ahrs.pitch_sensor;
    //zefiroControl.TCA9548A(0);
    
}


