/* 
    File: AP_MotionController.cpp
    Author: Dr. -Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Created: 07 Oct 2018
    Last Modified: 07 Oct 2018
    Contributation: Dr. -Ing. Ahmad Kamal Nasir, 
    Description: 
    This library is used to communicate with motion control board such as 
    roboclaw through serial port communication. It uses the serial_manager
    to get the first instance of the serial port containing ESC Telemetry 
    protocol (16). Some parameters are also defined which can be accessed
    through mission planner GUI interface in order to configure roboclaws
    settings.Up to four roboclaw boards can be connected to the same
    serial bus and can be managed by the library. Each roboclaw can control
    two motors, therefore, eight motors in total. 
 */
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>
#include "AP_MotionController.h"

extern const AP_HAL::HAL &hal; // External reference for console debugging

const AP_Param::GroupInfo AP_MotionController::var_info[] = {
    // @Param: MASK
    // @DisplayName: SERVO_MOCR_MASK
    // @Description: One of four boards enabling mask, 0:Disable, 1:One Board, 2:Two Boards, 3:Three Boards, 4:Four Boards
    // @Range: 0 4
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MASK", 0, AP_MotionController, bitmask, 0),

    AP_GROUPEND};

AP_MotionController::AP_MotionController()
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

AP_MotionController::~AP_MotionController()
{
}

void AP_MotionController::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ESCTelemetry, 0);
    roboclaw.init(port, 10000);
    //hal.console->printf("AP_MotionController initialized.\n");
    //gcs().send_text(MAV_SEVERITY_INFO, "AP_MotionController Initialized.");
}

double AP_MotionController::constrain_map(double x, double in_min, double in_max, double out_min, double out_max)
{
    //constrain the input value between in_min and in_max
    if (x < in_min)
        x = in_min;
    else if (x > in_max)
        x = in_max;
    //map the value from the scale (in_min->in_max) into scale (out_min->out_max)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void AP_MotionController::update(uint16_t steering, uint16_t throttle)
{
    static int counter = 0;
    uint32_t m1 = AP_MotionController::constrain_map(steering, SRV_Channels::srv_channel(1)->get_output_min(), SRV_Channels::srv_channel(1)->get_output_max(), 0, 127);
    uint32_t m2 = AP_MotionController::constrain_map(steering, SRV_Channels::srv_channel(3)->get_output_min(), SRV_Channels::srv_channel(3)->get_output_max(), 0, 127);
    roboclaw.ForwardBackwardM1(AP_RoboClaw::ROBOCLAW_ADDR, m1);
    roboclaw.ForwardBackwardM2(AP_RoboClaw::ROBOCLAW_ADDR, m2);
    //roboclaw.SpeedM1M2(AP_RoboClaw::ROBOCLAW_ADDR, steering, throttle);
    if (++counter >= 10) // Call every fifth of a second (50Hz calling rate)
    {
        //hal.console->printf("Time:%05.3f Steering:%d Throttle:%d\n", AP_HAL::millis64() / 1000.0, m1, m2);
        //port->printf("Time:%05.3f Steering:%d Throttle:%d\n", AP_HAL::millis64()/1000.0, steering, throttle);
        //gcs().send_text(MAV_SEVERITY_INFO, "Time:%05.3f Steering:%d Throttle:%d", AP_HAL::millis64()/1000.0, steering, throttle);
        DataFlash_Class::instance()->Log_Write("MOCR", "TimeUS,Steering,Throttle", "Qff", AP_HAL::micros64(), m1, m2);
        counter = 0;
    }
}