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
#include <AP_MotionController/AP_MotionController.h>

extern const AP_HAL::HAL &hal; // External reference for console debugging

const AP_Param::GroupInfo AP_MotionController::var_info[] = {
    // @Param: MASK
    // @DisplayName: MOCR_MASK
    // @Description: One of four boards enabling mask, 0:Disable, 1:One Board, 2:Two Boards, 3:Three Boards, 4:Four Boards
    // @Range: 0 4
    // @Units: DN
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOCR", 1, AP_MotionController, bitmask, 0),

    AP_GROUPEND    
};

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
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz, 0);
    //hal.console->printf("AP_MotionController initialized.\n");
    //gcs().send_text(MAV_SEVERITY_INFO, "AP_MotionController Initialized.");
}

void AP_MotionController::update(void)
{
    //hal.console->printf("AP_MotionController updated.\n");
    //gcs().send_text(MAV_SEVERITY_INFO, "AP_MotionController Updated.");
}