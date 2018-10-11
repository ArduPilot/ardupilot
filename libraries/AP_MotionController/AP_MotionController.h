/* 
    File: AP_MotionController.h
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

#ifndef AP_MOTIONCONTROLLER_H_
#define AP_MOTIONCONTROLLER_H_

#include "AP_RoboClaw.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>

class AP_MotionController
{
public:
  AP_MotionController();
  ~AP_MotionController();
  static const AP_Param::GroupInfo var_info[];
  void update(uint16_t steering, uint16_t throtle);
  void init(void);
  static double constrain_map(double x, double in_min, double in_max, double out_min, double out_max);

protected:
  AP_Int8 bitmask, bitmask1;
  AP_HAL::UARTDriver *port;

private:
  AP_RoboClaw roboclaw;
};

#endif /*AP_MOTIONCONTROLLER_H_*/