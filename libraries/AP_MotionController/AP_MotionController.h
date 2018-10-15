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

#define AP_MOTIONCONTROLLER_IX 0.2360f
#define AP_MOTIONCONTROLLER_IY 0.2645f
#define AP_MOTIONCONTROLLER_RW 0.0790f
#define AP_MOTIONCONTROLLER_QPPR 908
#define AP_MOTIONCONTROLLER_VEL_KP 1000.0f
#define AP_MOTIONCONTROLLER_VEL_KI 1000.0f
#define AP_MOTIONCONTROLLER_VEL_KD 0.0f
#define AP_MOTIONCONTROLLER_VEL_QPPS 908
#define AP_MOTIONCONTROLLER_POS_KP 100.0f
#define AP_MOTIONCONTROLLER_POS_KI 1.0f
#define AP_MOTIONCONTROLLER_POS_KD 0.0f
#define AP_MOTIONCONTROLLER_POS_IMAX 0.0f
#define AP_MOTIONCONTROLLER_POS_DEADBAND 0.0f
#define AP_MOTIONCONTROLLER_POS_MAXPOS 50.0f
#define AP_MOTIONCONTROLLER_POS_MINPOS -50.0f
#define AP_MOTIONCONTROLLER_POS_ACCEL 50.0f
#define AP_MOTIONCONTROLLER_POS_VEL 50.0f

class AP_MotionController
{
private:
  struct control
  {
    AP_Float kp, ki, kd;                           // PID controller constants
    int32_t setpoint;                              // Control setpoint
    AP_Int32 qpps, maxi, deadband, maxpos, minpos; // Additional ontroller parameters
  };

  struct motor
  {
    control vel, pos;                // Velocity and Position Controller parameters
    AP_Int32 velocity, acceleration; // Desired velocity, acceleration/deacceleration for position control
    uint32_t current, depth;         // Roboclaw sensor
    int32_t encoder, speed;          // Motor senors
    uint8_t stat;                    // Status flag
    bool valid;                      // Sent command status
  };

  struct settings
  {
    motor m1, m2;                          // Two motors per roboclaw board, m1 = velocity, m2 = position
    AP_Int16 address;                      // Address of the roboclaw board, Range:[0x80-0x90]
    uint16_t logicVoltage, batteryVoltage; // Logic and Main battery voltage measurement
    uint32_t temperature;                  // Temperature of the board
  };

public:
  AP_MotionController();
  ~AP_MotionController();
  static const AP_Param::GroupInfo var_info[];
  void update(uint16_t steering, uint16_t throtle);
  void init(void);
  static double constrain_map(double x, double in_min, double in_max, double out_min, double out_max);

protected:
  AP_Float Ix;              // Longitudinal wheel offset from robot center [m]
  AP_Float Iy;              // Lateral wheel offset from robot center [m]
  AP_Float Rw;              // Wheel Radius [m]
  AP_Float QPPR;            // Quadrature pulses per revolution
  AP_HAL::UARTDriver *port; // Serial port configured through serial_manager
  AP_RoboClaw roboclaw;     // Roboclaw communication protocol implementation
  settings rc[4];           // Structure variable to store roboclaw boards settings and data
};

#endif /*AP_MOTIONCONTROLLER_H_*/