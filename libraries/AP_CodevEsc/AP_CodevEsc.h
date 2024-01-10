/// @file	AP_CodevEsc.h
/// @brief	exectue the codev esc
#pragma once
#include "drv_codev_esc.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Motors/AP_Motors_Class.h>

#define BOARD_TAP_ESC_MODE 2


// allow the board to override the number (or maxiumum number) of LED's it has
#ifndef HAL_ESC_NUM
#define HAL_ESC_NUM 4
#endif

#define LED_ON_TIME_MS  50
#define LED_OFF_TIME_MS 1450

// Circular from back right in CCW direction
#define ESC_POS {2, 1, 0, 3, 4, 5, 6, 7}
// 0 is CW, 1 is CCW
#define ESC_DIR {0, 1, 1, 0, 1 ,1, 1, 1}
const uint8_t _device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;
const uint8_t _device_dir_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_DIR;

class AP_CodevEsc
{
public:
    AP_CodevEsc(/* args */);
    ~AP_CodevEsc();

    /// Startup initialisation.
    void init();

    static AP_CodevEsc *get_singleton() { return _singleton; }

    bool uart_state() { return uart==nullptr?false:true;}

    void set_output_pwm(uint8_t chan,uint16_t pwm){ motor_out[chan] = pwm;};

    void set_vehicle_control_mode(uint8_t mode) {control_mode = mode;};
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    void execute_codev_esc();
    void receive_esc_status();

    Esc_Status _esc_status[HAL_ESC_NUM] = {};


    enum CONTROL_MODE_TYPE {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-automous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
    };

private:

    static AP_CodevEsc *_singleton;

    int configure_esc();
    // strobe the corresponding buffer channel
	void select_responder(uint8_t channel);
    void send_esc_outputs();
    void read_data_from_uart(ESC_UART_BUF *const uart_buf);
    int parse_tap_esc_feedback(ESC_UART_BUF *const serial_buf, EscPacket *const packetdata);
    uint8_t crc_packet(EscPacket &p);
    uint8_t crc8_esc(uint8_t *p, uint8_t len);
    void set_led_status(uint8_t id,uint8_t control_mode,uint16_t& led_status);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t baudrate = 0;

    uint8_t    	  channels_count = 0; 		///< nnumber of ESC channels
    int8_t 	    responding_esc = -1;
    uint16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    uint8_t led_on_off = 0;
    uint8_t control_mode = 0;

    unsigned long _esc_led_on_time_us = 0;
};

namespace AP {
    AP_CodevEsc *codevesc();
};
