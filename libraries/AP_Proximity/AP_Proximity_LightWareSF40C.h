#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_SF40C_TIMEOUT_MS            200                               // requests timeout after 0.2 seconds

class AP_Proximity_LightWareSF40C : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_LightWareSF40C(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    enum RequestType {
        RequestType_None = 0,
        RequestType_Health,
        RequestType_MotorSpeed,
        RequestType_MotorDirection,
        RequestType_ForwardDirection,
        RequestType_DistanceMeasurement
    };

    // initialise sensor (returns true if sensor is successfully initialised)
    bool initialise();
    void init_sectors();
    void set_motor_speed(bool on_off);
    void set_motor_direction();
    void set_forward_direction();

    // send request for something from sensor
    void request_new_data();
    void send_request_for_health();
    bool send_request_for_distance();

    // check and process replies from sensor
    bool check_for_reply();
    bool process_reply();
    void clear_buffers();

    // reply related variables
    AP_HAL::UARTDriver *uart = nullptr;
    char element_buf[2][10];
    uint8_t element_len[2];
    uint8_t element_num;
    bool ignore_reply;                      // true if we should ignore the incoming message (because it is just echoing our command)
    bool wait_for_space;                    // space marks the start of returned data

    // request related variables
    enum RequestType _last_request_type;    // last request made to sensor
    uint8_t  _last_sector;                  // last sector requested
    uint32_t _last_request_ms;              // system time of last request
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    uint8_t _request_count;                 // counter used to interleave requests for distance with health requests

    // sensor health register
    union {
        struct PACKED {
            uint16_t motor_stopped : 1;
            uint16_t motor_dir : 1;          // 0 = clockwise, 1 = counter-clockwise
            uint16_t motor_fault : 1;
            uint16_t torque_control : 1;     // 0 = automatic, 1 = manual
            uint16_t laser_fault : 1;
            uint16_t low_battery : 1;
            uint16_t flat_battery : 1;
            uint16_t system_restarting : 1;
            uint16_t no_results_available : 1;
            uint16_t power_saving : 1;
            uint16_t user_flag1 : 1;
            uint16_t user_flag2 : 1;
            uint16_t unused1 : 1;
            uint16_t unused2 : 1;
            uint16_t spare_input : 1;
            uint16_t major_system_abnormal : 1;
        } _flags;
        uint16_t value;
    } _sensor_status;

    // sensor config
    uint8_t _motor_speed;               // motor speed as reported by lidar
    uint8_t _motor_direction = 99;      // motor direction as reported by lidar
    int16_t _forward_direction = 999;   // forward direction as reported by lidar
    bool _sector_initialised = false;
};
