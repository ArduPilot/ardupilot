
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include "analog_sensor.h"

#define BUTTON_ARRAY_LENGTH 4

class JoypadRemote {
public:

    void setup();
    void loop();

private:

    typedef struct __data_controller
    {
        uint8_t button_array[BUTTON_ARRAY_LENGTH];
        uint8_t dpad_left_on : 1;
        uint8_t dpad_up_on : 1;
        uint8_t dpad_right_on : 1;
        uint8_t dpad_down_on : 1;
        uint8_t dummy : 4;

        int16_t left_stick_x;
        int16_t left_stick_y;
        int16_t right_stick_x;
        int16_t right_stick_y;
        int16_t stick3_x;
        int16_t stick3_y;

    } data_controller_t;

    AP_Scheduler scheduler;
    AnalogSensor::state_t state[2];
    AnalogSensor *_analogsensor[2];

    data_controller_t controller_data_buffer1;
    data_controller_t controller_data_buffer2;
    data_controller_t controller_data;

    uint32_t ins_counter;
    static const AP_Scheduler::Task scheduler_tasks[];

    uint32_t nowmicros;
    uint8_t pin;
    int16_t filtered_value1, filtered_value2;

    volatile static bool _sending;

    void update_sensor(void);
    void send_data(void);
    void set_data(void);
    void live(void);
    data_controller_t get_empty_data_controller(void);
    void set_controller_data(data_controller_t controller_data_set);
};
