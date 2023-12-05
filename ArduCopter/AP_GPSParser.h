#pragma once

#ifndef AP_GPSParser_H
#define AP_GPSParser_H

#include <AP_HAL/AP_HAL.h>
//#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>



class AP_GPSParser {
public:
    AP_GPSParser();
    void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
    void setup();
    void process();
    void read_from_serial(AP_HAL::UARTDriver *uart, const char *name);
    void save_to_buffer(uint8_t data);
    bool get_isReady();
    uint8_t* get_buffer();
private:
    //extern AP_SerialManager serial_manager;
    bool is_ready;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver* uart;
    uint8_t mavlink_buffer[255];
    ssize_t mavlink_buffer_index;
};

#endif // AP_GPSParser_H
