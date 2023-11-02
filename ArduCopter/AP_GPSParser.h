#pragma once

#ifndef AP_GPSParser_H
#define AP_GPSParser_H

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
//#include <GPSData.h>

class AP_GPSParser {
public:
    AP_GPSParser();
    void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
    void setup();
    void process();
    void test_uart(AP_HAL::UARTDriver *uart, const char *name);
    void heleMoellen();
   // GPSData getGPSData();
private:
    AP_HAL::UARTDriver* uart;
    uint8_t mavlink_buffer[255];
    uint16_t mavlink_buffer_index;
    //GPSData gps_data;

    bool parseMavlinkByte(uint8_t byte);
    bool processMavlinkMessage(const uint8_t* buffer, uint16_t length);
};

#endif // AP_GPSParser_H
