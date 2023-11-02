#pragma once

#ifndef GPSPARSER_H
#define GPSPARSER_H

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
//#include <GPSData.h>

class GPSParser {
public:
    GPSParser();
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

#endif // GPSPARSER_H
