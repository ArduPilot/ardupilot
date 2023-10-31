#ifndef GPSPARSER_H
#define GPSPARSER_H

#include "AP_HAL.h"
#include "GPSData.h"

class GPSParser {
public:
    GPSParser(AP_HAL::UARTDriver* uartDriver);
    void begin(uint32_t baud);
    void process();
    GPSData getGPSData();

private:
    AP_HAL::UARTDriver* uart;
    uint8_t mavlink_buffer[255];
    uint16_t mavlink_buffer_index;
    GPSData gps_data;

    bool parseMavlinkByte(uint8_t byte);
    bool processMavlinkMessage(const uint8_t* buffer, uint16_t length);
};

#endif // GPSPARSER_H
