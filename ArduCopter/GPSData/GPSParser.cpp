#include "GPSParser.h"

GPSParser::GPSParser(AP_HAL::UARTDriver* uartDriver) : uart(uartDriver), mavlink_buffer_index(0) {
}

void GPSParser::begin(uint32_t baud) {
    uart->begin(baud);
}

void GPSParser::process() {
    uint32_t key = 123; // Replace with your desired key

    while (uart->available_locked(key) > 0) {
        uint8_t received_byte;
        ssize_t bytesRead = uart->read_locked(&received_byte, 1, key);

        // 100hz loop
        if (millis() - last_run_time < 20) {
            hal.scheduler->delay(10);
            continue;
        }
        last_run_time = millis();


        if (bytesRead > 0) {
            if (parseMavlinkByte(received_byte)) {
                if (processMavlinkMessage(mavlink_buffer, mavlink_buffer_index)) {
                    // Process the GPS data, e.g., update the gps_data structure
                }
                mavlink_buffer_index = 0;
            }
        }
    }
}

GPSData GPSParser::getGPSData() {
    return gps_data;
}

bool GPSParser::parseMavlinkByte(uint8_t byte) {
    if (mavlink_buffer_index < sizeof(mavlink_buffer)) {
        mavlink_buffer[mavlink_buffer_index] = byte;
        mavlink_buffer_index++;
        if (mavlink_buffer_index >= 6 + mavlink_buffer[1]) {
            return true;
        }
    }
    return false;
}

bool GPSParser::processMavlinkMessage(const uint8_t* buffer, uint16_t length) {
    // Implement your logic to parse the MAVLink message and update gps_data
    // Return true if GPS data is successfully extracted, otherwise false
    return false; // Replace with your parsing logic
    
}
