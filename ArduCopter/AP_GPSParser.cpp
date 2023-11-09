#include "AP_GPSParser.h"



AP_GPSParser::AP_GPSParser() {

}

void AP_GPSParser::setup(){
    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
}

void AP_GPSParser::setup_uart(AP_HAL::UARTDriver *uart_param, const char *name){
this->uart = uart_param;
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
        hal.console->println("åh nej ikke godt");
    }
    uart->begin(57600);
    hal.console->println("den virker");
    uart->lock_port(17, 16); //Write key, Read key
}

void AP_GPSParser::process() {

    test_uart(hal.serial(3), "SERIAL3");

    if (uart->available_locked(16) > 0) {
        uint8_t received_byte;
        ssize_t bytesRead = uart->read_locked(&received_byte, 1, 16);

        if (bytesRead > 0) {
            if (parseMavlinkByte(received_byte)) {
                if (processMavlinkMessage(mavlink_buffer, mavlink_buffer_index)) {
                }
                mavlink_buffer_index = 0;
            }
        }
    }
}

void AP_GPSParser::test_uart(AP_HAL::UARTDriver *uart_param, const char *name)
{
    // Check if the UART port is available
    if (uart == nullptr) {
        hal.console->println("UART port not available.");
        return;
    }

    uart_param->write_locked(mavlink_buffer,"teesting \n");
    
    // Read data from the UART
    size_t bytesRead = uart->read(mavlink_buffer, sizeof(mavlink_buffer));

    if (bytesRead > 0) {
        // Print the received data to the console
        for (size_t i = 0; i < bytesRead; i++) {
          // hal.console->printf("%c \n", mavlink_buffer[i]);
        }
    }
}

bool AP_GPSParser::parseMavlinkByte(uint8_t byte) {
    if (mavlink_buffer_index < sizeof(mavlink_buffer)) {
        mavlink_buffer[mavlink_buffer_index] = byte;
        mavlink_buffer_index++;
        if (mavlink_buffer_index >= 6 + mavlink_buffer[1]) {
            return true;
        }
    }
    return false;
}

bool AP_GPSParser::processMavlinkMessage(const uint8_t* buffer, uint16_t length) {
    // Implement your logic to parse the MAVLink message and update gps_data
    // Return true if GPS data is successfully extracted, otherwise false
    return false; // Replace with your parsing logic
  
}


