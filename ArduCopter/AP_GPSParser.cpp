#include "AP_GPSParser.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
AP_SerialManager serial_manager;
//AP_SerialManager.SerialProtocol *protocol = AP::serialmanager().SerialProtocol_MAVLink;
AP_HAL::UARTDriver *port = serial_manager.find_serial(AP::serialmanager().SerialProtocol_MAVLink,0);

AP_GPSParser::AP_GPSParser() :  uart(nullptr), mavlink_buffer_index(0){

} 

void AP_GPSParser::setup() {
    // Ensure that the uartA can be initialized
    hal.scheduler->delay_microseconds(10000);

    // Initialize serial ports using AP_SerialManager
    serial_manager.init();
  // serial_manager.init_console();
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
}

void AP_GPSParser::setup_uart(AP_HAL::UARTDriver *uart_param, const char *name){
    
this->uart = uart_param;
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        hal.console->println("åh nej ikke godt");
        return;
    }
    uart->begin(57600);
    hal.console->println("den virker");
}

void AP_GPSParser::process() {
    test_uart(port, "SERIAL3");
    if(serial_manager.have_serial(AP::serialmanager().SerialProtocol_MAVLink, 0)){
        hal.console->println("Serial 0 connection obtained");
    }
      if(serial_manager.have_serial(AP::serialmanager().SerialProtocol_MAVLink, 1)){
        hal.console->println("Serial 1 connection obtained");
    }
      if(serial_manager.have_serial(AP::serialmanager().SerialProtocol_MAVLink, 2)){
        hal.console->println("Serial 2 connection obtained");
    }
      if(serial_manager.have_serial(AP::serialmanager().SerialProtocol_MAVLink, 3)){
        hal.console->println("Serial 3 connection obtained");
    }
}

void AP_GPSParser::test_uart(AP_HAL::UARTDriver *uart_param, const char *name) {
    // Check if the UART port is available
    if (uart_param != nullptr) {
        // Test UART communication here
        // Example: Send a message
        const char *message = "Testing UART\n";
        uart_param->write(message);

        // Example: Read and print received data
        uint8_t buffer[50];
        size_t bytesRead = uart_param->read(buffer, sizeof(buffer));

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';  // Null-terminate the string
            hal.console->printf("Received on %s: %s", name, buffer);
        }
    } else {
        hal.console->printf("UART %s not available.\n", name);
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


