#include "AP_GPSParser.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
AP_SerialManager serial_manager;

AP_GPSParser::AP_GPSParser() :  uart(nullptr), mavlink_buffer_index(0){
} 

void AP_GPSParser::setup() {
    // Ensure that the uartA can be initialized
    // Initialize serial ports using AP_SerialManager
    serial_manager.init();  
    setup_uart(hal.serial(1), "SERIAL1");  // TELEM 1
}

void AP_GPSParser::setup_uart(AP_HAL::UARTDriver *uart_param, const char *name){
this->uart = uart_param;
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        hal.console->println("Failed to find serial");
        return;
    }  
    uart->begin(57600);
    hal.console->println("setup version kurt");
    }

void AP_GPSParser::process() {  
    test_uart(serial_manager.get_serial_by_id(1), "SERIAL1");
}

void AP_GPSParser::test_uart(AP_HAL::UARTDriver *uart_param, const char *name) {
    // Check if the UART port is available
    if (uart_param == nullptr) {
        //that UART doesn't exist on this platform
        hal.console->println("Failed to find serial");
        return;
    }else{
        hal.console->println("Found serial");
        //print the incoming data as a string
        uint8_t inc_data = uart_param->read();
        hal.console->printf("Recieved data: %c \n", inc_data);          

    if (inc_data == 0xFE) {
        hal.console->println("Found 0xFE");
        mavlink_buffer_index = 0;
    }
}
}

