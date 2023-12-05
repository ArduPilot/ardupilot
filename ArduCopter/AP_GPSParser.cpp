/******************
Class responsible for establishing connection to any unused serial port and read incoming data from it. Its saves the incoming to a one Byte buffer. 

TO use this class, you need to include the following in your code:

#include "AP_GPSParser.h"
get_isReady() returns true if the buffer is ready to be parsed
get_buffer() returns the buffer

process() processes the incoming data and saves it to the buffer
setup() sets up the serial port

You can then use the buffer in your code to parse the data
*******************/

#include "AP_GPSParser.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

AP_SerialManager serial_manager;

AP_GPSParser::AP_GPSParser() :  uart(nullptr), mavlink_buffer_index(0){
} 

void AP_GPSParser::setup() {
    // Initialize serial ports using AP_SerialManager
    serial_manager.init();  
    setup_uart(hal.serial(1), "SERIAL1");  // TELEM 1
}

void AP_GPSParser::setup_uart(AP_HAL::UARTDriver *uart_param, const char *name){
this->uart = uart_param;
    if (uart == nullptr) {
        hal.console->println("Failed to find serial");
        return;
    }  
    uart->begin(57600);
}

void AP_GPSParser::process() {  
    read_from_serial(serial_manager.get_serial_by_id(1), "SERIAL1");
    if (get_isReady()) {
        get_buffer();
        mavlink_buffer[0] = '\0';
        mavlink_buffer_index = 0;
    }   
}

void AP_GPSParser::read_from_serial(AP_HAL::UARTDriver *uart_param, const char *name) {
    if (uart_param == nullptr) {
        hal.console->println("Failed to find serial");
        return;
    } else{
   while (uart_param->available()) {
            uint8_t inc_data = uart_param->read();
            save_to_buffer(inc_data);
        }
    }
}

void AP_GPSParser::save_to_buffer(uint8_t data){
    if (mavlink_buffer_index < sizeof(mavlink_buffer) - 1) {
        mavlink_buffer[mavlink_buffer_index] = data;
        mavlink_buffer_index++;
        mavlink_buffer[mavlink_buffer_index] = '\0'; // Null-terminate the buffer
    }
    }

bool AP_GPSParser::get_isReady(){
    if (mavlink_buffer_index > 50) {
        return true;
    } else{
        return false;
    }
}

uint8_t* AP_GPSParser::get_buffer(){
    hal.console->printf("Buffer: %s\n", mavlink_buffer);
    return mavlink_buffer;
}