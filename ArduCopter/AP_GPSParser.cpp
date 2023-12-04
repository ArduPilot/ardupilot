#include "AP_GPSParser.h"
#include "GCS_Mavlink.h"

//AP_SerialManager serial_manager;

// Constructor
AP_GPSParser::AP_GPSParser() : uart(nullptr), mavlink_buffer_index(0), is_ready(false) {}

// Setup function for initializing the GPS parser
void AP_GPSParser::setup() {
  if (!is_ready) {
        // Ensure that the uartA can be initialized
        // Initialize serial ports using AP_SerialManager
        //serial_manager.init();    
        hal.console->printf("Setting up serial 1");
        setup_uart(hal.serial(1), "SERIAL1");  // TELEM 1
        is_ready = true;  // Set the flag to indicate that setup is done
    } else {
        hal.console->println("GPS parser is already ready");
    }
}

// Setup UART with the specified UART driver and name
void AP_GPSParser::setup_uart(AP_HAL::UARTDriver *uart_param, const char *name) {
    this->uart = uart_param;
    
    // Check if the UART exists on this platform
    if (uart == nullptr) {
        hal.console->println("Failed to find serial");
        return;
    }  

    // Initialize UART with a baud rate
    uart->begin(57600);
    hal.scheduler->delay_microseconds(1000);
}

// Process function to handle the main logic of the GPS parser
void AP_GPSParser::process() {
    // Test UART communication
    test_uart(hal.serial(1), "SERIAL1");
}

// Test UART communication and perform actions accordingly
void AP_GPSParser::test_uart(AP_HAL::UARTDriver *uart_param, const char *name) {
    // Check if the UART port is available
    if (uart_param != nullptr) {
        hal.console->printf("Serial 1 ready");
        is_ready = true;

        // Example: Read and print received data
        uint8_t buffer[50];
        size_t bytesRead = uart_param->read(buffer, sizeof(buffer));
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';  // Null-terminate the string
            hal.console->printf("Received on %s: %s", name, buffer); 

            // Process MAVLink message
            processMavlinkMessage(buffer, bytesRead);       
        }
    } else {
        is_ready = false;
        hal.console->printf("UART %s not available.\n", name);
    }
}

// Parse a MAVLink byte and return true if a complete message is received
bool AP_GPSParser::parseMavlinkByte(uint8_t byte) {
    if (mavlink_buffer_index < sizeof(mavlink_buffer)) {
        mavlink_buffer[mavlink_buffer_index] = byte;
        mavlink_buffer_index++;
        if (mavlink_buffer_index >= 6 + mavlink_buffer[1]) {
            return true;  // Complete MAVLink message received
        }
    }
    return false;  // Incomplete MAVLink message
}


bool AP_GPSParser::processMavlinkMessage(const uint8_t* buffer, uint16_t length) {
    // Parse the MAVLink message
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (uint16_t i = 0; i < length; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            // MAVLink message parsed successfully
            // You can access message fields like msg.sysid, msg.msgid, etc.
            // Update your data based on the message content
            // For example, print the received message ID
            hal.console->printf("Received MAVLink Message ID: %u\n", msg.msgid);
        }
    }
    
    // Return true if GPS data is successfully extracted, otherwise false
    return false;  // Replace with your parsing logic 
}

// Getter function for the is_ready variable
bool AP_GPSParser::get_isReady() {
    return is_ready;
}
