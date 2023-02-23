#pragma once

#include <AP_HAL/AP_HAL.h>


#ifndef AP_IQUART_ENABLED
#define AP_IQUART_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif

#include <AP_Param/AP_Param.h>
#include "generic_interface.hpp"
#include "client_communication.hpp"
#include <AP_IQMotor/brushless_drive_client.hpp>

class AP_IQMotor {
public:
    AP_IQMotor();

    AP_IQMotor(const AP_IQMotor &other) = delete;
    AP_IQMotor &operator=(const AP_IQMotor&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void update();
    // Add a client pointer to the list of clients. The only check right now is if the list is full.
    // Takes a ClientAbstract pointer which all clients are derived from.
    // Return 1 if the client was added, else return 0
	uint8_t add_client(ClientAbstract *new_client);

    // Find a client if it exists and give its pointer.
    // Takes a pointer to a ClientAbstract pointer, a client type, and a module ID
    // Returns a 1 if the client exists and a 0 if it doesn't
    // TODO: always return a client. If it exists already return that pointer, else create a new one. Maybe this should be a new function called "get_client"
	uint8_t find_client(ClientAbstract **set_client, uint8_t client_type, uint8_t obj_id);

    // Return the single com interface pointer owned by AP_IQMotor
    // Currently only one UART can be used, but maybe if more than one is required the behavior might need to change
    // Takes no input
    // Returns the pointer to the single GenericInterface
    GenericInterface  *get_com_interface() { return &com; };
    
    // This function returns whether the writing function is happening right now or not. The code isn't multithreaded, but maybe useful for interrupts?
    // Takes no input
    // Returns whether writing is happening
    bool get_writing() { return writing; };


    // Returns the AP_IQMotor object singleton. Trying to copy other ardupilot structure here.
    static AP_IQMotor *get_singleton(void) {
        return _singleton;
    }

private:
    static AP_IQMotor *_singleton;

    AP_HAL::UARTDriver *iq_uart;
    GenericInterface com;
    
    
    AP_Int8 motor_input_type;
    AP_Int8 broadcast_length;
    AP_Int16 telemetry_bitmask;
    AP_Int16 motor_dir_bitmask;
    
    static const uint8_t client_limit = 128;

    // Initialize the the AP IQmotor object
    void init(void);

    // unfinished. Desired behavior is to loop through the masked out motors and output to them
    void update_motor_outputs(void);

    void update_telemetry(void);

    void find_next_telem_motor(void);

    // This function reads the uart that is attached to the AP IQMotor object and populates client's messages if there is one for it
    void read(void);

    // Writes out the com buffer to the uart in the generic interface
    void write(void);

    bool initialized = false;

    bool writing = false;

    uint8_t tx_buf[128];
    uint8_t rx_buf[128];
    uint8_t ser_length;
    uint8_t total_channels = 0;
    BrushlessDriveClient motors[16] = {
        BrushlessDriveClient(0),
        BrushlessDriveClient(1),
        BrushlessDriveClient(2),
        BrushlessDriveClient(3),
        BrushlessDriveClient(4),
        BrushlessDriveClient(5),
        BrushlessDriveClient(6),
        BrushlessDriveClient(7),
        BrushlessDriveClient(8),
        BrushlessDriveClient(9),
        BrushlessDriveClient(0),
        BrushlessDriveClient(11),
        BrushlessDriveClient(12),
        BrushlessDriveClient(13),
        BrushlessDriveClient(14),
        BrushlessDriveClient(15),
    }; // TODO change how this works
    ClientAbstract *clients[client_limit]; // how do I make a list of virtual classes?
    uint8_t client_size = 0;
    uint8_t telem_motor_id = 0;
    uint32_t last_request_time = 0;
    uint32_t TELEM_TIMEOUT = 50;
    uint8_t first_telem_motor = 0;
    uint8_t last_telem_motor = 0;
    bool out_flag = false;
};