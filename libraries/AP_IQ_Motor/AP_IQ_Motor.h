/*
 * This header file defines all functions and variables necessary for creation of an AP_IQ_Motor object.
 * The AP_IQ_Motor object provides functions necessary to communicate with Vertiq modules.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>


#ifndef AP_IQUART_ENABLED
#define AP_IQUART_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>
#include "generic_interface.hpp"
#include "client_communication.hpp"
#include <AP_IQ_Motor/ardupilot_client.hpp>
#include <AP_IQ_Motor/ifci.hpp>

class AP_IQ_Motor : public AP_ESC_Telem_Backend {
public:

    //Create an AP_IQ_Motor object
    AP_IQ_Motor();

    //Delete an AP_IQ_Motor object
    AP_IQ_Motor(const AP_IQ_Motor &other) = delete;

    //Delete an AP_IQ_Motor object
    AP_IQ_Motor &operator=(const AP_IQ_Motor&) = delete;

    //Holds all of the information for our group variables
    static const struct AP_Param::GroupInfo var_info[];

    //Called periodically to update the motor outputs and update telemetry
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

    // Return the single com interface pointer owned by AP_IQ_Motor
    // Currently only one UART can be used, but maybe if more than one is required the behavior might need to change
    // Takes no input
    // Returns the pointer to the single GenericInterface
    GenericInterface *get_com_interface() { return &_com; }
    
    // This function returns whether the writing function is happening right now or not. The code isn't multithreaded, but maybe useful for interrupts?
    // Takes no input
    // Returns whether writing is happening
    bool get_writing() { return _writing; }


    // Returns the AP_IQ_Motor object singleton. Trying to copy other ardupilot structure here.
    static AP_IQ_Motor *get_singleton(void) { return _singleton; }

private:
    static AP_IQ_Motor *_singleton;

    AP_HAL::UARTDriver *iq_uart;
    GenericInterface _com;
    
    
    AP_Int8 _broadcast_length;
    AP_Int16 _telemetry_bitmask;
    AP_Int16 _motor_dir_bitmask;
    
    static const uint8_t _client_limit = 128;

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

    bool _initialized = false;

    bool _writing = false;

    uint8_t _tx_buf[128];
    uint8_t _rx_buf[128];
    uint8_t _ser_length;
    uint8_t _total_channels = 0;
    IFCI _motor_interface;
    ClientAbstract *_clients[_client_limit]; // how do I make a list of virtual classes?
    uint8_t _client_size = 0;
    uint8_t _telem_motor_id = 0;
    uint8_t _telem_request_id = 0;
    uint32_t _last_request_time = 0;
    static const uint32_t _telem_timeout = 50;
    uint8_t _first_telem_motor = 0;
    uint8_t _last_telem_motor = 0;

    static const uint8_t kSubCtrlCoast                  =  2;
    static const uint8_t kTypePropellerMotorControl     = 52;
    static const uint8_t kBroadcastID                   = 63;
};