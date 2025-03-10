/*
 * This header file defines all functions and variables necessary for creation of an AP_Vertiq object.
 * The AP_Vertiq object provides functions necessary to communicate with Vertiq modules.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_IQUART_ENABLED
#define AP_IQUART_ENABLED BOARD_FLASH_SIZE > 1024
#endif

#pragma GCC diagnostic ignored "-Wsuggest-override"

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>

#include <generic_interface.hpp>

#include "AP_VertiqSerialManager.h"
#include "AP_VertiqClientManager.h"
#include "AP_VertiqOutputManager.h"

/**
 * @brief This class is the top level module for controlling Vertiq modules with IFCI (https://iqmotion.readthedocs.io/en/latest/manual/manual_ifci_control.html). It contains
 * all necessary resources to send IFCI commands over a UART line, and to receive telemetry from user specified module IDs.
 *
 */
class AP_Vertiq: public AP_ESC_Telem_Backend
{

public:
    /**
     * @brief Create a new AP_Vertiq object
     */
    AP_Vertiq();

    /**
     * @brief Delete a new AP_Vertiq object
     */
    AP_Vertiq(const AP_Vertiq &other) = delete;

    /**
     * @brief Delete a new AP_Vertiq object
     */      AP_Vertiq &operator=(const AP_Vertiq&) = delete;

    //Holds all of the information for our group variables
    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    static AP_Vertiq *get_singleton()
    {
        return _singleton;
    }

    /**
     * @brief periodically called by SRV_Channels::push(). This acts as the main loop.
     */
    void update();

private:

    AP_Int8 _number_cvs; //The user set number of control values to output
    AP_Int32 _telemetry_bitmask; //The user set bitmask determining which module IDs to request telemetry from

    static AP_Vertiq *_singleton;
    bool _init_done = false; //A bool to make sure we only initialize once

    GenericInterface _iquart_interface; //A shared resource for creating and parsing IQUART messages

    AP_VertiqClientManager _client_manager; //An object to handle all client message reception
    AP_VertiqSerialManager _iquart_manager; //An object to handle the hardware end of IQUART
    AP_VertiqOutputManager _output_manager; //An object to handle setting all CVs and telemetry bytes

    /**
     * @brief Initializes the AP_Vertiq object
     */
    void init();

};