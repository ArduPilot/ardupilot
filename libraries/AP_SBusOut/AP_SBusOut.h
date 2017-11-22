/*
 * AP_SBusOut.h
 *
 *  Created on: Aug 19, 2017
 *      Author: Mark Whitehorn
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>

class AP_SBusOut {
public:
    static const struct AP_Param::GroupInfo var_info[];

    static AP_SBusOut create() {
        return AP_SBusOut{};
    }

    constexpr AP_SBusOut(AP_SBusOut &&other) = default;

    /* Do not allow copies */
    AP_SBusOut(const AP_SBusOut &other) = delete;
    AP_SBusOut &operator=(const AP_SBusOut&) = delete;

    void update();

private:
    AP_SBusOut();
    AP_HAL::UARTDriver *sbus1_uart;

    void init(void);

    uint16_t sbus_frame_interval;   // microseconds

    AP_Int16 sbus_rate;
    bool initialised;
};
