/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  UAVCAN GPS driver
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include "GPS_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class FixCb;
class Fix2Cb;
class AuxCb;
class HeadingCb;
class StatusCb;

class AP_GPS_UAVCAN : public AP_GPS_Backend {
public:
    AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state);
    ~AP_GPS_UAVCAN();

    bool read() override;

    bool is_healthy(void) const override;

    bool logging_healthy(void) const override;

    bool is_configured(void) const override;

    const char *name() const override { return _name; }

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_GPS_Backend* probe(AP_GPS &_gps, AP_GPS::GPS_State &_state);

    static void handle_fix_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const FixCb &cb);
    static void handle_fix2_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Fix2Cb &cb);
    static void handle_aux_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AuxCb &cb);
    static void handle_heading_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HeadingCb &cb);
    static void handle_status_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const StatusCb &cb);

    static bool backends_healthy(char failure_msg[], uint16_t failure_msg_len);
    void inject_data(const uint8_t *data, uint16_t len) override;

    bool get_error_codes(uint32_t &error_codes) const override { error_codes = error_code; return seen_status; };

private:
    void handle_fix_msg(const FixCb &cb);
    void handle_fix2_msg(const Fix2Cb &cb);
    void handle_aux_msg(const AuxCb &cb);
    void handle_heading_msg(const HeadingCb &cb);
    void handle_status_msg(const StatusCb &cb);

    static bool take_registry();
    static void give_registry();
    static AP_GPS_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

    bool _new_data;
    AP_GPS::GPS_State interim_state;

    HAL_Semaphore sem;

    uint8_t _detected_module;
    bool seen_message;
    bool seen_fix2;
    bool seen_aux;
    bool seen_status;

    bool healthy;
    uint32_t status_flags;
    uint32_t error_code;
    char _name[15];

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        uint8_t instance;
        AP_GPS_UAVCAN* driver;
    } _detected_modules[GPS_MAX_RECEIVERS];

    static HAL_Semaphore _sem_registry;
};
