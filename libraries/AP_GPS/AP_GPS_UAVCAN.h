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
class FixCb;
class AuxCb;
class AP_GPS_UAVCAN : public AP_GPS_Backend {
public:
    AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state);
    ~AP_GPS_UAVCAN();
    static void handle_fix_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const FixCb &cb);
    static void handle_aux_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AuxCb &cb);
    bool read() override;

    const char *name() const override { return "UAVCAN"; }
    static void subscribe_gps_uavcan_messages(AP_UAVCAN* ap_uavcan);
    static AP_GPS_Backend* probe(AP_GPS &_gps, AP_GPS::GPS_State &_state);

private:
    bool _new_data;

    //Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_GPS_UAVCAN* driver;
    } _detected_modules[GPS_MAX_RECEIVERS];
    static AP_HAL::Semaphore *_sem_registry;
    static bool take_registry();
    static void give_registry();
    static AP_GPS_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

    void handle_fix_msg(const FixCb &cb);
    void handle_aux_msg(const AuxCb &cb);

    uint8_t _detected_module;
    HAL_Semaphore sem;
    AP_GPS::GPS_State interim_state;
};
