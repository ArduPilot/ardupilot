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

   DJI FPV MSP telemetry library backend

   The DJI Air Unit polls the FC for the following MSP messages at around 4Hz.
   Note: messages are polled in ascending hex id order.

   Hex|Dec|Name
   ---------------------------
   03	03	MSP_FC_VERSION
   0a	10	MSP_NAME
   54	84	MSP_OSD_CONFIG
   5c	92	MSP_FILTER_CONFIG
   5e	94	MSP_PID_ADVANCED
   65	101	MSP_STATUS
   69	105	MSP_RC
   6a	106	MSP_RAW_GPS
   6b	107	MSP_COMP_GPS
   6c	108	MSP_ATTITUDE
   6d	109	MSP_ALTITUDE
   6e	110	MSP_ANALOG
   6f	111	MSP_RC_TUNING
   70	112	MSP_PID
   82	130	MSP_BATTERY_STATE
   86	134	MSP_ESC_SENSOR_DATA
   96	150	MSP_STATUS_EX
   f7	247	MSP_RTC
*/

#pragma once

#include "AP_MSP_Telem_Backend.h"

#if HAL_MSP_ENABLED

class AP_MSP_Telem_DJI : public AP_MSP_Telem_Backend
{
    using AP_MSP_Telem_Backend::AP_MSP_Telem_Backend;
public:
    bool init_uart() override;
    // implementation specific helpers
    bool is_scheduler_enabled() const override;
    AP_SerialManager::SerialProtocol get_serial_protocol() const override { return AP_SerialManager::SerialProtocol::SerialProtocol_DJI_FPV; };
    uint32_t get_osd_flight_mode_bitmask(void) override;
    void hide_osd_items(void) override;

    bool get_rssi(float &rssi) const override;
    void update_home_pos(home_state_t &home_state) override;
    void update_battery_state(battery_state_t &_battery_state) override;
    void update_gps_state(gps_state_t &gps_state) override;
    void update_airspeed(airspeed_state_t &airspeed_state) override;
    void update_flight_mode_str(char *flight_mode_str, uint8_t size, bool wind_enabled) override;

    MSP::MSPCommandResult msp_process_out_fc_variant(MSP::sbuf_t *dst) override;
    MSP::MSPCommandResult msp_process_out_esc_sensor_data(MSP::sbuf_t *dst) override;

    enum : uint8_t {
        DJI_FLAG_ARM = 0,
        DJI_FLAG_STAB,
        DJI_FLAG_HOR,
        DJI_FLAG_HEAD,
        DJI_FLAG_FS,
        DJI_FLAG_RESC,
    };
};

#endif //HAL_MSP_ENABLED
