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

   Code by Andy Piper, ported from betaflight vtx_tramp
*/

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_OSD/AP_OSD.h>

#ifndef AP_TRAMP_ENABLED
#define AP_TRAMP_ENABLED OSD_ENABLED && BOARD_FLASH_SIZE>1024 && !HAL_MINIMIZE_FEATURES
#endif

#if AP_TRAMP_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_VideoTX.h"

#define VTX_TRAMP_POWER_COUNT 5

#define VTX_TRAMP_MIN_FREQUENCY_MHZ 5000             //min freq in MHz
#define VTX_TRAMP_MAX_FREQUENCY_MHZ 5999             //max freq in MHz
// Maximum number of requests sent to try a config change
// Some VTX fail to respond to every request (like Matek FCHUB-VTX) so
// we sometimes need multiple retries to get the VTX to respond.
#define VTX_TRAMP_MAX_RETRIES (20)
// Race lock - settings can't be changed
#define TRAMP_CONTROL_RACE_LOCK (0x01)

class AP_Tramp
{
public:
    AP_Tramp();
    ~AP_Tramp() {}

    /* Do not allow copies */
    AP_Tramp(const AP_Tramp &other) = delete;
    AP_Tramp &operator=(const AP_Tramp&) = delete;

    static AP_Tramp *get_singleton(void) {
        return singleton;
    }

    bool init(void);
    void update();

    uint16_t get_current_actual_power();
    uint16_t get_current_temp();

private:
    uint8_t checksum(uint8_t *buf);
    // Check if race lock is enabled
    bool is_race_lock_enabled(void) {
        return cur_control_mode & TRAMP_CONTROL_RACE_LOCK;
    }
    void send_command(uint8_t cmd, uint16_t param);
    char handle_response();
    void reset_receiver();
    char receive_response();
    void send_query(uint8_t cmd);
    void process_requests();
    bool is_device_ready();
    void set_frequency(uint16_t freq);
    void set_power(uint16_t power);
    void set_pit_mode(uint8_t onoff);

    // serial interface
    AP_HAL::UARTDriver *port;                  // UART used to send data to Tramp VTX

    //Pointer to singleton
    static AP_Tramp* singleton;

    const static uint16_t TRAMP_BUF_SIZE = 16;

    // Serial transmit and receive buffers
    uint8_t request_buffer[TRAMP_BUF_SIZE];
    uint8_t response_buffer[TRAMP_BUF_SIZE];

    // Module state machine
    enum class TrampStatus {
        // Offline - device hasn't responded yet
        TRAMP_STATUS_OFFLINE = 0,
        // Init - fetching current settings from device
        TRAMP_STATUS_INIT,
        // Online - device is ready and being monitored - freq/power/pitmode
        TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT,
        // Online - device is ready and being monitored - temperature
        TRAMP_STATUS_ONLINE_MONITOR_TEMP,
        // Online - device is ready and config has just been updated
        TRAMP_STATUS_ONLINE_CONFIG
    };

    TrampStatus status = TrampStatus::TRAMP_STATUS_OFFLINE;

    void set_status(TrampStatus stat);

    // Device limits, read from device during init
    struct {
        uint32_t rf_freq_min;
        uint32_t rf_freq_max;
        uint32_t rf_power_max;
    } device_limits;

    uint16_t cur_act_power; // Actual power
    int16_t cur_temp;
    uint8_t cur_control_mode;

    // Retry count
    uint8_t retry_count = VTX_TRAMP_MAX_RETRIES;

    // Receive state machine
    enum class ReceiveState {
        S_WAIT_LEN = 0,   // Waiting for a packet len
        S_WAIT_CODE,      // Waiting for a response code
        S_DATA,           // Waiting for rest of the packet.
    };

    ReceiveState receive_state = ReceiveState::S_WAIT_LEN;

    // Receive buffer index
    int16_t receive_pos;

    // Last action time
    uint32_t last_time_us;
};

#endif
