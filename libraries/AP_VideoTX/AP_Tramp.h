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

#include "AP_VideoTX_config.h"

#if AP_TRAMP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_VideoTX.h"

#define VTX_TRAMP_MIN_FREQUENCY_MHZ 1000             //min freq in MHz
#define VTX_TRAMP_MAX_FREQUENCY_MHZ 5999             //max freq in MHz
// Maximum number of requests sent to try a config change
// Some VTX fail to respond to every request (like Matek FCHUB-VTX) so
// we sometimes need multiple retries to get the VTX to respond.
#define VTX_TRAMP_MAX_RETRIES (20)
// Retry period for a pit mode change the VTX is not accepting. Giving up
// strands the VTX in pit mode with no way back, so keep asking indefinitely.
// Also bounds how long video can stay dark once the VTX is ready again.
#define VTX_TRAMP_PITMODE_RETRY_MS (1000)
// only complain once the VTX has been refusing for this long: one that is
// merely still starting up recovers well inside it and should stay silent
#define VTX_TRAMP_OPTIONS_WARN_MS (30000)
// Re-arm period for a power change the VTX is not applying. The initial
// request burst can land while the VTX is still settling (e.g. just after a
// pit mode change), so keep re-arming instead of giving up: giving up strands
// the VTX at the old power until a different level is requested.
#define VTX_TRAMP_POWER_RETRY_MS (1000)
// Race lock - settings can't be changed
#define TRAMP_CONTROL_RACE_LOCK (0x01)

#define VTX_TRAMP_UART_BAUD            9600
#define VTX_TRAMP_SMARTBAUD_MIN        9120     // -5%
#define VTX_TRAMP_SMARTBAUD_MAX        10080    // +5%
#define VTX_TRAMP_SMARTBAUD_STEP       120

class AP_Tramp
{
public:
    AP_Tramp();
    ~AP_Tramp() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Tramp);

    static AP_Tramp *get_singleton(void) {
        return singleton;
    }

    bool init(void);
    void update();

    // actual transmitted power reported by the VTX in the 'v' response
    uint16_t get_current_actual_power() const { return cur_act_power; }
    int16_t get_current_temp() const { return cur_temp; }

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
    // the reported and configured pit modes disagree. A change to pit mode is
    // prioritised over every other pending change: update_power() refuses to
    // send power while in pit mode, and a frequency change cannot be seen on
    // a dark video feed.
    bool is_pitmode_disagreed() const;
    void set_frequency(uint16_t freq);
    // change baud automatically when request-response fails many times
    void update_baud_rate();

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
    bool _act_power_warned;

    // statistics
    uint16_t _packets_sent;
    uint16_t _packets_rcvd;

    // value for current baud adjust
    int32_t _smartbaud = VTX_TRAMP_UART_BAUD;
    enum class AutobaudDirection {
        UP = 1,
        DOWN = -1
    } _smartbaud_direction = AutobaudDirection::DOWN;

    // Retry count
    uint8_t retry_count = VTX_TRAMP_MAX_RETRIES;

    // retry counter only re-arms when one of these changes
    uint16_t _last_conf_freq;
    uint16_t _last_conf_power;
    uint16_t _last_conf_options;
    bool _power_warn_pending;
    // pit mode changes are retried periodically while the VTX disagrees, so
    // that a refused or missed request cannot strand the VTX with no way back
    uint32_t _last_pitmode_send_ms {0};
    // whether the current pit mode disagreement is being timed
    bool _pitmode_disagreement_started {false};
    // when the current pit mode disagreement began
    uint32_t _pitmode_disagree_ms {0};
    bool _pitmode_warned {false};
    // last time the retry budget was re-armed for a pending power change
    uint32_t _last_power_rearm_ms {0};

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
