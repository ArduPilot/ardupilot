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
/*
  AP_IBUS2_Slave: ArduPilot as an IBUS2 slave device (telemetry adapter role).

  In this role a FlySky receiver is the master. ArduPilot:
    - receives Frame 1 → extracts RC channel values
    - receives Frame 2 → parses command
    - sends    Frame 3 → vehicle telemetry (voltage, GPS, attitude, etc.)

  The device identity matches the FlySky telemetry adapter:
    VID = 0x01, PID = 0x03  (Telemetry_Adapter_Protocol___20240923_en_.pdf §4)

  Serial port: SerialProtocol_IBUS2_Slave (52), 1.5 Mbit/s, OPTION_HDPLEX.

  Usage:
    sim_vehicle.py -v ArduCopter -A --serial5=sim:ibus2master
    param set SERIAL5_PROTOCOL 52
    param set SERIAL5_BAUD 1500000
    param set SIM_IBUS2M_ENA 1
    reboot
*/

#pragma once

#include <AP_IBUS2/AP_IBUS2_config.h>

#if AP_IBUS2_SLAVE_ENABLED

#include <AP_IBUS2/AP_IBUS2.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>
#include <AP_GPS/AP_GPS_config.h>

class AP_IBUS2_Slave
{
public:
    AP_IBUS2_Slave();

    CLASS_NO_COPY(AP_IBUS2_Slave);

    void init();
    void update();

    // Number of valid RC channels received from the master
    uint8_t get_rc_channel_count() const { return _rc_channel_count; }

    // Read received channel values (PWM, 1000-2000 µs typical)
    // Returns true if any channels are valid
    bool get_rc_channels(uint16_t *channels, uint8_t &count) const;

private:
    AP_HAL::UARTDriver *_port;
    bool _initialized;

    // RC channels received in Frame 1
    uint16_t _rc_channels[32];
    uint8_t  _rc_channel_count;
    uint32_t _rc_last_update_ms;

    // Frame reception state machine
    enum class RxState : uint8_t {
        WAIT_HEADER,
        IN_FRAME1,
        IN_FRAME2,
    };
    RxState _rx_state;

    // Receive buffer large enough for the largest frame
    uint8_t _rx_buf[IBUS2_FRAME1_MAX];
    uint8_t _rx_len;
    uint8_t _frame1_expected_len;  // decoded from Frame 1 length byte

    // Pending Frame 2 command
    bool _response_pending;
    IBUS2_Frame2 _pending_cmd;
    uint32_t _frame2_end_us;       // micros() when Frame 2 reception completed

    // Response delay before sending Frame 3 (µs)
    static const uint32_t RESPONSE_DELAY_US = 160;

    void process_rx();
    void handle_frame1(const uint8_t *buf, uint8_t len);
    void handle_frame2(const IBUS2_Frame2 *f2);
    void send_frame3();
    void send_resp_get_type();
    void send_resp_get_value();
    void send_resp_get_param(const IBUS2_Cmd_GetParam *cmd);
    void send_resp_set_param(const IBUS2_Cmd_SetParam *cmd);

    // Populate sensor data into a GET_VALUE response value[14] buffer.
    // Returns number of data points written.
    uint8_t populate_sensor_data(uint8_t *value14);
};

#endif  // AP_IBUS2_SLAVE_ENABLED
