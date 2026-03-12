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
  AP_IBUS2_Master: ArduPilot as the IBUS2 master (host/receiver role).

  In this role ArduPilot sends:
    Frame 1 – channel values from SRV_Channel PWM outputs
    Frame 2 – commands to attached slave devices (controlled by SEND_MASK)
  and receives:
    Frame 3 – sensor/telemetry data from the slave device

  Serial port: SerialProtocol_IBUS2_Master (51), 1.5 Mbit/s, OPTION_HDPLEX.

  Usage:
    sim_vehicle.py -v ArduCopter -A --serial5=sim:ibus2slave
    param set SERIAL5_PROTOCOL 51
    param set SERIAL5_BAUD 1500000
    param set SIM_IBUS2S_ENA 1
    reboot
*/

#pragma once

#include <AP_IBUS2/AP_IBUS2_config.h>

#if AP_IBUS2_MASTER_ENABLED

#include <AP_IBUS2/AP_IBUS2.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_IBUS2_Master
{
public:
    AP_IBUS2_Master();

    CLASS_NO_COPY(AP_IBUS2_Master);

    void init();
    void update();

    static const struct AP_Param::GroupInfo var_info[];

    // Cached sensor value from last GET_VALUE response
    struct SensorData {
        uint8_t  device_type;    // IBUS2DeviceType
        uint8_t  value[14];      // raw value bytes from GET_VALUE response
        uint8_t  vid;
        uint8_t  pid;
        bool     valid;
        uint32_t last_update_ms;
    };

    // Access the cached data for device at AddressLevel1 addr (0-7)
    const SensorData &get_device_data(uint8_t addr) const { return _devices[addr & 0x7]; }

private:
    AP_Int8  _enable;
    AP_Int8  _send_mask;   // bitmask: bit0=GET_TYPE, bit1=GET_VALUE, bit2=GET_PARAM, bit3=SET_PARAM

    AP_HAL::UARTDriver *_port;
    bool _initialized;

    // The current device address being polled (0-7 for AddressLevel1)
    uint8_t _current_addr;

    // Known devices: device_known[i] = true after successful GET_TYPE response
    bool _device_known[8];

    // Cached data per device
    SensorData _devices[8];

    // State machine for frame reception
    enum class RxState : uint8_t {
        WAIT_HEADER,
        IN_FRAME3,
    };
    RxState _rx_state;
    uint8_t _rx_buf[IBUS2_FRAME3_SIZE];
    uint8_t _rx_len;

    // Timestamp when we sent the last Frame 2 (µs)
    uint32_t _frame2_sent_us;
    bool _waiting_response;

    // Number of TX bytes still to discard from RX (half-duplex echo)
    uint16_t _tx_pending_echo;

    void send_frame1();
    void send_frame2(uint8_t addr);
    void process_rx();
    void handle_frame3(const IBUS2_Frame3 *f3);

    // Send mask bit definitions
    static const uint8_t SEND_GET_TYPE  = (1U << 0);
    static const uint8_t SEND_GET_VALUE = (1U << 1);
    static const uint8_t SEND_GET_PARAM = (1U << 2);
    static const uint8_t SEND_SET_PARAM = (1U << 3);
};

#endif  // AP_IBUS2_MASTER_ENABLED
