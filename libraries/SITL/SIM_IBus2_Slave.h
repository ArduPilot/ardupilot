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
  Simulator for an IBUS2 slave device for testing AP_IBUS2_Master.

  This simulator acts as a single IBUS2-attached peripheral (e.g. a servo with
  telemetry). It:
    1. Reads Frame 1 (channel values) sent by ArduPilot
    2. Reads Frame 2 commands sent by ArduPilot
    3. Responds with Frame 3 containing synthetic sensor data

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter \
  -A --serial5=sim:ibus2slave --speedup=1 --console

param set SERIAL5_PROTOCOL 51
param set SERIAL5_BAUD 1500000
param set SIM_IBUS2S_ENA 1
reboot
*/

#pragma once

#include "SIM_SerialDevice.h"
#include <AP_Param/AP_Param.h>
#include <AP_IBus2/AP_IBus2.h>

namespace SITL {

// Abstract base class: handles Frame 1/2 reception, dispatches send_response()
class IBus2Slave : public SerialDevice {
public:
    IBus2Slave();

    void update(const class Aircraft &aircraft);

    bool enabled() const { return _enabled.get(); }

protected:
    AP_Int8  _enabled;

    // Receive channels from Frame 1
    uint16_t _channels[14];

    // Last received Frame 2 command — needed by send_response() in subclasses
    IBUS2_Pkt<IBUS2_Frame2> _pending_cmd;

private:
    // Frame 1 reception
    uint8_t _rx1_buf[IBUS2_FRAME1_MAX];
    uint8_t _rx1_len;
    uint8_t _rx1_expected_len;

    // Frame 2 reception
    uint8_t _rx2_buf[IBUS2_FRAME2_SIZE];
    uint8_t _rx2_len;

    enum class RxState : uint8_t {
        WAIT_HEADER,
        IN_FRAME1,
        IN_FRAME2,
    };
    RxState _rx_state;

    bool     _respond_pending;
    uint32_t _frame2_end_us;

    void process_rx(const class Aircraft &aircraft);
    virtual void send_response(const class Aircraft &aircraft) = 0;
};

// Concrete subclass — current role: respond to Frame 2 queries with telemetry (Frame 3)
class IBus2SlaveDevice : public IBus2Slave {
public:
    IBus2SlaveDevice() = default;

    static const AP_Param::GroupInfo var_info[];

private:
    void send_response(const class Aircraft &aircraft) override;
};

}  // namespace SITL
