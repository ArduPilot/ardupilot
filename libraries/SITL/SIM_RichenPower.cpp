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
  Simulator for the RichenPower Hybrid generators
*/

#include <AP_Math/AP_Math.h>

#include "SIM_RichenPower.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo RichenPower::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: RichenPower Generator sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the RichenPower simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, RichenPower, _enabled, 0),

    // @Param: CTRL_PIN
    // @DisplayName: Pin RichenPower is connectred to
    // @Description: The pin number that the RichenPower spinner servo is connected to. (start at 1)
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("CTRL", 2, RichenPower, _ctrl_pin, -1),

    AP_GROUPEND
};

RichenPower::RichenPower() : SerialDevice::SerialDevice()
{
    ASSERT_STORAGE_SIZE(RichenPacket, 70);

    AP_Param::setup_object_defaults(this, var_info);

    u.packet.magic1 = 0xAA;
    u.packet.magic2 = 0x55;

    u.packet.version_major = 0x0A;
    u.packet.version_minor = 0x00;

    u.packet.footermagic1 = 0x55;
    u.packet.footermagic2 = 0xAA;
}

void RichenPower::set_run_state(State newstate) {
    hal.console->printf("Moving to state %u from %u\n", (unsigned)newstate, (unsigned)_state);
    _state = newstate;
}

void RichenPower::update(const struct sitl_input &input)
{
    if (!_enabled.get()) {
        return;
    }
    update_control_pin(input);
    update_send();
}

void RichenPower::update_control_pin(const struct sitl_input &input)
{
    const uint32_t now = AP_HAL::millis();

    static const uint16_t INPUT_SERVO_PWM_STOP = 1200;
    static const uint16_t INPUT_SERVO_PWM_IDLE = 1500;
    // static const uint16_t INPUT_SERVO_PWM_RUN = 1900;

    // RICHENPOWER, 13:47
    // 1100~1300 for engine stop, 1300~1800 for idle, and 1800~2000 for run

    const uint16_t control_pwm = _ctrl_pin >= 1 ? input.servos[_ctrl_pin-1] : -1;
    if (_state != State::STOPPING) {
        if (control_pwm <= INPUT_SERVO_PWM_STOP && _state != State::STOP) {
            if (stop_start_ms == 0) {
                stop_start_ms = now;
            }
        } else {
            stop_start_ms = 0;
        }
    }
    // ::fprintf(stderr, "stop_start_ms=%u\n", stop_start_ms);
    State newstate;
    if (control_pwm <= INPUT_SERVO_PWM_STOP) {
        // stop
        if (stop_start_ms == 0 || now - stop_start_ms > 30000) {
            newstate = State::STOP;
        } else {
            newstate = State::STOPPING;
        }
    } else if (control_pwm <= INPUT_SERVO_PWM_IDLE) {
        newstate = State::IDLE;
    } else {
        newstate = State::RUN;
    }
    if (newstate != _state) {
        set_run_state(newstate);
    } else {
        if (_state == State::STOP) {
            // pass
        } else {
            _runtime_ms += now - _last_runtime_ms;
        }
        _last_runtime_ms = now;
    }

    // RICHENPOWER, 13:49
    // Idle RMP 4800 +-300, RUN RPM 13000 +- 1500

    switch (_state) {
    case State::STOP:
        generatorengine.desired_rpm = 0;
        break;
    case State::IDLE:
    case State::STOPPING:
        generatorengine.desired_rpm = 4800; // +/- 300
        break;
    case State::RUN:
        generatorengine.desired_rpm = 13000; // +/- 1500
        break;
    }

    _current_current = AP::sitl()->state.battery_current;
    _current_current = MIN(_current_current, max_current);

    generatorengine.current_current = _current_current;
    generatorengine.max_current = max_current;
    generatorengine.max_slew_rpm_per_second = 2000;

    generatorengine.update();

    _current_rpm = generatorengine.current_rpm;
}

void RichenPower::RichenUnion::update_checksum()
{
    packet.checksum = 0;
    for (uint8_t i=1; i<6; i++) {
        packet.checksum += htobe16(checksum_buffer[i]);
    }
    packet.checksum = htobe16(packet.checksum);
}

void RichenPower::update_send()
{
    // just send a chunk of data at 1Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 1000) {
        return;
    }
    last_sent_ms = now;

    u.packet.rpm = htobe16(_current_rpm);
    uint32_t runtime_seconds_remainder = _runtime_ms / 1000;
    u.packet.runtime_hours = runtime_seconds_remainder / 3600;
    runtime_seconds_remainder %= 3600;
    u.packet.runtime_minutes = runtime_seconds_remainder / 60;
    runtime_seconds_remainder %= 60;
    u.packet.runtime_seconds = runtime_seconds_remainder;

    u.packet.runtime_seconds = runtime_seconds_remainder;

    const int32_t seconds_until_maintenance = (original_seconds_until_maintenance - _runtime_ms/1000.0f);
    uint16_t errors = htobe16(u.packet.errors);
    if (seconds_until_maintenance <= 0) {
        u.packet.seconds_until_maintenance = htobe32(0);
        errors |= (1U<<(uint8_t(Errors::MaintenanceRequired)));
    } else {
        u.packet.seconds_until_maintenance = htobe32(seconds_until_maintenance);
        errors &= ~(1U<<(uint8_t(Errors::MaintenanceRequired)));
    }
    u.packet.errors = htobe16(errors);

    switch (_state) {
    case State::IDLE:
    case State::RUN:
        u.packet.output_current = htobe16(_current_current * 100);
        // +/- 3V, depending on draw
        u.packet.output_voltage = htobe16(100*base_supply_voltage - 3 * (_current_current / max_current));
        break;
    case State::STOP:
    default:
        u.packet.output_current = 0;
        u.packet.output_voltage = 0;
        break;
    }

    enum class Mode {
        IDLE = 0,
        RUN = 1,
        CHARGE = 2,
        BALANCE = 3,
        OFF = 4,
    };
    switch (_state) {
    case State::STOP:
        u.packet.mode = (uint8_t)Mode::OFF;
        break;
    case State::STOPPING:
    case State::IDLE:
        u.packet.mode = (uint8_t)Mode::IDLE;
        break;
    case State::RUN:
        u.packet.mode = (uint8_t)Mode::RUN;
        break;
    }

    u.update_checksum();

    // const uint8_t data[] = {
    //     0xAA, 0x55, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1E, 0xB0, 0x00, 0x10, 0x00, 0x00, 0x23, 0x7A, 0x23,
    //     0x7A, 0x11, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xBE, 0x55, 0xAA,
    // };

    if (write_to_autopilot((char*)u.parse_buffer, ARRAY_SIZE(u.parse_buffer)) != ARRAY_SIZE(u.parse_buffer)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
