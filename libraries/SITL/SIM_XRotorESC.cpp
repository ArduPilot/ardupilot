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
  Simulator for the serial telemetry from a HobbyWing XRotor or T-Motor
  Alpha ESC. Frame layout, rates and units follow a bench capture of a
  T-Motor Alpha.
*/

#include "SIM_XRotorESC.h"

#if AP_SIM_XROTOR_ESC_ENABLED

#include "SIM_Aircraft.h"
#include "SITL.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

using namespace SITL;

// bytes in one frame
static constexpr uint8_t FRAME_LEN = 24;

// the Alpha sent a frame every 60 ms with its motor turning and every 368 ms without
static constexpr uint32_t FRAME_INTERVAL_TURNING_MS = 60;
static constexpr uint32_t FRAME_INTERVAL_STOPPED_MS = 368;

// the default of ESC_TLM_SER_POLE, so that ArduPilot reads back the simulated RPM
static constexpr uint8_t MOTOR_POLES = 28;

// raw NTC reading for 25 degrees C
static constexpr uint8_t TEMPERATURE_RAW_25C = 211;

/*
  send a frame for the motor on our output when one is due. The ESC
  reports electrical RPM / 10 and the bus voltage in 0.1 V; the
  throttle and current fields are left at zero, as ArduPilot does not
  read them. The capacitor reads two counts hotter for each output
  number, so each of outputs 1-8 decodes to a different temperature and
  a test can tell which ESC's frames reached which output.
 */
void XRotorESC::update(const Aircraft &aircraft)
{
    const SIM *sitl = AP::sitl();
    const uint8_t motor = servo_channel - 1;
    if (sitl == nullptr || motor >= ARRAY_SIZE(sitl->state.rpm)) {
        return;
    }
    const float rpm = fabsf(sitl->state.rpm[motor]);

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t interval_ms = is_positive(rpm) ? FRAME_INTERVAL_TURNING_MS : FRAME_INTERVAL_STOPPED_MS;
    if (!AP_HAL::timeout_expired(last_frame_ms, now_ms, interval_ms)) {
        return;
    }
    last_frame_ms = now_ms;

    uint8_t frame[FRAME_LEN] {};
    frame[0] = 0x9B;                // header
    frame[1] = FRAME_LEN - 2;       // bytes after these two
    frame[2] = 0x01;                // version and command, as the Alpha sends
    frame[3] = 0x02;
    put_be16_ptr(&frame[4], frame_count++);
    put_be16_ptr(&frame[10], uint16_t(constrain_float(rpm * (MOTOR_POLES / 2) * 0.1f, 0, UINT16_MAX)));
    put_be16_ptr(&frame[12], uint16_t(constrain_float(aircraft.get_battery_voltage() * 10, 0, UINT16_MAX)));
    frame[18] = TEMPERATURE_RAW_25C;                    // MOSFET
    frame[19] = TEMPERATURE_RAW_25C - 2 * servo_channel;    // capacitor

    uint16_t sum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 2; i++) {
        sum += frame[i];
    }
    put_le16_ptr(&frame[FRAME_LEN - 2], sum);

    write_to_autopilot((const char *)frame, sizeof(frame));
}

#endif  // AP_SIM_XROTOR_ESC_ENABLED
