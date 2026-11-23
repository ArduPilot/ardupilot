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
  ESC telemetry from HobbyWing XRotor and T-Motor Alpha ESCs, each
  sending on its own serial port
 */

#include "AP_ESC_Telem_XRotor.h"

#if AP_ESC_TELEM_XROTOR_ENABLED

#include "AP_ESC_Telem.h"

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// 19200 baud delivers at most 20 bytes between 100 Hz updates; reading up
// to two frames per call keeps up even if the loop runs at half rate
static constexpr uint8_t MAX_BYTES_PER_UPDATE = 2 * AP_ESC_Telem_XRotor_Parser::FRAME_LEN;

/*
  pair each serial port set to XRotor ESC telemetry with an output in
  servo_mask: the lowest numbered port with the lowest output, and so
  on. The first output that cannot be paired, because it has no port or
  is beyond the outputs ESC telemetry covers, is kept for the pre-arm
  check, and it and the outputs after it get no telemetry. Returns
  false if there is no memory for the ESCs.

  The ports are opened here, from the thread that reads them, at the
  ESCs' fixed rate, which SERIALn_BAUD is set to show.
 */
bool AP_ESC_Telem_XRotor::init(uint32_t servo_mask)
{
    escs = (ESC *)calloc(__builtin_popcount(servo_mask), sizeof(ESC));
    if (escs == nullptr) {
        return false;
    }

    const auto protocol = AP_SerialManager::SerialProtocol_HobbyWing_XRotor;
    for (uint8_t i = 0; i < 32; i++) {
        if (!BIT_IS_SET(servo_mask, i)) {
            continue;
        }
        AP_HAL::UARTDriver *uart = nullptr;
        if (i < ESC_TELEM_MAX_ESCS) {
            uart = AP::serialmanager().find_serial(protocol, num_escs);
        }
        if (uart == nullptr) {
            unpaired_output = i + 1;
            break;
        }
        AP::serialmanager().set_and_default_baud(protocol, num_escs, AP_ESC_Telem_XRotor_Parser::BAUDRATE / 1000);
        uart->begin(AP_ESC_Telem_XRotor_Parser::BAUDRATE);
        escs[num_escs].uart = uart;
        escs[num_escs].esc_index = i;
        num_escs++;
    }
    return true;
}

/*
  read the bytes waiting on each port and publish every frame they
  complete. A pole count below 2 is treated as 2, so it can neither
  divide by zero nor reverse the RPM.
 */
void AP_ESC_Telem_XRotor::update(int8_t poles)
{
    const float motor_poles = MAX(poles, 2);
    uint8_t buf[MAX_BYTES_PER_UPDATE];
    for (uint8_t i = 0; i < num_escs; i++) {
        ESC &esc = escs[i];
        const ssize_t n = esc.uart->read(buf, sizeof(buf));
        for (ssize_t j = 0; j < n; j++) {
            if (esc.parser.parse_byte(buf[j])) {
                publish(esc, motor_poles);
            }
        }
    }
}

/*
  publish the frame just received from esc. Ten times the RPM field is
  electrical RPM: on a T-Motor Alpha it matched the ESC's one pulse per
  electrical revolution output to 1%, and with a 240 Kv 28-pole motor
  the RPM it gives rose at 0.9 x Kv x volts with throttle, as an
  unloaded motor's should. AP_Periph's fixed 5/7 scaling of the same
  field is this conversion for a 28-pole motor.

  The current fields are not published. AP_Periph scales them by 0.01 A
  for HobbyWing ESCs, while T-Motor's documentation for the Alpha gives
  1/64 A, and an Alpha drawing 0.1 to 0.3 A reported a flat 2 counts,
  which settles neither.
 */
void AP_ESC_Telem_XRotor::publish(const ESC &esc, float poles)
{
    const AP_ESC_Telem_XRotor_Parser::Frame &f = esc.parser.get_frame();

    // each turn of the motor is poles/2 electrical revolutions
    update_rpm(esc.esc_index, f.rpm * 20.0f / poles);

    TelemetryData t {};
    t.voltage = f.voltage * 0.1f;
    const uint8_t temp_C = MAX(AP_ESC_Telem_XRotor_Parser::temperature_decode(f.mos_temperature),
                               AP_ESC_Telem_XRotor_Parser::temperature_decode(f.cap_temperature));
    t.temperature_cdeg = int16_t(temp_C) * 100;
    update_telem_data(esc.esc_index, t, TelemetryType::VOLTAGE | TelemetryType::TEMPERATURE);
}

#endif  // AP_ESC_TELEM_XROTOR_ENABLED
