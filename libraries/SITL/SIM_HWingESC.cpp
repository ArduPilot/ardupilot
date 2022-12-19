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
  Simulator for the HWingESC

 - frame gap is used to strengthen protocol integrity in ArduPilot

*/

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "SIM_Aircraft.h"

#include "SIM_HWingESC.h"
#include "SITL.h"
#include "SITL_Input.h"
#include <AP_HAL/utility/sparse-endian.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

HWingESC::HWingESC(uint8_t _servo_channel_number) : SerialDevice::SerialDevice(), servo_channel_number{_servo_channel_number}
{
}

void HWingESC::Packet::update_checksum()
{
    crc = 0;
    for (uint8_t i=0; i<sizeof(*this)-2; i++) {
        crc += ((uint8_t*)this)[i];
    }
}

void HWingESC::update(const Aircraft &aircraft, const sitl_input &input)
{
    uint16_t pwm = 0;
    const uint16_t servo_channel_offset = servo_channel_number - 1;
    if (servo_channel_offset < ARRAY_SIZE(input.servos)) {
        pwm = input.servos[servo_channel_offset];
    }


    // FIXME: the vehicle models should be supplying this RPM!
    const uint16_t Kv = 1000;
    const float p = (pwm-1000)/1000.0;
    int16_t rpm = aircraft.get_battery_voltage() * Kv * p;

    // FIXME: a lot of stuff in here needs fixing
    packet.counter = htobe32(counter++);
    packet.throttle = htobe16(pwm - 1000);
    packet.throttle_req = htobe16(pwm - 1000);
    packet.rpm = htobe16(rpm * 7.0 / 5.0);  // scale from RPM to eRPM
    packet.voltage = aircraft.get_battery_voltage() * 100;
    packet.current = 0;
    packet.phase_current = 0;

    // FIXME: this may not be an entirely accurate model of the
    // temperature profile of these ESCs.
    packet.mos_temperature += pwm/100000;
    packet.mos_temperature *= 0.95;

    // FIXME: this may not be an entirely accurate model of the
    // temperature profile of these ESCs.
    packet.cap_temperature += pwm/100000;
    packet.cap_temperature *= 0.95;
    packet.status = 0;

    packet.update_checksum();

    write_to_autopilot((char*)&packet, sizeof(packet));
}

HWingESC *HWingESCs::create(uint8_t servo_channel_number)
{
    if (num_escs >= ARRAY_SIZE(escs)) {
        return nullptr;
    }
    escs[num_escs] = new HWingESC(servo_channel_number);  // starting at 1, typically 3 for throttle
    if (escs[num_escs] == nullptr) {
        return nullptr;
    }
    return escs[num_escs++];
}
