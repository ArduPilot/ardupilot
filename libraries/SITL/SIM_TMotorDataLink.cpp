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
  Simulator for the TMotorDataLink
*/

#include <AP_Math/AP_Math.h>
#include "SIM_Aircraft.h"
#include "SITL.h"
#include "SITL_Input.h"

#include "SIM_TMotorDataLink.h"

#if AP_SIM_TMOTOR_DATALINK_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

TMotorDataLink::TMotorDataLink() :
    SerialDevice::SerialDevice()
{
}

void TMotorDataLink::set_motors(const char *comma_separated_list_of_motors)
{
    char s[64];
    strncpy(s, comma_separated_list_of_motors, sizeof(s));

    uint8_t count = 0;

    char *saveptr = NULL;
    for (char *p = strtok_r(s, ",", &saveptr);
         p && count <ARRAY_SIZE(servo_channel_numbers);
         p = strtok_r(nullptr, ",\n", &saveptr)) {
        servo_channel_numbers[count++] = atoi(p);
    }
}


void TMotorDataLink::Packet::update_checksum()
{
    crc = crc_xmodem((uint8_t*)this, sizeof(*this)-2);
}

void TMotorDataLink::update(const Aircraft &aircraft, const sitl_input &input)
{
    // only sent every 20ms
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms < 20) {
        return;
    }
    last_send_ms = now_ms;

    for (uint8_t i=0; i<8; i++) {  // 8 being the number of motors in the packet...
        update_motor(i, aircraft, input);
        packet.motor[i].esc_channel_number = i;
    }

    packet.update_checksum();
    write_to_autopilot((char*)&packet, sizeof(packet));

    counter++;  // we share the counters for each ESC at the moment - i.e. every ESC is in every packet and everything has the same seqno
}

void TMotorDataLink::update_motor(uint8_t motor_number, const Aircraft &aircraft, const sitl_input &input)
{
    uint16_t pwm = 0;
    if (motor_number > num_servo_channels || servo_channel_numbers[motor_number] == 0) {
        // there's no servo associcated with this motor number; think
        // of it as there being no motor plugged into the physical
        // device.
        memset(&packet.motor[motor_number], '\0', sizeof(packet.motor[motor_number]));  // hmmm.  Is this necessary?
        return;
    }

    const uint8_t servo_channel_number = servo_channel_numbers[motor_number];
    const uint16_t servo_channel_offset = servo_channel_number - 1;
    if (servo_channel_offset < ARRAY_SIZE(input.servos)) {
        pwm = input.servos[servo_channel_offset];
    }


    SITL::SIM* sitl = AP::sitl();
    const int16_t rpm = sitl->state.rpm[servo_channel_offset];
    // ::printf("sitl rpm=%d\n", rpm);

    ESCInfo &escinfo = packet.motor[motor_number];

    // FIXME: a lot of stuff in here needs fixing.  Also note that this is essentially copied from HWing.cpp so it has the same problems.
    escinfo.seq = htobe16(counter);
    escinfo.throttle = htobe16(pwm - 1000);
    escinfo.throttle_req = htobe16(pwm - 1000);
    escinfo.rpm = htobe16(rpm * 14.0 / 10.0);  // scale from RPM to eRPM  (14 poes
    escinfo.voltage = aircraft.get_battery_voltage() * 100;
    escinfo.current = 0;
    escinfo.phase_current = 0;

    // FIXME: this may not be an entirely accurate model of the
    // temperature profile of these ESCs.
    escinfo.mos_temperature += pwm/100000;
    escinfo.mos_temperature *= 0.95;

    // FIXME: this may not be an entirely accurate model of the
    // temperature profile of these ESCs.
    escinfo.cap_temperature += pwm/100000;
    escinfo.cap_temperature *= 0.95;
    escinfo.status = 0;
}

#endif  // AP_SIM_TMOTOR_DATALINK_ENABLED
