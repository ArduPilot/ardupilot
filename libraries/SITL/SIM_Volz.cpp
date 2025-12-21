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
  Simulator for the Volz

  TODO:
    - check that -128-to-127 thing vs -100-to-100

*/

#include "SIM_config.h"

#if AP_SIM_VOLZ_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "SIM_Volz.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>

#include "SIM_Aircraft.h"

#include <errno.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Volz::var_info[] = {

    // @Param: ENA
    // @DisplayName: Volz simulator enable/disable
    // @Description: Allows you to enable (1) or disable (0) the Volz simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, Volz, _enabled, 0),

    // @Param: MASK
    // @DisplayName: Volz override mask
    // @Description: mask of servo output channels to override with values from Volz protocol.  Note these are indexed from 0 - so channel 3 (usually throttle) has value 4 in this bitmask (1<<2).
    // @User: Advanced
    AP_GROUPINFO("MASK", 2, Volz, _output_mask, (1U<<0)|(1U<<1)|(1U<<3)),

    // @Param: FMASK
    // @DisplayName: Volz fail mask
    // @Description: fail servo at current position.  Channel 1 is bit 0.
    // @User: Advanced
    AP_GROUPINFO("FMASK", 3, Volz, _failed_mask, 0),

    AP_GROUPEND
};

// this isn't in the header as inlcuding sparse_endian in the header
// causes compilation issues with redefinition of bswap16
void Volz::Command::update_checksum()
{
    crc = htobe16(calculate_checksum());
}

Volz::Volz() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise IDs
    for (uint8_t n=0; n<ARRAY_SIZE(servos); n++) {
        auto &s = servos[n];
        s.id = n+1;  // really should parameterise this
        s.pcb_temperature = 25;
        s.motor_temperature = 25;
        s.primary_current = 0.1;
        s.secondary_current = 0.1;
        s.primary_voltage = 1.1;
        s.secondary_voltage = 1.2;
    }
}

void Volz::update_servos(const class Aircraft &aircraft)
{
    const float ambient_degC = aircraft.ambient_temperature_degC();
    const uint32_t now_us = AP_HAL::micros();
    const uint32_t delta_t_us = now_us - last_servo_update_us;
    if (delta_t_us < 1000) {  // only update simulation at 1kHz
        return;
    }
    last_servo_update_us = now_us;
    const float dt = delta_t_us / 1000000.0;

    for (auto  &servo : servos) {
        const uint8_t idx = servo.id - 1;
        if (idx > ARRAY_SIZE(aircraft.servo_filter)) {
            continue;
        }
        servo.position = aircraft.servo_filter[idx].angle();

        // update primary current - proportional to airspeed and aileron deflection:
        float airspeed = aircraft.get_airspeed_pitot();
        servo.primary_current = abs(sinf(radians(servo.position))) * airspeed * 10;

        // update temperatures
        // servo.pcb_temperature += servo.secondary_current *0.01 * dt;
        servo.pcb_temperature -= (servo.pcb_temperature-ambient_degC) * 0.1 * dt;

#if HAL_LOGGING_ENABLED
        log_Servo(servo, now_us);
#endif  // HAL_LOGGING_ENABLED
    }
}

void Volz::update(const class Aircraft &aircraft)
{
    if (!_enabled.get()) {
        return;
    }

    update_servos(aircraft);

    update_input();
}

void Volz::consume_command()
{
    if (buflen < sizeof(Command)) {
        AP_HAL::panic("No command to consume");
    }
    if (buflen > sizeof(Command)) {
        memmove(&u.buffer[0], &u.buffer[sizeof(Command)], buflen-sizeof(Command));
    }
    buflen -= sizeof(Command);
}

bool Volz::shift_command_to_front_of_buffer()
{
    while (buflen >= sizeof(Command)) {
        if (u.command.calculate_checksum() == be16toh(u.command.crc)) {
            return true;
        }
        // AP_HAL::panic("Invalid input");

        // sync error; just shift everything by 1...
        memmove(&u.buffer[0], &u.buffer[1], buflen-1);
        buflen--;
    }
    return false;
}

void Volz::process_command(const Command &command)
{
    if (command.actuator_id >= ARRAY_SIZE(servos)+1) {
        AP_HAL::panic("Invalid actuator ID (%u) received", command.actuator_id);
    }
    Servo &servo = servos[command.actuator_id-1];

    Command response {};
    response.actuator_id = command.actuator_id;

    switch (command.command_id) {
    case CommandId::SET_EXTENDED_POSITION: {
        servo.desired_position = linear_interpolate(-1, 1, UINT16_VALUE(command.arg1, command.arg2), 0x80, 0x0F80);
        response.command_id = CommandId::EXTENDED_POSITION_RESPONSE;
        const uint16_t scaled_position = linear_interpolate(0x80, 0x0F80, servo.position, -1, 1);
        response.arg1 = HIGHBYTE(scaled_position);
        response.arg2 = LOWBYTE(scaled_position);
        break;
    }
    case CommandId::READ_TEMPERATURE:
        // nb. MAX(50) may not be correct.... unknown how true device performs
        response.command_id = CommandId::TEMPERATURE_RESPONSE;
        response.arg1 = MAX(servo.motor_temperature, -50.0) + 50.0;
        response.arg2 = MAX(servo.pcb_temperature, -50.0) + 50.0;
        break;
    case CommandId::READ_CURRENT:
        response.command_id = CommandId::CURRENT_RESPONSE;
        response.arg1 = MIN(servo.primary_current * 50, 255);  // realistic?
        response.arg2 = MIN(servo.secondary_current * 50, 255);
        break;
    case CommandId::READ_VOLTAGE:
        response.command_id = CommandId::VOLTAGE_RESPONSE;
        response.arg1 = MIN(servo.primary_voltage * 5, 255);
        response.arg2 = MIN(servo.secondary_voltage * 5, 255);
        break;
    case CommandId::CURRENT_RESPONSE:
    case CommandId::VOLTAGE_RESPONSE:
    case CommandId::TEMPERATURE_RESPONSE:
    case CommandId::EXTENDED_POSITION_RESPONSE:
        AP_HAL::panic("Response 'command' %u received", (unsigned)command.command_id);
        break;
    default:
        AP_HAL::panic("Unknown command %u received", (unsigned)command.command_id);
    }

    response.update_checksum();

    if (write_to_autopilot((char*)&response, sizeof(response)) != sizeof(response)) {
        AP_HAL::panic("short write");
    }
}

void Volz::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&u.buffer[buflen], ARRAY_SIZE(u.buffer) - buflen);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }

    while (shift_command_to_front_of_buffer()) {
        process_command(u.command);
        consume_command();
    }
}

void Volz::update_sitl_input_pwm(struct sitl_input &input)
{
    // overwrite the SITL input values passed through from
    // sitl_model->update_model with those we're receiving serially
    for (auto &servo : servos) {
        const uint8_t idx = servo.id - 1;
        if (idx > ARRAY_SIZE(input.servos)) {
            // silently ignore; input.servos is 12-long, we are
            // usually 16-long
            continue;
        }
        if ((_output_mask.get() & (1U<<idx)) == 0) {
            continue;
        }
        servo.failed = _failed_mask.get() & (1U<<idx);
        input.servos[idx] = servo.pwm();
    }
}

#if HAL_LOGGING_ENABLED
void Volz::log_Servo(const Volz::Servo &servo, const uint32_t now_us)
{
    // @LoggerMessage: SMVZ
    // @Description: Simulated Volz servo information
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Volz servo ID
    // @Field: Pos: Current Simulated Position
    // @Field: DesPos: Desired Simulated Position
    // @Field: V: simulated servo voltage
    // @Field: A: simulated servo current
    // @Field: PCBT: simulated PCB Temperature
    // @Field: MotT: simulated motor Temperature
    AP::logger().WriteStreaming(
        "SMVZ",
        "TimeUS," "Id," "Pos," "DesPos," "V," "A," "PCBT," "MotT" ,
        "s"       "#"   "%"    "%"       "v"  "A"  "O"     "O"    ,
        "F"       "-"   "0"    "0"       "0"  "0"  "0"     "0"    ,
        "Q"       "B"   "f"    "f"       "f"  "f"  "f"     "f"    ,
        now_us,
        servo.id - 1,
        servo.position * 100,
        servo.desired_position * 100,
        servo.primary_voltage,
        servo.primary_current,
        servo.pcb_temperature,
        servo.motor_temperature
        );
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_SIM_VOLZ_ENABLED
