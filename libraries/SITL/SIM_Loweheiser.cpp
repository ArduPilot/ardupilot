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
  Simulator for the Loweheiser generators
*/

#include <AP_Math/AP_Math.h>

#include "SIM_Loweheiser.h"
#include "SITL.h"

#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

Loweheiser::Loweheiser() : SerialDevice::SerialDevice()
{
}

void Loweheiser::update()
{
    // if (!_enabled.get()) {
    //     return;
    // }
    maybe_send_heartbeat();
    update_receive();
    update_send();
}

void Loweheiser::maybe_send_heartbeat()
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_ms < 100) {
        // we only provide a heartbeat every so often
        return;
    }
    last_heartbeat_ms = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id,
                               component_id,
                               &msg,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID,
                               0,
                               0,
                               0);

    uint8_t buf[300];
    uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);

    if (write_to_autopilot((const char*)&buf, buf_len) != buf_len) {
        ::fprintf(stderr, "write failure\n");
    }
}

void Loweheiser::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t pkt;
        mavlink_msg_command_long_decode(&msg, &pkt);

        if (pkt.target_system != system_id ||
            pkt.target_component != component_id) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Not for me");
            return;
        }

        switch (pkt.command) {
        case MAV_CMD_LOWEHEISER_SET_STATE:
            // decode the desired run state:
            // param2 physically turns power on/off to the EFI!
            switch ((uint8_t)pkt.param2) {
            case 0:
                autopilot_desired_engine_run_state = EngineRunState::OFF;
                break;
            case 1:
                autopilot_desired_engine_run_state = EngineRunState::ON;
                break;
            default:
                AP_HAL::panic("Bad desired engine run state");
            }
            switch ((uint8_t)pkt.param3) {
            case 0:
                autopilot_desired_governor_state = GovernorState::OFF;
                break;
            case 1:
                autopilot_desired_governor_state = GovernorState::ON;
                break;
            default:
                AP_HAL::panic("Bad desired governor state");
            }
            switch ((uint8_t)pkt.param5) {
            case 0:
                autopilot_desired_startup_state = StartupState::OFF;
                break;
            case 1:
                autopilot_desired_startup_state = StartupState::ON;
                break;
            default:
                AP_HAL::panic("Bad electronic startup state");
            }
            manual_throttle_pct = pkt.param4;
            mavlink_message_t ack;
            mavlink_msg_command_ack_pack(
                system_id,
                component_id,
                &ack,
                MAV_CMD_LOWEHEISER_SET_STATE,
                MAV_RESULT_ACCEPTED,
                0,
                0,
                msg.sysid,
                msg.compid
                );
            uint8_t buf[300];
            uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &ack);

            if (write_to_autopilot((const char*)&buf, buf_len) != buf_len) {
                ::fprintf(stderr, "write failure\n");
            }
            break;
        }
    };
    }
}

void Loweheiser::update_receive()
{
    char buf[128];
    const ssize_t bytes_read = read_from_autopilot(buf, sizeof(buf));
    if (bytes_read <= 0) {
        return;
    }
    for (uint16_t i=0; i<bytes_read; i++) {
        if (mavlink_frame_char_buffer(
                &rxmsg,
                &rxstatus,
                buf[i],
                &rxmsg,
                &rxstatus) == MAVLINK_FRAMING_OK) {
            handle_message(rxmsg);
        }
    }
}

void Loweheiser::update_fuel_level()
{
    const uint32_t now_ms = AP_HAL::millis();

    const uint32_t tdelta = now_ms - last_fuel_update_ms;
    last_fuel_update_ms = now_ms;

    const float fuel_consumption_lperh_at_8000rpm = 10;
    const float fuel_consumption_lpers_at_8000rpm = fuel_consumption_lperh_at_8000rpm / 3600;

    fuel_flow_lps = generatorengine.current_rpm/8000 * fuel_consumption_lpers_at_8000rpm;
    const float fuel_delta = tdelta/1000.0 * fuel_flow_lps;  // litres
    fuel_consumed += fuel_delta;
    fuel_level -= fuel_delta;
}

void Loweheiser::update_send()
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 200) {
        return;
    }
    last_sent_ms = now;

    auto sitl = AP::sitl();
    if (!sitl || sitl->efi_type == SIM::EFIType::EFI_TYPE_NONE) {
        return;
    }

    float throttle = 0;
    float throttle_output = 0;

    switch (autopilot_desired_engine_run_state) {
    case EngineRunState::OFF:
        generatorengine.desired_rpm = 0;
        break;
    case EngineRunState::ON:
        switch (autopilot_desired_governor_state) {
        case GovernorState::OFF: {
            throttle = manual_throttle_pct;
            throttle_output = throttle;
            if (is_positive(generatorengine.desired_rpm) ||
                autopilot_desired_startup_state == StartupState::ON) {
                const uint16_t manual_rpm_min = 0;
                const uint16_t manual_rpm_max = 8000;
                generatorengine.desired_rpm = linear_interpolate(
                    manual_rpm_min,
                    manual_rpm_max,
                    throttle,
                    0,
                    100
                    );
            }
            break;
        }
        case GovernorState::ON:
            // should probably base this on current draw from battery
            // / motor throttle output?
            throttle = 80;
            throttle_output = 80;
            generatorengine.desired_rpm = 8000;
            break;
        }
    }

    _current_current = AP::sitl()->state.battery_current;
    _current_current = MIN(_current_current, max_current);

    generatorengine.current_current = _current_current;
    generatorengine.max_current = max_current;
    generatorengine.max_slew_rpm_per_second = 2000;

    generatorengine.update();

    update_fuel_level();

    float efi_pw = std::numeric_limits<float>::quiet_NaN();
    float efi_fuel_flow = std::numeric_limits<float>::quiet_NaN();
    float efi_fuel_consumed = std::numeric_limits<float>::quiet_NaN();
    float efi_fuel_level = std::numeric_limits<float>::quiet_NaN();
    float efi_baro = std::numeric_limits<float>::quiet_NaN();
    float efi_mat = std::numeric_limits<float>::quiet_NaN();
    float efi_clt = std::numeric_limits<float>::quiet_NaN();
    float efi_tps = std::numeric_limits<float>::quiet_NaN();
    float efi_egt = std::numeric_limits<float>::quiet_NaN();
    float efi_batt = std::numeric_limits<float>::quiet_NaN();

    float curr_batt = -0.3;
    float curr_gen = 10.12;

    // Current from the generator is the  battery charging current (defined to be negative) plus the generator current
    float curr_rot = (curr_batt < 0 ? -curr_batt : 0.0) + curr_gen;

    // controlled by param2, this turns on/off the DC/DC component which
    // powers the efi
    if (autopilot_desired_engine_run_state == EngineRunState::ON) {
        efi_baro = AP::baro().get_pressure() / 1000.0;
        efi_mat = AP::baro().get_temperature();
        efi_clt = generatorengine.temperature;
        efi_tps = MAX(throttle_output, 40);
        efi_batt = 12.5;
        efi_fuel_flow = fuel_flow_lps * 3600;  // litres/second -> litres/hour
    }

    if (false) {
        efi_fuel_level = fuel_level;
        efi_fuel_consumed = fuel_consumed;
    }

    // +/- 3V, depending on draw
    const float volt_batt = base_supply_voltage - (3 * (_current_current / max_current));

    const mavlink_loweheiser_gov_efi_t loweheiser_efi_gov{
        volt_batt, // volt_batt
        curr_batt, // curr_batt
        curr_gen, // curr_gen
        curr_rot, // curr_rot
        efi_fuel_level, // fuel_level in litres
        throttle, // throttle
        UINT32_MAX, // runtime in seconds
        INT32_MAX, // time until maintenance
        std::numeric_limits<float>::quiet_NaN(),  // rectifier temperature
        std::numeric_limits<float>::quiet_NaN(),  // generator temperature
        efi_batt, // efi_batt
        generatorengine.current_rpm, // efi_rpm
        efi_pw, // efi_pw
        efi_fuel_flow, // efi_fuelflow
        efi_fuel_consumed, // efi_fuel_consumed
        efi_baro, // efi_baro
        efi_mat, // efi_mat
        efi_clt, // efi_clt
        efi_tps, // efi_tps
        efi_egt, // efi_exhaust_gas_temperature
        1,  // EFI index
        0, // generator_status
        0  // EFI status
    };

    mavlink_message_t msg;
    mavlink_msg_loweheiser_gov_efi_encode_status(
        system_id,
        component_id,
        &mav_status,
        &msg,
        &loweheiser_efi_gov
        );

    uint8_t buf[300];
    uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);

    if (write_to_autopilot((char*)buf, buf_len) != buf_len) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
