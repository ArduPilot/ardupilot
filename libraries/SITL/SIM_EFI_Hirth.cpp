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
  simulate Hirth EFI system
*/

#include "SIM_Aircraft.h"
#include <SITL/SITL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include "SIM_EFI_Hirth.h"

using namespace SITL;

// assume SERVO3 is throttle
#define HIRTH_RPM_INDEX 2

void EFI_Hirth::update_receive()
{
    const ssize_t num_bytes_read = read_from_autopilot((char*)&receive_buf[receive_buf_ofs], ARRAY_SIZE(receive_buf) - receive_buf_ofs);
    if (num_bytes_read < 0) {
        return;
    }
    receive_buf_ofs += num_bytes_read;

    if (receive_buf_ofs < 1) {
        return;
    }

    const uint8_t expected_bytes_in_message = receive_buf[0];

    if (expected_bytes_in_message == 0) {
        AP_HAL::panic("zero bytes expected is unexpected");
    }

    if (expected_bytes_in_message > ARRAY_SIZE(receive_buf)) {
        AP_HAL::panic("Unexpectedly large byte count");
    }

    if (receive_buf_ofs < expected_bytes_in_message) {
        return;
    }

    // checksum is sum of all bytes except the received checksum:
    const uint8_t expected_checksum = 256U - crc_sum_of_bytes(receive_buf, expected_bytes_in_message-1);
    const uint8_t received_checksum = receive_buf[expected_bytes_in_message-1];
    if (expected_checksum == received_checksum) {
        PacketCode received_packet_code = PacketCode(receive_buf[1]);
        if (received_packet_code == PacketCode::SetValues) {
            // do this synchronously for now
            handle_set_values();
        } else {
            assert_receive_size(3);
            if (requested_data_record.time_ms != 0) {
                AP_HAL::panic("Requesting too fast?");
            }
            requested_data_record.code = received_packet_code;
            requested_data_record.time_ms = AP_HAL::millis();
        }
    } else {
        AP_HAL::panic("checksum failed");
        // simply throw these bytes away.  What the actual device does in the
        // face of weird data is unknown.
    }
    memmove(&receive_buf[0], &receive_buf[expected_bytes_in_message], receive_buf_ofs - expected_bytes_in_message);
    receive_buf_ofs -= expected_bytes_in_message;
}

void EFI_Hirth::assert_receive_size(uint8_t receive_size)
{
    if (receive_buf[0] != receive_size) {
        AP_HAL::panic("Expected %u message size, got %u message size", receive_size, receive_buf[0]);
    }
}

void EFI_Hirth::handle_set_values()
{
    assert_receive_size(23);
    static_assert(sizeof(settings) == 20, "correct number of bytes in settings");
    memcpy((void*)&settings, &receive_buf[2], sizeof(settings));
}

void EFI_Hirth::update_send()
{
    if (requested_data_record.time_ms == 0) {
        // no outstanding request
        return;
    }
    if (AP_HAL::millis() - requested_data_record.time_ms < 20) {
        // 20ms to service a request
        return;
    }
    requested_data_record.time_ms = 0;

    switch (requested_data_record.code) {
    case PacketCode::DataRecord1:
        send_record1();
        break;
    case PacketCode::DataRecord2:
        send_record2();
        break;
    case PacketCode::DataRecord3:
        send_record3();
        break;
    default:
        AP_HAL::panic("Unknown data record (%u) requested", (unsigned)requested_data_record.code);
    }
}

void EFI_Hirth::update_engine_model()
{
    auto sitl = AP::sitl();

    // FIXME: this should come from simulation, not baro.  baro gets
    // warmed by the simulated electronics!
    const float ambient = AP::baro().get_temperature();

    const uint32_t now_ms = AP_HAL::millis();

    const float delta_t = (now_ms - engine.last_update_ms) * 1e-6;
    engine.last_update_ms = now_ms;

    // lose heat to environment (air-cooling due to airspeed and prop
    // airflow could be taken into account here)
    const float ENV_LOSS_FACTOR = 25;
    engine.cht1_temperature -= (engine.cht1_temperature - ambient) * delta_t * ENV_LOSS_FACTOR;
    engine.cht2_temperature -= (engine.cht2_temperature - ambient) * delta_t * ENV_LOSS_FACTOR;

    const float rpm = sitl->state.rpm[HIRTH_RPM_INDEX];
    const float RPM_GAIN_FACTOR_CHT1 = 10;
    const float RPM_GAIN_FACTOR_CHT2 = 8;
    engine.cht1_temperature += rpm * delta_t * RPM_GAIN_FACTOR_CHT1;
    engine.cht2_temperature += rpm * delta_t * RPM_GAIN_FACTOR_CHT2;
}

void EFI_Hirth::init()
{
    // auto sitl = AP::sitl();

    if (is_zero(AP::baro().get_temperature())) {
        // defer until the baro has had a chance to update....
        return;
    }

    engine.cht1_temperature = AP::baro().get_temperature();
    engine.cht2_temperature = AP::baro().get_temperature();

    init_done = true;
}

void EFI_Hirth::update()
{
    const auto *sitl = AP::sitl();
    if (!sitl || sitl->efi_type != SIM::EFI_TYPE_HIRTH) {
        return;
    }

    if (!init_done) {
        init();
    }

    // update throttle; interim thing to make life a little more interesting
    throttle = 0.9 * throttle + 0.1 * settings.throttle/10;

    update_engine_model();

    update_receive();
    update_send();
}

uint16_t EFI_Hirth::engine_status_field_value() const
{
    return (
        0U << 0 |  // engine temperature sensor
        1U << 1 |  // air temperature sensor
        1U << 2 |  // air pressure sensor
        1U << 3    // throttle sensor OK
        );
}

void SITL::EFI_Hirth::send_record1()
{
    const auto *sitl = AP::sitl();

    // notionally the field updates should happen in the update()
    // method, but here to save CPU for now:
    auto &r = packed_record1.record;
    r.engine_status = engine_status_field_value();
    r.rpm = sitl->state.rpm[HIRTH_RPM_INDEX];
    r.air_temperature = AP::baro().get_temperature();
    r.throttle = settings.throttle / 10;  // just echo this back

    packed_record1.update_checksum();

    write_to_autopilot((char*)&packed_record1, sizeof(packed_record1));

    ASSERT_STORAGE_SIZE(Record1, 84);
}

void SITL::EFI_Hirth::send_record2()
{
    const auto *sitl = AP::sitl();

    // notionally the field updates should happen in the update()
    // method, but here to save CPU for now:
    auto &r = packed_record2.record;
    r.throttle_percent_times_10 = throttle * 10.0;
    r.fuel_consumption = ((MAX(sitl->state.rpm[HIRTH_RPM_INDEX] - 1500.0, 0)) /2200.0) * 10;  // from log, very rough

    packed_record2.update_checksum();

    write_to_autopilot((char*)&packed_record2, sizeof(packed_record2));

    ASSERT_STORAGE_SIZE(Record2, 98);
}


void SITL::EFI_Hirth::send_record3()
{
    // notionally the field updates should happen in the update()
    // method, but here to save CPU for now:
    auto &r = packed_record3.record;
    r.excess_temperature_1 = engine.cht1_temperature;  // cht1
    r.excess_temperature_2 = engine.cht2_temperature;  // cht2
    r.excess_temperature_3 = 39;  // egt1
    r.excess_temperature_4 = 41;  // egt2

    packed_record3.update_checksum();

    write_to_autopilot((char*)&packed_record3, sizeof(packed_record3));

    ASSERT_STORAGE_SIZE(Record3, 100);
}
