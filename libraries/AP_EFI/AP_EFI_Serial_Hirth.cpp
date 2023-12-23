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



#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_HIRTH_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_EFI/AP_EFI_Serial_Hirth.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Math/definitions.h>
#include <AP_Logger/AP_Logger.h>

#define HIRTH_MAX_PKT_SIZE 100
#define HIRTH_MAX_RAW_PKT_SIZE 103

#define SERIAL_WAIT_TIMEOUT_MS 100

#define ENGINE_RUNNING 4
#define THROTTLE_POSITION_FACTOR 10
#define CRANK_SHAFT_SENSOR_OK 0x0F
#define INJECTION_TIME_RESOLUTION 0.8
#define THROTTLE_POSITION_RESOLUTION 0.1
#define VOLTAGE_RESOLUTION 0.0049       /* 5/1024 */
#define ADC_CALIBRATION (5.0/1024.0)
#define MAP_HPA_PER_VOLT_FACTOR 248.2673
#define HPA_TO_KPA 0.1
#define TPS_SCALE 0.70

// request/response status constants
#define QUANTITY_REQUEST_STATUS    0x03
#define QUANTITY_SET_VALUE         0x17
#define CODE_REQUEST_STATUS_1      0x04
#define CODE_REQUEST_STATUS_2      0x0B
#define CODE_REQUEST_STATUS_3      0x0D
#define CODE_SET_VALUE             0xC9
#define CHECKSUM_REQUEST_STATUS_1  0xF9
#define CHECKSUM_REQUEST_STATUS_2  0xF2
#define CHECKSUM_REQUEST_STATUS_3  0xF0
#define QUANTITY_RESPONSE_STATUS_1 0x57
#define QUANTITY_RESPONSE_STATUS_2 0x65
#define QUANTITY_RESPONSE_STATUS_3 0x67
#define QUANTITY_ACK_SET_VALUES    0x03

extern const AP_HAL::HAL& hal;

/**
 * @brief Constructor with port initialization
 * 
 * @param _frontend 
 */
AP_EFI_Serial_Hirth::AP_EFI_Serial_Hirth(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
    set_default_coef1(1.0);
}

/**
 * @brief checks for response from or makes requests to Hirth ECU periodically
 * 
 */
void AP_EFI_Serial_Hirth::update()
{
    if (port == nullptr) {
        return;
    }

    // parse response from Hirth
    check_response();

    // send request
    send_request();
}

/**
 * @brief Checks if required bytes are available and proceeds with parsing
 * 
 */
void AP_EFI_Serial_Hirth::check_response()
{
    const uint32_t now = AP_HAL::millis();

    // waiting for response
    if (!waiting_response) {
        return;
    }

    const uint32_t num_bytes = port->available();

    // if already requested
    if (num_bytes >= expected_bytes) {
        // read data from buffer
        uint8_t computed_checksum = 0;
        computed_checksum += res_data.quantity = port->read();
        computed_checksum += res_data.code = port->read();

        if (res_data.code == requested_code) {
            for (int i = 0; i < (res_data.quantity - QUANTITY_REQUEST_STATUS); i++) {
                computed_checksum += raw_data[i] = port->read();
            }
        }

        res_data.checksum = port->read();
        if (res_data.checksum != (256 - computed_checksum)) {
            crc_fail_cnt++;
            port->discard_input();
        } else {
            uptime = now - last_packet_ms;
            last_packet_ms = now;
            internal_state.last_updated_ms = now;
            decode_data();
            copy_to_frontend();
            port->discard_input();
        }

        waiting_response = false;

#if HAL_LOGGING_ENABLED
        log_status();
#endif
    }

    // reset request if no response for SERIAL_WAIT_TIMEOUT_MS
    if (waiting_response &&
        now - last_request_ms > SERIAL_WAIT_TIMEOUT_MS) {
        waiting_response = false;
        last_request_ms = now;

        port->discard_input();
        ack_fail_cnt++;
    }
}

/**
 * @brief Send Throttle and Telemetry requests to Hirth
 * 
 */
void AP_EFI_Serial_Hirth::send_request()
{
    if (waiting_response) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    bool request_was_sent;

    // get new throttle value
    const uint16_t new_throttle = (uint16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

    // check for change or timeout for throttle value
    if ((new_throttle != last_throttle) || (now - last_req_send_throttle_ms > 500)) {
        // send new throttle value, only when ARMED
        bool allow_throttle = hal.util->get_soft_armed();
        if (!allow_throttle) {
#if AP_ICENGINE_ENABLED
            const auto *ice = AP::ice();
            if (ice != nullptr) {
                allow_throttle = ice->allow_throttle_while_disarmed();
            }
#endif  // AP_ICENGINE_ENABLED
        }
        if (allow_throttle) {
            request_was_sent = send_target_values(new_throttle);
        } else {
            request_was_sent = send_target_values(0);
        }

        last_throttle = new_throttle;
        last_req_send_throttle_ms = now;
    } else {
        // request Status request at the driver update rate if no throttle commands
        request_was_sent = send_request_status();
    }

    if (request_was_sent) {
        waiting_response = true;
        last_request_ms = now;
    }
}


/**
 * @brief sends the new throttle command to Hirth ECU
 * 
 * @param thr - new throttle value given by SRV_Channel::k_throttle
 * @return true - if success
 * @return false - currently not implemented
 */
bool AP_EFI_Serial_Hirth::send_target_values(uint16_t thr)
{
    uint8_t computed_checksum = 0;

    // clear buffer
    memset(raw_data, 0, ARRAY_SIZE(raw_data));

#if AP_EFI_THROTTLE_LINEARISATION_ENABLED
    // linearise throttle input
    thr = linearise_throttle(thr);
#endif

    const uint16_t throttle = thr * THROTTLE_POSITION_FACTOR;

    uint8_t idx = 0;

    // set Quantity + Code + "20 bytes of records to set" + Checksum
    computed_checksum += raw_data[idx++] = QUANTITY_SET_VALUE;
    computed_checksum += raw_data[idx++] = requested_code = CODE_SET_VALUE;
    computed_checksum += raw_data[idx++] = throttle & 0xFF;
    computed_checksum += raw_data[idx++] = (throttle >> 8) & 0xFF;

    // checksum calculation
    raw_data[QUANTITY_SET_VALUE - 1] = (256 - computed_checksum);
    
    expected_bytes = QUANTITY_ACK_SET_VALUES;
    // write data
    port->write(raw_data, QUANTITY_SET_VALUE);

    return true;
}


/**
 * @brief cyclically sends different Status requests to Hirth ECU
 * 
 * @return true - when successful
 * @return false  - not implemented
 */
bool AP_EFI_Serial_Hirth::send_request_status() {

    uint8_t requested_quantity;
    uint8_t requested_checksum;

    switch (requested_code)
    {
    case CODE_REQUEST_STATUS_1:
        requested_quantity = QUANTITY_REQUEST_STATUS;
        requested_code = CODE_REQUEST_STATUS_2;
        requested_checksum = CHECKSUM_REQUEST_STATUS_2;
        expected_bytes = QUANTITY_RESPONSE_STATUS_2;
        break;
    case CODE_REQUEST_STATUS_2:
        requested_quantity = QUANTITY_REQUEST_STATUS;
        requested_code = CODE_REQUEST_STATUS_3;
        requested_checksum = CHECKSUM_REQUEST_STATUS_3;
        expected_bytes = QUANTITY_RESPONSE_STATUS_3;
        break;
    case CODE_REQUEST_STATUS_3:
        requested_quantity = QUANTITY_REQUEST_STATUS;
        requested_code = CODE_REQUEST_STATUS_1;
        requested_checksum = CHECKSUM_REQUEST_STATUS_1;
        expected_bytes = QUANTITY_RESPONSE_STATUS_1;
        break;
    default:
        requested_quantity = QUANTITY_REQUEST_STATUS;
        requested_code = CODE_REQUEST_STATUS_1;
        requested_checksum = CHECKSUM_REQUEST_STATUS_1;
        expected_bytes = QUANTITY_RESPONSE_STATUS_1;
        break;
    }
    raw_data[0] = requested_quantity;
    raw_data[1] = requested_code;
    raw_data[2] = requested_checksum;

    port->write(raw_data, QUANTITY_REQUEST_STATUS);

    return true;
}


/**
 * @brief parses the response from Hirth ECU and updates the internal state instance
 * 
 */
void AP_EFI_Serial_Hirth::decode_data()
{
    const uint32_t now = AP_HAL::millis();

    switch (res_data.code) {
    case CODE_REQUEST_STATUS_1: {
        struct Record1 *record1 = (Record1*)raw_data;

        internal_state.engine_speed_rpm = record1->rpm;
        internal_state.throttle_out = record1->throttle;

        // EFI2 log
        internal_state.engine_state = (Engine_State)record1->engine_status;
        internal_state.crankshaft_sensor_status = (record1->sensor_ok & CRANK_SHAFT_SENSOR_OK) ? Crankshaft_Sensor_Status::OK : Crankshaft_Sensor_Status::ERROR;

        // ECYL log
        internal_state.cylinder_status.injection_time_ms = record1->injection_time * INJECTION_TIME_RESOLUTION;
        internal_state.cylinder_status.ignition_timing_deg = record1->ignition_angle;

        // EFI3 log
        internal_state.ignition_voltage = record1->battery_voltage * VOLTAGE_RESOLUTION;

        engine_temperature_sensor_status = (record1->sensor_ok & 0x01) != 0;
        air_temperature_sensor_status = (record1->sensor_ok & 0x02) != 0;
        air_pressure_sensor_status = (record1->sensor_ok & 0x04) != 0;
        throttle_sensor_status = (record1->sensor_ok & 0x08) != 0;

        // resusing mavlink variables as required for Hirth
        // add in ADC voltage of MAP sensor > convert to MAP in kPa
        internal_state.intake_manifold_pressure_kpa = record1->voltage_int_air_pressure * (ADC_CALIBRATION * MAP_HPA_PER_VOLT_FACTOR * HPA_TO_KPA);
        internal_state.intake_manifold_temperature = C_TO_KELVIN(record1->air_temperature);
        break;
    }

    case CODE_REQUEST_STATUS_2: {
        struct Record2 *record2 = (Record2*)raw_data;

        // EFI log
        const float fuel_consumption_rate_lph = record2->fuel_consumption * 0.1;

        internal_state.fuel_consumption_rate_cm3pm = (fuel_consumption_rate_lph * 1000.0 / 60.0) * get_coef1();

        if (last_fuel_integration_ms != 0) {
            // estimated_consumed_fuel_volume_cm3 is in cm3/pm
            const float dt_minutes = (now - last_fuel_integration_ms)*(0.001/60);
            internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * dt_minutes;
        }
        last_fuel_integration_ms = now;

        internal_state.throttle_position_percent = record2->throttle_percent_times_10 * 0.1;
        break;
    }

    case CODE_REQUEST_STATUS_3: {
        struct Record3 *record3 = (Record3*)raw_data;

        // EFI3 Log
        CHT_1_error_excess_temperature_status = (record3->error_excess_temperature_bitfield & 0x0007) != 0;
        CHT_2_error_excess_temperature_status = (record3->error_excess_temperature_bitfield & 0x0038) != 0;
        EGT_1_error_excess_temperature_status = (record3->error_excess_temperature_bitfield & 0x01C0) != 0;
        EGT_2_error_excess_temperature_status = (record3->error_excess_temperature_bitfield & 0x0E00) != 0;

        // ECYL log
        internal_state.cylinder_status.cylinder_head_temperature = C_TO_KELVIN(record3->excess_temperature_1);
        internal_state.cylinder_status.cylinder_head_temperature2 = C_TO_KELVIN(record3->excess_temperature_2);
        internal_state.cylinder_status.exhaust_gas_temperature = C_TO_KELVIN(record3->excess_temperature_3);
        internal_state.cylinder_status.exhaust_gas_temperature2 = C_TO_KELVIN(record3->excess_temperature_4);
        break;
    }
        
    // case CODE_SET_VALUE:
    //     // Do nothing for now
    //     break;
    }
}

#if HAL_LOGGING_ENABLED
void AP_EFI_Serial_Hirth::log_status(void)
{
    // @LoggerMessage: EFIS
    // @Description: Electronic Fuel Injection data - Hirth specific Status information
    // @Field: TimeUS: Time since system startup
    // @Field: ETS1: Status of EGT1 excess temperature error
    // @Field: ETS2: Status of EGT2 excess temperature error
    // @Field: CTS1: Status of CHT1 excess temperature error
    // @Field: CTS2: Status of CHT2 excess temperature error
    // @Field: ETSS: Status of Engine temperature sensor
    // @Field: ATSS: Status of Air temperature sensor
    // @Field: APSS: Status of Air pressure sensor
    // @Field: TSS: Status of Temperature sensor
    // @Field: CRCF: CRC failure count
    // @Field: AckF: ACK failure count
    // @Field: Up: Uptime between 2 messages
    // @Field: ThrO: Throttle output as received by the engine
    AP::logger().WriteStreaming("EFIS",
                                "TimeUS,ETS1,ETS2,CTS1,CTS2,ETSS,ATSS,APSS,TSS,CRCF,AckF,Up,ThrO",
                                "s------------",
                                "F------------",
                                "QBBBBBBBBIIIf",
                                AP_HAL::micros64(),
                                uint8_t(EGT_1_error_excess_temperature_status),
                                uint8_t(EGT_2_error_excess_temperature_status),
                                uint8_t(CHT_1_error_excess_temperature_status),
                                uint8_t(CHT_2_error_excess_temperature_status),
                                uint8_t(engine_temperature_sensor_status),
                                uint8_t(air_temperature_sensor_status),
                                uint8_t(air_pressure_sensor_status),
                                uint8_t(throttle_sensor_status),
                                uint32_t(crc_fail_cnt),
                                uint32_t(ack_fail_cnt),
                                uint32_t(uptime),
                                float(internal_state.throttle_out));
}
#endif // HAL_LOGGING_ENABLED

#endif // AP_EFI_SERIAL_HIRTH_ENABLED
