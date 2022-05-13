/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev
 */

#include <AP_CYPHAL/AP_CYPHAL_esc.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


extern const AP_HAL::HAL& hal;


bool CyphalEscController::init(CyphalSubscriberManager &sub_manager,
                               CyphalPublisherManager &pub_manager,
                               CanardInstance &ins,
                               CanardTxQueue& tx_queue)
{
    _pub_setpoint = new CyphalSetpointPublisher(ins, tx_queue, _registers.getPortIdByIndex(UAVCAN_PUB_SETPOINT_ID));
    if (!pub_manager.add_publisher(_pub_setpoint)) {
        return false;
    }

    _pub_readiness = new CyphalReadinessPublisher(ins, tx_queue, _registers.getPortIdByIndex(UAVCAN_PUB_READINESS_ID));
    if (!pub_manager.add_publisher(_pub_readiness)) {
        return false;
    }

    const uint8_t power_register_indexes[4] = {
        UAVCAN_SUB_POWER_0_ID,
        UAVCAN_SUB_POWER_1_ID,
        UAVCAN_SUB_POWER_2_ID,
        UAVCAN_SUB_POWER_3_ID,
    };

    const uint8_t dynamics_register_indexes[4] = {
        UAVCAN_SUB_DYNAMICS_0_ID,
        UAVCAN_SUB_DYNAMICS_1_ID,
        UAVCAN_SUB_DYNAMICS_2_ID,
        UAVCAN_SUB_DYNAMICS_3_ID,
    };

    const uint8_t status_register_indexes[4] = {
        UAVCAN_SUB_STATUS_0_ID,
        UAVCAN_SUB_STATUS_1_ID,
        UAVCAN_SUB_STATUS_2_ID,
        UAVCAN_SUB_STATUS_3_ID,
    };

    int16_t port_id;
    for (auto esc_idx = 0; esc_idx < 4; esc_idx++) {
        port_id = _registers.getPortIdByIndex(power_register_indexes[esc_idx]);
        _sub_power[esc_idx] = new CyphalPowerSubscriber(ins, tx_queue, port_id, esc_idx);
        if (!sub_manager.add_subscriber(_sub_power[esc_idx])) {
            return false;
        }

        port_id = _registers.getPortIdByIndex(dynamics_register_indexes[esc_idx]);
        _sub_dynamics[esc_idx] = new CyphalDynamicsSubscriber(ins, tx_queue, port_id, esc_idx);
        if (!sub_manager.add_subscriber(_sub_dynamics[esc_idx])) {
            return false;
        }

        port_id = _registers.getPortIdByIndex(status_register_indexes[esc_idx]);
        _sub_status[esc_idx] = new CyphalStatusSubscriber(ins, tx_queue, port_id, esc_idx);
        if (!sub_manager.add_subscriber(_sub_status[esc_idx])) {
            return false;
        }
    }

    _is_inited = true;
    return true;
}


void CyphalEscController::SRV_push_servos()
{
    if (!_is_inited) {
        if (AP_HAL::millis() > last_log_ts_ms + 5000) {
            last_log_ts_ms = AP_HAL::millis();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: esc is not inited.");
        }
        return;
    }

    WITH_SEMAPHORE(SRV_sem);

    for (uint8_t src_idx = 0; src_idx < NUM_SERVO_CHANNELS; src_idx++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(src_idx)) {
            _SRV_conf[src_idx].pulse = SRV_Channels::srv_channel(src_idx)->get_output_pwm();
            _SRV_conf[src_idx].esc_pending = true;
            _SRV_conf[src_idx].servo_pending = true;
        }
    }

    _pub_setpoint->set_setpoint(_SRV_conf);
    _pub_readiness->update_readiness();
}


/**
 * @note reg.udral.physics.dynamics.rotation.PlanarTs_0_1
 */
void CyphalDynamicsSubscriber::subscribe()
{
    subscribeOnMessage(reg_udral_physics_dynamics_rotation_PlanarTs_0_1_EXTENT_BYTES_);
}

void CyphalDynamicsSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_physics_dynamics_rotation_PlanarTs_0_1 msg;
    if (reg_udral_physics_dynamics_rotation_PlanarTs_0_1_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    constexpr float RAD_PER_SEC_TO_RPM = 60.0f / (3.14159f * 2.0f);
    uint16_t rpm = msg.value.kinematics.angular_velocity.radian_per_second * RAD_PER_SEC_TO_RPM;
    update_rpm(_esc_idx, rpm);
}


/**
 * @note reg.udral.physics.electricity.PowerTs_0_1
 */
void CyphalPowerSubscriber::subscribe()
{
    subscribeOnMessage(reg_udral_physics_electricity_PowerTs_0_1_EXTENT_BYTES_);
}

void CyphalPowerSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_physics_electricity_PowerTs_0_1 msg;
    if (reg_udral_physics_electricity_PowerTs_0_1_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    uint16_t telem_data_mask = TelemetryType::CURRENT | TelemetryType::VOLTAGE;
    TelemetryData telemetry_data;
    telemetry_data.voltage = msg.value.voltage.volt;
    telemetry_data.current = msg.value.current.ampere;
    update_telem_data(_esc_idx, telemetry_data, telem_data_mask);
}


/**
 * @note reg.udral.service.actuator.common.Status_0_1
 */
void CyphalStatusSubscriber::subscribe()
{
    subscribeOnMessage(reg_udral_service_actuator_common_Status_0_1_EXTENT_BYTES_);
}

void CyphalStatusSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_service_actuator_common_Status_0_1 msg;
    if (reg_udral_service_actuator_common_Status_0_1_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    uint16_t telem_data_mask = TelemetryType::TEMPERATURE;
    TelemetryData telemetry_data;
    telemetry_data.temperature_cdeg = int16_t((KELVIN_TO_C(msg.motor_temperature.kelvin) * 100));
    update_telem_data(_esc_idx, telemetry_data, telem_data_mask);
}


/**
 * @note reg.udral.service.actuator.common.sp.*
 */
CyphalSetpointPublisher::CyphalSetpointPublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
    CyphalBasePublisher(ins, tx_queue, port_id)
{
    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
}

void CyphalSetpointPublisher::set_setpoint(SrvConfig *srv_config)
{
    for (uint_fast8_t sp_idx = 0; sp_idx < 4; sp_idx++) {
        _vector4_sp.value[sp_idx] = (hal.rcout->scale_esc_to_unity(srv_config[sp_idx].pulse) + 1.0) / 2.0;
    }
}

void CyphalSetpointPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalSetpointPublisher::publish()
{
    uint8_t buf[reg_udral_service_actuator_common_sp_Vector4_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = reg_udral_service_actuator_common_sp_Vector4_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;

    _transfer_metadata.port_id = _port_id;

    auto result = reg_udral_service_actuator_common_sp_Vector4_0_1_serialize_(&_vector4_sp, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push(buf_size, buf);
    }

    _transfer_metadata.transfer_id++;
}


/**
 * @note reg.udral.service.common_Readiness_0_1
 */
CyphalReadinessPublisher::CyphalReadinessPublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
    CyphalBasePublisher(ins, tx_queue, port_id)
{
    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;

    _readiness.value = reg_udral_service_common_Readiness_0_1_SLEEP;
}

void CyphalReadinessPublisher::update_readiness()
{
    if (hal.util->safety_switch_state() == AP_HAL::Util::safety_state::SAFETY_NONE) {
        _readiness.value = reg_udral_service_common_Readiness_0_1_SLEEP;
    } else if (!hal.util->get_soft_armed()) {
        _readiness.value = reg_udral_service_common_Readiness_0_1_STANDBY;
    } else {
        _readiness.value = reg_udral_service_common_Readiness_0_1_ENGAGED;
    }
}

void CyphalReadinessPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalReadinessPublisher::publish()
{
    _transfer_metadata.port_id = _port_id;

    uint8_t buf[reg_udral_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = reg_udral_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;

    auto result = reg_udral_service_common_Readiness_0_1_serialize_(&_readiness, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push(buf_size, buf);
    }

    _transfer_metadata.transfer_id++;
}

#endif // HAL_ENABLE_CYPHAL_DRIVERS
