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

#include <AP_UAVCAN_V1/AP_UAVCAN_V1_esc.h>

#if HAL_ENABLE_LIBUAVCAN_V1_DRIVERS

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


extern const AP_HAL::HAL& hal;


bool UavcanEscController::init(UavcanSubscriberManager &sub_manager,
                               UavcanPublisherManager &pub_manager,
                               CanardInstance &ins,
                               CanardTxQueue& tx_queue)
{
    _pub_setpoint = new UavcanSetpointPublisher(ins, tx_queue, _registers.getPortIdByIndex(UAVCAN_PUB_SETPOINT_ID));
    if (!pub_manager.add_publisher(_pub_setpoint)) {
        return false;
    }

    _pub_readiness = new UavcanReadinessPublisher(ins, tx_queue, _registers.getPortIdByIndex(UAVCAN_PUB_READINESS_ID));
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
        _sub_power[esc_idx] = new UavcanElectricityPowerTsSubscriber(ins, tx_queue, port_id);
        if (!sub_manager.add_subscriber(_sub_power[esc_idx])) {
            return false;
        }

        port_id = _registers.getPortIdByIndex(dynamics_register_indexes[esc_idx]);
        _sub_dynamics[esc_idx] = new UavcanDynamicsSubscriber(ins, tx_queue, port_id);
        if (!sub_manager.add_subscriber(_sub_dynamics[esc_idx])) {
            return false;
        }

        port_id = _registers.getPortIdByIndex(status_register_indexes[esc_idx]);
        _sub_status[esc_idx] = new UavcanStatusSubscriber(ins, tx_queue, port_id);
        if (!sub_manager.add_subscriber(_sub_status[esc_idx])) {
            return false;
        }
    }

    _is_inited = true;
    return true;
}


void UavcanEscController::SRV_push_servos()
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

float UavcanEscController::get_voltage(uint8_t esc_idx)
{
    return (esc_idx < 4) ? _sub_power[esc_idx]->get_voltage() : 0;
}

float UavcanEscController::get_current(uint8_t esc_idx)
{
    return (esc_idx < 4) ? _sub_power[esc_idx]->get_current() : 0;
}

uint16_t UavcanEscController::get_rpm(uint8_t esc_idx)
{
    if (esc_idx >= 4 || _sub_dynamics[esc_idx]->get_last_recv_timestamp() + 1000 < AP_HAL::millis()) {
        return 0;
    }
    return _sub_dynamics[esc_idx]->get_angular_velocity() * 9.5493;
}

float UavcanEscController::get_temperature(uint8_t esc_idx)
{
    return (esc_idx < 4) ? _sub_status[esc_idx]->get_motor_temperature() : 0;
}

uint16_t UavcanEscController::get_avaliable_data_mask(uint8_t esc_idx)
{
    uint16_t telem_data_mask = 0;

    if (esc_idx < 4) {
        if (_sub_power[esc_idx]->get_last_recv_timestamp() + 1000 > AP_HAL::millis()) {
            telem_data_mask |= AP_ESC_Telem_Backend::TelemetryType::CURRENT;
            telem_data_mask |= AP_ESC_Telem_Backend::TelemetryType::VOLTAGE;
        }

        if (_sub_status[esc_idx]->get_last_recv_timestamp() + 1000 > AP_HAL::millis()) {
            telem_data_mask |= AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE;
        }
    }

    return telem_data_mask;
}


/**
 * @note reg.udral.physics.dynamics.rotation.PlanarTs_0_1
 */
void UavcanDynamicsSubscriber::subscribe()
{
    subscribeOnMessage(reg_udral_physics_dynamics_rotation_PlanarTs_0_1_EXTENT_BYTES_);
}

void UavcanDynamicsSubscriber::handler(const CanardRxTransfer* transfer)
{
    _last_recv_ts_ms = AP_HAL::millis();
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_physics_dynamics_rotation_PlanarTs_0_1_deserialize_(&msg, payload, &payload_len);
}

uint32_t UavcanDynamicsSubscriber::get_last_recv_timestamp()
{
    return _last_recv_ts_ms;
}

float UavcanDynamicsSubscriber::get_angular_position()
{
    return msg.value.kinematics.angular_position.radian;
}

float UavcanDynamicsSubscriber::get_angular_velocity()
{
    return msg.value.kinematics.angular_velocity.radian_per_second;
}

float UavcanDynamicsSubscriber::get_angular_acceleration()
{
    return msg.value.kinematics.angular_acceleration.radian_per_second_per_second;
}

float UavcanDynamicsSubscriber::get_torque()
{
    return msg.value._torque.newton_meter;
}


/**
 * @note reg.udral.physics.electricity.PowerTs_0_1
 */
void UavcanElectricityPowerTsSubscriber::subscribe()
{
    subscribeOnMessage(reg_udral_physics_electricity_PowerTs_0_1_EXTENT_BYTES_);
}

void UavcanElectricityPowerTsSubscriber::handler(const CanardRxTransfer* transfer)
{
    _last_recv_ts_ms = AP_HAL::millis();
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_physics_electricity_PowerTs_0_1_deserialize_(&msg, payload, &payload_len);
}

uint32_t UavcanElectricityPowerTsSubscriber::get_last_recv_timestamp()
{
    return _last_recv_ts_ms;
}

float UavcanElectricityPowerTsSubscriber::get_voltage()
{
    return msg.value.voltage.volt;
}

float UavcanElectricityPowerTsSubscriber::get_current()
{
    return msg.value.current.ampere;
}


/**
 * @note reg.udral.service.actuator.common.Status_0_1
 */
void UavcanStatusSubscriber::subscribe()
{
    subscribeOnMessage(reg_udral_service_actuator_common_Status_0_1_EXTENT_BYTES_);
}

void UavcanStatusSubscriber::handler(const CanardRxTransfer* transfer)
{
    _last_recv_ts_ms = AP_HAL::millis();
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_service_actuator_common_Status_0_1_deserialize_(&_msg, payload, &payload_len);
}

uint32_t UavcanStatusSubscriber::get_last_recv_timestamp()
{
    return _last_recv_ts_ms;
}

float UavcanStatusSubscriber::get_motor_temperature()
{
    return _msg.motor_temperature.kelvin;
}


/**
 * @note reg.udral.service.actuator.common.sp.*
 */
UavcanSetpointPublisher::UavcanSetpointPublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
    UavcanBasePublisher(ins, tx_queue, port_id)
{
    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
}

void UavcanSetpointPublisher::set_setpoint(SrvConfig *srv_config)
{
    for (uint_fast8_t sp_idx = 0; sp_idx < 4; sp_idx++) {
        _vector4_sp.value[sp_idx] = (hal.rcout->scale_esc_to_unity(srv_config[sp_idx].pulse) + 1.0) / 2.0;
    }
}

void UavcanSetpointPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void UavcanSetpointPublisher::publish()
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
UavcanReadinessPublisher::UavcanReadinessPublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
    UavcanBasePublisher(ins, tx_queue, port_id)
{
    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;

    _readiness.value = reg_udral_service_common_Readiness_0_1_SLEEP;
}

void UavcanReadinessPublisher::update_readiness()
{
    if (hal.util->safety_switch_state() == AP_HAL::Util::safety_state::SAFETY_NONE) {
        _readiness.value = reg_udral_service_common_Readiness_0_1_SLEEP;
    } else if (!hal.util->get_soft_armed()) {
        _readiness.value = reg_udral_service_common_Readiness_0_1_STANDBY;
    } else {
        _readiness.value = reg_udral_service_common_Readiness_0_1_ENGAGED;
    }
}

void UavcanReadinessPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void UavcanReadinessPublisher::publish()
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

#endif // HAL_ENABLE_LIBUAVCAN_V1_DRIVERS
