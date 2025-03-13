#include <AP_Cyphal/AP_Cyphal_Pub.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_Cyphal.h"  // чтобы видеть back_motor_add_vel

#include <AP_HAL/AP_HAL.h>

using namespace AP_HAL;
extern const HAL& hal;

void CyphalPublisherManager::init(CypInstance&ins, CypTxQueue& tx_queue)
{
    CyphalBasePublisher *publisher;

    ///< Minimal requrement
    publisher = new CyphalHeartbeatPublisher(ins, tx_queue);
    add_publisher(publisher);

    publisher = new CyphalSetpointPublisher(ins, tx_queue);
    add_publisher(publisher);

    publisher = new CyphalReadinessPublisherFront(ins, tx_queue);
    add_publisher(publisher);

    publisher = new CyphalReadinessPublisherBack(ins, tx_queue);
    add_publisher(publisher);

}

bool CyphalPublisherManager::add_publisher(CyphalBasePublisher *publisher)
{
    if (publisher == nullptr || number_of_publishers >= max_number_of_publishers) {
        return false;
    }

    publishers[number_of_publishers] = publisher;
    number_of_publishers++;
    return true;
}

void CyphalPublisherManager::process_all()
{
    for (uint_fast8_t pub_idx = 0; pub_idx < number_of_publishers; pub_idx++) {
        if (publishers[pub_idx] != nullptr && publishers[pub_idx]->get_port_id() != 0) {
            publishers[pub_idx]->update();
        }
    }
}

void CyphalBasePublisher::push(size_t buf_size, uint8_t* buf)
{
    auto result = canardTxPush(&_tx_queue, &_canard, 0, &_transfer_metadata, buf_size, buf);
    if (result < 0) {
        // An error has occurred: either an argument is invalid, the TX queue is full, or we've
        // run out of memory. It is possible to statically prove that an out-of-memory will
        // never occur for a given application if the heap is sized correctly; for background,
        // refer to the Robson's Proof and the documentation for O1Heap.
    }
}

CyphalHeartbeatPublisher::CyphalHeartbeatPublisher(CypInstance&ins, CypTxQueue& tx_queue) :
    CyphalBasePublisher(ins, tx_queue, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_)
{
    msg.health.value = uavcan_node_Health_1_0_NOMINAL;
    msg.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
    msg.vendor_specific_status_code = static_cast<uint8_t>(0);

    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}


void CyphalHeartbeatPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalHeartbeatPublisher::publish()
{
    msg.uptime = AP_HAL::millis() / 1000;

    uint8_t buf[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    int32_t result = uavcan_node_Heartbeat_1_0_serialize_(&msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push(buf_size, buf);
    }

    _transfer_metadata.transfer_id++;
}

CyphalReadinessPublisherBack::CyphalReadinessPublisherBack(CypInstance &ins,
                                                             CypTxQueue& tx_queue) :
    CyphalBasePublisher(ins, tx_queue, 11)   // порт = 10 (можно вынести в параметр)
{
    memset(&msg, 0, sizeof(msg));

    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = 11;   // тот же, что в .h
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void CyphalReadinessPublisherBack::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalReadinessPublisherBack::publish()
{
    // По умолчанию ставим SLEEP
    msg.value = zubax_service_Readiness_1_0_SLEEP;

    // Проверка условий: арминг, emergency stop и значение PWM на 5 канале
    const bool armed = hal.util->get_soft_armed();
    const bool emergency = SRV_Channels::get_emergency_stop();  // замени на свой API если нужно

    uint16_t pwm = 1000;  // значение по умолчанию
    bool have_pwm = SRV_Channels::get_output_pwm_chan(4, pwm);

    if (armed && !emergency && have_pwm && pwm != 1000) {
        msg.value = zubax_service_Readiness_1_0_ENGAGED;
    }

    uint8_t buf[zubax_service_Readiness_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = sizeof(buf);
    int32_t res = zubax_service_Readiness_1_0_serialize_(&msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == res) {
        // ⚡ Лог в GCS — какое значение реально отправляем
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Cyphal Readiness: %u (armed=%d, emergency=%d, pwm=%u)",
                      (unsigned)msg.value,
                      (int)armed,
                      (int)emergency,
                      (unsigned)pwm);

        push(buf_size, buf);
        _transfer_metadata.transfer_id++;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "Cyphal: Readiness serialize fail (%ld)", (long)res);
    }
}

CyphalReadinessPublisherFront::CyphalReadinessPublisherFront(CypInstance &ins,
                                                             CypTxQueue& tx_queue) :
    CyphalBasePublisher(ins, tx_queue, 10)   // порт = 10 (можно вынести в параметр)
{
    memset(&msg, 0, sizeof(msg));

    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = 10;   // тот же, что в .h
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void CyphalReadinessPublisherFront::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalReadinessPublisherFront::publish()
{
    // По умолчанию ставим SLEEP
    msg.value = zubax_service_Readiness_1_0_SLEEP;

    // Проверка условий: арминг и отсутствие emergency stop
    const bool armed = hal.util->get_soft_armed();
    const bool emergency = SRV_Channels::get_emergency_stop();  // если у тебя другой API – замени здесь

    if (armed && !emergency) {
        msg.value = zubax_service_Readiness_1_0_ENGAGED;
    }

    uint8_t buf[zubax_service_Readiness_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = sizeof(buf);
    int32_t res = zubax_service_Readiness_1_0_serialize_(&msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == res) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Cyphal Readiness: %u (armed=%d, emergency=%d)",
                      (unsigned)msg.value,
                      (int)armed,
                      (int)emergency);
        push(buf_size, buf);
        _transfer_metadata.transfer_id++;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "Cyphal: Readiness serialize fail (%ld)", (long)res);
    }
}

CyphalSetpointPublisher::CyphalSetpointPublisher(CypInstance &ins, CypTxQueue &tx_queue)
    : CyphalBasePublisher(ins, tx_queue, 113)  // порт 113
{
    memset(&msg, 0, sizeof(msg));

    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = 113;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void CyphalSetpointPublisher::update() {
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalSetpointPublisher::publish() {
    uint16_t pwm = 0;

    // Канал 1 → msg.value[0]
    if (SRV_Channels::get_output_pwm_chan(0, pwm)) {
        msg.value[0] = static_cast<float>(pwm);
    } else {
        msg.value[0] = 0.0f;
    }

    // Канал 2 → msg.value[1]
    if (SRV_Channels::get_output_pwm_chan(1, pwm)) {
        msg.value[1] = static_cast<float>(pwm);
    } else {
        msg.value[1] = 0.0f;
    }

    // Канал 3 → msg.value[3] (с проверкой <1050)
    if (SRV_Channels::get_output_pwm_chan(2, pwm)) {
        msg.value[3] = (pwm < 1050) ? 50.0f : static_cast<float>(pwm - 1000);
    } else {
        msg.value[3] = 0.0f;
    }

    // Канал 4 → msg.value[2] (с проверкой <1050)
    if (SRV_Channels::get_output_pwm_chan(3, pwm)) {
        msg.value[2] = (pwm < 1050) ? 50.0f : static_cast<float>(pwm - 1000);
    } else {
        msg.value[2] = 0.0f;
    }

    // Канал 5 → msg.value[4] с добавлением back_motor_add_vel
    if (SRV_Channels::get_output_pwm_chan(4, pwm)) {
        msg.value[4] = static_cast<float>(pwm - 1000 + 80);
    } else {
        msg.value[4] = 0.0f;
    }

    // Сериализация
    uint8_t buf[zubax_primitive_real16_Vector31_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = sizeof(buf);
    int32_t res = zubax_primitive_real16_Vector31_1_0_serialize_(&msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == res) {
        push(buf_size, buf);
        _transfer_metadata.transfer_id++;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Cyphal: serialize fail (%ld)", (long)res);
    }
}