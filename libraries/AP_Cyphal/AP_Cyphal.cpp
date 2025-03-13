#include "AP_Cyphal.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <stdlib.h>   // malloc/free
#include <malloc.h>   // memalign
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Math/AP_Math.h>   

#include "zubax/telega/Temperatures_1_0.h"
#include "zubax/physics/dynamics/DoF3rdTs_1_0.h"
#include "zubax/physics/electricity/PowerTs_1_0.h"


struct ESC_State {
    float temperature = NAN;  // °C
    float current     = NAN;  // A
    float voltage     = NAN;  // V
    float rpm         = NAN;  // RPM
    bool has_temp     = false;
    bool has_dyn      = false;
    bool has_power    = false;
};

// поддерживаем три ESC: 124, 125, 126
static ESC_State esc_states[3];

// утилита для маппинга node_id -> индекс массива
static ESC_State* getEscState(uint8_t node_id) {
    switch (node_id) {
        case 124: return &esc_states[0];
        case 125: return &esc_states[1];
        case 126: return &esc_states[2];
        default: return nullptr;
    }
}

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Cyphal::var_info[] = {
    // @Param: NODE
    // @DisplayName: Cyphal node that is used for this network
    // @Description: Cyphal node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE",1, AP_Cyphal, node_id_autopilot, 40),
    // @Param: BMA
    // @DisplayName: Cyphal back motor add velocity
    // @Description: Cyphal back motor add velocity
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("BMA",2,AP_Cyphal, back_motor_add_vel, 80),
    AP_GROUPEND
};

//--------------------------------------------------------
AP_Cyphal::AP_Cyphal(uint8_t driver_index) : _driver_index(driver_index) {
    AP_Param::setup_object_defaults(this, var_info);
}

//--------------------------------------------------------
// кастомный new/delete для правильного выравнивания
void* AP_Cyphal::operator new(size_t size) {
    return memalign(alignof(AP_Cyphal), size);
}
void AP_Cyphal::operator delete(void* ptr) noexcept {
    free(ptr);
}
void* AP_Cyphal::operator new(size_t size, const std::nothrow_t&) noexcept {
    return memalign(alignof(AP_Cyphal), size);
}
void AP_Cyphal::operator delete(void* ptr, const std::nothrow_t&) noexcept {
    free(ptr);
}

//--------------------------------------------------------
bool AP_Cyphal::add_interface(AP_HAL::CANIface* can_iface) {
    if (_iface != nullptr) return false;
    _iface = can_iface;
    _transport_iface.attach_can_iface(can_iface);
    return true;
}

//--------------------------------------------------------
void* AP_Cyphal::memAllocate(CypInstance* const canard_, size_t amount) {
    auto* self = static_cast<AP_Cyphal*>(canard_->user_reference);
    return o1heapAllocate(self->_allocator, amount);
}

void AP_Cyphal::memFree(CypInstance* const canard_, void* pointer) {
    auto* self = static_cast<AP_Cyphal*>(canard_->user_reference);
    o1heapFree(self->_allocator, pointer);
}
//--------------------------------------------------------
void AP_Cyphal::init(uint8_t, bool) {
    if (_initialized) return;

    if (!_iface) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Cyphal: no CAN iface");
        return;
    }

    _allocator = o1heapInit(_heap_memory, CYPHAL_HEAP_SIZE);
    if (!_allocator) {
        test = false;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Cyphal: O1Heap init failed");
        return;
    }
    test = true;

    _canard = cypInit(memAllocate, memFree);
    _canard.user_reference = this;
    _canard.node_id = node_id_autopilot;

    int32_t res = canardRxSubscribe(
        &_canard,
        CanardTransferKindMessage,   // сообщение, а не сервис
        301,                         // subject-ID
        62,                          // макс. размер полезной нагрузки
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
        &sub_motor_temp); // temp

    res = canardRxSubscribe(
        &_canard,
        CanardTransferKindMessage,
        302,                         // dynamics
        62,
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
        &sub_motor_dyn);

    res = canardRxSubscribe(
        &_canard,
        CanardTransferKindMessage,
        303,                         // power
        62,
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
        &sub_motor_power);

    if (res < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                    "Cyphal: subscription failed %ld",
                    (long)res);
    }

    _tx_queue = canardTxInit(100, CANARD_MTU_CAN_CLASSIC);

    publisher_manager.init(_canard, _tx_queue);

    // запуск потока только один раз
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_Cyphal::thread_loop, void),
        "cyphal", CYPHAL_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_CAN, 0);

    _initialized = true;
}

//--------------------------------------------------------
void AP_Cyphal::reset() {
    _initialized = false;
    _iface = nullptr;
    _allocator = nullptr;
    _canard = {};
    _tx_queue = {};
    // поток остаётся, он будет ждать _initialized == true
}

//--------------------------------------------------------
void AP_Cyphal::thread_loop() {
    while (true) {
        if (!_initialized || !_iface) {
            hal.scheduler->delay(500); // 0.5 сек
            continue;
        }
        if (spinReceive(1000) < 0) {
            hal.scheduler->delay_microseconds(2);
        }
        publisher_manager.process_all();
        spinTransmit();
        hal.scheduler->delay(2);
    }
}

//--------------------------------------------------------
void AP_Cyphal::spinTransmit() {
    for (const CypTxQueueItem* ti = nullptr; (ti = canardTxPeek(&_tx_queue)) != nullptr;) {
        if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > AP_HAL::micros())) {
            if (!_transport_iface.send(ti)) break;
        }
        _canard.memory_free(&_canard, canardTxPop(&_tx_queue, ti));
    }
}

int8_t AP_Cyphal::spinReceive(uint16_t duration_us)
{
    uint64_t deadline_us = AP_HAL::micros64() + duration_us;
    while (AP_HAL::micros64() <= deadline_us) {
        CanardFrame canard_frame;
        if (!_transport_iface.receive(&canard_frame)) {
            // There is no avaliable data means fifo is empty
            hal.scheduler->delay_microseconds(100);
            continue;
        }

        CypRxTransfer transfer;
        int8_t result = canardRxAccept(&_canard, AP_HAL::micros64(), &canard_frame, 0, &transfer, NULL);
        if (result < 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
              "AP_CYPHAL: recv err: %d",
              (int)result);
            // An error has occurred: either an argument is invalid or we've ran out of memory.
            // It is possible to statically prove that an out-of-memory will never occur for a given
            // application if the heap is sized correctly; for background, refer to the Robson's Proof
            // and the documentation for O1Heap. Reception of an invalid frame is NOT an error.
            return result;
        } else if (result == 1) {
            processReceivedTransfer(0, &transfer);
            _canard.memory_free(&_canard, transfer.payload);
        } else {
            // Nothing to do.
            // The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
            // Reception of an invalid frame is NOT reported as an error because it is not an error.
        }
    }

    return 0;
}
void AP_Cyphal::processReceivedTransfer(const uint8_t iface_index, const CypRxTransfer* transfer)
{
    switch (transfer->metadata.port_id) {
        case 301:  // MotorFeedback
            handleMotorTemp(transfer);
            break;
        case 302:  // MotorDynamics
            handleMotorDyn(transfer);
            break;
        case 303:
            handleMotorPower(transfer);
            break;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "Cyphal: recv port %u", 
                          (unsigned)transfer->metadata.port_id);
            break;
    }
}
void AP_Cyphal::handleMotorTemp(const CypRxTransfer* transfer) {
    zubax_telega_Temperatures_1_0 msg;
    size_t size = transfer->payload_size;
    if (0 == zubax_telega_Temperatures_1_0_deserialize_(&msg, (const uint8_t*)transfer->payload, &size)) {
        uint8_t node_id = transfer->metadata.remote_node_id;
        ESC_State* st = getEscState(node_id);
        if (st) {
            st->temperature = (float)msg.power_stage - 273.15f;  // К → °C
            st->has_temp = true;
            maybeSendTelemetry(node_id, st);
        }
    }
}

void AP_Cyphal::handleMotorDyn(const CypRxTransfer* transfer) {
    zubax_physics_dynamics_DoF3rdTs_1_0 msg;
    size_t size = transfer->payload_size;
    if (0 == zubax_physics_dynamics_DoF3rdTs_1_0_deserialize_(&msg, (const uint8_t*)transfer->payload, &size)) {
        uint8_t node_id = transfer->metadata.remote_node_id;
        ESC_State* st = getEscState(node_id);
        if (st) {
            float rad_per_sec = msg.value.kinematics.base.velocity;
            st->rpm = rad_per_sec * 60.0f / (2.0f * M_PI);  // rad/s → RPM
            st->has_dyn = true;
            maybeSendTelemetry(node_id, st);
        }
    }
}

void AP_Cyphal::handleMotorPower(const CypRxTransfer* transfer) {
    zubax_physics_electricity_PowerTs_1_0 msg;
    size_t size = transfer->payload_size;
    if (0 == zubax_physics_electricity_PowerTs_1_0_deserialize_(&msg, (const uint8_t*)transfer->payload, &size)) {
        uint8_t node_id = transfer->metadata.remote_node_id;
        ESC_State* st = getEscState(node_id);
        if (st) {
            st->current = msg.value.current.ampere;
            st->voltage = msg.value.voltage.volt;
            st->has_power = true;
            maybeSendTelemetry(node_id, st);
        }
    }
}

void AP_Cyphal::maybeSendTelemetry(uint8_t node_id, ESC_State* st) {
    if (st->has_temp && st->has_dyn && st->has_power) {
        const int esc_index = (node_id == 124) ? 0 :
                              (node_id == 125) ? 1 :
                              (node_id == 126) ? 2 : -1;
        if (esc_index < 0) {
            return; // неизвестный node_id
        }

        AP_ESC_Telem_Backend::TelemetryData new_data{};
        uint16_t mask = 0;

        // температура силовой части (°C → cdeg)
        if (!isnan(st->temperature)) {
            new_data.temperature_cdeg = (int16_t)(st->temperature * 100.0f);
            mask |= AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE;
        }

        // ток (А)
        if (!isnan(st->current)) {
            new_data.current = st->current;
            mask |= AP_ESC_Telem_Backend::TelemetryType::CURRENT;
        }

        // напряжение (В)
        if (!isnan(st->voltage)) {
            new_data.voltage = st->voltage;
            mask |= AP_ESC_Telem_Backend::TelemetryType::VOLTAGE;
        }

        // время работы (сек) — у тебя пока нет, можно 0
        new_data.usage_s = 0;
        mask |= AP_ESC_Telem_Backend::TelemetryType::USAGE;

        // обновляем телеметрию
        AP::esc_telem().update_telem_data(esc_index, new_data, mask);

        // RPM
        if (!isnan(st->rpm)) {
            AP::esc_telem().update_rpm(esc_index, st->rpm, 0.0f);
        }

        // сброс после отправки
        *st = ESC_State{};
    }
}