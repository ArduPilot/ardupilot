#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Cyphal/AP_Cyphal_IFaceManager.h>
#include <AP_Cyphal/AP_Cyphal_Pub.h>

#include "cyphal/canard.h"
#include "o1heap/o1heap.h"

#ifndef CYPHAL_TX_QUEUE_FRAME_SIZE
#define CYPHAL_TX_QUEUE_FRAME_SIZE 512
#endif

#ifndef CYPHAL_STACK_SIZE
#define CYPHAL_STACK_SIZE 768
#endif

#ifndef CYPHAL_HEAP_SIZE
#define CYPHAL_HEAP_SIZE (1024 * 2)
#endif

struct ESC_State;

class AP_Cyphal : public AP_CANDriver {
public:
    AP_Cyphal(uint8_t driver_index);
    bool test;
    static const struct AP_Param::GroupInfo var_info[];

    bool add_interface(AP_HAL::CANIface* can_iface) override;
    void init(uint8_t driver_index, bool enable_filters) override;

    CypInstance& get_canard_instance() { return _canard; }
    CypTxQueue& get_tx_queue() { return _tx_queue; }
    uint8_t get_driver_index() const { return _driver_index; }

    // сброс для безопасного ресета
    void reset();

    // кастомный new/delete для выравнивания
    void* operator new(size_t size);
    void operator delete(void* ptr) noexcept;
    void* operator new(size_t size, const std::nothrow_t&) noexcept;
    void operator delete(void* ptr, const std::nothrow_t&) noexcept;
    int16_t get_back_motor_add_vel() const { return back_motor_add_vel.get(); }

private:
    bool _initialized = false;

    CyphalTransportIface _transport_iface;
    uint8_t _driver_index;
    AP_HAL::CANIface* _iface = nullptr;
    AP_Int8 _loglevel;


    int8_t spinReceive(uint16_t us);
    void processReceivedTransfer(const uint8_t iface_index, const CypRxTransfer* transfer);
    void thread_loop();
    void spinTransmit();

    // Heap для Cyphal
    uint8_t _heap_memory[CYPHAL_HEAP_SIZE] __attribute__ ((aligned(O1HEAP_ALIGNMENT)));
    O1HeapInstance* _allocator = nullptr;

    // Cyphal instance
    CypInstance _canard;
    CypTxQueue _tx_queue;

    void send_heartbeat();

    CyphalPublisherManager publisher_manager;

    // memory callbacks
    static void* memAllocate(CypInstance* const canard_, size_t amount);
    static void memFree(CypInstance* const canard_, void* pointer);
    // Params settings
    CanardRxSubscription sub_motor_temp{};
    CanardRxSubscription sub_motor_dyn{};
    CanardRxSubscription sub_motor_power{};
    
    AP_Int16 node_id_autopilot;
    AP_Int16 back_motor_add_vel;

    void handleMotorTemp(const CypRxTransfer* transfer);
    void handleMotorDyn(const CypRxTransfer* transfer);
    void handleMotorPower(const CypRxTransfer* transfer);

    void maybeSendTelemetry(uint8_t node_id, ESC_State* st);
};