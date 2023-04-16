#pragma once
#include <AP_HAL/AP_HAL.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <canard/interface.h>

class AP_DroneCAN;
class CanardInterface : public Canard::Interface {
    friend class AP_DroneCAN;
public:

    /// @brief delete copy constructor and assignment operator
    CanardInterface(const CanardInterface&) = delete;
    CanardInterface& operator=(const CanardInterface&) = delete;

    CanardInterface(uint8_t driver_index);

    void init(void* mem_arena, size_t mem_arena_size, uint8_t node_id);

    /// @brief broadcast message to all listeners on Interface
    /// @param bc_transfer
    /// @return true if message was added to the queue
    bool broadcast(const Canard::Transfer &bcast_transfer) override;

    /// @brief request message from
    /// @param destination_node_id
    /// @param req_transfer
    /// @return true if request was added to the queue
    bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;

    /// @brief respond to a request
    /// @param destination_node_id
    /// @param res_transfer
    /// @return true if response was added to the queue
    bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

    void processTx();
    void processRx();

    void process(uint32_t duration);

    static void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    bool add_interface(AP_HAL::CANIface *can_drv);

#if AP_TEST_DRONECAN_DRIVERS
    static CanardInterface& get_test_iface() { return test_iface; }
    static void processTestRx();
#endif

    uint8_t get_node_id() const override { return canard.node_id; }
private:
    CanardInstance canard;
    AP_HAL::CANIface* ifaces[HAL_NUM_CAN_IFACES];
#if AP_TEST_DRONECAN_DRIVERS
    static CanardInterface* canard_ifaces[3];
    static CanardInterface test_iface;
#endif
    uint8_t num_ifaces;
    HAL_EventHandle _event_handle;
    bool initialized;
    HAL_Semaphore _sem;
};
#endif // HAL_ENABLE_DRONECAN_DRIVERS