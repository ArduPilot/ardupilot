#pragma once

#include <canard/interface.h>
#include <canard.h>
#include <canard/transfer_object.h>

namespace Canard {

class CanardInterface : public Interface {

public:
    CanardInterface(uint8_t index) :
    Interface(index) {}

    /// @brief delete copy constructor and assignment operator
    CanardInterface(const CanardInterface&) = delete;
    CanardInterface& operator=(const CanardInterface&) = delete;
    CanardInterface() = delete;

    void init(void* mem_arena, size_t mem_arena_size);

    /// @brief broadcast message to all listeners on Interface
    /// @param bc_transfer
    /// @return true if message was added to the queue
    bool broadcast(const Transfer &bcast_transfer) override;

    /// @brief request message from
    /// @param destination_node_id
    /// @param req_transfer
    /// @return true if request was added to the queue
    bool request(uint8_t destination_node_id, const Transfer &req_transfer) override;

    /// @brief respond to a request
    /// @param destination_node_id
    /// @param res_transfer
    /// @return true if response was added to the queue
    bool respond(uint8_t destination_node_id, const Transfer &res_transfer) override;

    void handle_frame(const CanardCANFrame &frame, uint64_t timestamp_usec);

    virtual void update_tx(uint64_t timestamp_usec) = 0;

    static void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    uint8_t get_node_id() const override { return canard.node_id; }

    CanardInstance canard {};
};

class CanardTestInterface;
class CanardTestNetwork {
    friend class CanardTestInterface;
public:
    static constexpr int N = CANARD_NUM_HANDLERS;
    CanardTestNetwork() {}
    void route_frame(CanardTestInterface *send_iface, const CanardCANFrame &frame, uint64_t timestamp_usec);
    CanardTestNetwork(const CanardTestNetwork&) = delete;
    CanardTestNetwork& operator=(const CanardTestNetwork&) = delete;

    static CanardTestNetwork& get_network() {
        static CanardTestNetwork network;
        return network;
    }
private:
    CanardTestInterface *ifaces[N];
};

class CanardTestInterface : public CanardInterface {
public:
    CanardTestInterface(uint8_t index) :
    CanardInterface(index) {
        CanardTestNetwork::get_network().ifaces[index] = this;
    }
    // push tx frame queue to network
    void update_tx(uint64_t timestamp_usec) override;
    // set node id
    void set_node_id(uint8_t node_id);

    void free() {
        // free transfer ids
        Canard::TransferObject::free_tid_ptr(get_index());
    }
};

} // namespace CubeFramework

/// @brief get singleton instance of interface, used by node to publish messages
#define CANARD_TEST_INTERFACE_DEFINE(index) \
    Canard::CanardTestInterface test_interface_##index {index};

#define CANARD_TEST_INTERFACE(index) test_interface_##index