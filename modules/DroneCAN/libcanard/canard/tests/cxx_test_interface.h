#pragma once
#include <canard/interface.h>
#include <canard/transfer_object.h>

namespace Canard {

class CoreTestInterface;
class TestNetwork {
    friend class CoreTestInterface;
public:
    static constexpr int N = 10;
    TestNetwork() {}
    void route_msg(CoreTestInterface *send_iface, uint8_t source_node_id, uint8_t destination_node_id, Transfer transfer, CanardTransferType transfer_type);
    TestNetwork(const TestNetwork&) = delete;
    TestNetwork& operator=(const TestNetwork&) = delete;

    static TestNetwork& get_network() {
        static TestNetwork network;
        return network;
    }
private:
    CoreTestInterface *ifaces[N];
};

class CoreTestInterface : public Interface {
public:
    CoreTestInterface(uint8_t index) :
    Interface(index) {
        TestNetwork::get_network().ifaces[index] = this;
    }

    /// @brief delete copy constructor and assignment operator
    CoreTestInterface(const CoreTestInterface&) = delete;
    CoreTestInterface& operator=(const CoreTestInterface&) = delete;
    CoreTestInterface() = delete;

    virtual ~CoreTestInterface() {}

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

    /// @brief set node id
    /// @param node_id
    void set_node_id(uint8_t node_id);

    /// @brief get node id
    /// @return node id
    uint8_t get_node_id() const override { return node_id; }

    void handle_transfer(CanardRxTransfer &transfer, CanardTransferType transfer_type);

    void free() {
        // free transfer ids
        Canard::TransferObject::free_tid_ptr(get_index());
    }
private:
    uint8_t node_id;
};

} // namespace Canard

/// @brief get singleton instance of interface, used by node to publish messages
#define CXX_TEST_INTERFACE_DEFINE(index) \
    Canard::CoreTestInterface test_interface_##index {index};

#define CXX_TEST_INTERFACE(index) test_interface_##index
