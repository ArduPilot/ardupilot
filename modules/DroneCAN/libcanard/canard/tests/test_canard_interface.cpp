#include "common.h"
#include "canard_interface.h"
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <dronecan_msgs.h>
#include <gtest/gtest.h>
#include <canard/service_server.h>
#include <canard/service_client.h>
#include <time.h>

DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

using namespace Canard;
namespace StaticCanardTest {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
CanardRxState rx_state  {
        .next = CANARD_BUFFER_IDX_NONE,
        .buffer_blocks = CANARD_BUFFER_IDX_NONE,
        .dtid_tt_snid_dnid = 0,
    };
#pragma GCC diagnostic pop

///////////// TESTS for Subscriber and Publisher //////////////
static bool called_handle_node_status = false;
static uavcan_protocol_NodeStatus sent_msg;
static CanardRxTransfer last_transfer;
void handle_node_status(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &msg);
void handle_node_status(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &msg) {
    called_handle_node_status = true;
    last_transfer = transfer;
    // check if message is correct
    ASSERT_EQ(memcmp(&msg, &sent_msg, sizeof(uavcan_protocol_NodeStatus)), 0);
}

TEST(StaticCanardTest, test_publish_subscribe) {
    CANARD_TEST_INTERFACE_DEFINE(0);
    CANARD_TEST_INTERFACE_DEFINE(1);
    // create publisher for message uavcan_protocol_NodeStatus on interface CoreTestInterface
    // with callback function handle_node_status
    Publisher<uavcan_protocol_NodeStatus> node_status_pub0(CANARD_TEST_INTERFACE(0));
    // create publisher for message uavcan_protocol_NodeStatus on different interface instance CoreTestInterface
    // with callback function handle_node_status
    Publisher<uavcan_protocol_NodeStatus> node_status_pub1(CANARD_TEST_INTERFACE(1));

    // create subscriber for message uavcan_protocol_NodeStatus on interface CoreTestInterface
    auto static_cb0 = allocate_sub_static_callback(&handle_node_status, 0);

    // create subscriber for message uavcan_protocol_NodeStatus on different interface instance CoreTestInterface
    // with callback function handle_node_status
    auto static_cb1 = allocate_sub_static_callback(&handle_node_status, 1);

    uint8_t buffer0[1024] {};
    uint8_t buffer1[1024] {};
    CANARD_TEST_INTERFACE(0).init(buffer0, sizeof(buffer0));
    CANARD_TEST_INTERFACE(1).init(buffer1, sizeof(buffer1));

    // set node id for interfaces
    CANARD_TEST_INTERFACE(0).set_node_id(1);
    CANARD_TEST_INTERFACE(1).set_node_id(2);

    // create message
    uavcan_protocol_NodeStatus msg {};
    msg.uptime_sec = 1;
    msg.health = 2;
    msg.mode = 3;
    msg.sub_mode = 4;
    msg.vendor_specific_status_code = 5;
    sent_msg = msg;
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp = (uint64_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);

    called_handle_node_status = false;
    // publish message
    ASSERT_TRUE(node_status_pub0.broadcast(msg));
    CANARD_TEST_INTERFACE(0).update_tx(timestamp);

    // check if message was received
    ASSERT_TRUE(called_handle_node_status);

    deallocate(static_cb0);
    deallocate(static_cb1);
    CANARD_TEST_INTERFACE(0).free();
    CANARD_TEST_INTERFACE(1).free();
}

///////////// TESTS for Service Server and Client //////////////
class TestClient0 {
    Canard::CanardTestInterface &interface;
public:
    TestClient0(Canard::CanardTestInterface &_interface) : interface(_interface) {
    }
    static int call_counts;
    void handle_get_node_info_response(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoResponse &res) {
        (void)transfer;
        ASSERT_EQ(res.status.uptime_sec, 1);
        ASSERT_EQ(res.status.health, 2);
        ASSERT_EQ(res.status.mode, 3);
        ASSERT_EQ(res.status.sub_mode, 4);
        ASSERT_EQ(res.status.vendor_specific_status_code, 5);
        ASSERT_EQ(res.software_version.major, 1);
        ASSERT_EQ(res.software_version.minor, 2);
        ASSERT_EQ(res.hardware_version.major, 3);
        ASSERT_EQ(res.hardware_version.minor, 4);
        ASSERT_EQ(res.name.len, strlen("helloworld"));
        ASSERT_EQ(memcmp(res.name.data, "helloworld", res.name.len), 0);
        call_counts++;
    }
    Canard::ObjCallback<TestClient0, uavcan_protocol_GetNodeInfoResponse> get_node_info_client_callback{this, &TestClient0::handle_get_node_info_response};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client{interface, get_node_info_client_callback};
};

class TestClient1 {
    Canard::CanardTestInterface &interface;
public:
    TestClient1(Canard::CanardTestInterface &_interface) : interface(_interface) {
    }
    static int call_counts;
    void handle_get_node_info_response(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoResponse &res) {
        (void)transfer;
        ASSERT_EQ(res.status.uptime_sec, 1);
        ASSERT_EQ(res.status.health, 2);
        ASSERT_EQ(res.status.mode, 3);
        ASSERT_EQ(res.status.sub_mode, 4);
        ASSERT_EQ(res.status.vendor_specific_status_code, 5);
        ASSERT_EQ(res.software_version.major, 1);
        ASSERT_EQ(res.software_version.minor, 2);
        ASSERT_EQ(res.hardware_version.major, 3);
        ASSERT_EQ(res.hardware_version.minor, 4);
        ASSERT_EQ(res.name.len, strlen("helloworld"));
        ASSERT_EQ(memcmp(res.name.data, "helloworld", res.name.len), 0);
        call_counts++;
    }
    Canard::ObjCallback<TestClient1, uavcan_protocol_GetNodeInfoResponse> get_node_info_client_callback{this, &TestClient1::handle_get_node_info_response};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client{interface, get_node_info_client_callback};
};

int TestClient0::call_counts = 0;
int TestClient1::call_counts = 0;

class TestServer0 {
    Canard::CanardTestInterface &interface;
public:
    TestServer0(Canard::CanardTestInterface &_interface) : interface(_interface) {
    }
    static int call_counts;
    void handle_get_node_info_request(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req) {
        (void)req;
        call_counts++;
        uavcan_protocol_GetNodeInfoResponse res {};
        res.status.uptime_sec = 1;
        res.status.health = 2;
        res.status.mode = 3;
        res.status.sub_mode = 4;
        res.status.vendor_specific_status_code = 5;
        res.software_version.major = 1;
        res.software_version.minor = 2;
        res.hardware_version.major = 3;
        res.hardware_version.minor = 4;
        res.name.len = strlen("helloworld");
        memcpy(res.name.data, "helloworld", res.name.len);
        get_node_info_server.respond(transfer, res);
    }
    Canard::ObjCallback<TestServer0, uavcan_protocol_GetNodeInfoRequest> get_node_info_server_callback{this, &TestServer0::handle_get_node_info_request};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server{interface, get_node_info_server_callback};
};

class TestServer1 {
    Canard::CanardTestInterface &interface;
    Canard::ObjCallback<TestServer1, uavcan_protocol_GetNodeInfoRequest> get_node_info_server_callback{this, &TestServer1::handle_get_node_info_request};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server{interface, get_node_info_server_callback};
public:
    TestServer1(Canard::CanardTestInterface &_interface) : interface(_interface) {
    }
    static int call_counts;
    void handle_get_node_info_request(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req) {
        (void)req;
        call_counts++;
        uavcan_protocol_GetNodeInfoResponse res {};
        res.status.uptime_sec = 1;
        res.status.health = 2;
        res.status.mode = 3;
        res.status.sub_mode = 4;
        res.status.vendor_specific_status_code = 5;
        res.software_version.major = 1;
        res.software_version.minor = 2;
        res.hardware_version.major = 3;
        res.hardware_version.minor = 4;
        res.name.len = strlen("helloworld");
        memcpy(res.name.data, "helloworld", res.name.len);
        get_node_info_server.respond(transfer, res);
    }
};

int TestServer0::call_counts = 0;
int TestServer1::call_counts = 0;

TEST(StaticCanardTest, test_multiple_clients) {
    CANARD_TEST_INTERFACE_DEFINE(0);
    CANARD_TEST_INTERFACE_DEFINE(1);
    // create multiple clients
    TestClient0 *client0[2];
    for (auto &i : client0) {
        i = new TestClient0{test_interface_0};
    }
    TestClient1 *client1[2];
    for (auto &i : client1) {
        i = new TestClient1{test_interface_1};
    }
    // create servers
    TestServer0 server0 __attribute__((unused)){test_interface_0};
    TestServer1 server1 __attribute__((unused)){test_interface_1};
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp = (uint64_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);

    uint8_t buffer0[2048] {};
    uint8_t buffer1[2048] {};
    CANARD_TEST_INTERFACE(0).init(buffer0, sizeof(buffer0));
    CANARD_TEST_INTERFACE(1).init(buffer1, sizeof(buffer1));

    // set node id
    CANARD_TEST_INTERFACE(0).set_node_id(1);
    CANARD_TEST_INTERFACE(1).set_node_id(2);

    // send request
    uavcan_protocol_GetNodeInfoRequest req {};
    EXPECT_TRUE(client0[0]->get_node_info_client.request(2, req));
    CANARD_TEST_INTERFACE(0).update_tx(timestamp);
    CANARD_TEST_INTERFACE(1).update_tx(timestamp);
    ASSERT_EQ(TestClient0::call_counts, 1);
    EXPECT_TRUE(client0[1]->get_node_info_client.request(2, req));
    CANARD_TEST_INTERFACE(0).update_tx(timestamp);
    CANARD_TEST_INTERFACE(1).update_tx(timestamp);
    ASSERT_EQ(TestClient0::call_counts, 2);
    ASSERT_EQ(TestServer1::call_counts, 2);

    EXPECT_TRUE(client1[0]->get_node_info_client.request(1, req));
    CANARD_TEST_INTERFACE(1).update_tx(timestamp);
    CANARD_TEST_INTERFACE(0).update_tx(timestamp);
    ASSERT_EQ(TestClient1::call_counts, 1);
    EXPECT_TRUE(client1[1]->get_node_info_client.request(1, req));
    CANARD_TEST_INTERFACE(1).update_tx(timestamp);
    CANARD_TEST_INTERFACE(0).update_tx(timestamp);
    ASSERT_EQ(TestClient1::call_counts, 2);
    ASSERT_EQ(TestServer0::call_counts, 2);

    for (auto &i : client0) {
        delete i;
    }
    for (auto &i : client1) {
        delete i;
    }
    CANARD_TEST_INTERFACE(0).free();
    CANARD_TEST_INTERFACE(1).free();
}

#if CANARD_ENABLE_DEADLINE
static uint8_t test_var = 0;
static void test_node_status_server_callback(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &req) {
    (void)transfer;
    (void)req;
    test_var++;
}

TEST(StaticCanardTest, test_CleanupStaleTransfers)
{
    CANARD_TEST_INTERFACE_DEFINE(0);
    CANARD_TEST_INTERFACE_DEFINE(1);

    uavcan_protocol_NodeStatus node_status {};
    node_status.uptime_sec = 1;
    node_status.health = 2;
    node_status.mode = 3;
    node_status.sub_mode = 4;
    node_status.vendor_specific_status_code = 5;

    uint8_t buffer0[4096] {};
    uint8_t buffer1[4096] {};
    CANARD_TEST_INTERFACE(0).init(buffer0, sizeof(buffer0));
    CANARD_TEST_INTERFACE(1).init(buffer1, sizeof(buffer1));

    // set node id
    CANARD_TEST_INTERFACE(0).set_node_id(1);
    CANARD_TEST_INTERFACE(1).set_node_id(2);

    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub_0{CANARD_TEST_INTERFACE(0)};
    auto static_test_cb = Canard::allocate_static_callback(test_node_status_server_callback);
    Canard::Subscriber<uavcan_protocol_NodeStatus> node_status_sub_1{*static_test_cb, 1};

    node_status_pub_0.set_timeout_ms(1);
    for (int i = 0; i < 20; i++) {
        if (i % 2) {
            node_status_pub_0.set_timeout_ms(10);
        } else {
            node_status_pub_0.set_timeout_ms(1);
        }
        ASSERT_TRUE(node_status_pub_0.broadcast(node_status));
    }
    // sleep for 2ms
    usleep(2000);
    // current time in us
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp = uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL;
    // cleanup stale transfers
    canardCleanupStaleTransfers(&CANARD_TEST_INTERFACE(0).canard, timestamp);
    // update tx
    CANARD_TEST_INTERFACE(0).update_tx(timestamp);

    // 10 messages should have been dropped due to timeout
    ASSERT_EQ(test_var, 10);

    CANARD_TEST_INTERFACE(0).free();
    CANARD_TEST_INTERFACE(1).free();
    deallocate(static_test_cb);
}
#endif

} // namespace StaticCanardTest