#include "common.h"
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <dronecan_msgs.h>
#include <gtest/gtest.h>
#include <canard/service_server.h>
#include <canard/service_client.h>
#include "cxx_test_interface.h"

using namespace Canard;

DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

namespace StaticCoreTest {

///////////// TESTS for Subscriber and Publisher //////////////
static bool called_handle_node_status = false;
static uavcan_protocol_NodeStatus sent_msg;
static CanardRxTransfer last_transfer;
static void handle_node_status(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &msg) {
    called_handle_node_status = true;
    last_transfer = transfer;
    // check if message is correct
    ASSERT_EQ(memcmp(&msg, &sent_msg, sizeof(uavcan_protocol_NodeStatus)), 0);
}

static CXX_TEST_INTERFACE_DEFINE(0);
static CXX_TEST_INTERFACE_DEFINE(1);

TEST(StaticCoreTest, test_publish_subscribe) {
    // create publisher for message uavcan_protocol_NodeStatus on interface CoreTestInterface
    // with callback function handle_node_status
    CANARD_PUBLISHER(CXX_TEST_INTERFACE(0), node_status_pub0, uavcan_protocol_NodeStatus);
    // create publisher for message uavcan_protocol_NodeStatus on different interface instance CoreTestInterface
    // with callback function handle_node_status
    CANARD_PUBLISHER(CXX_TEST_INTERFACE(1), node_status_pub1, uavcan_protocol_NodeStatus);

    // create subscriber for message uavcan_protocol_NodeStatus on interface CoreTestInterface
    auto handle_node_status_cb_1 = Canard::allocate_sub_static_callback(handle_node_status, 0);
    // create subscriber for message uavcan_protocol_NodeStatus on different interface instance CoreTestInterface
    // with callback function handle_node_status
    auto handle_node_status_cb_2 = Canard::allocate_sub_static_callback(handle_node_status, 1);

    // set node id for interfaces
    CXX_TEST_INTERFACE(0).set_node_id(1);
    CXX_TEST_INTERFACE(1).set_node_id(2);

    // create message
    uavcan_protocol_NodeStatus msg {};
    msg.uptime_sec = 1;
    msg.health = 2;
    msg.mode = 3;
    msg.sub_mode = 4;
    msg.vendor_specific_status_code = 5;
    sent_msg = msg;

    called_handle_node_status = false;
    // publish message
    ASSERT_TRUE(node_status_pub0.broadcast(msg));
    // check if message was received
    ASSERT_TRUE(called_handle_node_status);

    deallocate(handle_node_status_cb_1);
    deallocate(handle_node_status_cb_2);
}

// test multiple subscribers

// default test subscriber without
class TestSubscriber0 {
public:
    TestSubscriber0() {
        handle_node_status_cb = Canard::allocate_sub_obj_callback(this, &StaticCoreTest::TestSubscriber0::handle_node_status, 0);
    }
    ~TestSubscriber0() {
        deallocate(handle_node_status_cb);
    }
    void handle_node_status(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &msg);
    static int call_counts;

private:
    Canard::SubscriberObjCb<TestSubscriber0,uavcan_protocol_NodeStatus>  *handle_node_status_cb;
};

void TestSubscriber0::handle_node_status(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &msg) {
    called_handle_node_status = true;
    last_transfer = transfer;
    // check if message is correct
    ASSERT_EQ(memcmp(&msg, &sent_msg, sizeof(uavcan_protocol_NodeStatus)), 0);
    call_counts++;
}

int TestSubscriber0::call_counts = 0;

// test subsriber with index 1
class TestSubscriber1 {
public:
    TestSubscriber1() {
        handle_node_status_cb = Canard::allocate_sub_obj_callback(this, &StaticCoreTest::TestSubscriber1::handle_node_status, 1);
    }
    ~TestSubscriber1() {
        deallocate(handle_node_status_cb);
    }
    void handle_node_status(const CanardRxTransfer &transfer, const uavcan_protocol_NodeStatus &msg) {
        called_handle_node_status = true;
        last_transfer = transfer;
        // check if message is correct
        ASSERT_EQ(memcmp(&msg, &sent_msg, sizeof(uavcan_protocol_NodeStatus)), 0);
        call_counts++;
    }
    static int call_counts;
private:
    Canard::SubscriberObjCb<TestSubscriber1,uavcan_protocol_NodeStatus> *handle_node_status_cb;
};

int TestSubscriber1::call_counts = 0;

// Create 5 subscribers and check if all of them are called
TEST(StaticCoreTest, test_multiple_subscribers) {
    // create publisher for message uavcan_protocol_NodeStatus on interface CoreTestInterface
    // with callback function handle_node_status
    CANARD_PUBLISHER(CXX_TEST_INTERFACE(0), node_status_pub0, uavcan_protocol_NodeStatus);
    // create publisher for message uavcan_protocol_NodeStatus on different interface instance CoreTestInterface
    // with callback function handle_node_status
    CANARD_PUBLISHER(CXX_TEST_INTERFACE(1), node_status_pub1, uavcan_protocol_NodeStatus);

    TestSubscriber0 test_subscriber0[5] __attribute__((unused)) {};
    TestSubscriber1 test_subscriber1[5] __attribute__((unused)) {};

    // set node id for interfaces
    CXX_TEST_INTERFACE(0).set_node_id(1);
    CXX_TEST_INTERFACE(1).set_node_id(2);

    // create message
    uavcan_protocol_NodeStatus msg {};
    msg.uptime_sec = 1;
    msg.health = 2;
    msg.mode = 3;
    msg.sub_mode = 4;
    msg.vendor_specific_status_code = 5;
    sent_msg = msg;

    // publish message
    ASSERT_TRUE(node_status_pub0.broadcast(msg));
    // check if message was received 5 times
    ASSERT_EQ(TestSubscriber1::call_counts, 5);
    // publish message
    ASSERT_TRUE(node_status_pub1.broadcast(msg));
    // check if message was received 5 times
    ASSERT_EQ(TestSubscriber0::call_counts, 5);
}

//////////// TESTS FOR SERVICE //////////////

// test single server single client
bool handle_get_node_info_response_called = false;
void handle_get_node_info_response(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoResponse &res);
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
    handle_get_node_info_response_called = true;
}

template <typename msgtype>
class StaticCallbackWrapper {
public:
    StaticCallbackWrapper(void (*_cb)(const CanardRxTransfer &transfer, const msgtype &req)) {
        cb = Canard::allocate_static_callback(_cb);
    }
    ~StaticCallbackWrapper() {
        deallocate(cb);
    }
    Canard::StaticCallback<msgtype> *cb;
};

void handle_get_node_info_request0(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req);
static StaticCallbackWrapper<uavcan_protocol_GetNodeInfoRequest> static_callback_wrapper0(handle_get_node_info_request0);
Canard::Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server0(CXX_TEST_INTERFACE(0), *static_callback_wrapper0.cb);

void handle_get_node_info_request1(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req);
static StaticCallbackWrapper<uavcan_protocol_GetNodeInfoRequest> static_callback_wrapper1(handle_get_node_info_request1);
Canard::Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server1(CXX_TEST_INTERFACE(1), *static_callback_wrapper1.cb);

void handle_get_node_info_request0(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req) {
    (void)req;
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
    get_node_info_server0.respond(transfer, res);
}

void handle_get_node_info_request1(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req) {
    (void)req;
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
    get_node_info_server1.respond(transfer, res);
}

TEST(StaticCoreTest, test_service) {
    // create client for service uavcan_protocol_GetNodeInfo on interface CoreTestInterface
    // with response callback function handle_get_node_info_response
    auto static_cb0 = allocate_static_callback(handle_get_node_info_response);
    Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client0(CXX_TEST_INTERFACE(0), *static_cb0);

    // create client for service uavcan_protocol_GetNodeInfo on different interface instance CoreTestInterface
    // with response callback function handle_get_node_info_response
    auto static_cb1 = allocate_static_callback(handle_get_node_info_response);
    Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client1(CXX_TEST_INTERFACE(1), *static_cb1);

    // set node id for interfaces
    CXX_TEST_INTERFACE(0).set_node_id(1);
    CXX_TEST_INTERFACE(1).set_node_id(2);

    // create request
    uavcan_protocol_GetNodeInfoRequest req {};
    handle_get_node_info_response_called = false;
    // send request
    ASSERT_TRUE(get_node_info_client0.request(2, req));
    // check if response was received
    ASSERT_TRUE(handle_get_node_info_response_called);

    handle_get_node_info_response_called = false;
    // send request
    ASSERT_TRUE(get_node_info_client1.request(1, req));
    // check if response was received
    ASSERT_TRUE(handle_get_node_info_response_called);
    deallocate(static_cb0);
    deallocate(static_cb1);
    CXX_TEST_INTERFACE(0).free();
    CXX_TEST_INTERFACE(1).free();
}

// test single server multiple clients
class TestClient0 {
public:
    TestClient0() {}
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

    ObjCallback<TestClient0, uavcan_protocol_GetNodeInfoResponse> get_node_info_response_cb{this, &StaticCoreTest::TestClient0::handle_get_node_info_response};
    Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client{CXX_TEST_INTERFACE(0), get_node_info_response_cb};
};

class TestClient1 {
public:
    TestClient1() {}
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
    ObjCallback<TestClient1, uavcan_protocol_GetNodeInfoResponse> get_node_info_response_cb{this, &StaticCoreTest::TestClient1::handle_get_node_info_response};
    Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client{CXX_TEST_INTERFACE(1), get_node_info_response_cb};
};

int TestClient0::call_counts = 0;
int TestClient1::call_counts = 0;

class TestServer0 {
public:
    TestServer0() {}
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
    ObjCallback<TestServer0, uavcan_protocol_GetNodeInfoRequest> get_node_info_request_cb{this, &StaticCoreTest::TestServer0::handle_get_node_info_request};
    Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server{CXX_TEST_INTERFACE(0), get_node_info_request_cb};
};

class TestServer1 {
public:
    TestServer1() {}
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
private:
    ObjCallback<TestServer1, uavcan_protocol_GetNodeInfoRequest> get_node_info_request_cb{this, &StaticCoreTest::TestServer1::handle_get_node_info_request};
    Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server{CXX_TEST_INTERFACE(1), get_node_info_request_cb};
};

int TestServer0::call_counts = 0;
int TestServer1::call_counts = 0;

TEST(StaticCoreTest, test_multiple_clients) {
    // create multiple clients
    TestClient0 client0[2];
    TestClient1 client1[2];
    // create servers
    TestServer0 server0 __attribute__((unused));
    TestServer1 server1 __attribute__((unused));

    // set node id
    CXX_TEST_INTERFACE(0).set_node_id(1);
    CXX_TEST_INTERFACE(1).set_node_id(2);

    // send request
    uavcan_protocol_GetNodeInfoRequest req {};
    EXPECT_TRUE(client0[0].get_node_info_client.request(2, req));
    ASSERT_EQ(TestClient0::call_counts, 1);
    EXPECT_TRUE(client0[1].get_node_info_client.request(2, req));
    ASSERT_EQ(TestClient0::call_counts, 2);
    ASSERT_EQ(TestServer1::call_counts, 2);

    EXPECT_TRUE(client1[0].get_node_info_client.request(1, req));
    ASSERT_EQ(TestClient1::call_counts, 1);
    EXPECT_TRUE(client1[1].get_node_info_client.request(1, req));
    ASSERT_EQ(TestClient1::call_counts, 2);
    ASSERT_EQ(TestServer0::call_counts, 2);

    // free memory
    CXX_TEST_INTERFACE(0).free();
    CXX_TEST_INTERFACE(1).free();
}

} // namespace StaticCoreTest