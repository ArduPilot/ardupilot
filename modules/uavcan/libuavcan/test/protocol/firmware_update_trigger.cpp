/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/firmware_update_trigger.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"

using namespace uavcan::protocol::file;

struct FirmwareVersionChecker : public uavcan::IFirmwareVersionChecker
{
    unsigned should_request_cnt;
    unsigned should_retry_cnt;
    unsigned confirmation_cnt;

    std::string firmware_path;

    int retry_quota;
    std::string expected_node_name_to_update;

    BeginFirmwareUpdate::Response last_error_response;

    FirmwareVersionChecker()
        : should_request_cnt(0)
        , should_retry_cnt(0)
        , confirmation_cnt(0)
        , retry_quota(0)
    { }

    virtual bool shouldRequestFirmwareUpdate(uavcan::NodeID node_id,
                                             const uavcan::protocol::GetNodeInfo::Response& node_info,
                                             FirmwareFilePath& out_firmware_file_path)
    {
        should_request_cnt++;
        std::cout << "REQUEST? " << int(node_id.get()) << "\n" << node_info << std::endl;
        out_firmware_file_path = firmware_path.c_str();
        return node_info.name == expected_node_name_to_update;
    }

    virtual bool shouldRetryFirmwareUpdate(uavcan::NodeID node_id,
                                           const BeginFirmwareUpdate::Response& error_response,
                                           FirmwareFilePath& out_firmware_file_path)
    {
        last_error_response = error_response;
        std::cout << "RETRY? " << int(node_id.get()) << "\n" << error_response << std::endl;
        should_retry_cnt++;

        EXPECT_STREQ(firmware_path.c_str(), out_firmware_file_path.c_str());

        if (retry_quota > 0)
        {
            retry_quota--;
            return true;
        }
        else
        {
            return false;
        }
    }

    virtual void handleFirmwareUpdateConfirmation(uavcan::NodeID node_id,
                                                  const BeginFirmwareUpdate::Response& response)
    {
        confirmation_cnt++;
        std::cout << "CONFIRMED " << int(node_id.get()) << "\n" << response << std::endl;
    }
};

struct BeginFirmwareUpdateServer
{
    uint8_t response_error_code;

    BeginFirmwareUpdateServer() : response_error_code(0) { }

    void handleRequest(const uavcan::ReceivedDataStructure<typename BeginFirmwareUpdate::Request>& req,
                       uavcan::ServiceResponseDataStructure<typename BeginFirmwareUpdate::Response>& res) const
    {
        std::cout << "REQUEST\n" << req << std::endl;
        res.error = response_error_code;
        res.optional_error_message = "foobar";
    }

    typedef uavcan::MethodBinder<BeginFirmwareUpdateServer*,
        void (BeginFirmwareUpdateServer::*)(
            const uavcan::ReceivedDataStructure<typename BeginFirmwareUpdate::Request>&,
            uavcan::ServiceResponseDataStructure<typename BeginFirmwareUpdate::Response>&) const> Callback;

    Callback makeCallback() { return Callback(this, &BeginFirmwareUpdateServer::handleRequest); }
};


TEST(FirmwareUpdateTrigger, Basic)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<BeginFirmwareUpdate> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg3;

    InterlinkedTestNodesWithSysClock nodes;

    FirmwareVersionChecker checker;

    uavcan::NodeInfoRetriever node_info_retriever(nodes.a);            // On the same node

    uavcan::FirmwareUpdateTrigger trigger(nodes.a, checker);
    std::cout << "sizeof(uavcan::FirmwareUpdateTrigger): " << sizeof(uavcan::FirmwareUpdateTrigger) << std::endl;

    std::unique_ptr<uavcan::NodeStatusProvider> provider(new uavcan::NodeStatusProvider(nodes.b));    // Other node

    /*
     * Initializing
     */
    ASSERT_LE(0, trigger.start(node_info_retriever, "/path_prefix/"));

    ASSERT_LE(0, node_info_retriever.start());
    ASSERT_EQ(1, node_info_retriever.getNumListeners());

    uavcan::protocol::HardwareVersion hwver;
    hwver.unique_id[0] = 123;
    hwver.unique_id[4] = 213;
    hwver.unique_id[8] = 45;

    provider->setName("Ivan");
    provider->setHardwareVersion(hwver);

    ASSERT_LE(0, provider->startAndPublish());

    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());

    /*
     * Updating one node
     * The server that can confirm the request is not running yet
     */
    checker.firmware_path = "firmware_path";
    checker.expected_node_name_to_update = "Ivan";
    checker.retry_quota = 1000;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(1, trigger.getNumPendingNodes());

    ASSERT_EQ(1, checker.should_request_cnt);
    ASSERT_EQ(0, checker.should_retry_cnt);
    ASSERT_EQ(0, checker.confirmation_cnt);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    // Still running!
    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(1, trigger.getNumPendingNodes());

    /*
     * Starting the firmware update server that returns an error
     * The checker will instruct the trigger to repeat
     */
    uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateServer::Callback> srv(nodes.b);
    BeginFirmwareUpdateServer srv_impl;

    ASSERT_LE(0, srv.start(srv_impl.makeCallback()));

    srv_impl.response_error_code = BeginFirmwareUpdate::Response::ERROR_UNKNOWN;
    checker.retry_quota = 1000;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));

    ASSERT_EQ(1, checker.should_request_cnt);
    ASSERT_EQ(1, checker.should_retry_cnt);
    ASSERT_EQ(0, checker.confirmation_cnt);

    // Still running!
    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(1, trigger.getNumPendingNodes());

    /*
     * Trying again, this time with ERROR_IN_PROGRESS
     */
    srv_impl.response_error_code = BeginFirmwareUpdate::Response::ERROR_IN_PROGRESS;
    checker.retry_quota = 0;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2100));

    ASSERT_EQ(1, checker.should_request_cnt);
    ASSERT_EQ(1, checker.should_retry_cnt);
    ASSERT_EQ(1, checker.confirmation_cnt);

    // Stopped!
    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());

    /*
     * Restarting the node info provider
     * Now it doesn't need an update
     */
    provider.reset(new uavcan::NodeStatusProvider(nodes.b));

    provider->setName("Dmitry");
    provider->setHardwareVersion(hwver);

    ASSERT_LE(0, provider->startAndPublish());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2100));

    ASSERT_EQ(2, checker.should_request_cnt);
    ASSERT_EQ(1, checker.should_retry_cnt);
    ASSERT_EQ(1, checker.confirmation_cnt);

    // Stopped!
    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());

    /*
     * Final checks
     */
    ASSERT_EQ(0, nodes.a.internal_failure_count);
    ASSERT_EQ(0, nodes.b.internal_failure_count);
}


TEST(FirmwareUpdateTrigger, MultiNode)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<BeginFirmwareUpdate> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg3;

    TestNetwork<5> nodes;

    // The trigger node
    FirmwareVersionChecker checker;
    uavcan::NodeInfoRetriever node_info_retriever(nodes[0]);
    uavcan::FirmwareUpdateTrigger trigger(nodes[0], checker);

    // The client nodes
    std::unique_ptr<uavcan::NodeStatusProvider> provider_a(new uavcan::NodeStatusProvider(nodes[1]));
    std::unique_ptr<uavcan::NodeStatusProvider> provider_b(new uavcan::NodeStatusProvider(nodes[2]));
    std::unique_ptr<uavcan::NodeStatusProvider> provider_c(new uavcan::NodeStatusProvider(nodes[3]));
    std::unique_ptr<uavcan::NodeStatusProvider> provider_d(new uavcan::NodeStatusProvider(nodes[4]));

    uavcan::protocol::HardwareVersion hwver;

    /*
     * Initializing
     */
    ASSERT_LE(0, trigger.start(node_info_retriever, "/path_prefix/"));

    ASSERT_LE(0, node_info_retriever.start());
    ASSERT_EQ(1, node_info_retriever.getNumListeners());

    hwver.unique_id[0] = 0xAA;
    provider_a->setHardwareVersion(hwver);
    provider_a->setName("Victor");
    ASSERT_LE(0, provider_a->startAndPublish());

    hwver.unique_id[0] = 0xBB;
    provider_b->setHardwareVersion(hwver);
    provider_b->setName("Victor");
    ASSERT_LE(0, provider_b->startAndPublish());

    hwver.unique_id[0] = 0xCC;
    provider_c->setHardwareVersion(hwver);
    provider_c->setName("Alexey");
    ASSERT_LE(0, provider_c->startAndPublish());

    hwver.unique_id[0] = 0xDD;
    provider_d->setHardwareVersion(hwver);
    provider_d->setName("Victor");
    ASSERT_LE(0, provider_d->startAndPublish());

    checker.expected_node_name_to_update = "Victor";    // Victors will get updated, others will not
    checker.firmware_path = "abc";

    /*
     * Running - 3 will timout, 1 will be ignored
     */
    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());

    nodes.spinAll(uavcan::MonotonicDuration::fromMSec(4100));

    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(3, trigger.getNumPendingNodes());

    ASSERT_EQ(4, checker.should_request_cnt);
    ASSERT_EQ(0, checker.should_retry_cnt);
    ASSERT_EQ(0, checker.confirmation_cnt);

    /*
     * Initializing the BeginFirmwareUpdate servers
     */
    uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateServer::Callback> srv_a(nodes[1]);
    uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateServer::Callback> srv_b(nodes[2]);
    uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateServer::Callback> srv_c(nodes[3]);
    uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateServer::Callback> srv_d(nodes[4]);

    BeginFirmwareUpdateServer srv_a_impl;
    BeginFirmwareUpdateServer srv_b_impl;
    BeginFirmwareUpdateServer srv_c_impl;
    BeginFirmwareUpdateServer srv_d_impl;

    ASSERT_LE(0, srv_a.start(srv_a_impl.makeCallback()));
    ASSERT_LE(0, srv_b.start(srv_b_impl.makeCallback()));
    ASSERT_LE(0, srv_c.start(srv_c_impl.makeCallback()));
    ASSERT_LE(0, srv_d.start(srv_d_impl.makeCallback()));

    srv_a_impl.response_error_code = BeginFirmwareUpdate::Response::ERROR_INVALID_MODE; // retry
    srv_b_impl.response_error_code = BeginFirmwareUpdate::Response::ERROR_INVALID_MODE; // retry
    srv_c_impl.response_error_code = BeginFirmwareUpdate::Response::ERROR_INVALID_MODE; // ignore (see below)
    srv_d_impl.response_error_code = BeginFirmwareUpdate::Response::ERROR_OK;           // OK

    /*
     * Spinning, now we're getting some errors
     * This also checks correctness of the round-robin selector
     */
    checker.retry_quota = 2;
    nodes.spinAll(uavcan::MonotonicDuration::fromMSec(4200));   // Two will retry, one drop, one confirm

    ASSERT_TRUE(trigger.isTimerRunning());

    nodes.spinAll(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_EQ(0, trigger.getNumPendingNodes());         // All removed now

    EXPECT_EQ(4, checker.should_request_cnt);
    EXPECT_EQ(4, checker.should_retry_cnt);
    EXPECT_EQ(1, checker.confirmation_cnt);

    /*
     * Waiting for the timer to stop
     */
    nodes.spinAll(uavcan::MonotonicDuration::fromMSec(1100));

    ASSERT_FALSE(trigger.isTimerRunning());
}
