/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"


struct AdHocNodeStatusUpdater : public uavcan::IAdHocNodeStatusUpdater
{
    uavcan::uint64_t invokations;

    AdHocNodeStatusUpdater() : invokations(0) { }

    virtual void updateNodeStatus()
    {
        invokations++;
    }
};


TEST(NodeStatusProvider, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::NodeStatusProvider nsp(nodes.a);

    /*
     * Initialization
     */
    uavcan::protocol::HardwareVersion hwver;
    hwver.major = 3;
    hwver.minor = 14;

    uavcan::protocol::SoftwareVersion swver;
    swver.major = 2;
    swver.minor = 18;
    swver.vcs_commit = 0x600DF00D;

    nsp.setHardwareVersion(hwver);
    nsp.setSoftwareVersion(swver);

    ASSERT_TRUE(nsp.getName().empty());
    nsp.setName("superluminal_communication_unit");
    ASSERT_STREQ("superluminal_communication_unit", nsp.getName().c_str());

    ASSERT_EQ(uavcan::protocol::NodeStatus::HEALTH_OK, nsp.getHealth());
    ASSERT_EQ(uavcan::protocol::NodeStatus::MODE_INITIALIZATION, nsp.getMode());
    nsp.setHealthError();
    nsp.setModeOperational();
    ASSERT_EQ(uavcan::protocol::NodeStatus::HEALTH_ERROR, nsp.getHealth());
    ASSERT_EQ(uavcan::protocol::NodeStatus::MODE_OPERATIONAL, nsp.getMode());

    // Will fail - types are not registered
    uavcan::GlobalDataTypeRegistry::instance().reset();
    ASSERT_GT(0, nsp.startAndPublish());

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    ASSERT_LE(0, nsp.startAndPublish());

    // Checking the publishing rate settings
    ASSERT_EQ(uavcan::MonotonicDuration::fromMSec(uavcan::protocol::NodeStatus::MAX_BROADCASTING_PERIOD_MS),
              nsp.getStatusPublicationPeriod());

    nsp.setStatusPublicationPeriod(uavcan::MonotonicDuration());
    ASSERT_EQ(uavcan::MonotonicDuration::fromMSec(uavcan::protocol::NodeStatus::MIN_BROADCASTING_PERIOD_MS),
              nsp.getStatusPublicationPeriod());

    nsp.setStatusPublicationPeriod(uavcan::MonotonicDuration::fromMSec(3600 * 1000 * 24));
    ASSERT_EQ(uavcan::MonotonicDuration::fromMSec(uavcan::protocol::NodeStatus::MAX_BROADCASTING_PERIOD_MS),
              nsp.getStatusPublicationPeriod());

    AdHocNodeStatusUpdater ad_hoc;
    ASSERT_EQ(UAVCAN_NULLPTR, nsp.getAdHocNodeStatusUpdater());
    nsp.setAdHocNodeStatusUpdater(&ad_hoc);
    ASSERT_EQ(&ad_hoc, nsp.getAdHocNodeStatusUpdater());

    /*
     * Initial status publication
     */
    SubscriberWithCollector<uavcan::protocol::NodeStatus> status_sub(nodes.b);

    ASSERT_LE(0, status_sub.start());
    ASSERT_FALSE(status_sub.collector.msg.get());  // No data yet

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(status_sub.collector.msg.get());  // Was published at startup
    ASSERT_EQ(uavcan::protocol::NodeStatus::HEALTH_ERROR, status_sub.collector.msg->health);
    ASSERT_EQ(0, status_sub.collector.msg->vendor_specific_status_code);
    ASSERT_GE(1, status_sub.collector.msg->uptime_sec);

    ASSERT_EQ(0, ad_hoc.invokations);   // Not invoked from startAndPublish()

    /*
     * Altering the vendor-specific status code, forcePublish()-ing it and checking the result
     */
    ASSERT_EQ(0, nsp.getVendorSpecificStatusCode());
    nsp.setVendorSpecificStatusCode(1234);
    ASSERT_EQ(1234, nsp.getVendorSpecificStatusCode());

    ASSERT_LE(0, nsp.forcePublish());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_EQ(uavcan::protocol::NodeStatus::HEALTH_ERROR, status_sub.collector.msg->health);
    ASSERT_EQ(1234, status_sub.collector.msg->vendor_specific_status_code);
    ASSERT_GE(1, status_sub.collector.msg->uptime_sec);

    ASSERT_EQ(0, ad_hoc.invokations);   // Not invoked from forcePublish()

    /*
     * Explicit node info request
     */
    ServiceClientWithCollector<uavcan::protocol::GetNodeInfo> gni_cln(nodes.b);

    nsp.setHealthCritical();

    ASSERT_FALSE(gni_cln.collector.result.get());  // No data yet
    ASSERT_LE(0, gni_cln.call(1, uavcan::protocol::GetNodeInfo::Request()));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_TRUE(gni_cln.collector.result.get());   // Response must have been delivered

    ASSERT_TRUE(gni_cln.collector.result->isSuccessful());
    ASSERT_EQ(1, gni_cln.collector.result->getCallID().server_node_id.get());

    ASSERT_EQ(uavcan::protocol::NodeStatus::HEALTH_CRITICAL,
              gni_cln.collector.result->getResponse().status.health);

    ASSERT_TRUE(hwver == gni_cln.collector.result->getResponse().hardware_version);
    ASSERT_TRUE(swver == gni_cln.collector.result->getResponse().software_version);

    ASSERT_EQ("superluminal_communication_unit", gni_cln.collector.result->getResponse().name);

    ASSERT_EQ(0, ad_hoc.invokations);   // No timer-triggered publications happened yet

    /*
     * Timer triggered publication
     */
    EXPECT_EQ(3, nodes.a.getDispatcher().getTransferPerfCounter().getTxTransferCount());

    nodes.spinBoth(nsp.getStatusPublicationPeriod());

    EXPECT_EQ(1, ad_hoc.invokations);   // No timer-triggered publications happened yet
    EXPECT_EQ(4, nodes.a.getDispatcher().getTransferPerfCounter().getTxTransferCount());
}
