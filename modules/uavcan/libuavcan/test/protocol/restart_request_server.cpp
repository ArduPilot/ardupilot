/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/restart_request_server.hpp>
#include "helpers.hpp"


struct RestartHandler : public uavcan::IRestartRequestHandler
{
    bool accept;

    bool handleRestartRequest(uavcan::NodeID request_source)
    {
        std::cout << "Restart request from " << int(request_source.get()) << " will be "
                  << (accept ? "accepted" : "rejected") << std::endl;
        return accept;
    }
};


TEST(RestartRequestServer, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::RestartRequestServer rrs(nodes.a);

    ServiceClientWithCollector<uavcan::protocol::RestartNode> rrs_cln(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::RestartNode> _reg1;

    ASSERT_LE(0, rrs.start());

    uavcan::protocol::RestartNode::Request request;
    request.magic_number = uavcan::protocol::RestartNode::Request::MAGIC_NUMBER;

    /*
     * Rejected - handler was not set
     */
    ASSERT_LE(0, rrs_cln.call(1, request));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(rrs_cln.collector.result.get());
    ASSERT_TRUE(rrs_cln.collector.result->isSuccessful());
    ASSERT_FALSE(rrs_cln.collector.result->getResponse().ok);

    /*
     * Accepted
     */
    RestartHandler handler;
    handler.accept = true;
    rrs.setHandler(&handler);

    ASSERT_LE(0, rrs_cln.call(1, request));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(rrs_cln.collector.result->isSuccessful());
    ASSERT_TRUE(rrs_cln.collector.result->getResponse().ok);

    /*
     * Rejected by handler
     */
    handler.accept = false;

    ASSERT_LE(0, rrs_cln.call(1, request));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(rrs_cln.collector.result->isSuccessful());
    ASSERT_FALSE(rrs_cln.collector.result->getResponse().ok);

    /*
     * Rejected because of invalid magic number
     */
    handler.accept = true;

    ASSERT_LE(0, rrs_cln.call(1, uavcan::protocol::RestartNode::Request()));
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_TRUE(rrs_cln.collector.result->isSuccessful());
    ASSERT_FALSE(rrs_cln.collector.result->getResponse().ok);
}
