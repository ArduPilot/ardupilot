/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include "event_tracer.hpp"


TEST(dynamic_node_id_server_EventTracer, EventCodeToString)
{
    using namespace uavcan::dynamic_node_id_server;

    // Simply checking some error codes
    ASSERT_STREQ("Error",                        IEventTracer::getEventName(TraceError));
    ASSERT_STREQ("RaftAppendEntriesCallFailure", IEventTracer::getEventName(TraceRaftAppendEntriesCallFailure));
    ASSERT_STREQ("RaftDiscoveryReceived",        IEventTracer::getEventName(TraceRaftDiscoveryReceived));
    ASSERT_STREQ("DiscoveryNodeRestartDetected", IEventTracer::getEventName(TraceDiscoveryNodeRestartDetected));
    ASSERT_STREQ("AllocationUnexpectedStage",    IEventTracer::getEventName(TraceAllocationUnexpectedStage));
}


TEST(dynamic_node_id_server_EventTracer, EnvironmentSelfTest)
{
    using namespace uavcan::dynamic_node_id_server;

    EventTracer tracer;

    ASSERT_EQ(0, tracer.getNumEvents());

    tracer.onEvent(TraceRaftAppendEntriesCallFailure, 123);
    ASSERT_EQ(1, tracer.getNumEvents());
    tracer.onEvent(TraceRaftAppendEntriesCallFailure, -456);
    ASSERT_EQ(2, tracer.getNumEvents());
    tracer.onEvent(TraceError, -0xFFFFFFFFFFFFFFFLL);
    ASSERT_EQ(3, tracer.getNumEvents());

    ASSERT_EQ(0, tracer.countEvents(TraceAllocationActivity));
    ASSERT_EQ(2, tracer.countEvents(TraceRaftAppendEntriesCallFailure));
    ASSERT_EQ(1, tracer.countEvents(TraceError));

    ASSERT_EQ(-456, tracer.getLastEventArgumentOrFail(TraceRaftAppendEntriesCallFailure));

    ASSERT_EQ(-0xFFFFFFFFFFFFFFFLL, tracer.getLastEventArgumentOrFail(TraceError));
}
