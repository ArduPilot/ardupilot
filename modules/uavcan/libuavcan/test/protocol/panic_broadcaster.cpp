/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/panic_broadcaster.hpp>
#include "helpers.hpp"


TEST(PanicBroadcaster, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::PanicBroadcaster panicker(nodes.a);

    SubscriberWithCollector<uavcan::protocol::Panic> sub(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::Panic> _reg1;

    ASSERT_LE(0, sub.start());

    panicker.panic("I lost my towel!");  // Only the first 7 chars allowed

    ASSERT_STREQ("I lost ", panicker.getReason().c_str());
    ASSERT_TRUE(panicker.isPanicking());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_STREQ("I lost ", sub.collector.msg->reason_text.c_str());
    sub.collector.msg.reset();

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(300));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_STREQ("I lost ", sub.collector.msg->reason_text.c_str());
    sub.collector.msg.reset();

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(300));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_STREQ("I lost ", sub.collector.msg->reason_text.c_str());
    sub.collector.msg.reset();

    panicker.dontPanic();
    ASSERT_FALSE(panicker.isPanicking());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(300));
    ASSERT_FALSE(sub.collector.msg.get());
}
