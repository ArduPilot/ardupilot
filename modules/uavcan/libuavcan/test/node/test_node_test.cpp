/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "test_node.hpp"


TEST(TestNode, TestNetwork)
{
    TestNetwork<4> nwk;

    uavcan::CanFrame frame;
    for (uint8_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }
    frame.id = 1234U;

    ASSERT_EQ(1, nwk.nodes[0]->can_driver.send(frame, uavcan::MonotonicTime(), uavcan::CanIOFlags()));

    for (int i = 1; i < 4; i++)
    {
        uavcan::CanFrame rx;
        uavcan::MonotonicTime ts_mono;
        uavcan::UtcTime ts_utc;
        uavcan::CanIOFlags flags = 0;
        ASSERT_EQ(1, nwk.nodes[i]->can_driver.receive(rx, ts_mono, ts_utc, flags));

        ASSERT_TRUE(rx == frame);
    }
}
