/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "can.hpp"

TEST(CanDriverMock, Basic)
{
    using uavcan::CanFrame;
    using uavcan::CanSelectMasks;

    SystemClockMock clockmock;
    CanDriverMock driver(3, clockmock);

    const uavcan::CanFrame* pending_tx[uavcan::MaxCanIfaces] = { };

    ASSERT_EQ(3, driver.getNumIfaces());

    // All WR, no RD
    CanSelectMasks masks;
    masks.write = 7;
    masks.read = 7;
    EXPECT_LT(0, driver.select(masks, pending_tx, uavcan::MonotonicTime::fromUSec(100)));
    EXPECT_EQ(7, masks.write);
    EXPECT_EQ(0, masks.read);

    for (unsigned i = 0; i < 3; i++)
    {
        driver.ifaces.at(i).writeable = false;
    }

    // No WR, no RD
    masks.write = 7;
    masks.read = 7;
    EXPECT_EQ(0, driver.select(masks, pending_tx, uavcan::MonotonicTime::fromUSec(100)));
    EXPECT_EQ(0, masks.write);
    EXPECT_EQ(0, masks.read);
    EXPECT_EQ(100, clockmock.monotonic);
    EXPECT_EQ(100, clockmock.utc);

    // No WR, #1 RD
    const CanFrame fr1 = makeCanFrame(123, "foo", EXT);
    driver.ifaces.at(1).pushRx(fr1);
    masks.write = 7;
    masks.read = 6;
    EXPECT_LT(0, driver.select(masks, pending_tx, uavcan::MonotonicTime::fromUSec(100)));
    EXPECT_EQ(0, masks.write);
    EXPECT_EQ(2, masks.read);
    CanFrame fr2;
    uavcan::MonotonicTime ts_monotonic;
    uavcan::UtcTime ts_utc;
    uavcan::CanIOFlags flags = 0;
    EXPECT_EQ(1, driver.getIface(1)->receive(fr2, ts_monotonic, ts_utc, flags));
    EXPECT_EQ(0, flags);
    EXPECT_EQ(fr1, fr2);
    EXPECT_EQ(100, ts_monotonic.toUSec());
    EXPECT_EQ(0, ts_utc.toUSec());

    // #0 WR, #1 RD, Select failure
    driver.ifaces.at(0).writeable = true;
    driver.select_failure = true;
    masks.write = 1;
    masks.read = 7;
    EXPECT_EQ(-1, driver.select(masks, pending_tx, uavcan::MonotonicTime::fromUSec(100)));
    EXPECT_EQ(1, masks.write);                               // Leaving masks unchanged - the library must ignore them
    EXPECT_EQ(7, masks.read);
}

TEST(CanDriverMock, Loopback)
{
    using uavcan::CanFrame;
    using uavcan::CanSelectMasks;

    SystemClockMock clockmock;
    CanDriverMock driver(1, clockmock);

    const uavcan::CanFrame* pending_tx[uavcan::MaxCanIfaces] = { };

    CanSelectMasks masks;
    masks.write = 1;
    masks.read = 1;
    EXPECT_LT(0, driver.select(masks, pending_tx, uavcan::MonotonicTime::fromUSec(100)));
    EXPECT_EQ(1, masks.write);
    EXPECT_EQ(0, masks.read);

    clockmock.advance(200);

    CanFrame fr1;
    fr1.id = 123 | CanFrame::FlagEFF;
    EXPECT_EQ(1, driver.getIface(0)->send(fr1, uavcan::MonotonicTime::fromUSec(10000), uavcan::CanIOFlagLoopback));

    masks.write = 0;
    masks.read = 1;
    EXPECT_LT(0, driver.select(masks, pending_tx, uavcan::MonotonicTime::fromUSec(100)));
    EXPECT_EQ(0, masks.write);
    EXPECT_EQ(1, masks.read);

    CanFrame fr2;
    uavcan::MonotonicTime ts_monotonic;
    uavcan::UtcTime ts_utc;
    uavcan::CanIOFlags flags = 0;
    EXPECT_EQ(1, driver.getIface(0)->receive(fr2, ts_monotonic, ts_utc, flags));
    EXPECT_EQ(uavcan::CanIOFlagLoopback, flags);
    EXPECT_EQ(fr1, fr2);
    EXPECT_EQ(200, ts_monotonic.toUSec());
    EXPECT_EQ(0, ts_utc.toUSec());
}
