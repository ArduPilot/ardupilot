/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer_listener.hpp>
#include "../clock.hpp"
#include "transfer_test_helpers.hpp"


static uavcan::RxFrame makeFrame()
{
    uavcan::RxFrame frame(uavcan::Frame(123, uavcan::TransferTypeMessageBroadcast, 1, uavcan::NodeID::Broadcast, 0),
                          tsMono(123), tsUtc(456), 0);
    uint8_t data[8];
    for (uint8_t i = 0; i < sizeof(data); i++)
    {
        data[i] = i;
    }
    frame.setPayload(data, sizeof(data));
    frame.setEndOfTransfer(true);
    return frame;
}


static bool match(const uavcan::IncomingTransfer& it, const uavcan::RxFrame& frame,
                  const uint8_t* payload, unsigned payload_len)
{
    // Fields extracted from the frame struct
    EXPECT_EQ(it.getMonotonicTimestamp(), frame.getMonotonicTimestamp());
    EXPECT_EQ(it.getUtcTimestamp(),       frame.getUtcTimestamp());
    EXPECT_EQ(it.getSrcNodeID(),          frame.getSrcNodeID());
    EXPECT_EQ(it.getTransferID(),         frame.getTransferID());
    EXPECT_EQ(it.getTransferType(),       frame.getTransferType());

    // Payload comparison
    static const unsigned BUFLEN = 1024;
    uint8_t buf_reference[BUFLEN], buf_actual[BUFLEN];

    if (payload_len > BUFLEN)
    {
        std::cout << "match(): Payload is too long" << std::endl;
        exit(1);
    }

    std::fill(buf_reference, buf_reference + BUFLEN, 0);
    std::fill(buf_actual, buf_actual + BUFLEN, 0);
    std::copy(payload, payload + payload_len, buf_reference);

    EXPECT_EQ(payload_len, it.read(0, buf_actual, payload_len * 3));
    EXPECT_EQ(0, it.read(payload_len, buf_actual, payload_len * 3));

    return std::equal(buf_reference, buf_reference + BUFLEN, buf_actual);
}


TEST(SingleFrameIncomingTransfer, Basic)
{
    using uavcan::RxFrame;
    using uavcan::SingleFrameIncomingTransfer;

    const RxFrame frame = makeFrame();
    SingleFrameIncomingTransfer it(frame);

    ASSERT_TRUE(match(it, frame, frame.getPayloadPtr(), frame.getPayloadLen()));
}


TEST(MultiFrameIncomingTransfer, Basic)
{
    using uavcan::RxFrame;
    using uavcan::MultiFrameIncomingTransfer;

    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 100, uavcan::MemPoolBlockSize> poolmgr;
    uavcan::TransferBufferManager bufmgr(256, poolmgr);

    const RxFrame frame = makeFrame();
    uavcan::TransferBufferManagerKey bufmgr_key(frame.getSrcNodeID(), frame.getTransferType());
    uavcan::TransferBufferAccessor tba(bufmgr, bufmgr_key);

    MultiFrameIncomingTransfer it(frame.getMonotonicTimestamp(), frame.getUtcTimestamp(), frame, tba);

    /*
     * Empty read must fail
     */
    uint8_t data_byte = 0;
    ASSERT_GT(0, it.read(0, &data_byte, 1));  // Error - no such buffer

    /*
     * Filling the test data
     */
    const std::string data = "123Hello world";
    const uint8_t* const data_ptr = reinterpret_cast<const uint8_t*>(data.c_str());
    ASSERT_FALSE(bufmgr.access(bufmgr_key));
    ASSERT_TRUE(bufmgr.create(bufmgr_key));
    ASSERT_EQ(data.length(), bufmgr.access(bufmgr_key)->write(0, data_ptr, unsigned(data.length())));

    /*
     * Check
     */
    ASSERT_TRUE(match(it, frame, data_ptr, unsigned(data.length())));

    /*
     * Buffer release
     */
    ASSERT_TRUE(bufmgr.access(bufmgr_key));
    it.release();
    ASSERT_FALSE(bufmgr.access(bufmgr_key));
}
