/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer_receiver.hpp>
#include "../clock.hpp"
#include "transfer_test_helpers.hpp"

/*
 * Beware!
 * The code you're about to look at desperately needs some cleaning.
 */

enum SotEotToggle
{
    SET000 = 0,
    SET001 = 1,
    SET010 = 2,
    SET011 = 3,
    SET100 = 4,
    SET101 = 5, // Illegal
    SET110 = 6,
    SET111 = 7  // Illegal
};

struct RxFrameGenerator
{
    static const uavcan::TransferBufferManagerKey DEFAULT_KEY;
    enum { TARGET_NODE_ID = 126 };

    uint16_t data_type_id;
    uavcan::TransferBufferManagerKey bufmgr_key;

    RxFrameGenerator(uint16_t data_type_id, const uavcan::TransferBufferManagerKey& bufmgr_key = DEFAULT_KEY)
        : data_type_id(data_type_id)
        , bufmgr_key(bufmgr_key)
    { }

    /// iface_index, data, set, transfer_id, ts_monotonic [, ts_utc]
    uavcan::RxFrame operator()(uint8_t iface_index, const std::string& data, SotEotToggle set,
                               uint8_t transfer_id, uint64_t ts_monotonic, uint64_t ts_utc = 0)
    {
        const uavcan::NodeID dst_nid =
            (bufmgr_key.getTransferType() == uavcan::TransferTypeMessageBroadcast) ?
            uavcan::NodeID::Broadcast : TARGET_NODE_ID;

        uavcan::Frame frame(data_type_id, bufmgr_key.getTransferType(), bufmgr_key.getNodeID(),
                            dst_nid, transfer_id);

        frame.setStartOfTransfer((set & (1 << 2)) != 0);
        frame.setEndOfTransfer((set & (1 << 1)) != 0);

        if ((set & (1 << 0)) != 0)
        {
            frame.flipToggle();
        }

        EXPECT_EQ(data.length(),
                  frame.setPayload(reinterpret_cast<const uint8_t*>(data.c_str()), unsigned(data.length())));

        uavcan::RxFrame output(frame, uavcan::MonotonicTime::fromUSec(ts_monotonic),
                               uavcan::UtcTime::fromUSec(ts_utc), iface_index);
        //std::cout << "Generated frame: " << output.toString() << std::endl;

        return output;
    }
};

const uavcan::TransferBufferManagerKey RxFrameGenerator::DEFAULT_KEY(42, uavcan::TransferTypeMessageBroadcast);


template <unsigned BufSize>
struct Context
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 100, uavcan::MemPoolBlockSize> pool;
    uavcan::TransferReceiver receiver;  // Must be default constructible and copyable
    uavcan::TransferBufferManager bufmgr;

    Context() :
        bufmgr(BufSize, pool)
    { }

    ~Context()
    {
        // We need to destroy the receiver before its buffer manager
        receiver = uavcan::TransferReceiver();
    }
};


static bool matchBufferContent(const uavcan::ITransferBuffer* tbb, const std::string& content)
{
    uint8_t data[1024];
    std::fill(data, data + sizeof(data), 0);
    if (content.length() > sizeof(data))
    {
        std::cerr << "matchBufferContent(): Content is too long" << std::endl;
        std::exit(1);
    }
    tbb->read(0, data, unsigned(content.length()));
    if (std::equal(content.begin(), content.end(), data))
    {
        return true;
    }
    std::cout << "Buffer content mismatch:"
              << "\n\tExpected: " << content
              << "\n\tActually: " << reinterpret_cast<const char*>(data)
              << std::endl;
    return false;
}


#define CHECK_NOT_COMPLETE(x) ASSERT_EQ(uavcan::TransferReceiver::ResultNotComplete, (x))
#define CHECK_COMPLETE(x)     ASSERT_EQ(uavcan::TransferReceiver::ResultComplete, (x))
#define CHECK_SINGLE_FRAME(x) ASSERT_EQ(uavcan::TransferReceiver::ResultSingleFrame, (x))

TEST(TransferReceiver, Basic)
{
    using uavcan::TransferReceiver;
    Context<32> context;
    RxFrameGenerator gen(789);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::TransferBufferManager& bufmgr = context.bufmgr;
    uavcan::TransferBufferAccessor bk(context.bufmgr, RxFrameGenerator::DEFAULT_KEY);

    std::cout << "sizeof(TransferReceiver): " << sizeof(TransferReceiver) << std::endl;

    /*
     * Empty
     */
    ASSERT_EQ(TransferReceiver::getDefaultTransferInterval(), rcv.getInterval());
    ASSERT_EQ(0, rcv.getLastTransferTimestampMonotonic().toUSec());

    /*
     * Single frame transfer with zero ts, must be ignored
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "Foo", SET110, 0, 0), bk));
    ASSERT_EQ(TransferReceiver::getDefaultTransferInterval(), rcv.getInterval());
    ASSERT_EQ(0, rcv.getLastTransferTimestampMonotonic().toUSec());

    /*
     * Valid compound transfer
     * Args: iface_index, data, set, transfer_id, ts_monotonic
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "\x34\x12" "34567", SET100, 0, 100), bk));
    CHECK_COMPLETE(    rcv.addFrame(gen(0, "foo",              SET011, 0, 200), bk));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567foo"));
    ASSERT_EQ(0x1234, rcv.getLastTransferCrc());
    ASSERT_EQ(TransferReceiver::getDefaultTransferInterval(), rcv.getInterval());           // Not initialized yet
    ASSERT_EQ(100, rcv.getLastTransferTimestampMonotonic().toUSec());

    /*
     * Compound transfer mixed with invalid frames; buffer was not released explicitly
     * Args: iface_index, data, set, transfer_id, ts_monotonic
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",     SET100, 0, 300), bk));    // Previous TID, rejected
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "rty",     SET100, 0, 300), bk));    // Previous TID, wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "\x9a\x78" "34567", SET100, 1, 1000), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwertyu", SET100, 1, 1100), bk));   // Old toggle
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwertyu", SET000, 1, 1100), bk));   // Old toggle
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "abcdefg", SET001, 1, 1200), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "4567891", SET001, 2, 1300), bk));   // Next TID, but not SOT
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",        SET010, 1, 1300), bk));   // Wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "",        SET001, 1, 1300), bk));   // Unexpected toggle
    CHECK_COMPLETE(    rcv.addFrame(gen(0, "",        SET010, 1, 1300), bk));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567abcdefg"));
    ASSERT_EQ(0x789A, rcv.getLastTransferCrc());
    ASSERT_GE(TransferReceiver::getDefaultTransferInterval(), rcv.getInterval());
    ASSERT_LE(TransferReceiver::getMinTransferInterval(), rcv.getInterval());
    ASSERT_EQ(1000, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_FALSE(rcv.isTimedOut(tsMono(1000)));
    ASSERT_FALSE(rcv.isTimedOut(tsMono(5000)));
    ASSERT_TRUE(rcv.isTimedOut(tsMono(60000000)));

    /*
     * Single-frame transfers
     * Args: iface_index, data, set, transfer_id, ts_monotonic
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",      SET110, 1, 2000), bk));   // Previous TID
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwe",      SET110, 2, 2100), bk));   // Wrong iface
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "qwe",      SET110, 2, 2200), bk));

    ASSERT_FALSE(bufmgr.access(gen.bufmgr_key));          // Buffer must be removed
    ASSERT_GT(TransferReceiver::getDefaultTransferInterval(), rcv.getInterval());
    ASSERT_EQ(2200, rcv.getLastTransferTimestampMonotonic().toUSec());

    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         SET110, 3, 2500), bk));
    ASSERT_EQ(2500, rcv.getLastTransferTimestampMonotonic().toUSec());

    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         SET110, 0, 3000), bk));
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         SET110, 1, 3100), bk));
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         SET110, 3, 3200), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         SET110, 0, 3300), bk));   // Old TID, wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         SET110, 2, 3400), bk));   // Old TID, wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "",         SET110, 3, 3500), bk));   // Old TID, wrong iface
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "",         SET110, 4, 3600), bk));
    ASSERT_EQ(3600, rcv.getLastTransferTimestampMonotonic().toUSec());

    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    /*
     * Timeouts
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwe",      SET110, 1, 5000), bk));    // Wrong iface - ignored
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(1, "qwe",      SET110, 6, 1500000), bk)); // Accepted due to iface timeout
    ASSERT_EQ(1500000, rcv.getLastTransferTimestampMonotonic().toUSec());

    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",      SET110, 7, 1500100), bk)); // Ignored - old iface 0
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(1, "qwe",      SET110, 7, 1500100), bk));
    ASSERT_EQ(1500100, rcv.getLastTransferTimestampMonotonic().toUSec());

    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwe",      SET110, 7, 1500100), bk));   // Old TID
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "qwe",      SET110, 7, 100000000), bk)); // Accepted - global timeout
    ASSERT_EQ(100000000, rcv.getLastTransferTimestampMonotonic().toUSec());

    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    CHECK_SINGLE_FRAME(rcv.addFrame(gen(0, "qwe",      SET110, 8, 100000100), bk));
    ASSERT_EQ(100000100, rcv.getLastTransferTimestampMonotonic().toUSec());

    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    ASSERT_TRUE(rcv.isTimedOut(tsMono(900000000)));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "\x78\x56" "34567", SET100, 0, 900000000), bk)); // Global timeout
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "1234567",          SET100, 0, 900000100), bk)); // Wrong iface
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwe",              SET011, 0, 900000200), bk)); // Wrong iface
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "qwe",              SET011, 0, 900000200), bk));

    ASSERT_EQ(900000000, rcv.getLastTransferTimestampMonotonic().toUSec());
    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    ASSERT_FALSE(rcv.isTimedOut(tsMono(1000)));
    ASSERT_FALSE(rcv.isTimedOut(tsMono(900000300)));
    ASSERT_TRUE(rcv.isTimedOut(tsMono(9990000000)));

    ASSERT_LT(TransferReceiver::getDefaultTransferInterval(), rcv.getInterval());
    ASSERT_LE(TransferReceiver::getMinTransferInterval(), rcv.getInterval());
    ASSERT_GE(TransferReceiver::getMaxTransferInterval(), rcv.getInterval());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567qwe"));
    ASSERT_EQ(0x5678, rcv.getLastTransferCrc());

    /*
     * Destruction
     */
    ASSERT_TRUE(bufmgr.access(gen.bufmgr_key));
    context.receiver.~TransferReceiver();         // TransferReceiver does not own the buffer, it must not be released!
    ASSERT_TRUE(bufmgr.access(gen.bufmgr_key));   // Making sure that the buffer is still there
}


TEST(TransferReceiver, OutOfBufferSpace_32bytes)
{
    Context<32> context;
    RxFrameGenerator gen(789);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::TransferBufferManager& bufmgr = context.bufmgr;
    uavcan::TransferBufferAccessor bk(context.bufmgr, RxFrameGenerator::DEFAULT_KEY);

    /*
     * Simple transfer, maximum buffer length
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, 1, 100000000), bk)); // 5
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET001, 1, 100000100), bk)); // 12
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET000, 1, 100000200), bk)); // 19
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET001, 1, 100000300), bk)); // 26
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "123456",  SET010, 1, 100000400), bk)); // 32

    ASSERT_EQ(100000000, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567123456712345671234567123456"));
    ASSERT_EQ(0x3231, rcv.getLastTransferCrc());                   // CRC from "12", which is 0x3231 in little endian

    /*
     * Transfer longer than available buffer space
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, 2, 100001000), bk)); // 5
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET001, 2, 100001100), bk)); // 12
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET000, 2, 100001200), bk)); // 19
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET001, 2, 100001200), bk)); // 26
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET010, 2, 100001300), bk)); // 33 // EOT, ignored - lost sync

    ASSERT_EQ(100000000, rcv.getLastTransferTimestampMonotonic().toUSec());  // Timestamp will not be overriden
    ASSERT_FALSE(bufmgr.access(gen.bufmgr_key));                    // Buffer should be removed

    ASSERT_EQ(1, rcv.yieldErrorCount());
    ASSERT_EQ(0, rcv.yieldErrorCount());
}


TEST(TransferReceiver, OutOfOrderFrames)
{
    Context<32> context;
    RxFrameGenerator gen(789);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::TransferBufferManager& bufmgr = context.bufmgr;
    uavcan::TransferBufferAccessor bk(context.bufmgr, RxFrameGenerator::DEFAULT_KEY);

    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, 7, 100000000), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "-------", SET000, 7, 100000100), bk));  // Out of order
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "-------", SET010, 7, 100000200), bk));  // Out of order
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwertyu", SET001, 7, 100000300), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "-------", SET011, 7, 100000200), bk));  // Out of order
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "abcd",    SET010, 7, 100000400), bk));

    ASSERT_EQ(100000000, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567qwertyuabcd"));
    ASSERT_EQ(0x3231, rcv.getLastTransferCrc());

    ASSERT_EQ(3, rcv.yieldErrorCount());
    ASSERT_EQ(0, rcv.yieldErrorCount());
}


TEST(TransferReceiver, IntervalMeasurement)
{
    Context<32> context;
    RxFrameGenerator gen(789);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::TransferBufferManager& bufmgr = context.bufmgr;
    uavcan::TransferBufferAccessor bk(context.bufmgr, RxFrameGenerator::DEFAULT_KEY);

    static const int INTERVAL = 1000;
    uavcan::TransferID tid;
    uint64_t timestamp = 100000000;

    for (int i = 0; i < 1000; i++)
    {
        CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, tid.get(), timestamp), bk));
        CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwertyu", SET001, tid.get(), timestamp), bk));
        CHECK_COMPLETE(    rcv.addFrame(gen(1, "abcd",    SET010, tid.get(), timestamp), bk));

        ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567qwertyuabcd"));
        ASSERT_EQ(0x3231, rcv.getLastTransferCrc());
        ASSERT_EQ(timestamp, rcv.getLastTransferTimestampMonotonic().toUSec());

        timestamp += uint64_t(INTERVAL);
        tid.increment();
    }

    ASSERT_EQ(INTERVAL, rcv.getInterval().toUSec());
}


TEST(TransferReceiver, Restart)
{
    Context<32> context;
    RxFrameGenerator gen(789);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::TransferBufferManager& bufmgr = context.bufmgr;
    uavcan::TransferBufferAccessor bk(context.bufmgr, RxFrameGenerator::DEFAULT_KEY);

    /*
     * This transfer looks complete, but must be ignored because of large delay after the first frame
     * Args: iface_index, data, set, transfer_id, ts_monotonic [, ts_utc]
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "-------", SET100, 0, 100), bk));       // Begin
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "-------", SET001, 0, 100000100), bk)); // Continue 100 sec later, expired
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "-------", SET010, 0, 100000200), bk)); // Ignored

    /*
     * Begins immediately after, encounters a delay 0.9 sec but completes OK
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, 0, 100000300), bk)); // Begin
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET001, 0, 100900300), bk)); // 0.9 sec later
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "1234567", SET010, 0, 100900400), bk)); // OK nevertheless

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "3456712345671234567"));
    ASSERT_EQ(0x3231, rcv.getLastTransferCrc());

    std::cout << "Interval: " << rcv.getInterval().toString() << std::endl;

    /*
     * Begins OK, gets a timeout, switches to another iface
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "-------", SET100, 1, 103000500), bk)); // Begin
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "-------", SET001, 1, 105000500), bk)); // 2 sec later, timeout
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "-------", SET001, 1, 105000600), bk)); // Same TID, another iface - ignore
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "-------", SET001, 2, 105000700), bk)); // Not first frame - ignore
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "1234567", SET100, 2, 105000800), bk)); // First, another iface - restart
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "-------", SET010, 1, 105000600), bk)); // Old iface - ignore
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "1234567", SET001, 2, 105000900), bk)); // Continuing
    CHECK_COMPLETE(    rcv.addFrame(gen(0, "1234567", SET010, 2, 105000910), bk)); // Done

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "3456712345671234567"));
    ASSERT_EQ(0x3231, rcv.getLastTransferCrc());

    ASSERT_EQ(4, rcv.yieldErrorCount());
    ASSERT_EQ(0, rcv.yieldErrorCount());
}


TEST(TransferReceiver, UtcTransferTimestamping)
{
    Context<32> context;
    RxFrameGenerator gen(789);
    uavcan::TransferReceiver& rcv = context.receiver;
    uavcan::TransferBufferManager& bufmgr = context.bufmgr;
    uavcan::TransferBufferAccessor bk(context.bufmgr, RxFrameGenerator::DEFAULT_KEY);

    /*
     * Zero UTC timestamp must be preserved
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, 0, 1, 0), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwertyu", SET001, 0, 2, 0), bk));
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "abcd",    SET010, 0, 3, 0), bk));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567qwertyuabcd"));
    ASSERT_EQ(1, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_EQ(0, rcv.getLastTransferTimestampUtc().toUSec());

    /*
     * Non-zero UTC timestamp
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "1234567", SET100, 1, 4, 123), bk)); // This UTC is going to be preserved
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(1, "qwertyu", SET001, 1, 5, 0), bk));   // Following are ignored
    CHECK_COMPLETE(    rcv.addFrame(gen(1, "abcd",    SET010, 1, 6, 42), bk));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567qwertyuabcd"));
    ASSERT_EQ(4, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_EQ(123, rcv.getLastTransferTimestampUtc().toUSec());

    /*
     * Single-frame transfers
     * iface_index, data, set, transfer_id, ts_monotonic
     */
    CHECK_SINGLE_FRAME(rcv.addFrame(gen(1, "abc", SET110, 2, 10, 100000000), bk)); // Exact value is irrelevant
    ASSERT_EQ(10, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_EQ(100000000, rcv.getLastTransferTimestampUtc().toUSec());

    /*
     * Restart recovery
     */
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "1234567", SET100, 1, 100000000, 800000000), bk));
    CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "qwertyu", SET001, 1, 100000001, 300000000), bk));
    CHECK_COMPLETE(    rcv.addFrame(gen(0, "abcd",    SET010, 1, 100000002, 900000000), bk));

    ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567qwertyuabcd"));
    ASSERT_EQ(100000000, rcv.getLastTransferTimestampMonotonic().toUSec());
    ASSERT_EQ(800000000, rcv.getLastTransferTimestampUtc().toUSec());
}


TEST(TransferReceiver, HeaderParsing)
{
    static const std::string SFT_PAYLOAD = "1234567";

    uavcan::TransferID tid;

    /*
     * Broadcast
     */
    {
        Context<32> context;
        RxFrameGenerator gen(123);
        uavcan::TransferReceiver& rcv = context.receiver;
        uavcan::TransferBufferManager& bufmgr = context.bufmgr;

        /*
         * MFT, message broadcasting
         */
        {
            gen.bufmgr_key =
                uavcan::TransferBufferManagerKey(gen.bufmgr_key.getNodeID(), uavcan::TransferTypeMessageBroadcast);
            uavcan::TransferBufferAccessor bk1(context.bufmgr, gen.bufmgr_key);

            CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "1234567", SET100, tid.get(), 1), bk1));
            CHECK_COMPLETE(    rcv.addFrame(gen(0, "abcd",     SET011,  tid.get(), 2), bk1));

            ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567abcd"));
            ASSERT_EQ(0x3231, rcv.getLastTransferCrc());

            tid.increment();
            bk1.remove();
        }

        /*
         * SFT, message broadcasting
         */
        {
            gen.bufmgr_key =
                uavcan::TransferBufferManagerKey(gen.bufmgr_key.getNodeID(), uavcan::TransferTypeMessageBroadcast);
            uavcan::TransferBufferAccessor bk(context.bufmgr, gen.bufmgr_key);

            const uavcan::RxFrame frame = gen(0, SFT_PAYLOAD, SET110, tid.get(), 1000);

            CHECK_SINGLE_FRAME(rcv.addFrame(frame, bk));
            ASSERT_EQ(0x0000, rcv.getLastTransferCrc());                                     // Default value - zero

            // All bytes are payload, zero overhead
            ASSERT_TRUE(std::equal(SFT_PAYLOAD.begin(), SFT_PAYLOAD.end(), frame.getPayloadPtr()));

            tid.increment();
        }
    }

    /*
     * Unicast
     */
    {
        Context<32> context;
        RxFrameGenerator gen(123);
        uavcan::TransferReceiver& rcv = context.receiver;
        uavcan::TransferBufferManager& bufmgr = context.bufmgr;

        static const uavcan::TransferType ADDRESSED_TRANSFER_TYPES[2] =
        {
            uavcan::TransferTypeServiceRequest,
            uavcan::TransferTypeServiceResponse
        };

        /*
         * MFT, service request/response
         */
        for (unsigned i = 0; i < (sizeof(ADDRESSED_TRANSFER_TYPES) / sizeof(ADDRESSED_TRANSFER_TYPES[0])); i++)
        {
            gen.bufmgr_key =
                uavcan::TransferBufferManagerKey(gen.bufmgr_key.getNodeID(), ADDRESSED_TRANSFER_TYPES[i]);
            uavcan::TransferBufferAccessor bk2(context.bufmgr, gen.bufmgr_key);

            const uint64_t ts_monotonic = i + 10;

            CHECK_NOT_COMPLETE(rcv.addFrame(gen(0, "1234567", SET100, tid.get(), ts_monotonic), bk2));
            CHECK_COMPLETE(    rcv.addFrame(gen(0, "abcd",    SET011,  tid.get(), ts_monotonic), bk2));

            ASSERT_TRUE(matchBufferContent(bufmgr.access(gen.bufmgr_key), "34567abcd"));
            ASSERT_EQ(0x3231, rcv.getLastTransferCrc());

            tid.increment();
            bk2.remove();
        }

        /*
         * SFT, message unicast, service request/response
         */
        for (unsigned i = 0; i < int(sizeof(ADDRESSED_TRANSFER_TYPES) / sizeof(ADDRESSED_TRANSFER_TYPES[0])); i++)
        {
            gen.bufmgr_key =
                uavcan::TransferBufferManagerKey(gen.bufmgr_key.getNodeID(), ADDRESSED_TRANSFER_TYPES[i]);
            uavcan::TransferBufferAccessor bk(context.bufmgr, gen.bufmgr_key);

            const uavcan::RxFrame frame = gen(0, SFT_PAYLOAD, SET110, tid.get(), i + 10000U);

            CHECK_SINGLE_FRAME(rcv.addFrame(frame, bk));
            ASSERT_EQ(0x0000, rcv.getLastTransferCrc());                                     // Default value - zero

            // First byte must be ignored
            ASSERT_TRUE(std::equal(SFT_PAYLOAD.begin(), SFT_PAYLOAD.end(), frame.getPayloadPtr()));

            tid.increment();
        }
    }


}
