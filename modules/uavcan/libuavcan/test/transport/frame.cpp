/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <string>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/crc.hpp>
#include "../clock.hpp"
#include "can/can.hpp"


TEST(Frame, MessageParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    /*
     * Priority
     * Message Type ID
     * Service Not Message
     * Source Node ID
     */
    const uint32_t can_id =
        (16 << 24) |    // Priority
        (20000 << 8) |  // Message Type ID
        (0 << 7) |      // Service Not Message
        (42 << 0);      // Source Node ID

    const std::string payload_string = "hello\xD4"; // SET = 110, TID = 20

    /*
     * Parse
     */
    // Invalid CAN frames
    ASSERT_FALSE(frame.parse(CanFrame(can_id | CanFrame::FlagRTR, reinterpret_cast<const uint8_t*>(""), 0)));
    ASSERT_FALSE(frame.parse(makeCanFrame(can_id, payload_string, STD)));

    // Valid
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(20), frame.getTransferID());
    EXPECT_TRUE(frame.isStartOfTransfer());
    EXPECT_TRUE(frame.isEndOfTransfer());
    EXPECT_FALSE(frame.getToggle());
    EXPECT_EQ(uavcan::NodeID(42), frame.getSrcNodeID());
    EXPECT_TRUE(frame.getDstNodeID().isBroadcast());
    EXPECT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    EXPECT_EQ(20000, frame.getDataTypeID().get());
    EXPECT_EQ(16, frame.getPriority().get());

    EXPECT_EQ(payload_string.length() - 1, frame.getPayloadLen());
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           payload_string.begin()));

    std::cout << frame.toString() << std::endl;

    /*
     * Compile
     */
    CanFrame can_frame;
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    ASSERT_TRUE(frame.compile(can_frame));
    ASSERT_EQ(can_frame, makeCanFrame(can_id, payload_string, EXT));

    EXPECT_EQ(payload_string.length(), can_frame.dlc);
    std::cout << can_frame.toString() << std::endl;
    /*
     * FUN FACT: comparison of uint8_t with char may fail on the character 0xD4 (depending on the locale),
     * because it will be considered a Unicode character. Hence, we do reinterpret_cast<>.
     */
    EXPECT_TRUE(std::equal(can_frame.data, can_frame.data + can_frame.dlc,
                           reinterpret_cast<const uint8_t*>(&payload_string[0])));

    /*
     * Comparison
     */
    ASSERT_FALSE(Frame() == frame);
    ASSERT_TRUE(Frame() != frame);
    frame = Frame();
    ASSERT_TRUE(Frame() == frame);
    ASSERT_FALSE(Frame() != frame);
}


TEST(Frame, ServiceParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    /*
     * Priority
     * Service Type ID
     * Request Not Response
     * Destination Node ID
     * Service Not Message
     * Source Node ID
     */
    const uint32_t can_id =
        (31 << 24) |    // Priority
        (200 << 16) |   // Service Type ID
        (1 << 15) |     // Request Not Response
        (0x42 << 8) |   // Destination Node ID
        (1 << 7) |      // Service Not Message
        (42 << 0);      // Source Node ID

    const std::string payload_string = "hello\x6a"; // SET = 011, TID = 10

    /*
     * Parse
     */
    // Invalid CAN frames
    ASSERT_FALSE(frame.parse(CanFrame(can_id | CanFrame::FlagRTR, reinterpret_cast<const uint8_t*>(""), 0)));
    ASSERT_FALSE(frame.parse(makeCanFrame(can_id, payload_string, STD)));

    // Valid
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(10), frame.getTransferID());
    EXPECT_FALSE(frame.isStartOfTransfer());
    EXPECT_TRUE(frame.isEndOfTransfer());
    EXPECT_TRUE(frame.getToggle());
    EXPECT_EQ(uavcan::NodeID(42), frame.getSrcNodeID());
    EXPECT_EQ(uavcan::NodeID(0x42), frame.getDstNodeID());
    EXPECT_EQ(uavcan::TransferTypeServiceRequest, frame.getTransferType());
    EXPECT_EQ(200, frame.getDataTypeID().get());
    EXPECT_EQ(31, frame.getPriority().get());

    EXPECT_EQ(payload_string.length(), frame.getPayloadLen() + 1);
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           reinterpret_cast<const uint8_t*>(&payload_string[0])));

    std::cout << frame.toString() << std::endl;

    /*
     * Compile
     */
    CanFrame can_frame;
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    ASSERT_TRUE(frame.compile(can_frame));
    ASSERT_EQ(can_frame, makeCanFrame(can_id, payload_string, EXT));

    EXPECT_EQ(payload_string.length(), can_frame.dlc);
    EXPECT_TRUE(std::equal(can_frame.data, can_frame.data + can_frame.dlc,
                           reinterpret_cast<const uint8_t*>(&payload_string[0])));

    /*
     * Comparison
     */
    ASSERT_FALSE(Frame() == frame);
    ASSERT_TRUE(Frame() != frame);
    frame = Frame();
    ASSERT_TRUE(Frame() == frame);
    ASSERT_FALSE(Frame() != frame);
}


TEST(Frame, AnonymousParseCompile)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::TransferID;
    using uavcan::TransferType;

    Frame frame;

    /*
     * Priority
     * Discriminator
     * Message Type ID
     * Service Not Message
     * Source Node ID
     */
    const uint32_t can_id =
        (16383 << 10) | // Discriminator
        (1 << 8);       // Message Type ID

    const std::string payload_string = "hello\xd4"; // SET = 110, TID = 20

    uavcan::TransferCRC payload_crc;
    payload_crc.add(reinterpret_cast<const uint8_t*>(payload_string.c_str()), unsigned(payload_string.length()));

    /*
     * Parse
     */
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    EXPECT_EQ(TransferID(20), frame.getTransferID());
    EXPECT_TRUE(frame.isStartOfTransfer());
    EXPECT_TRUE(frame.isEndOfTransfer());
    EXPECT_FALSE(frame.getToggle());
    EXPECT_TRUE(frame.getSrcNodeID().isBroadcast());
    EXPECT_TRUE(frame.getDstNodeID().isBroadcast());
    EXPECT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());
    EXPECT_EQ(1, frame.getDataTypeID().get());
    EXPECT_EQ(0, frame.getPriority().get());

    EXPECT_EQ(payload_string.length() - 1, frame.getPayloadLen());
    EXPECT_TRUE(std::equal(frame.getPayloadPtr(), frame.getPayloadPtr() + frame.getPayloadLen(),
                           reinterpret_cast<const uint8_t*>(&payload_string[0])));

    std::cout << frame.toString() << std::endl;

    /*
     * Compile
     */
    const uint32_t DiscriminatorMask = 0x00FFFC00;
    const uint32_t NoDiscriminatorMask = 0xFF0003FF;

    CanFrame can_frame;
    ASSERT_TRUE(frame.parse(makeCanFrame(can_id, payload_string, EXT)));

    ASSERT_TRUE(frame.compile(can_frame));
    ASSERT_EQ(can_id & NoDiscriminatorMask & uavcan::CanFrame::MaskExtID,
              can_frame.id & NoDiscriminatorMask & uavcan::CanFrame::MaskExtID);

    EXPECT_EQ(payload_string.length(), can_frame.dlc);
    EXPECT_TRUE(std::equal(can_frame.data, can_frame.data + can_frame.dlc,
                           reinterpret_cast<const uint8_t*>(&payload_string[0])));

    EXPECT_EQ((can_frame.id & DiscriminatorMask & uavcan::CanFrame::MaskExtID) >> 10, payload_crc.get() & 16383);

    /*
     * Comparison
     */
    ASSERT_FALSE(Frame() == frame);
    ASSERT_TRUE(Frame() != frame);
    frame = Frame();
    ASSERT_TRUE(Frame() == frame);
    ASSERT_FALSE(Frame() != frame);
}


TEST(Frame, FrameParsing)
{
    using uavcan::Frame;
    using uavcan::CanFrame;
    using uavcan::NodeID;
    using uavcan::TransferID;

    CanFrame can;
    Frame frame;
    ASSERT_FALSE(frame.parse(can));

    for (unsigned i = 0; i < sizeof(CanFrame::data); i++)
    {
        can.data[i] = uint8_t(i | (i << 4));
    }

    /*
     * Message CAN ID fields and offsets:
     *   24 Priority
     *   8  Message Type ID
     *   7  Service Not Message (0)
     *   0  Source Node ID
     *
     * Service CAN ID fields and offsets:
     *   24 Priority
     *   16 Service Type ID
     *   15 Request Not Response
     *   8  Destination Node ID
     *   7  Service Not Message (1)
     *   0  Source Node ID
     */

    /*
     * SFT message broadcast
     */
    can.id = CanFrame::FlagEFF |
             (2 << 24) |
             (456 << 8) |
             (0 << 7) |
             (42 << 0);

    can.data[7] = 0xcf; // SET=110, TID=0

    ASSERT_FALSE(frame.parse(can));
    can.dlc = 8;

    ASSERT_TRUE(frame.parse(can));
    EXPECT_TRUE(frame.isStartOfTransfer());
    EXPECT_TRUE(frame.isEndOfTransfer());
    EXPECT_FALSE(frame.getToggle());
    ASSERT_EQ(2, frame.getPriority().get());
    ASSERT_EQ(NodeID(42), frame.getSrcNodeID());
    ASSERT_EQ(NodeID::Broadcast, frame.getDstNodeID());
    ASSERT_EQ(456, frame.getDataTypeID().get());
    ASSERT_EQ(TransferID(15), frame.getTransferID());
    ASSERT_EQ(uavcan::TransferTypeMessageBroadcast, frame.getTransferType());

    // TODO: test service frames
    // TODO: test malformed frames
}


TEST(Frame, RxFrameParse)
{
    using uavcan::Frame;
    using uavcan::RxFrame;
    using uavcan::CanFrame;
    using uavcan::CanRxFrame;

    CanRxFrame can_rx_frame;
    RxFrame rx_frame;

    // Failure
    ASSERT_FALSE(rx_frame.parse(can_rx_frame));

    // Valid
    can_rx_frame.ts_mono = uavcan::MonotonicTime::fromUSec(1);  // Zero is not allowed
    can_rx_frame.id = CanFrame::FlagEFF |
        (2 << 24) |
        (456 << 8) |
        (0 << 7) |
        (42 << 0);

    ASSERT_FALSE(rx_frame.parse(can_rx_frame));

    can_rx_frame.data[0] = 0xc0; // SOT, EOT
    can_rx_frame.dlc = 1;

    ASSERT_TRUE(rx_frame.parse(can_rx_frame));
    ASSERT_EQ(1, rx_frame.getMonotonicTimestamp().toUSec());
    ASSERT_EQ(0, rx_frame.getIfaceIndex());

    can_rx_frame.ts_mono = tsMono(123);
    can_rx_frame.iface_index = 2;

    Frame frame(456, uavcan::TransferTypeMessageBroadcast, 1, uavcan::NodeID::Broadcast, 0);
    ASSERT_TRUE(frame.compile(can_rx_frame));

    ASSERT_TRUE(rx_frame.parse(can_rx_frame));
    ASSERT_EQ(123, rx_frame.getMonotonicTimestamp().toUSec());
    ASSERT_EQ(2, rx_frame.getIfaceIndex());
    ASSERT_EQ(456, rx_frame.getDataTypeID().get());
    ASSERT_EQ(uavcan::TransferTypeMessageBroadcast, rx_frame.getTransferType());
}


TEST(Frame, FrameToString)
{
    using uavcan::Frame;
    using uavcan::RxFrame;

    // RX frame default
    RxFrame rx_frame;
    EXPECT_EQ("prio=255 dtid=65535 tt=3 snid=255 dnid=255 sot=0 eot=0 togl=0 tid=0 payload=[] ts_m=0.000000 ts_utc=0.000000 iface=0",
              rx_frame.toString());

    // RX frame max len
    rx_frame = RxFrame(Frame(uavcan::DataTypeID::MaxPossibleDataTypeIDValue, uavcan::TransferTypeMessageBroadcast,
                             uavcan::NodeID::Max, 0, uavcan::TransferID::Max),
                       uavcan::MonotonicTime::getMax(), uavcan::UtcTime::getMax(), 3);

    uint8_t data[8];
    for (unsigned i = 0; i < sizeof(data); i++)
    {
        data[i] = uint8_t(i);
    }
    rx_frame.setPayload(data, sizeof(data));

    rx_frame.setStartOfTransfer(true);
    rx_frame.setEndOfTransfer(true);
    rx_frame.flipToggle();
    rx_frame.setPriority(uavcan::TransferPriority::NumericallyMax);

    EXPECT_EQ("prio=31 dtid=65535 tt=2 snid=127 dnid=0 sot=1 eot=1 togl=1 tid=31 payload=[00 01 02 03 04 05 06] "
              "ts_m=18446744073709.551615 ts_utc=18446744073709.551615 iface=3",
              rx_frame.toString());

    // Plain frame default
    Frame frame;
    EXPECT_EQ("prio=255 dtid=65535 tt=3 snid=255 dnid=255 sot=0 eot=0 togl=0 tid=0 payload=[]", frame.toString());

    // Plain frame max len
    frame = rx_frame;
    EXPECT_EQ("prio=31 dtid=65535 tt=2 snid=127 dnid=0 sot=1 eot=1 togl=1 tid=31 payload=[00 01 02 03 04 05 06]",
              frame.toString());
}
