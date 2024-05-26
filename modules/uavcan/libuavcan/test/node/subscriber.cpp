/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <root_ns_a/EmptyMessage.hpp>
#include <root_ns_a/MavlinkMessage.hpp>
#include "../clock.hpp"
#include "../transport/can/can.hpp"
#include "test_node.hpp"


template <typename DataType>
struct SubscriptionListener
{
    typedef uavcan::ReceivedDataStructure<DataType> ReceivedDataStructure;

    struct ReceivedDataStructureCopy
    {
        uavcan::MonotonicTime ts_monotonic;
        uavcan::UtcTime ts_utc;
        uavcan::TransferType transfer_type;
        uavcan::TransferID transfer_id;
        uavcan::NodeID src_node_id;
        uavcan::uint8_t iface_index;
        DataType msg;

        ReceivedDataStructureCopy(const ReceivedDataStructure& s)
            : ts_monotonic(s.getMonotonicTimestamp())
            , ts_utc(s.getUtcTimestamp())
            , transfer_type(s.getTransferType())
            , transfer_id(s.getTransferID())
            , src_node_id(s.getSrcNodeID())
            , iface_index(s.getIfaceIndex())
            , msg(s)
        { }
    };

    std::vector<DataType> simple;
    std::vector<ReceivedDataStructureCopy> extended;

    void receiveExtended(ReceivedDataStructure& msg)
    {
        extended.push_back(msg);
    }

    void receiveSimple(DataType& msg)
    {
        simple.push_back(msg);
    }

    typedef SubscriptionListener<DataType> SelfType;
    typedef uavcan::MethodBinder<SelfType*, void (SelfType::*)(ReceivedDataStructure&)> ExtendedBinder;
    typedef uavcan::MethodBinder<SelfType*, void (SelfType::*)(DataType&)> SimpleBinder;

    ExtendedBinder bindExtended() { return ExtendedBinder(this, &SelfType::receiveExtended); }
    SimpleBinder bindSimple() { return SimpleBinder(this, &SelfType::receiveSimple); }
};


TEST(Subscriber, Basic)
{
    // Manual type registration - we can't rely on the GDTR state
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::MavlinkMessage> _registrator;

    SystemClockDriver clock_driver;
    CanDriverMock can_driver(2, clock_driver);
    TestNode node(can_driver, clock_driver, 1);

    typedef SubscriptionListener<root_ns_a::MavlinkMessage> Listener;

    uavcan::Subscriber<root_ns_a::MavlinkMessage, Listener::ExtendedBinder> sub_extended(node);
    uavcan::Subscriber<root_ns_a::MavlinkMessage, Listener::ExtendedBinder> sub_extended2(node); // Not used
    uavcan::Subscriber<root_ns_a::MavlinkMessage, Listener::SimpleBinder> sub_simple(node);
    uavcan::Subscriber<root_ns_a::MavlinkMessage, Listener::SimpleBinder> sub_simple2(node);     // Not used

    std::cout <<
        "sizeof(uavcan::Subscriber<root_ns_a::MavlinkMessage, Listener::ExtendedBinder>): " <<
        sizeof(uavcan::Subscriber<root_ns_a::MavlinkMessage, Listener::ExtendedBinder>) << std::endl;

    // Null binder - will fail
    ASSERT_EQ(-uavcan::ErrInvalidParam, sub_extended.start(Listener::ExtendedBinder(UAVCAN_NULLPTR, UAVCAN_NULLPTR)));

    Listener listener;

    /*
     * Message layout:
     * uint8 seq
     * uint8 sysid
     * uint8 compid
     * uint8 msgid
     * uint8[<256] payload
     */
    root_ns_a::MavlinkMessage expected_msg;
    expected_msg.seq = 0x42;
    expected_msg.sysid = 0x72;
    expected_msg.compid = 0x08;
    expected_msg.msgid = 0xa5;
    expected_msg.payload = "Msg";

    const uint8_t transfer_payload[] = {0x42, 0x72, 0x08, 0xa5, 'M', 's', 'g'};

    /*
     * RxFrame generation
     */
    std::vector<uavcan::RxFrame> rx_frames;
    for (uint8_t i = 0; i < 4; i++)
    {
        uavcan::TransferType tt = uavcan::TransferTypeMessageBroadcast;
        uavcan::NodeID dni = (tt == uavcan::TransferTypeMessageBroadcast) ?
                             uavcan::NodeID::Broadcast : node.getDispatcher().getNodeID();
        // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
        // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
        uavcan::Frame frame(root_ns_a::MavlinkMessage::DefaultDataTypeID, tt, uavcan::NodeID(uint8_t(i + 100)),
                            dni, i);
        frame.setStartOfTransfer(true);
        frame.setEndOfTransfer(true);
        frame.setPayload(transfer_payload, 7);
        uavcan::RxFrame rx_frame(frame, clock_driver.getMonotonic(), clock_driver.getUtc(), 0);
        rx_frames.push_back(rx_frame);
    }

    /*
     * Reception
     */
    ASSERT_EQ(0, node.getDispatcher().getNumMessageListeners());

    ASSERT_EQ(0, sub_extended.start(listener.bindExtended()));
    ASSERT_EQ(0, sub_extended2.start(listener.bindExtended()));
    ASSERT_EQ(0, sub_simple.start(listener.bindSimple()));
    ASSERT_EQ(0, sub_simple2.start(listener.bindSimple()));

    ASSERT_EQ(4, node.getDispatcher().getNumMessageListeners());

    sub_extended2.stop();  // These are not used - making sure they aren't receiving anything
    sub_simple2.stop();

    ASSERT_EQ(2, node.getDispatcher().getNumMessageListeners());

    for (unsigned i = 0; i < rx_frames.size(); i++)
    {
        can_driver.ifaces[0].pushRx(rx_frames[i]);
        can_driver.ifaces[1].pushRx(rx_frames[i]);
    }

    ASSERT_LE(0, node.spin(clock_driver.getMonotonic() + durMono(10000)));

    /*
     * Validation
     */
    ASSERT_EQ(listener.extended.size(), rx_frames.size());
    for (unsigned i = 0; i < rx_frames.size(); i++)
    {
        const Listener::ReceivedDataStructureCopy s = listener.extended.at(i);
        ASSERT_TRUE(s.msg == expected_msg);
        ASSERT_EQ(rx_frames[i].getSrcNodeID(), s.src_node_id);
        ASSERT_EQ(rx_frames[i].getTransferID(), s.transfer_id);
        ASSERT_EQ(rx_frames[i].getTransferType(), s.transfer_type);
        ASSERT_EQ(rx_frames[i].getMonotonicTimestamp(), s.ts_monotonic);
        ASSERT_EQ(rx_frames[i].getIfaceIndex(), s.iface_index);
    }

    ASSERT_EQ(listener.simple.size(), rx_frames.size());
    for (unsigned i = 0; i < rx_frames.size(); i++)
    {
        ASSERT_TRUE(listener.simple.at(i) == expected_msg);
    }

    ASSERT_EQ(0, sub_extended.getFailureCount());
    ASSERT_EQ(0, sub_simple.getFailureCount());

    /*
     * Unregistration
     */
    ASSERT_EQ(2, node.getDispatcher().getNumMessageListeners());

    sub_extended.stop();
    sub_extended2.stop();
    sub_simple.stop();
    sub_simple2.stop();

    ASSERT_EQ(0, node.getDispatcher().getNumMessageListeners());
}


static void panickingSink(const uavcan::ReceivedDataStructure<root_ns_a::MavlinkMessage>&)
{
    FAIL() << "I just went mad";
}


TEST(Subscriber, FailureCount)
{
    // Manual type registration - we can't rely on the GDTR state
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::MavlinkMessage> _registrator;

    SystemClockDriver clock_driver;
    CanDriverMock can_driver(2, clock_driver);
    TestNode node(can_driver, clock_driver, 1);

    {
        uavcan::Subscriber<root_ns_a::MavlinkMessage> sub(node);
        ASSERT_EQ(0, node.getDispatcher().getNumMessageListeners());
        sub.start(panickingSink);
        ASSERT_EQ(1, node.getDispatcher().getNumMessageListeners());

        ASSERT_EQ(0, sub.getFailureCount());

        for (uint8_t i = 0; i < 4; i++)
        {
            // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
            // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
            uavcan::Frame frame(root_ns_a::MavlinkMessage::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                                uavcan::NodeID(uint8_t(i + 100)), uavcan::NodeID::Broadcast, i);
            frame.setStartOfTransfer(true);
            frame.setEndOfTransfer(true);
            // No payload - broken transfer
            uavcan::RxFrame rx_frame(frame, clock_driver.getMonotonic(), clock_driver.getUtc(), 0);
            can_driver.ifaces[0].pushRx(rx_frame);
            can_driver.ifaces[1].pushRx(rx_frame);
        }

        ASSERT_LE(0, node.spin(clock_driver.getMonotonic() + durMono(10000)));

        ASSERT_EQ(4, sub.getFailureCount());

        ASSERT_EQ(1, node.getDispatcher().getNumMessageListeners()); // Still there
    }
    ASSERT_EQ(0, node.getDispatcher().getNumMessageListeners());     // Removed
}


TEST(Subscriber, SingleFrameTransfer)
{
    // Manual type registration - we can't rely on the GDTR state
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::EmptyMessage> _registrator;

    SystemClockDriver clock_driver;
    CanDriverMock can_driver(2, clock_driver);
    TestNode node(can_driver, clock_driver, 1);

    typedef SubscriptionListener<root_ns_a::EmptyMessage> Listener;

    uavcan::Subscriber<root_ns_a::EmptyMessage, Listener::SimpleBinder> sub(node);

    std::cout <<
        "sizeof(uavcan::Subscriber<root_ns_a::EmptyMessage, Listener::SimpleBinder>): " <<
        sizeof(uavcan::Subscriber<root_ns_a::EmptyMessage, Listener::SimpleBinder>) << std::endl;

    Listener listener;

    sub.start(listener.bindSimple());

    for (uint8_t i = 0; i < 4; i++)
    {
        // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
        // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
        uavcan::Frame frame(root_ns_a::EmptyMessage::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                            uavcan::NodeID(uint8_t(i + 100)), uavcan::NodeID::Broadcast, i);
        frame.setStartOfTransfer(true);
        frame.setEndOfTransfer(true);
        // No payload - message is empty
        uavcan::RxFrame rx_frame(frame, clock_driver.getMonotonic(), clock_driver.getUtc(), 0);
        can_driver.ifaces[0].pushRx(rx_frame);
        can_driver.ifaces[1].pushRx(rx_frame);
    }

    ASSERT_LE(0, node.spin(clock_driver.getMonotonic() + durMono(10000)));

    ASSERT_EQ(0, sub.getFailureCount());

    ASSERT_EQ(4, listener.simple.size());
    for (unsigned i = 0; i < 4; i++)
    {
        ASSERT_TRUE(listener.simple.at(i) == root_ns_a::EmptyMessage());
    }
}
