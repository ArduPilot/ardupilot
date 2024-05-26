/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <memory>
#include <vector>
#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"
#include "can/can.hpp"
#include <uavcan/transport/dispatcher.hpp>


class DispatcherTransferEmulator : public IncomingTransferEmulatorBase
{
    CanDriverMock& target_;

public:
    DispatcherTransferEmulator(CanDriverMock& target, uavcan::NodeID dst_node_id = 127)
        : IncomingTransferEmulatorBase(dst_node_id)
        , target_(target)
    { }

    void sendOneFrame(const uavcan::RxFrame& frame)
    {
        CanIfaceMock* const iface = static_cast<CanIfaceMock*>(target_.getIface(frame.getIfaceIndex()));
        EXPECT_TRUE(iface);
        if (iface)
        {
            iface->pushRx(frame);
        }
    }
};


struct RxFrameListener : public uavcan::IRxFrameListener
{
    std::vector<uavcan::CanRxFrame> rx_frames;

    virtual void handleRxFrame(const uavcan::CanRxFrame& frame, uavcan::CanIOFlags flags)
    {
        std::cout << "RX frame [flags=" << flags << "]: " << frame.toString() << std::endl;
        if ((flags & uavcan::CanIOFlagLoopback) == 0)
        {
            rx_frames.push_back(frame);
        }
    }
};


static const uavcan::NodeID SELF_NODE_ID(64);


TEST(Dispatcher, Reception)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 100, uavcan::MemPoolBlockSize> pool;

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::Dispatcher dispatcher(driver, pool, clockmock);
    ASSERT_TRUE(dispatcher.setNodeID(SELF_NODE_ID));  // Can be set only once
    ASSERT_FALSE(dispatcher.setNodeID(SELF_NODE_ID));
    ASSERT_EQ(SELF_NODE_ID, dispatcher.getNodeID());

    DispatcherTransferEmulator emulator(driver, SELF_NODE_ID);

    /*
     * RX listener
     */
    RxFrameListener rx_listener;
    ASSERT_FALSE(dispatcher.getRxFrameListener());
    dispatcher.installRxFrameListener(&rx_listener);
    ASSERT_TRUE(dispatcher.getRxFrameListener());
    ASSERT_TRUE(rx_listener.rx_frames.empty());

    /*
     * Test environment
     */
    static const uavcan::DataTypeDescriptor TYPES[4] =
    {
        makeDataType(uavcan::DataTypeKindMessage, 1),
        makeDataType(uavcan::DataTypeKindMessage, 2),
        makeDataType(uavcan::DataTypeKindService, 1),
        makeDataType(uavcan::DataTypeKindService, 1)
    };

    typedef std::unique_ptr<TestListener> TestListenerPtr;
    static const int MaxBufSize = 512;
    static const int NumSubscribers = 6;
    TestListenerPtr subscribers[NumSubscribers] =
    {
        TestListenerPtr(new TestListener(dispatcher.getTransferPerfCounter(), TYPES[0], MaxBufSize, pool)), // msg
        TestListenerPtr(new TestListener(dispatcher.getTransferPerfCounter(), TYPES[0], MaxBufSize, pool)), // msg // Two similar
        TestListenerPtr(new TestListener(dispatcher.getTransferPerfCounter(), TYPES[1], MaxBufSize, pool)), // msg
        TestListenerPtr(new TestListener(dispatcher.getTransferPerfCounter(), TYPES[2], MaxBufSize, pool)), // srv
        TestListenerPtr(new TestListener(dispatcher.getTransferPerfCounter(), TYPES[3], MaxBufSize, pool)), // srv
        TestListenerPtr(new TestListener(dispatcher.getTransferPerfCounter(), TYPES[3], MaxBufSize, pool))  // srv // Repeat again
    };

    static const std::string DATA[6] =
    {
        "Yes, man is mortal, but that would be only half the trouble. "
        "The worst of it is that he's sometimes unexpectedly mortal - there's the trick!",

        "In fact, I'm beginning to fear that this confusion will go on for a long time. "
        "And all because he writes down what I said incorrectly.",

        "I had the pleasure of meeting that young man at the Patriarch's Ponds. "
        "He almost drove me mad myself, proving to me that I don't exist. But you do believe that it is really I?",

        "He was a dreamer, a thinker, a speculative philosopher... or, as his wife would have it, an idiot.",

        "The only way to get ideas for stories is to drink way too much coffee and buy a desk that doesn't "
        "collapse when you beat your head against it",

        ""
    };

    for (unsigned i = 0; i < sizeof(DATA) / sizeof(DATA[0]); i++)
    {
        std::cout << "Size of test data chunk " << i << ": " << DATA[i].length() << std::endl;
    }

    const Transfer transfers[8] =
    {
        emulator.makeTransfer(0,  uavcan::TransferTypeMessageBroadcast, 10, DATA[0], TYPES[0]),
        emulator.makeTransfer(5,  uavcan::TransferTypeMessageBroadcast, 11, DATA[1], TYPES[1]),
        emulator.makeTransfer(10, uavcan::TransferTypeServiceRequest,   12, DATA[2], TYPES[2]),
        emulator.makeTransfer(15, uavcan::TransferTypeServiceResponse,  13, DATA[4], TYPES[3]),
        emulator.makeTransfer(20, uavcan::TransferTypeMessageBroadcast, 14, DATA[3], TYPES[0]),
        emulator.makeTransfer(25, uavcan::TransferTypeMessageBroadcast, 15, DATA[5], TYPES[1]),
        // Wrongly addressed:
        emulator.makeTransfer(29, uavcan::TransferTypeServiceResponse,  10, DATA[0], TYPES[3], 100),
        emulator.makeTransfer(31, uavcan::TransferTypeServiceRequest,   11, DATA[4], TYPES[2], 101)
    };

    /*
     * Registration
     */
    for (int i = 0; i < NumSubscribers; i++)
    {
        ASSERT_FALSE(dispatcher.hasSubscriber(subscribers[i]->getDataTypeDescriptor().getID()));
        ASSERT_FALSE(dispatcher.hasPublisher(subscribers[i]->getDataTypeDescriptor().getID()));
        ASSERT_FALSE(dispatcher.hasServer(subscribers[i]->getDataTypeDescriptor().getID()));
    }

    ASSERT_TRUE(dispatcher.registerMessageListener(subscribers[0].get()));
    ASSERT_TRUE(dispatcher.registerMessageListener(subscribers[1].get()));
    ASSERT_TRUE(dispatcher.registerMessageListener(subscribers[2].get()));
    ASSERT_TRUE(dispatcher.registerServiceRequestListener(subscribers[3].get()));
    ASSERT_TRUE(dispatcher.registerServiceResponseListener(subscribers[4].get()));
    ASSERT_TRUE(dispatcher.registerServiceResponseListener(subscribers[5].get()));

    for (int i = 0; i < NumSubscribers; i++)
    {
        ASSERT_FALSE(dispatcher.hasPublisher(subscribers[i]->getDataTypeDescriptor().getID()));
    }

    // Subscribers
    ASSERT_TRUE(dispatcher.hasSubscriber(subscribers[0]->getDataTypeDescriptor().getID()));
    ASSERT_TRUE(dispatcher.hasSubscriber(subscribers[1]->getDataTypeDescriptor().getID()));
    ASSERT_TRUE(dispatcher.hasSubscriber(subscribers[2]->getDataTypeDescriptor().getID()));

    // Servers
    ASSERT_TRUE(dispatcher.hasServer(subscribers[3]->getDataTypeDescriptor().getID()));

    /*
     * Sending the transfers
     */
    // Multiple service request listeners are not allowed
    ASSERT_FALSE(dispatcher.registerServiceRequestListener(subscribers[3].get()));

    // Item count validation
    ASSERT_EQ(3, dispatcher.getNumMessageListeners());
    ASSERT_EQ(1, dispatcher.getNumServiceRequestListeners());
    ASSERT_EQ(2, dispatcher.getNumServiceResponseListeners());

    for (int i = 0; i < NumSubscribers; i++)
    {
        ASSERT_TRUE(subscribers[i]->isEmpty());
    }

    emulator.send(transfers);
    emulator.send(transfers);  // Just for fun, they will be ignored anyway.

    while (true)
    {
        const int res = dispatcher.spinOnce();
        ASSERT_LE(0, res);
        clockmock.advance(100);
        if (res == 0)
        {
            break;
        }
    }

    /*
     * Matching.
     */
    ASSERT_TRUE(subscribers[0]->matchAndPop(transfers[4]));
    ASSERT_TRUE(subscribers[0]->matchAndPop(transfers[0]));

    ASSERT_TRUE(subscribers[1]->matchAndPop(transfers[4]));
    ASSERT_TRUE(subscribers[1]->matchAndPop(transfers[0]));

    ASSERT_TRUE(subscribers[2]->matchAndPop(transfers[5]));
    ASSERT_TRUE(subscribers[2]->matchAndPop(transfers[1]));

    ASSERT_TRUE(subscribers[3]->matchAndPop(transfers[2]));

    ASSERT_TRUE(subscribers[4]->matchAndPop(transfers[3]));

    ASSERT_TRUE(subscribers[5]->matchAndPop(transfers[3]));

    for (int i = 0; i < NumSubscribers; i++)
    {
        ASSERT_TRUE(subscribers[i]->isEmpty());
    }

    /*
     * Unregistering all transfers
     */
    dispatcher.unregisterMessageListener(subscribers[0].get());
    dispatcher.unregisterMessageListener(subscribers[1].get());
    dispatcher.unregisterMessageListener(subscribers[2].get());
    dispatcher.unregisterServiceRequestListener(subscribers[3].get());
    dispatcher.unregisterServiceResponseListener(subscribers[4].get());
    dispatcher.unregisterServiceResponseListener(subscribers[5].get());

    ASSERT_EQ(0, dispatcher.getNumMessageListeners());
    ASSERT_EQ(0, dispatcher.getNumServiceRequestListeners());
    ASSERT_EQ(0, dispatcher.getNumServiceResponseListeners());

    /*
     * Perf counters
     */
    EXPECT_LT(0, dispatcher.getTransferPerfCounter().getErrorCount());   // Repeated transfers
    EXPECT_EQ(0, dispatcher.getTransferPerfCounter().getTxTransferCount());
    EXPECT_EQ(9, dispatcher.getTransferPerfCounter().getRxTransferCount());

    /*
     * RX listener
     */
    std::cout << "Num received frames: " << rx_listener.rx_frames.size() << std::endl;
    ASSERT_EQ(292, rx_listener.rx_frames.size());
}


TEST(Dispatcher, Transmission)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::Dispatcher dispatcher(driver, pool, clockmock);
    ASSERT_TRUE(dispatcher.setNodeID(SELF_NODE_ID));  // Can be set only once
    ASSERT_FALSE(dispatcher.setNodeID(SELF_NODE_ID));

    /*
     * RX listener
     */
    RxFrameListener rx_listener;
    dispatcher.installRxFrameListener(&rx_listener);

    /*
     * Transmission
     */
    static const uavcan::MonotonicTime TX_DEADLINE = tsMono(123456);

    // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
    // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame = false
    uavcan::Frame frame(123, uavcan::TransferTypeServiceRequest, SELF_NODE_ID, 2, 0);
    frame.setPayload(reinterpret_cast<const uint8_t*>("123"), 3);

    ASSERT_FALSE(dispatcher.hasPublisher(123));
    ASSERT_FALSE(dispatcher.hasPublisher(456));
    const uavcan::OutgoingTransferRegistryKey otr_key(123, uavcan::TransferTypeMessageBroadcast, 0);
    ASSERT_TRUE(dispatcher.getOutgoingTransferRegistry().accessOrCreate(otr_key,
                                                                        uavcan::MonotonicTime::fromMSec(1000000)));
    ASSERT_TRUE(dispatcher.hasPublisher(123));
    ASSERT_FALSE(dispatcher.hasPublisher(456));

    ASSERT_EQ(2, dispatcher.send(frame, TX_DEADLINE, tsMono(0), uavcan::CanTxQueue::Volatile, 0, 0xFF));

    /*
     * Validation
     */
    uavcan::CanFrame expected_can_frame;
    ASSERT_TRUE(frame.compile(expected_can_frame));

    ASSERT_TRUE(driver.ifaces.at(0).matchAndPopTx(expected_can_frame, TX_DEADLINE));
    ASSERT_TRUE(driver.ifaces.at(1).matchAndPopTx(expected_can_frame, TX_DEADLINE));

    ASSERT_TRUE(driver.ifaces.at(0).tx.empty());
    ASSERT_TRUE(driver.ifaces.at(1).tx.empty());

    /*
     * Perf counters - all empty because dispatcher itself does not count TX transfers
     */
    EXPECT_EQ(0, dispatcher.getTransferPerfCounter().getErrorCount());
    EXPECT_EQ(0, dispatcher.getTransferPerfCounter().getTxTransferCount());
    EXPECT_EQ(0, dispatcher.getTransferPerfCounter().getRxTransferCount());

    /*
     * RX listener
     */
    ASSERT_TRUE(rx_listener.rx_frames.empty());
}


TEST(Dispatcher, Spin)
{
    NullAllocator poolmgr;

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::Dispatcher dispatcher(driver, poolmgr, clockmock);
    ASSERT_TRUE(dispatcher.setNodeID(SELF_NODE_ID));  // Can be set only once
    ASSERT_FALSE(dispatcher.setNodeID(SELF_NODE_ID));

    clockmock.monotonic_auto_advance = 100;

    ASSERT_EQ(100, clockmock.monotonic);
    ASSERT_EQ(0, dispatcher.spin(tsMono(1000)));
    ASSERT_LE(1000, clockmock.monotonic);
    ASSERT_EQ(0, dispatcher.spinOnce());
    ASSERT_LE(1000, clockmock.monotonic);
    ASSERT_EQ(0, dispatcher.spin(tsMono(1100)));
    ASSERT_LE(1100, clockmock.monotonic);
}


struct DispatcherTestLoopbackFrameListener : public uavcan::LoopbackFrameListenerBase
{
    uavcan::RxFrame last_frame;
    unsigned count;

    DispatcherTestLoopbackFrameListener(uavcan::Dispatcher& dispatcher)
        : uavcan::LoopbackFrameListenerBase(dispatcher)
        , count(0)
    { }

    using uavcan::LoopbackFrameListenerBase::startListening;
    using uavcan::LoopbackFrameListenerBase::isListening;

    void handleLoopbackFrame(const uavcan::RxFrame& frame)
    {
        std::cout << "DispatcherTestLoopbackFrameListener: " << frame.toString() << std::endl;
        last_frame = frame;
        count++;
    }
};

TEST(Dispatcher, Loopback)
{
    NullAllocator poolmgr;

    SystemClockMock clockmock(100);
    CanDriverMock driver(2, clockmock);

    uavcan::Dispatcher dispatcher(driver, poolmgr, clockmock);
    ASSERT_TRUE(dispatcher.setNodeID(SELF_NODE_ID));

    {
        DispatcherTestLoopbackFrameListener listener(dispatcher);
        ASSERT_FALSE(listener.isListening());
        listener.startListening();
        ASSERT_TRUE(listener.isListening());

        // uint_fast16_t data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
        // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame = false
        uavcan::Frame frame(123, uavcan::TransferTypeServiceResponse, SELF_NODE_ID, 2, 0);
        frame.setPayload(reinterpret_cast<const uint8_t*>("123"), 3);

        ASSERT_TRUE(listener.last_frame == uavcan::RxFrame());

        ASSERT_LE(0, dispatcher.send(frame, tsMono(1000), tsMono(0), uavcan::CanTxQueue::Persistent,
                                     uavcan::CanIOFlagLoopback, 0xFF));

        ASSERT_EQ(0, dispatcher.spin(tsMono(1000)));

        ASSERT_TRUE(listener.last_frame != uavcan::RxFrame());
        ASSERT_TRUE(listener.last_frame == frame);
        ASSERT_EQ(1, listener.last_frame.getIfaceIndex());  // Last iface
        ASSERT_EQ(2, listener.count);

        ASSERT_EQ(1, dispatcher.getLoopbackFrameListenerRegistry().getNumListeners());
    }
    ASSERT_EQ(0, dispatcher.getLoopbackFrameListenerRegistry().getNumListeners());
}
