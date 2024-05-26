/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"
#include "../clock.hpp"


TEST(TransferTestHelpers, Transfer)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;

    uavcan::TransferBufferManager mgr(128, pool);
    uavcan::TransferBufferAccessor tba(mgr, uavcan::TransferBufferManagerKey(0, uavcan::TransferTypeMessageBroadcast));

    uavcan::RxFrame frame(uavcan::Frame(123, uavcan::TransferTypeMessageBroadcast, 1, 0, 0),
                          uavcan::MonotonicTime(), uavcan::UtcTime(), 0);
    frame.setEndOfTransfer(true);
    uavcan::MultiFrameIncomingTransfer mfit(tsMono(10), tsUtc(1000), frame, tba);

    // Filling the buffer with data
    static const std::string TEST_DATA = "Kaneda! What do you see? Kaneda! What do you see? Kaneda! Kaneda!!!";
    ASSERT_TRUE(tba.create());
    ASSERT_EQ(TEST_DATA.length(), tba.access()->write(0, reinterpret_cast<const uint8_t*>(TEST_DATA.c_str()),
                                                      unsigned(TEST_DATA.length())));

    // Reading back
    const Transfer transfer(mfit, uavcan::DataTypeDescriptor());
    ASSERT_EQ(TEST_DATA, transfer.payload);
}


TEST(TransferTestHelpers, MFTSerialization)
{
    uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "Foo");

    static const std::string DATA = "To go wrong in one's own way is better than to go right in someone else's.";
    const Transfer transfer(1, 100000, 10,
                            uavcan::TransferTypeServiceRequest, 2, 42, 127, DATA, type);

    const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer);

    std::cout << "Serialized transfer:\n";
    for (std::vector<uavcan::RxFrame>::const_iterator it = ser.begin(); it != ser.end(); ++it)
    {
        std::cout << "\t" << it->toString() << "\n";
    }

    for (std::vector<uavcan::RxFrame>::const_iterator it = ser.begin(); it != ser.end(); ++it)
    {
        std::cout << "\t'";
        for (unsigned i = 0; i < it->getPayloadLen(); i++)
        {
            uint8_t ch = it->getPayloadPtr()[i];
            if (ch < 0x20 || ch > 0x7E)
            {
                ch = '.';
            }
            std::cout << static_cast<char>(ch);
        }
        std::cout << "'\n";
    }
    std::cout << std::flush;
}


TEST(TransferTestHelpers, SFTSerialization)
{
    uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "Foo");

    {
        const Transfer transfer(1, 100000, 10,
                                uavcan::TransferTypeMessageBroadcast, 7, 42, 0, "Nvrfrgt", type);
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, 11,
                                uavcan::TransferTypeServiceRequest, 7, 42, 127, "7-chars", type);
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, 12,
                                uavcan::TransferTypeMessageBroadcast, 7, 42, 0, "", type);
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, 13,
                                uavcan::TransferTypeServiceResponse, 7, 42, 127, "", type);
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
}
