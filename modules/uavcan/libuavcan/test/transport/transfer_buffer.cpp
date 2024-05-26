/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
# pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <algorithm>
#include <gtest/gtest.h>
#include <memory>
#include <uavcan/transport/transfer_buffer.hpp>

static const std::string TEST_DATA =
    "It was like this: I asked myself one day this question - what if Napoleon, for instance, had happened to be in my "
    "place, and if he had not had Toulon nor Egypt nor the passage of Mont Blanc to begin his career with, but "
    "instead of all those picturesque and monumental things, there had simply been some ridiculous old hag, a "
    "pawnbroker, who had to be murdered too to get money from her trunk (for his career, you understand). "
    "Well, would he have brought himself to that if there had been no other means?";

template <typename T, unsigned Size>
static bool allEqual(const T (&a)[Size])
{
    unsigned n = Size;
    while ((--n > 0) && (a[n] == a[0])) { }
    return n == 0;
}

template <typename T, unsigned Size, typename R>
static void fill(T (&a)[Size], R value)
{
    for (unsigned i = 0; i < Size; i++)
    {
        a[i] = T(value);
    }
}

static bool matchAgainst(const std::string& data, const uavcan::ITransferBuffer& tbb,
                         unsigned offset = 0, int len = -1)
{
    uint8_t local_buffer[1024];
    fill(local_buffer, 0);
    assert((len < 0) || (sizeof(local_buffer) >= static_cast<unsigned>(len)));

    if (len < 0)
    {
        const int res = tbb.read(offset, local_buffer, sizeof(local_buffer));
        if (res < 0)
        {
            std::cout << "matchAgainst(): res " << res << std::endl;
            return false;
        }
        len = res;
    }
    else
    {
        const int res = tbb.read(offset, local_buffer, unsigned(len));
        if (res != len)
        {
            std::cout << "matchAgainst(): res " << res << " expected " << len << std::endl;
            return false;
        }
    }
    const bool equals = std::equal(local_buffer, local_buffer + len, data.begin() + offset);
    if (!equals)
    {
        std::cout << "local_buffer:\n\t" << local_buffer << std::endl;
        std::cout << "test_data:\n\t" << std::string(data.begin() + offset, data.begin() + offset + len) << std::endl;
    }
    return equals;
}

static bool matchAgainstTestData(const uavcan::ITransferBuffer& tbb, unsigned offset, int len = -1)
{
    return matchAgainst(TEST_DATA, tbb, offset, len);
}

TEST(TransferBuffer, TestDataValidation)
{
    ASSERT_LE(4, TEST_DATA.length() / uavcan::MemPoolBlockSize);
    uint8_t local_buffer[50];
    std::copy(TEST_DATA.begin(), TEST_DATA.begin() + sizeof(local_buffer), local_buffer);
    ASSERT_FALSE(allEqual(local_buffer));
}

static const int TEST_BUFFER_SIZE = 200;

TEST(StaticTransferBuffer, Basic)
{
    using uavcan::StaticTransferBuffer;
    StaticTransferBuffer<TEST_BUFFER_SIZE> buf;

    uint8_t local_buffer[TEST_BUFFER_SIZE * 2];
    const uint8_t* const test_data_ptr = reinterpret_cast<const uint8_t*>(TEST_DATA.c_str());

    // Empty reads
    fill(local_buffer, 0xA5);
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(999, local_buffer, 0));
    ASSERT_TRUE(allEqual(local_buffer));

    // Bulk write
    ASSERT_EQ(TEST_BUFFER_SIZE, buf.write(0, test_data_ptr, unsigned(TEST_DATA.length())));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2, TEST_BUFFER_SIZE / 4));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 4, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, 0, TEST_BUFFER_SIZE / 4));

    // Reset
    fill(local_buffer, 0xA5);
    buf.reset();
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_TRUE(allEqual(local_buffer));

    // Random write
    ASSERT_EQ(21, buf.write(12, test_data_ptr + 12, 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 12, 21));

    ASSERT_EQ(12, buf.write(0, test_data_ptr, 12));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));

    ASSERT_EQ(0, buf.write(21, test_data_ptr + 21, 0));
    ASSERT_EQ(TEST_BUFFER_SIZE - 21, buf.write(21, test_data_ptr + 21, 999));
    ASSERT_TRUE(matchAgainstTestData(buf, 21, TEST_BUFFER_SIZE - 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));
}


TEST(TransferBufferManagerEntry, Basic)
{
    using uavcan::TransferBufferManagerEntry;

    static const int MAX_SIZE = TEST_BUFFER_SIZE;
    static const int POOL_BLOCKS = 8;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;

    TransferBufferManagerEntry buf(pool, MAX_SIZE);

    uint8_t local_buffer[TEST_BUFFER_SIZE * 2];
    const uint8_t* const test_data_ptr = reinterpret_cast<const uint8_t*>(TEST_DATA.c_str());

    // Empty reads
    fill(local_buffer, 0xA5);
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(999, local_buffer, 0));
    ASSERT_TRUE(allEqual(local_buffer));

    // Bulk write
    ASSERT_EQ(MAX_SIZE, buf.write(0, test_data_ptr, unsigned(TEST_DATA.length())));

    ASSERT_LT(0, pool.getNumUsedBlocks());      // Making sure some memory was used

    ASSERT_TRUE(matchAgainstTestData(buf, 0));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 2, TEST_BUFFER_SIZE / 4));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE / 4, TEST_BUFFER_SIZE / 2));
    ASSERT_TRUE(matchAgainstTestData(buf, 0, TEST_BUFFER_SIZE / 4));

    // Reset
    fill(local_buffer, 0xA5);
    buf.reset();
    ASSERT_EQ(0, buf.read(0, local_buffer, 0));
    ASSERT_EQ(0, buf.read(0, local_buffer, 999));
    ASSERT_TRUE(allEqual(local_buffer));
    ASSERT_EQ(0, pool.getNumUsedBlocks());

    // Random write
    ASSERT_EQ(21, buf.write(12, test_data_ptr + 12, 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 12, 21));

    ASSERT_EQ(60, buf.write(TEST_BUFFER_SIZE - 60, test_data_ptr + TEST_BUFFER_SIZE - 60, 60));
    ASSERT_TRUE(matchAgainstTestData(buf, TEST_BUFFER_SIZE - 60));

    // Now we have two empty regions: empty-data-empty-data

    ASSERT_EQ(0, buf.write(0, test_data_ptr, 0));
    ASSERT_EQ(TEST_BUFFER_SIZE - 21, buf.write(21, test_data_ptr + 21, TEST_BUFFER_SIZE - 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 21, TEST_BUFFER_SIZE - 21));

    // Now: empty-data-data-data

    ASSERT_EQ(21, buf.write(0, test_data_ptr, 21));
    ASSERT_TRUE(matchAgainstTestData(buf, 0));

    // Destroying the object; memory should be released
    ASSERT_LT(0, pool.getNumUsedBlocks());
    buf.~TransferBufferManagerEntry();
    ASSERT_EQ(0, pool.getNumUsedBlocks());
}


static const std::string MGR_TEST_DATA[4] =
{
    "I thought you would cry out again \'don\'t speak of it, leave off.\'\" Raskolnikov gave a laugh, but rather a "
    "forced one. \"What, silence again?\" he asked a minute later. \"We must talk about something, you know. ",

    "It would be interesting for me to know how you would decide a certain \'problem\' as Lebeziatnikov would say.\" "
    "(He was beginning to lose the thread.) \"No, really, I am serious. Imagine, Sonia, that you had known all ",

    "Luzhin\'s intentions beforehand. Known, that is, for a fact, that they would be the ruin of Katerina Ivanovna "
    "and the children and yourself thrown in--since you don\'t count yourself for anything--Polenka too... for ",

    "she\'ll go the same way. Well, if suddenly it all depended on your decision whether he or they should go on "
    "living, that is whether Luzhin should go on living and doing wicked things, or Katerina Ivanovna should die? "
    "How would you decide which of them was to die? I ask you?"
};

static const int MGR_MAX_BUFFER_SIZE = 100;

TEST(TransferBufferManager, TestDataValidation)
{
    for (unsigned i = 0; i < sizeof(MGR_TEST_DATA) / sizeof(MGR_TEST_DATA[0]); i++)
    {
        ASSERT_LT(MGR_MAX_BUFFER_SIZE, MGR_TEST_DATA[i].length());
    }
}


static int fillTestData(const std::string& data, uavcan::ITransferBuffer* tbb)
{
    return tbb->write(0, reinterpret_cast<const uint8_t*>(data.c_str()), unsigned(data.length()));
}

TEST(TransferBufferManager, Basic)
{
    using uavcan::TransferBufferManager;
    using uavcan::TransferBufferManagerKey;
    using uavcan::ITransferBuffer;

    static const int POOL_BLOCKS = 100;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;

    std::unique_ptr<TransferBufferManager> mgr(new TransferBufferManager(MGR_MAX_BUFFER_SIZE, pool));

    // Empty
    ASSERT_FALSE(mgr->access(TransferBufferManagerKey(0, uavcan::TransferTypeMessageBroadcast)));
    ASSERT_FALSE(mgr->access(TransferBufferManagerKey(127, uavcan::TransferTypeServiceRequest)));

    ITransferBuffer* tbb = UAVCAN_NULLPTR;

    const TransferBufferManagerKey keys[5] =
    {
        TransferBufferManagerKey(0, uavcan::TransferTypeServiceRequest),
        TransferBufferManagerKey(1, uavcan::TransferTypeMessageBroadcast),
        TransferBufferManagerKey(2, uavcan::TransferTypeServiceRequest),
        TransferBufferManagerKey(127, uavcan::TransferTypeServiceResponse),
        TransferBufferManagerKey(64, uavcan::TransferTypeMessageBroadcast)
    };

    ASSERT_TRUE((tbb = mgr->create(keys[0])));
    ASSERT_EQ(MGR_MAX_BUFFER_SIZE, fillTestData(MGR_TEST_DATA[0], tbb));
    ASSERT_EQ(1, mgr->getNumBuffers());

    ASSERT_TRUE((tbb = mgr->create(keys[1])));
    ASSERT_EQ(MGR_MAX_BUFFER_SIZE, fillTestData(MGR_TEST_DATA[1], tbb));
    ASSERT_EQ(2, mgr->getNumBuffers());
    ASSERT_LT(2, pool.getNumUsedBlocks());

    ASSERT_TRUE((tbb = mgr->create(keys[2])));
    ASSERT_EQ(MGR_MAX_BUFFER_SIZE, fillTestData(MGR_TEST_DATA[2], tbb));
    ASSERT_EQ(3, mgr->getNumBuffers());

    std::cout << "TransferBufferManager - Basic: Pool usage: " << pool.getNumUsedBlocks() << std::endl;

    ASSERT_TRUE((tbb = mgr->create(keys[3])));

    ASSERT_LT(0, fillTestData(MGR_TEST_DATA[3], tbb));
    ASSERT_EQ(4, mgr->getNumBuffers());

    // Making sure all buffers contain proper data
    ASSERT_TRUE((tbb = mgr->access(keys[0])));
    ASSERT_TRUE(matchAgainst(MGR_TEST_DATA[0], *tbb));

    ASSERT_TRUE((tbb = mgr->access(keys[1])));
    ASSERT_TRUE(matchAgainst(MGR_TEST_DATA[1], *tbb));

    ASSERT_TRUE((tbb = mgr->access(keys[2])));
    ASSERT_TRUE(matchAgainst(MGR_TEST_DATA[2], *tbb));

    ASSERT_TRUE((tbb = mgr->access(keys[3])));
    ASSERT_TRUE(matchAgainst(MGR_TEST_DATA[3], *tbb));

    mgr->remove(keys[1]);
    ASSERT_FALSE(mgr->access(keys[1]));
    ASSERT_EQ(3, mgr->getNumBuffers());
    ASSERT_LT(0, pool.getNumFreeBlocks());

    mgr->remove(keys[0]);
    ASSERT_FALSE(mgr->access(keys[0]));
    ASSERT_EQ(2, mgr->getNumBuffers());

    // At this time we have the following NodeID: 2, 127
    ASSERT_TRUE((tbb = mgr->access(keys[2])));
    ASSERT_TRUE(matchAgainst(MGR_TEST_DATA[2], *tbb));

    ASSERT_TRUE((tbb = mgr->access(keys[3])));
    ASSERT_TRUE(matchAgainst(MGR_TEST_DATA[3], *tbb));

    // These were deleted: 0, 1; 3 is still there
    ASSERT_FALSE(mgr->access(keys[1]));
    ASSERT_FALSE(mgr->access(keys[0]));
    ASSERT_TRUE(mgr->access(keys[3]));

    // Filling the memory again in order to check the destruction below
    ASSERT_TRUE((tbb = mgr->create(keys[1])));
    ASSERT_LT(0, fillTestData(MGR_TEST_DATA[1], tbb));

    // Deleting the object; all memory must be freed
    ASSERT_NE(0, pool.getNumUsedBlocks());
    mgr.reset();
    ASSERT_EQ(0, pool.getNumUsedBlocks());
}
