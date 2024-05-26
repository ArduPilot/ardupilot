/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include "memory_storage_backend.hpp"

TEST(dynamic_node_id_server_StorageMarshaller, Basic)
{
    MemoryStorageBackend st;

    uavcan::dynamic_node_id_server::StorageMarshaller marshaler(st);

    uavcan::dynamic_node_id_server::IStorageBackend::String key;

    /*
     * uint32
     */
    uint32_t u32 = 0;

    key = "foo";
    u32 = 0;
    ASSERT_LE(0, marshaler.setAndGetBack(key, u32));
    ASSERT_EQ(0, u32);

    key = "bar";
    u32 = 0xFFFFFFFF;
    ASSERT_LE(0, marshaler.setAndGetBack(key, u32));
    ASSERT_EQ(0xFFFFFFFF, u32);
    ASSERT_LE(0, marshaler.get(key, u32));
    ASSERT_EQ(0xFFFFFFFF, u32);

    key = "foo";
    ASSERT_LE(0, marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    key = "the_cake_is_a_lie";
    ASSERT_GT(0, marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    /*
     * uint8[16]
     */
    uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id array;

    key = "a";
    // Set zero
    ASSERT_LE(0, marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(0, array[i]);
    }

    // Make sure this will not be interpreted as uint32
    ASSERT_GT(0, marshaler.get(key, u32));
    ASSERT_EQ(0, u32);

    // Set pattern
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = uint8_t(i + 1);
    }
    ASSERT_LE(0, marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(i + 1, array[i]);
    }

    // Set another pattern
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = uint8_t(i | (i << 4));
    }
    ASSERT_LE(0, marshaler.setAndGetBack(key, array));
    for (uint8_t i = 0; i < 16; i++)
    {
        ASSERT_EQ(uint8_t(i | (i << 4)), array[i]);
    }

    // Make sure uint32 cannot be interpreted as an array
    key = "foo";
    ASSERT_GT(0, marshaler.get(key, array));

    // Nonexistent key
    key = "the_cake_is_a_lie";
    ASSERT_GT(0, marshaler.get(key, array));
}
