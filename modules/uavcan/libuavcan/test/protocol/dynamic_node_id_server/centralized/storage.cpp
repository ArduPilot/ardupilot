/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/centralized/storage.hpp>
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"


TEST(dynamic_node_id_server_centralized_Storage, Initialization)
{
    using namespace uavcan::dynamic_node_id_server::centralized;
    using namespace uavcan::dynamic_node_id_server;

    // No data in the storage - initializing empty
    {
        MemoryStorageBackend storage;
        Storage stor(storage);

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_LE(0, stor.init());

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_EQ(0, stor.getSize());

        ASSERT_FALSE(stor.isNodeIDOccupied(1));
        ASSERT_FALSE(stor.isNodeIDOccupied(0));
    }
    // Nonempty storage, many items
    {
        MemoryStorageBackend storage;
        Storage stor(storage);

        storage.set("occupation_mask", "0e000000000000000000000000000000"); // node ID 1, 2, 3
        ASSERT_LE(0, stor.init());     // OK

        ASSERT_EQ(1, storage.getNumKeys());
        ASSERT_EQ(3, stor.getSize());

        ASSERT_TRUE(stor.isNodeIDOccupied(1));
        ASSERT_TRUE(stor.isNodeIDOccupied(2));
        ASSERT_TRUE(stor.isNodeIDOccupied(3));
        ASSERT_FALSE(stor.isNodeIDOccupied(0));
        ASSERT_FALSE(stor.isNodeIDOccupied(4));

        UniqueID uid_1;
        uid_1[0] = 1;

        UniqueID uid_2;
        uid_2[0] = 2;

        UniqueID uid_3;
        uid_3[0] = 3;

        ASSERT_FALSE(stor.getNodeIDForUniqueID(uid_1).isValid());
        ASSERT_FALSE(stor.getNodeIDForUniqueID(uid_2).isValid());
        ASSERT_FALSE(stor.getNodeIDForUniqueID(uid_3).isValid());
        ASSERT_FALSE(stor.isNodeIDOccupied(127));

        storage.set("01000000000000000000000000000000", "1");
        storage.set("02000000000000000000000000000000", "2");
        storage.set("03000000000000000000000000000000", "3");

        ASSERT_EQ(1, stor.getNodeIDForUniqueID(uid_1).get());
        ASSERT_EQ(2, stor.getNodeIDForUniqueID(uid_2).get());
        ASSERT_EQ(3, stor.getNodeIDForUniqueID(uid_3).get());
        ASSERT_FALSE(stor.isNodeIDOccupied(127));
    }
}


TEST(dynamic_node_id_server_centralized_Storage, Basic)
{
    using namespace uavcan::dynamic_node_id_server::centralized;

    MemoryStorageBackend storage;
    Storage stor(storage);

    ASSERT_EQ(0, storage.getNumKeys());
    ASSERT_LE(0, stor.init());
    storage.print();
    ASSERT_EQ(0, storage.getNumKeys());

    /*
     * Adding one entry to the log, making sure it appears in the storage
     */
    uavcan::dynamic_node_id_server::UniqueID unique_id;
    unique_id[0] = 1;
    ASSERT_LE(0, stor.add(1, unique_id));

    ASSERT_EQ("02000000000000000000000000000000", storage.get("occupation_mask"));
    ASSERT_EQ("1",                                storage.get("01000000000000000000000000000000"));

    ASSERT_EQ(2, storage.getNumKeys());
    ASSERT_EQ(1, stor.getSize());

    /*
     * Adding another entry while storage is failing
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(2, storage.getNumKeys());

    unique_id[0] = 2;
    ASSERT_GT(0, stor.add(2, unique_id));

    ASSERT_EQ(2, storage.getNumKeys());  // No new entries, we failed

    ASSERT_EQ(1, stor.getSize());

    /*
     * Adding a lot of entries
     */
    storage.failOnSetCalls(false);

    uavcan::NodeID node_id(2);
    while (stor.getSize() < 127)
    {
        ASSERT_LE(0, stor.add(node_id, unique_id));

        ASSERT_TRUE(stor.getNodeIDForUniqueID(unique_id).isValid());
        ASSERT_TRUE(stor.isNodeIDOccupied(node_id));

        node_id = uint8_t(uavcan::min(node_id.get() + 1U, 127U));
        unique_id[0] = uint8_t(unique_id[0] + 1U);
    }

    storage.print();
}
