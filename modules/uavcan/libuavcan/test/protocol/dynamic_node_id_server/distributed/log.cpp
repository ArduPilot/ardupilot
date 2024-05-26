/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/distributed/log.hpp>
#include "../event_tracer.hpp"
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"


static const unsigned NumEntriesInStorageWithEmptyLog = 4;  // last index + 3 items per log entry


TEST(dynamic_node_id_server_Log, Initialization)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    EventTracer tracer;
    // No log data in the storage - initializing empty log
    {
        MemoryStorageBackend storage;
        uavcan::dynamic_node_id_server::distributed::Log log(storage, tracer);

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_LE(0, log.init());
        ASSERT_EQ(NumEntriesInStorageWithEmptyLog, storage.getNumKeys());
        ASSERT_EQ(0, log.getLastIndex());
        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
        ASSERT_EQ(0, log.getEntryAtIndex(0)->node_id);
        ASSERT_EQ(uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id(),
                  log.getEntryAtIndex(0)->unique_id);
    }
    // Nonempty storage, one item
    {
        MemoryStorageBackend storage;
        Log log(storage, tracer);

        storage.set("log_last_index", "0");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Expected one entry, none found
        ASSERT_EQ(1, storage.getNumKeys());

        storage.set("log0_term",      "0");
        storage.set("log0_unique_id", "00000000000000000000000000000000");
        storage.set("log0_node_id",   "0");
        ASSERT_LE(0, log.init());                       // OK now
        ASSERT_EQ(NumEntriesInStorageWithEmptyLog, storage.getNumKeys());
        ASSERT_EQ(0, log.getLastIndex());
        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
    }
    // Nonempty storage, broken data
    {
        MemoryStorageBackend storage;
        Log log(storage, tracer);

        storage.set("log_last_index", "foobar");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Bad value

        storage.set("log_last_index", "128");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Bad value

        storage.set("log_last_index", "0");
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // No log items
        ASSERT_EQ(1, storage.getNumKeys());

        storage.set("log0_term",      "0");
        storage.set("log0_unique_id", "00000000000000000000000000000000");
        storage.set("log0_node_id",   "128");           // Bad value (127 max)
        ASSERT_LE(-uavcan::ErrFailure, log.init());     // Failed
        ASSERT_EQ(0, log.getLastIndex());
        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
        ASSERT_EQ(4, storage.getNumKeys());
    }
    // Nonempty storage, many items
    {
        MemoryStorageBackend storage;
        Log log(storage, tracer);

        storage.set("log_last_index", "1");  // 2 items - 0, 1
        storage.set("log0_term",      "0");
        storage.set("log0_unique_id", "00000000000000000000000000000000");
        storage.set("log0_node_id",   "0");
        storage.set("log1_term",      "1");
        storage.set("log1_unique_id", "0123456789abcdef0123456789abcdef");
        storage.set("log1_node_id",   "127");

        ASSERT_LE(0, log.init());                       // OK now
        ASSERT_EQ(7, storage.getNumKeys());
        ASSERT_EQ(1, log.getLastIndex());

        ASSERT_EQ(0, log.getEntryAtIndex(0)->term);
        ASSERT_EQ(0, log.getEntryAtIndex(0)->node_id);
        ASSERT_EQ(uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id(),
                  log.getEntryAtIndex(0)->unique_id);

        ASSERT_EQ(1, log.getEntryAtIndex(1)->term);
        ASSERT_EQ(127, log.getEntryAtIndex(1)->node_id);
        uavcan::protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id uid;
        uid[0] = 0x01;
        uid[1] = 0x23;
        uid[2] = 0x45;
        uid[3] = 0x67;
        uid[4] = 0x89;
        uid[5] = 0xab;
        uid[6] = 0xcd;
        uid[7] = 0xef;
        uavcan::copy(uid.begin(), uid.begin() + 8, uid.begin() + 8);
        ASSERT_EQ(uid, log.getEntryAtIndex(1)->unique_id);
    }
}


TEST(dynamic_node_id_server_Log, Append)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    EventTracer tracer;
    MemoryStorageBackend storage;
    Log log(storage, tracer);

    ASSERT_EQ(0, storage.getNumKeys());
    ASSERT_LE(0, log.init());
    storage.print();
    ASSERT_EQ(NumEntriesInStorageWithEmptyLog, storage.getNumKeys());

    /*
     * Entry at the index 0 always exists, and it's always zero-initialized.
     */
    ASSERT_EQ("0",                                storage.get("log_last_index"));
    ASSERT_EQ("0",                                storage.get("log0_term"));
    ASSERT_EQ("00000000000000000000000000000000", storage.get("log0_unique_id"));
    ASSERT_EQ("0",                                storage.get("log0_node_id"));

    /*
     * Adding one entry to the log, making sure it appears in the storage
     */
    uavcan::protocol::dynamic_node_id::server::Entry entry;
    entry.term = 1;
    entry.node_id = 1;
    entry.unique_id[0] = 1;
    ASSERT_LE(0, log.append(entry));

    ASSERT_EQ("1",                                storage.get("log_last_index"));
    ASSERT_EQ("1",                                storage.get("log1_term"));
    ASSERT_EQ("01000000000000000000000000000000", storage.get("log1_unique_id"));
    ASSERT_EQ("1",                                storage.get("log1_node_id"));

    ASSERT_EQ(1, log.getLastIndex());
    ASSERT_TRUE(entry == *log.getEntryAtIndex(1));

    /*
     * Adding another entry while storage is failing
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(7, storage.getNumKeys());

    entry.term = 2;
    entry.node_id = 2;
    entry.unique_id[0] = 2;
    ASSERT_GT(0, log.append(entry));

    ASSERT_EQ(7, storage.getNumKeys());  // No new entries, we failed

    ASSERT_EQ(1, log.getLastIndex());

    /*
     * Making sure append() fails when the log is full
     */
    storage.failOnSetCalls(false);

    while (log.getLastIndex() < (log.Capacity - 1))
    {
        ASSERT_LE(0, log.append(entry));
        ASSERT_TRUE(entry == *log.getEntryAtIndex(log.getLastIndex()));

        entry.term += 1;
        entry.node_id = uint8_t(entry.node_id + 1U);
        entry.unique_id[0] = uint8_t(entry.unique_id[0] + 1U);
    }

    ASSERT_GT(0, log.append(entry));  // Failing because full

    storage.print();
}


TEST(dynamic_node_id_server_Log, Remove)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    EventTracer tracer;
    MemoryStorageBackend storage;
    Log log(storage, tracer);

    /*
     * Filling the log fully
     */
    uavcan::protocol::dynamic_node_id::server::Entry entry;
    entry.term = 1;
    entry.node_id = 1;
    entry.unique_id[0] = 1;

    while (log.getLastIndex() < (log.Capacity - 1))
    {
        ASSERT_LE(0, log.append(entry));
        ASSERT_TRUE(entry == *log.getEntryAtIndex(log.getLastIndex()));

        entry.term += 1;
        entry.node_id = uint8_t(entry.node_id + 1U);
        entry.unique_id[0] = uint8_t(entry.unique_id[0] + 1U);
    }

    /*
     * Removal will fail as the storage is failing to update
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(log.Capacity - 1, log.getLastIndex());
    ASSERT_GT(0, log.removeEntriesWhereIndexGreaterOrEqual(60));  // Failing
    ASSERT_EQ(log.Capacity - 1, log.getLastIndex());

    /*
     * Now removal must work
     */
    storage.failOnSetCalls(false);

    ASSERT_EQ(log.Capacity - 1, log.getLastIndex());

    ASSERT_LE(0, log.removeEntriesWhereIndexGreaterOrEqual(60));
    ASSERT_EQ(59, log.getLastIndex());
    ASSERT_EQ("59", storage.get("log_last_index"));

    ASSERT_LE(0, log.removeEntriesWhereIndexGreater(30));
    ASSERT_EQ(30, log.getLastIndex());
    ASSERT_EQ("30", storage.get("log_last_index"));

    ASSERT_LE(0, log.removeEntriesWhereIndexGreaterOrEqual(1));
    ASSERT_EQ(0, log.getLastIndex());
    ASSERT_EQ("0", storage.get("log_last_index"));

    storage.print();
}
