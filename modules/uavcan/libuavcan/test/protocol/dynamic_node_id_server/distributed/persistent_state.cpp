/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/distributed/persistent_state.hpp>
#include "../event_tracer.hpp"
#include "../memory_storage_backend.hpp"


TEST(dynamic_node_id_server_PersistentState, Initialization)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    EventTracer tracer;
    /*
     * First initialization
     */
    {
        MemoryStorageBackend storage;
        PersistentState pers(storage, tracer);

        ASSERT_EQ(0, storage.getNumKeys());
        ASSERT_LE(0, pers.init());

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("0", storage.get("log_last_index"));
        ASSERT_EQ("0", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
    /*
     * Partial recovery - only empty log is recovered
     */
    {
        MemoryStorageBackend storage;

        {
            // This log is used to initialize the storage
            Log log(storage, tracer);
            ASSERT_LE(0, log.init());
        }
        ASSERT_LE(1, storage.getNumKeys());

        PersistentState pers(storage, tracer);

        ASSERT_LE(0, pers.init());

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("0", storage.get("log_last_index"));
        ASSERT_EQ("0", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
    /*
     * Partial recovery - log and current term are recovered
     */
    {
        MemoryStorageBackend storage;

        {
            // This log is used to initialize the storage
            Log log(storage, tracer);
            ASSERT_LE(0, log.init());
        }
        ASSERT_LE(1, storage.getNumKeys());

        storage.set("current_term", "1");

        PersistentState pers(storage, tracer);

        ASSERT_GT(0, pers.init());              // Fails because current term is not zero

        storage.set("current_term", "0");

        ASSERT_LE(0, pers.init());              // OK now

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("0", storage.get("log_last_index"));
        ASSERT_EQ("0", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
    /*
     * Full recovery
     */
    {
        MemoryStorageBackend storage;

        {
            // This log is used to initialize the storage
            Log log(storage, tracer);
            ASSERT_LE(0, log.init());

            uavcan::protocol::dynamic_node_id::server::Entry entry;
            entry.term = 1;
            entry.node_id = 1;
            entry.unique_id[0] = 1;
            ASSERT_LE(0, log.append(entry));
        }
        ASSERT_LE(4, storage.getNumKeys());

        PersistentState pers(storage, tracer);

        ASSERT_GT(0, pers.init());              // Fails because log is not empty

        storage.set("current_term", "0");
        storage.set("voted_for", "0");
        ASSERT_GT(0, pers.init());              // Fails because of bad currentTerm

        storage.set("current_term", "1");       // OK
        storage.set("voted_for", "128");        // Invalid value
        ASSERT_GT(0, pers.init());              // Fails because of bad votedFor

        storage.set("voted_for", "0");          // OK now
        ASSERT_LE(0, pers.init());

        ASSERT_LE(3, storage.getNumKeys());
        ASSERT_EQ("1", storage.get("log_last_index"));
        ASSERT_EQ("1", storage.get("current_term"));
        ASSERT_EQ("0", storage.get("voted_for"));
    }
}


TEST(dynamic_node_id_server_PersistentState, Basic)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    EventTracer tracer;
    MemoryStorageBackend storage;
    PersistentState pers(storage, tracer);

    /*
     * Initializing
     */
    ASSERT_LE(0, pers.init());

    ASSERT_EQ("0", storage.get("log_last_index"));
    ASSERT_EQ("0", storage.get("current_term"));
    ASSERT_EQ("0", storage.get("voted_for"));

    /*
     * Inserting some log entries
     */
    uavcan::protocol::dynamic_node_id::server::Entry entry;
    entry.term = 1;
    entry.node_id = 1;
    entry.unique_id[0] = 1;
    ASSERT_LE(0, pers.getLog().append(entry));

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("0", storage.get("current_term"));
    ASSERT_EQ("0", storage.get("voted_for"));

    /*
     * Changing current term
     */
    ASSERT_EQ(0, pers.getCurrentTerm());
    ASSERT_LE(0, pers.setCurrentTerm(2));
    ASSERT_EQ(2, pers.getCurrentTerm());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
    ASSERT_EQ("0", storage.get("voted_for"));

    /*
     * Changing votedFor
     */
    ASSERT_FALSE(pers.isVotedForSet());
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_LE(0, pers.setVotedFor(0));
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_LE(0, pers.setVotedFor(45));
    ASSERT_EQ(45, pers.getVotedFor().get());
    ASSERT_TRUE(pers.isVotedForSet());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
    ASSERT_EQ("45", storage.get("voted_for"));

    ASSERT_LE(0, pers.resetVotedFor());
    ASSERT_EQ(0, pers.getVotedFor().get());
    ASSERT_FALSE(pers.isVotedForSet());
    ASSERT_EQ("0", storage.get("voted_for"));

    ASSERT_LE(0, pers.setVotedFor(45));
    ASSERT_TRUE(pers.isVotedForSet());
    ASSERT_EQ("45", storage.get("voted_for"));

    /*
     * Handling errors
     */
    storage.failOnSetCalls(true);

    ASSERT_EQ(2, pers.getCurrentTerm());
    ASSERT_GT(0, pers.setCurrentTerm(7893));
    ASSERT_EQ(2, pers.getCurrentTerm());

    ASSERT_EQ(45, pers.getVotedFor().get());
    ASSERT_GT(0, pers.setVotedFor(78));
    ASSERT_EQ(45, pers.getVotedFor().get());

    ASSERT_EQ("1", storage.get("log_last_index"));
    ASSERT_EQ("2", storage.get("current_term"));
    ASSERT_EQ("45", storage.get("voted_for"));

    /*
     * Final checks
     */
    ASSERT_GT(10, storage.getNumKeys());  // Making sure there's some sane number of keys in the storage
}
