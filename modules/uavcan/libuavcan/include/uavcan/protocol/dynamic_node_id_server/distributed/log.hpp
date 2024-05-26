/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_LOG_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_LOG_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{
/**
 * Raft log.
 * This class transparently replicates its state to the storage backend, keeping the most recent state in memory.
 * Writes are slow, reads are instantaneous.
 */
class Log
{
public:
    typedef uint8_t Index;

    enum { Capacity = NodeID::Max + 1 };

private:
    IStorageBackend& storage_;
    IEventTracer& tracer_;
    Entry entries_[Capacity];
    Index last_index_;             // Index zero always contains an empty entry

    static IStorageBackend::String getLastIndexKey() { return "log_last_index"; }

    static IStorageBackend::String makeEntryKey(Index index, const char* postfix)
    {
        IStorageBackend::String str;
        // "log0_foobar"
        str += "log";
        str.appendFormatted("%d", int(index));
        str += "_";
        str += postfix;
        return str;
    }

    int readEntryFromStorage(Index index, Entry& out_entry)
    {
        const StorageMarshaller io(storage_);

        // Term
        if (io.get(makeEntryKey(index, "term"), out_entry.term) < 0)
        {
            return -ErrFailure;
        }

        // Unique ID
        if (io.get(makeEntryKey(index, "unique_id"), out_entry.unique_id) < 0)
        {
            return -ErrFailure;
        }

        // Node ID
        uint32_t node_id = 0;
        if (io.get(makeEntryKey(index, "node_id"), node_id) < 0)
        {
            return -ErrFailure;
        }
        if (node_id > NodeID::Max)
        {
            return -ErrFailure;
        }
        out_entry.node_id = static_cast<uint8_t>(node_id);

        return 0;
    }

    int writeEntryToStorage(Index index, const Entry& entry)
    {
        Entry temp = entry;

        StorageMarshaller io(storage_);

        // Term
        if (io.setAndGetBack(makeEntryKey(index, "term"), temp.term) < 0)
        {
            return -ErrFailure;
        }

        // Unique ID
        if (io.setAndGetBack(makeEntryKey(index, "unique_id"), temp.unique_id) < 0)
        {
            return -ErrFailure;
        }

        // Node ID
        uint32_t node_id = entry.node_id;
        if (io.setAndGetBack(makeEntryKey(index, "node_id"), node_id) < 0)
        {
            return -ErrFailure;
        }
        temp.node_id = static_cast<uint8_t>(node_id);

        return (temp == entry) ? 0 : -ErrFailure;
    }

    int initEmptyLogStorage()
    {
        StorageMarshaller io(storage_);

        /*
         * Writing the zero entry - it must always be default-initialized
         */
        entries_[0] = Entry();
        int res = writeEntryToStorage(0, entries_[0]);
        if (res < 0)
        {
            return res;
        }

        /*
         * Initializing last index
         * Last index must be written AFTER the zero entry, otherwise if the write fails here the storage will be
         * left in an inconsistent state.
         */
        last_index_ = 0;
        uint32_t stored_index = 0;
        res = io.setAndGetBack(getLastIndexKey(), stored_index);
        if (res < 0)
        {
            return res;
        }
        if (stored_index != 0)
        {
            return -ErrFailure;
        }

        return 0;
    }

public:
    Log(IStorageBackend& storage, IEventTracer& tracer)
        : storage_(storage)
        , tracer_(tracer)
        , last_index_(0)
    { }

    int init()
    {
        StorageMarshaller io(storage_);

        // Reading max index
        {
            uint32_t value = 0;
            if (io.get(getLastIndexKey(), value) < 0)
            {
                if (storage_.get(getLastIndexKey()).empty())
                {
                    UAVCAN_TRACE("dynamic_node_id_server::distributed::Log", "Initializing empty storage");
                    return initEmptyLogStorage();
                }
                else
                {
                    // There's some data in the storage, but it cannot be parsed - reporting an error
                    UAVCAN_TRACE("dynamic_node_id_server::distributed::Log", "Failed to read last index");
                    return -ErrFailure;
                }
            }
            if (value >= Capacity)
            {
                return -ErrFailure;
            }
            last_index_ = Index(value);
        }

        tracer_.onEvent(TraceRaftLogLastIndexRestored, last_index_);

        // Restoring log entries - note that index 0 always exists
        for (Index index = 0; index <= last_index_; index++)
        {
            const int result = readEntryFromStorage(index, entries_[index]);
            if (result < 0)
            {
                UAVCAN_TRACE("dynamic_node_id_server::distributed::Log", "Failed to read entry at index %u: %d",
                             unsigned(index), result);
                return result;
            }
        }

        UAVCAN_TRACE("dynamic_node_id_server::distributed::Log", "Restored %u log entries", unsigned(last_index_));
        return 0;
    }

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the entry was successfully appended.
     */
    int append(const Entry& entry)
    {
        if ((last_index_ + 1) >= Capacity)
        {
            return -ErrLogic;
        }

        tracer_.onEvent(TraceRaftLogAppend, last_index_ + 1U);

        // If next operations fail, we'll get a dangling entry, but it's absolutely OK.
        int res = writeEntryToStorage(Index(last_index_ + 1), entry);
        if (res < 0)
        {
            return res;
        }

        // Updating the last index
        StorageMarshaller io(storage_);
        uint32_t new_last_index = last_index_ + 1U;
        res = io.setAndGetBack(getLastIndexKey(), new_last_index);
        if (res < 0)
        {
            return res;
        }
        if (new_last_index != last_index_ + 1U)
        {
            return -ErrFailure;
        }
        entries_[new_last_index] = entry;
        last_index_ = Index(new_last_index);

        UAVCAN_TRACE("dynamic_node_id_server::distributed::Log", "New entry, index %u, node ID %u, term %u",
                     unsigned(last_index_), unsigned(entry.node_id), unsigned(entry.term));
        return 0;
    }

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the requested operation has been carried out successfully.
     */
    int removeEntriesWhereIndexGreaterOrEqual(Index index)
    {
        UAVCAN_ASSERT(last_index_ < Capacity);

        if (((index) >= Capacity) || (index <= 0))
        {
            return -ErrLogic;
        }

        uint32_t new_last_index = index - 1U;

        tracer_.onEvent(TraceRaftLogRemove, new_last_index);

        if (new_last_index != last_index_)
        {
            StorageMarshaller io(storage_);
            int res = io.setAndGetBack(getLastIndexKey(), new_last_index);
            if (res < 0)
            {
                return res;
            }
            if (new_last_index != index - 1U)
            {
                return -ErrFailure;
            }
            UAVCAN_TRACE("dynamic_node_id_server::distributed::Log", "Entries removed, last index %u --> %u",
                         unsigned(last_index_), unsigned(new_last_index));
            last_index_ = Index(new_last_index);
        }

        // Removal operation leaves dangling entries in storage, it's OK
        return 0;
    }

    int removeEntriesWhereIndexGreater(Index index)
    {
        return removeEntriesWhereIndexGreaterOrEqual(Index(index + 1U));
    }

    /**
     * Returns nullptr if there's no such index.
     * This method does not use storage IO.
     */
    const Entry* getEntryAtIndex(Index index) const
    {
        UAVCAN_ASSERT(last_index_ < Capacity);
        return (index <= last_index_) ? &entries_[index] : UAVCAN_NULLPTR;
    }

    Index getLastIndex() const { return last_index_; }

    bool isOtherLogUpToDate(Index other_last_index, Term other_last_term) const
    {
        UAVCAN_ASSERT(last_index_ < Capacity);
        // Terms are different - the one with higher term is more up-to-date
        if (other_last_term != entries_[last_index_].term)
        {
            return other_last_term > entries_[last_index_].term;
        }
        // Terms are equal - longer log wins
        return other_last_index >= last_index_;
    }
};

}
}
}

#endif // Include guard
