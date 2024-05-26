/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_STORAGE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_STORAGE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <uavcan/util/bitset.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace centralized
{
/**
 * This class transparently replicates its state to the storage backend, keeping the most recent state in memory.
 * Writes are slow, reads are instantaneous.
 */
class Storage
{
    typedef BitSet<NodeID::Max + 1> OccupationMask;
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeStatic,
                  BitLenToByteLen<NodeID::Max + 1>::Result>
            OccupationMaskArray;

    IStorageBackend& storage_;
    OccupationMask occupation_mask_;

    static IStorageBackend::String getOccupationMaskKey() { return "occupation_mask"; }

    static OccupationMask maskFromArray(const OccupationMaskArray& array)
    {
        OccupationMask mask;
        for (uint8_t byte = 0; byte < array.size(); byte++)
        {
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                mask[byte * 8U + bit] = (array[byte] & (1U << bit)) != 0;
            }
        }
        return mask;
    }

    static OccupationMaskArray maskToArray(const OccupationMask& mask)
    {
        OccupationMaskArray array;
        for (uint8_t byte = 0; byte < array.size(); byte++)
        {
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (mask[byte * 8U + bit])
                {
                    array[byte] = static_cast<uint8_t>(array[byte] | (1U << bit));
                }
            }
        }
        return array;
    }

public:
    Storage(IStorageBackend& storage) :
        storage_(storage)
    { }

    /**
     * This method reads only the occupation mask from the storage.
     */
    int init()
    {
        StorageMarshaller io(storage_);
        OccupationMaskArray array;
        io.get(getOccupationMaskKey(), array);
        occupation_mask_ = maskFromArray(array);
        return 0;
    }

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the entry was successfully appended.
     */
    int add(const NodeID node_id, const UniqueID& unique_id)
    {
        if (!node_id.isUnicast())
        {
            return -ErrInvalidParam;
        }

        StorageMarshaller io(storage_);

        // If next operations fail, we'll get a dangling entry, but it's absolutely OK.
        {
            uint32_t node_id_int = node_id.get();
            int res = io.setAndGetBack(StorageMarshaller::convertUniqueIDToHex(unique_id), node_id_int);
            if (res < 0)
            {
                return res;
            }
            if (node_id_int != node_id.get())
            {
                return -ErrFailure;
            }
        }

        // Updating the mask in the storage
        OccupationMask new_occupation_mask = occupation_mask_;
        new_occupation_mask[node_id.get()] = true;
        OccupationMaskArray occupation_array = maskToArray(new_occupation_mask);

        int res = io.setAndGetBack(getOccupationMaskKey(), occupation_array);
        if (res < 0)
        {
            return res;
        }
        if (occupation_array != maskToArray(new_occupation_mask))
        {
            return -ErrFailure;
        }

        // Updating the cached mask only if the storage was updated successfully
        occupation_mask_ = new_occupation_mask;

        return 0;
    }

    /**
     * Returns an invalid node ID if there's no such allocation.
     */
    NodeID getNodeIDForUniqueID(const UniqueID& unique_id) const
    {
        StorageMarshaller io(storage_);
        uint32_t node_id = 0;
        io.get(StorageMarshaller::convertUniqueIDToHex(unique_id), node_id);
        return (node_id > 0 && node_id <= NodeID::Max) ? NodeID(static_cast<uint8_t>(node_id)) : NodeID();
    }

    bool isNodeIDOccupied(NodeID node_id) const { return occupation_mask_[node_id.get()]; }

    uint8_t getSize() const { return static_cast<uint8_t>(occupation_mask_.count()); }
};

}
}
}

#endif // Include guard
