/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_STORAGE_BACKEND_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_STORAGE_BACKEND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/marshal/types.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * This interface is used by the server to read and write stable storage.
 * The storage is represented as a key-value container, where keys and values are ASCII strings up to 32
 * characters long, not including the termination byte. Fixed block size allows for absolutely straightforward
 * and efficient implementation of storage backends, e.g. based on text files.
 * Keys and values may contain only non-whitespace, non-formatting printable characters.
 */
class UAVCAN_EXPORT IStorageBackend
{
public:
    /**
     * Maximum length of keys and values. One pair takes twice as much space.
     */
    enum { MaxStringLength = 32 };

    /**
     * It is guaranteed that the server will never require more than this number of key/value pairs.
     * Total storage space needed is (MaxKeyValuePairs * MaxStringLength * 2), not including storage overhead.
     */
    enum { MaxKeyValuePairs = 512 };

    /**
     * This type is used to exchange data chunks with the backend.
     * It doesn't use any dynamic memory; please refer to the Array<> class for details.
     */
    typedef MakeString<MaxStringLength>::Type String;

    /**
     * Read one value from the storage.
     * If such key does not exist, or if read failed, an empty string will be returned.
     * This method should not block for more than 50 ms.
     */
    virtual String get(const String& key) const = 0;

    /**
     * Create or update value for the given key. Empty value should be regarded as a request to delete the key.
     * This method should not block for more than 50 ms.
     * Failures will be ignored.
     */
    virtual void set(const String& key, const String& value) = 0;

    virtual ~IStorageBackend() { }
};

}
}

#endif // Include guard
