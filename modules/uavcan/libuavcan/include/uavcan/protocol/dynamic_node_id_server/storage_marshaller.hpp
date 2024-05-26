/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_STORAGE_MARSHALLER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_STORAGE_MARSHALLER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_backend.hpp>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>
#include <cstdlib>

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <cerrno>
#endif

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * This class extends the storage backend interface with serialization/deserialization functionality.
 */
class StorageMarshaller
{
    IStorageBackend& storage_;

    static uint8_t convertLowerCaseHexCharToNibble(char ch)
    {
        const uint8_t ret = (ch > '9') ? static_cast<uint8_t>(ch - 'a' + 10) : static_cast<uint8_t>(ch - '0');
        UAVCAN_ASSERT(ret < 16);
        return ret;
    }

public:
    StorageMarshaller(IStorageBackend& storage)
        : storage_(storage)
    { }

    /**
     * Turns a unique ID into a hex string that can be used as a key or as a value.
     */
    static IStorageBackend::String convertUniqueIDToHex(const UniqueID& key)
    {
        IStorageBackend::String serialized;
        for (uint8_t i = 0; i < UniqueID::MaxSize; i++)
        {
            serialized.appendFormatted("%02x", key.at(i));
        }
        UAVCAN_ASSERT(serialized.size() == UniqueID::MaxSize * 2);
        return serialized;
    }

    /**
     * These methods set the value and then immediately read it back.
     *  1. Serialize the value.
     *  2. Update the value on the backend.
     *  3. Call get() with the same value argument.
     * The caller then is supposed to check whether the argument has the desired value.
     */
    int setAndGetBack(const IStorageBackend::String& key, uint32_t& inout_value)
    {
        IStorageBackend::String serialized;
        serialized.appendFormatted("%lu", inout_value);

        UAVCAN_TRACE("StorageMarshaller", "Set %s = %s", key.c_str(), serialized.c_str());
        storage_.set(key, serialized);

        return get(key, inout_value);
    }

    int setAndGetBack(const IStorageBackend::String& key, UniqueID& inout_value)
    {
        const IStorageBackend::String serialized = convertUniqueIDToHex(inout_value);

        UAVCAN_TRACE("StorageMarshaller", "Set %s = %s", key.c_str(), serialized.c_str());
        storage_.set(key, serialized);

        return get(key, inout_value);
    }

    /**
     * Getters simply read and deserialize the value.
     *  1. Read the value back from the backend; return false if read fails.
     *  2. Deserealize the newly read value; return false if deserialization fails.
     *  3. Update the argument with deserialized value.
     *  4. Return true.
     */
    int get(const IStorageBackend::String& key, uint32_t& out_value) const
    {
        /*
         * Reading the storage
         */
        const IStorageBackend::String val = storage_.get(key);
        if (val.empty())
        {
            return -ErrFailure;
        }

        /*
         * Per MISRA C++ recommendations, checking the inputs instead of relying solely on errno.
         * The value must contain only numeric characters.
         */
        for (IStorageBackend::String::const_iterator it = val.begin(); it != val.end(); ++it)
        {
            if (static_cast<char>(*it) < '0' || static_cast<char>(*it) > '9')
            {
                return -ErrFailure;
            }
        }

        if (val.size() > 10) // len(str(0xFFFFFFFF))
        {
            return -ErrFailure;
        }

        /*
         * Conversion is carried out here
         */
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        errno = 0;
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        const unsigned long long x = std::strtoull(val.c_str(), UAVCAN_NULLPTR, 10);
#else
        // There was no strtoull() before C++11, so we need to resort to strtoul()
        StaticAssert<(sizeof(unsigned long) >= sizeof(uint32_t))>::check();
        const unsigned long x = std::strtoul(val.c_str(), UAVCAN_NULLPTR, 10);
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        if (errno != 0)
        {
            return -ErrFailure;
        }
#endif

        out_value = static_cast<uint32_t>(x);
        return 0;
    }

    int get(const IStorageBackend::String& key, UniqueID& out_value) const
    {
        static const uint8_t NumBytes = UniqueID::MaxSize;

        /*
         * Reading the storage
         */
        IStorageBackend::String val = storage_.get(key);
        if (val.size() != NumBytes * 2)
        {
            return -ErrFailure;
        }

        /*
         * The value must contain only hexadecimal numbers.
         */
        val.convertToLowerCaseASCII();
        for (IStorageBackend::String::const_iterator it = val.begin(); it != val.end(); ++it)
        {
            if ((static_cast<char>(*it) < '0' || static_cast<char>(*it) > '9') &&
                (static_cast<char>(*it) < 'a' || static_cast<char>(*it) > 'f'))
            {
                return -ErrFailure;
            }
        }

        /*
         * Conversion is carried out here
         */
        IStorageBackend::String::const_iterator it = val.begin();

        for (uint8_t byte_index = 0; byte_index < NumBytes; byte_index++)
        {
            out_value[byte_index] =
                static_cast<uint8_t>(convertLowerCaseHexCharToNibble(static_cast<char>(*it++)) << 4);
            out_value[byte_index] =
                static_cast<uint8_t>(convertLowerCaseHexCharToNibble(static_cast<char>(*it++)) | out_value[byte_index]);
        }

        return 0;
    }
};

}
}

#endif // Include guard
