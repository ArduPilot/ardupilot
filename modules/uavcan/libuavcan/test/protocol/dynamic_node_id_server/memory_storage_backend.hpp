/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <map>
#include <uavcan/protocol/dynamic_node_id_server/storage_backend.hpp>

class MemoryStorageBackend : public uavcan::dynamic_node_id_server::IStorageBackend
{
    typedef std::map<String, String> Container;
    Container container_;

    bool fail_;

public:
    MemoryStorageBackend()
        : fail_(false)
    { }

    virtual String get(const String& key) const
    {
        const Container::const_iterator it = container_.find(key);
        if (it == container_.end())
        {
            return String();
        }
        return it->second;
    }

    virtual void set(const String& key, const String& value)
    {
        if (!fail_)
        {
            container_[key] = value;
        }
    }

    void failOnSetCalls(bool really) { fail_ = really; }

    void reset() { container_.clear(); }

    unsigned getNumKeys() const { return unsigned(container_.size()); }

    void print() const
    {
        for (Container::const_iterator it = container_.begin(); it != container_.end(); ++it)
        {
            std::cout << it->first.c_str() << "\t" << it->second.c_str() << std::endl;
        }
    }
};
