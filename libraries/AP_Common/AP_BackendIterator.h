#pragma once

#include <stdint.h>

template <typename T>
class AP_BackendIterator
{
public:
    AP_BackendIterator(T**_backends, uint8_t &_num_backends) :
        backends{_backends},
        num_backends{_num_backends}
        {}

    // see
    // https://www.internalpointers.com/post/writing-custom-iterators-modern-cpp
    // - search for "done by implementing some custom"
    struct Iterator {
        Iterator(T **_backend) : backend{_backend} {}
        T& operator*() const { return **backend; }
        T* operator->() { return *backend; }

        // Prefix increment
        Iterator& operator++() { backend++; return *this; };
        // Postfix increment
        Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

        friend bool operator== (const Iterator& a, const Iterator& b) { return a.backend == b.backend; };
        friend bool operator!= (const Iterator& a, const Iterator& b) { return a.backend != b.backend; };
    private:
        T **backend;
    };

    Iterator begin() { return Iterator(&backends[0]); }
    Iterator end()   { return Iterator(&backends[num_backends]); }

    Iterator begin() const { return Iterator(&backends[0]); }
    Iterator end() const { return Iterator(&backends[num_backends]); }

private:

    T**backends;
    uint8_t &num_backends;
};
