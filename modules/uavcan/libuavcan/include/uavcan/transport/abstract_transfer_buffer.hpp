/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_ABSTRACT_TRANSFER_BUFFER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_ABSTRACT_TRANSFER_BUFFER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/std.hpp>

namespace uavcan
{
/**
 * API for transfer buffer users.
 */
class UAVCAN_EXPORT ITransferBuffer
{
public:
    virtual ~ITransferBuffer() { }

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const = 0;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len) = 0;
};

}

#endif // UAVCAN_TRANSPORT_ABSTRACT_TRANSFER_BUFFER_HPP_INCLUDED
