/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/error.hpp>
#include <cassert>
#include <cstdlib>

#ifndef UAVCAN_EXCEPTIONS
# error UAVCAN_EXCEPTIONS
#endif

#if UAVCAN_EXCEPTIONS
# include <stdexcept>
#endif

namespace uavcan
{

void handleFatalError(const char* msg)
{
#if UAVCAN_EXCEPTIONS
    throw std::runtime_error(msg);
#else
    (void)msg;
    UAVCAN_ASSERT(0);
    std::abort();
#endif
}

}
