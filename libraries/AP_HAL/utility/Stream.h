
#ifndef __AP_HAL_UTILITY_STREAM_H__
#define __AP_HAL_UTILITY_STREAM_H__

#include "../AP_HAL_Namespace.h"
#include "Print.h"

/* A simple Stream library modeled after the bits we actually use
 * from Arduino Stream */

class AP_HAL::Stream : public AP_HAL::Print {
public:
    virtual int  available() = 0;
    /* return value for read() and peek() :
     * -1 if nothing available, uint8_t value otherwise. */
    virtual int  read() = 0;
    virtual int  peek() = 0;
};

#endif // __AP_HAL_UTILITY_STREAM_H__

