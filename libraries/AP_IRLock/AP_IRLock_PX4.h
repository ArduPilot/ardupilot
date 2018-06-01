/*
 * AP_IRLock_PX4.h
 *
 *  Created on: Nov 10, 2014
 *      Author: MLandes
 */

#ifndef AP_IRLOCK_PX4_H_
#define AP_IRLOCK_PX4_H_

#include "IRLock.h"

class AP_IRLock_PX4 : public IRLock
{
public:
    AP_IRLock_PX4();

    // init - initialize sensor library
    virtual void init();

    // retrieve latest sensor data - returns true if new data is available
    virtual bool update();

private:
    int _fd;
    uint64_t _last_timestamp;
};

#endif /* AP_IRLOCK_PX4_H_ */
