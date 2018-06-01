
#ifndef __AP_HAL_EMPTY_PRIVATE_MEMBER_H__
#define __AP_HAL_EMPTY_PRIVATE_MEMBER_H__

/* Just a stub as an example of how to implement a private member of an
 * AP_HAL module */

#include "AP_HAL_Empty.h"

class Empty::EmptyPrivateMember {
public:
    EmptyPrivateMember(uint16_t foo);
    void init();
private:
    uint16_t _foo;
};

#endif // __AP_HAL_EMPTY_PRIVATE_MEMBER_H__

