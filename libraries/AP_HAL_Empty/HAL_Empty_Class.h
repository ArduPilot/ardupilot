
#ifndef __AP_HAL_EMPTY_CLASS_H__
#define __AP_HAL_EMPTY_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_Empty_Namespace.h"
#include "PrivateMember.h"

class HAL_Empty : public AP_HAL::HAL {
public:
    HAL_Empty();
    void init(int argc, char * const * argv) const;
private:
    Empty::EmptyPrivateMember *_member;
};

extern const HAL_Empty AP_HAL_Empty;

#endif // __AP_HAL_EMPTY_CLASS_H__

