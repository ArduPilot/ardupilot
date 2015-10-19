
#ifndef __AP_HAL_LINUX_CLASS_H__
#define __AP_HAL_LINUX_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux_Namespace.h"

class HAL_Linux : public AP_HAL::HAL {
public:
    HAL_Linux();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

extern const HAL_Linux AP_HAL_Linux;

#endif // __AP_HAL_LINUX_CLASS_H__

