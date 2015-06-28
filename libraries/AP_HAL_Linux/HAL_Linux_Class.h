
#ifndef __AP_HAL_LINUX_CLASS_H__
#define __AP_HAL_LINUX_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_Linux_Namespace.h"

class HAL_Linux : public AP_HAL::HAL {
public:
    HAL_Linux();
    void init(int argc, char * const * argv) const;
    const char* get_custom_log_directory() { return custom_log_directory; }
    const char* get_custom_terrain_directory() { return custom_terrain_directory; }

private:
    mutable const char* custom_log_directory = NULL;
    mutable const char* custom_terrain_directory = NULL;	
};

extern const HAL_Linux AP_HAL_Linux;

#endif // __AP_HAL_LINUX_CLASS_H__

