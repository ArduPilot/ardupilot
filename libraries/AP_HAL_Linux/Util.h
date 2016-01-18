
#ifndef __AP_HAL_LINUX_UTIL_H__
#define __AP_HAL_LINUX_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"

class Linux::LinuxUtil : public AP_HAL::Util {
public:
    void init(int argc, char * const *argv) {
        saved_argc = argc;
        saved_argv = argv;
    }

    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }

    /**
       return commandline arguments, if available
     */
    void commandline_arguments(uint8_t &argc, char * const *&argv);

private:
    int saved_argc;
    char* const *saved_argv;
};

#endif // __AP_HAL_LINUX_UTIL_H__
