#pragma once

#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Util.h"


class Linux::LinuxUtilRPI : public Linux::LinuxUtil {
public:
    LinuxUtilRPI();

    static LinuxUtilRPI *from(AP_HAL::Util *util) {
        return static_cast<LinuxUtilRPI*>(util);
    }

    /* return the Raspberry Pi version */
    int get_rpi_version() const;
    
    /* kill pigpio if running */
    int terminate_process(const char* name);

protected:
    // Called in the constructor once
    int _check_rpi_version();
    // Get state of pigpio
    pid_t _find_process(const char* name);

private:
    int _rpi_version = 0;
};
