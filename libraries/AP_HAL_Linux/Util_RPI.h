#pragma once

#include "Util.h"

class Linux::LinuxUtilRPI : public Linux::LinuxUtil {
public:
    LinuxUtilRPI();

    static LinuxUtilRPI *from(AP_HAL::Util *util) {
        return static_cast<LinuxUtilRPI*>(util);
    }

    /* return the Raspberry Pi version */
    int get_rpi_version() const;

protected:
    // Called in the constructor once
    int _check_rpi_version();

private:
    int _rpi_version = 0;
};
