#pragma once

#include "Util.h"

namespace Linux {

class UtilRPI : public Util {
public:
    UtilRPI();

    static UtilRPI *from(AP_HAL::Util *util) {
        return static_cast<UtilRPI*>(util);
    }

    /* return the Raspberry Pi version */
    int get_rpi_version() const;

protected:
    // Called in the constructor once
    int _check_rpi_version();

private:
    int _rpi_version = 0;
};

}
