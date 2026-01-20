#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_RP_Namespace.h"
#include "WSPIDevice.h"

class HAL_RP : public AP_HAL::HAL {
public:
    HAL_RP();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;

    RP::NAND_PIO_Driver* get_nand_pio();
};
