#pragma once

#include <AP_HAL/HAL.h>

#include <stdbool.h>

#include "usb_mass_mal.h"

extern "C" {
    extern int usb_close(void);
    extern void SCSI_Init();
}

namespace F4Light {

class MassStorage {
public:
    MassStorage() {}
    
    void setup() const;
};

} // namespace
