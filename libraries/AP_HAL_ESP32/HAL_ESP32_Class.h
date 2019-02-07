#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_ESP32/HAL_ESP32_Namespace.h>

class HAL_ESP32 : public AP_HAL::HAL {
public:
    HAL_ESP32();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
