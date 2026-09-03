#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_SITL/HAL_SITL_Class.h>
#include "AP_HAL_WASM_Namespace.h"

/*
 * HAL_WASM extends HAL_SITL, reusing all simulation physics and drivers.
 * The constructor provides a ring-buffer driver for MAVLink/serial0 so JS can
 * exchange raw MAVLink bytes with the autopilot via functions retained with
 * EMSCRIPTEN_KEEPALIVE.
 *
 * run() is inherited from HAL_SITL unchanged; with PROXY_TO_PTHREAD the
 * while(true) main loop runs in a Web Worker, not blocking the browser UI.
 */
class HALWASM::HAL_WASM : public HAL_SITL {
public:
    HAL_WASM();
};
