#include <AP_HAL/AP_HAL.h>

#include <emscripten/emscripten.h>

#include "HAL_WASM_Class.h"
#include "UARTDriver.h"

using namespace HALWASM;

static HALWASM::UARTDriver wasm_serial0;

extern "C" {

    EMSCRIPTEN_KEEPALIVE size_t ardupilot_serial0_write(const uint8_t *buf, size_t len)
    {
        return wasm_serial0.js_write(buf, len);
    }

    EMSCRIPTEN_KEEPALIVE size_t ardupilot_serial0_read(uint8_t *buf, size_t max_len)
    {
        return wasm_serial0.js_read(buf, max_len);
    }

    EMSCRIPTEN_KEEPALIVE size_t ardupilot_serial0_read_available(void)
    {
        return wasm_serial0.js_read_available();
    }

    EMSCRIPTEN_KEEPALIVE void *ardupilot_malloc(size_t size)
    {
        return malloc(size);
    }

    EMSCRIPTEN_KEEPALIVE void ardupilot_free(void *ptr)
    {
        free(ptr);
    }

} // extern "C"

HAL_WASM::HAL_WASM() :
    HAL_SITL(&wasm_serial0)
{}

void HAL_WASM::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    // Browser hosts cannot provide the default TCP listeners for serial1/2.
    get_sitl_state()->_serial_path[1] = "none";
    get_sitl_state()->_serial_path[2] = "none";
    HAL_SITL::run(argc, argv, callbacks);
}

static HAL_WASM hal_wasm_inst;

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_wasm_inst;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_wasm_inst;
}
