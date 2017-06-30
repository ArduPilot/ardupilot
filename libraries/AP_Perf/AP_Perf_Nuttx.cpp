#include "AP_Perf_Nuttx.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <AP_HAL_PX4/AP_HAL_PX4.h>
using namespace PX4;
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_HAL_VRBRAIN/AP_HAL_VRBRAIN.h>
using namespace VRBRAIN;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
AP_Perf::perf_counter_t AP_Perf_Nuttx::add(perf_counter_type t, const char *name)
{
    ::perf_counter_type nuttx_t;
    switch (t) {
    case AP_Perf::PC_COUNT:
        nuttx_t = ::PC_COUNT;
        break;
    case AP_Perf::PC_ELAPSED:
        nuttx_t = ::PC_ELAPSED;
        break;
    case AP_Perf::PC_INTERVAL:
        nuttx_t = ::PC_INTERVAL;
        break;
    default:
        return nullptr;
    }

    return (perf_counter_t)::perf_alloc(nuttx_t, name);
}

void AP_Perf_Nuttx::begin(perf_counter_t h)
{
    ::perf_begin((::perf_counter_t)h);
}

void AP_Perf_Nuttx::end(perf_counter_t h)
{
    ::perf_end((::perf_counter_t)h);
}

void AP_Perf_Nuttx::count(perf_counter_t h)
{
    ::perf_count((::perf_counter_t)h);
}
#endif