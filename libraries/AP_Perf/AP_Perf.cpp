#include "AP_Perf.h"
#include "AP_Perf_Backend.h"
#include "AP_Perf_Lttng.h"
#include "AP_Perf_Nuttx.h"
#include "AP_Perf_Linux.h"

AP_Perf AP_Perf::_instance{};

AP_Perf::AP_Perf()
{
#ifdef HAVE_LTTNG_UST
    _backend = new AP_Perf_Lttng();
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    _backend = new AP_Perf_Nuttx();
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    _backend = new AP_Perf_Linux();
#else
    // An empty definition.
    _backend = new AP_Perf_Backend();
#endif
}

AP_Perf::perf_counter_t AP_Perf::add(perf_counter_type type, const char *name)
{
    return _backend->add(type, name);
}

void AP_Perf::begin(perf_counter_t pc)
{
    _backend->begin(pc);
}

void AP_Perf::end(perf_counter_t pc)
{
    _backend->end(pc);
}

void AP_Perf::count(perf_counter_t pc)
{
    _backend->count(pc);
}