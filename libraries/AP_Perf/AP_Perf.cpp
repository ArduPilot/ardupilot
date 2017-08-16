/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Perf.h"
#include "AP_Perf_Backend.h"
#include "AP_Perf_Dummy.h"
#include "AP_Perf_Linux.h"
#include "AP_Perf_Nuttx.h"

AP_Perf AP_Perf::_instance{};

AP_Perf::AP_Perf()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    _backend = new AP_Perf_Nuttx();
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    _backend = new AP_Perf_Linux();
#else
    // An empty definition.
    _backend = new AP_Perf_Dummy();
#endif
}

AP_Perf::perf_counter_t AP_Perf::add(perf_counter_type type, const char *name) {
    return _backend->add(type, name);
}

void AP_Perf::begin(perf_counter_t pc) {
    _backend->begin(pc);
}

void AP_Perf::end(perf_counter_t pc) {
    _backend->end(pc);
}

void AP_Perf::count(perf_counter_t pc) {
    _backend->count(pc);
}
