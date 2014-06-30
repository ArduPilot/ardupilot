// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include "AP_Terrain.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Terrain::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Terrain following enable
    // @Description: enable terrain following
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, AP_Terrain, enable, 0),

    AP_GROUPEND
};

// constructor
AP_Terrain::AP_Terrain(const AP_AHRS &_ahrs) :
    ahrs(_ahrs),
    last_grid_spacing(0),
    grids_allocated(0),
    grids(NULL),
    last_request_time_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  allocate terrain object grid memory if enabled.
 */
void AP_Terrain::allocate(void)
{
    if (enable == 0) {
        return;
    }
    // constrain grid size to avoid 16 bit overflow
    if (grid_width > 150) {
        grid_width.set(150);
    }
    if (grid_width < 0) {
        return;
    }
    uint16_t grids_needed = sq((grid_width+4) / 5);
    uint16_t memory_needed = grids_needed * sizeof(struct grid);
    if (hal.util->available_memory() < memory_needed+512) {
        // refuse to allocate last bit of memory, we need some for
        // stack
        return;
    }
    if (grids != NULL && grids_needed == grids_allocated) {
        // already allocated
        return;
    }
    if (grids != NULL) {
        free(grids);
    }
    grids = (struct grid *)calloc(grids_needed, sizeof(struct grid));
    if (grids == NULL) {
        // not enough memory
        return;
    }
}

/*
  update terrain data. Check if we need to request more grids. This
  should be called at 1Hz
 */
void AP_Terrain::update(void)
{
    if (enable == 0) {
        // not enabled
        return;
    }
    // re-allocate if need be
    allocate();
}
