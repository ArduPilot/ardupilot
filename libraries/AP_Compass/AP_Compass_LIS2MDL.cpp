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
#include "AP_Compass_config.h"

#if AP_COMPASS_LIS2MDL_ENABLED

#include "AP_Compass_LIS2MDL.h"

AP_Compass_Backend *AP_Compass_LIS2MDL::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_LIS2MDL *sensor = NEW_NOTHROW AP_Compass_LIS2MDL(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

#endif  // AP_COMPASS_LIS2MDL_ENABLED
