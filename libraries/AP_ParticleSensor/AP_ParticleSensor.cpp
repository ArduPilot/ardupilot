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

#include "AP_ParticleSensor.h"
#include "AP_ParticleSensor_SDS021.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

void AP_ParticleSensor::init()
{
    AP_HAL::UARTDriver *port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SDS021, 0);
    if (port != nullptr) {
        backend = new AP_ParticleSensor_SDS021(*port);
    }
}

void AP_ParticleSensor::update()
{
    if (backend != nullptr) {
        backend->update();
    }
}
