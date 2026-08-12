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

#include "AP_SwarmMesh_Serial.h"

#if AP_SWARMMESH_SERIAL_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

AP_SwarmMesh_Serial::AP_SwarmMesh_Serial(AP_SwarmMesh &frontend) :
    AP_SwarmMesh_Backend(frontend)
{
    const AP_SerialManager &serialmanager = AP::serialmanager();
    uart = serialmanager.find_serial(AP_SerialManager::SerialProtocol_SwarmMesh, 0);
    if (uart == nullptr) {
        return;
    }
    uart->begin(serialmanager.find_baudrate(AP_SerialManager::SerialProtocol_SwarmMesh, 0));
}

bool AP_SwarmMesh_Serial::transport_ready() const
{
    return uart != nullptr;
}

uint32_t AP_SwarmMesh_Serial::transport_available()
{
    return uart->available();
}

int16_t  AP_SwarmMesh_Serial::transport_read()
{
    return uart->read();
}

uint32_t AP_SwarmMesh_Serial::transport_txspace()
{
    return uart->txspace();
}

void AP_SwarmMesh_Serial::transport_write(const uint8_t *buf, uint16_t len)
{
    uart->write(buf, len);
}

#endif  // AP_SWARMMESH_SERIAL_ENABLED