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

#pragma once

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_SERIAL_ENABLED

class AP_SwarmMesh_Serial : public AP_SwarmMesh_Backend
{
public:
    AP_SwarmMesh_Serial(AP_SwarmMesh &frontend);

protected:
    bool     transport_ready() const override;
    uint32_t transport_available() override;
    int16_t  transport_read() override;
    uint32_t transport_txspace() override;
    void     transport_write(const uint8_t *buf, uint16_t len) override;

private:
    AP_HAL::UARTDriver *uart;
};

#endif  // AP_SWARMMESH_SERIAL_ENABLED
