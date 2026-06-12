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

#include <stdint.h>

namespace AP_EZKontrolCAN_Protocol {

static constexpr uint8_t HANDSHAKE_START = 0x55;
static constexpr uint8_t HANDSHAKE_ACK = 0xAA;

static constexpr uint8_t HEARTBEAT_PERIOD_MS = 50;

static constexpr uint8_t VCU_ADDRESS = 0xD0;
static constexpr uint8_t PRIORITY_VCU_TO_MCU = 0x0C;
static constexpr uint8_t PRIORITY_MCU_TO_VCU = 0x18;

static constexpr uint8_t MSG_HANDSHAKE = 0x01;
static constexpr uint8_t MSG_COMMAND = 0x01;
static constexpr uint8_t MSG_TELEM_1 = 0x01;
static constexpr uint8_t MSG_TELEM_2 = 0x02;

static constexpr uint8_t COMMAND_FLAG_RUN = 0x01;
static constexpr uint8_t COMMAND_FLAG_SPEED_MODE = 0x02;

static inline uint32_t make_extended_id(uint8_t priority, uint8_t message, uint8_t dest, uint8_t src)
{
    return (uint32_t(priority) << 24) | (uint32_t(message) << 16) | (uint32_t(dest) << 8) | uint32_t(src);
}

} // namespace AP_EZKontrolCAN_Protocol
