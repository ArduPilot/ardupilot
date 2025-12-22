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

// TODO Stage 2: Populate protocol constants from EZKontrol CAN PDFs.
// Placeholder values should not be used for real control.
static constexpr uint8_t HANDSHAKE_START = 0x55;
static constexpr uint8_t HANDSHAKE_ACK = 0xAA;

static constexpr uint8_t HEARTBEAT_PERIOD_MS = 50;

// TODO Stage 2: Define extended CAN IDs, life counter, and payload layouts.

} // namespace AP_EZKontrolCAN_Protocol
