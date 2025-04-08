/*
     Copyright (C) 2021    Kraus Hamdani Aerospace Inc. All rights reserved.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.    If not, see <http://www.gnu.org/licenses/>.


    Author: GDL90/UCP protocol by uAvionix, 2021.
    Implemented by: Tom Pittenger
 */

#ifndef _GDL90_H_
#define _GDL90_H_

#include <stdint.h>

#define GDL90_QUEUE_LENGTH            (2)

#define GDL90_FLAG_BYTE               (0x7E)
#define GDL90_CONTROL_ESCAPE_BYTE     (0x7D)
#define GDL90_STUFF_BYTE              (0x20)
#define GDL90_OVERHEAD_LENGTH         (3) // Not counting framing bytes


// Transmit message sizes
#define GDL90_TX_MAX_PAYLOAD_LENGTH   (552)
#define GDL90_TX_MAX_PACKET_LENGTH    (GDL90_TX_MAX_PAYLOAD_LENGTH + GDL90_OVERHEAD_LENGTH)
#define GDL90_TX_MAX_FRAME_LENGTH     (2 + ((15 * GDL90_TX_MAX_PACKET_LENGTH) / 10)) // IF every other byte was stuffed

// Receive message sizes
#define GDL90_RX_MAX_PAYLOAD_LENGTH   (128)
#define GDL90_RX_MAX_PACKET_LENGTH    (GDL90_RX_MAX_PAYLOAD_LENGTH + GDL90_OVERHEAD_LENGTH)

typedef union __attribute__((__packed__))
{
    struct __attribute__((__packed__))
    {
        GDL90_MESSAGE_ID messageId;
        uint8_t          payload[GDL90_TX_MAX_PAYLOAD_LENGTH];
        uint16_t         crc; // Actually CRC location varies. This is a placeholder
    };
    uint8_t raw[GDL90_TX_MAX_PACKET_LENGTH];
} GDL90_TX_MESSAGE;

typedef union __attribute__((__packed__))
{
    struct __attribute__((__packed__))
    {
        GDL90_MESSAGE_ID messageId;
        uint8_t          payload[GDL90_RX_MAX_PAYLOAD_LENGTH];
        uint16_t         crc; // Actually CRC location varies. This is a placeholder
    };
    uint8_t raw[GDL90_RX_MAX_PACKET_LENGTH];
} GDL90_RX_MESSAGE;

typedef enum
{
    GDL90_RX_IDLE,
    GDL90_RX_IN_PACKET,
    GDL90_RX_UNSTUFF,
} GDL90_RX_STATE;

typedef struct
{
    GDL90_RX_STATE state;
    uint16_t       length;
    uint8_t        prev_data;
} GDL90_RX_STATUS;

#endif
