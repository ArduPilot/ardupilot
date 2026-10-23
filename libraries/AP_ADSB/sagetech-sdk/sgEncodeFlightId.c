/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file sgEncodeFlightId.c
 * @author Jacob.Garrison
 *
 * @date Feb 25, 2021
 *
 */

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_PAYLOAD_LEN_FLIGHT SG_MSG_LEN_FLIGHT - 5 /// the payload length.

#define PBASE 4       /// the payload offset.
#define OFFSET_ID 0   /// the flight id offset in the payload.
#define OFFSET_RSVD 8 /// the reserved field offset in the payload.

#define ID_LEN 8 /// the length of the flight identification field.

/*
 * Documented in the header file.
 */
bool sgEncodeFlightId(uint8_t *buffer, sg_flightid_t *id, uint8_t msgId)
{
    // populate header
    buffer[0] = SG_MSG_START_BYTE;
    buffer[1] = SG_MSG_TYPE_HOST_FLIGHT;
    buffer[2] = msgId;
    buffer[3] = SG_PAYLOAD_LEN_FLIGHT;

    // populate flight identification
    charArray2Buf(&buffer[PBASE + OFFSET_ID], id->flightId, ID_LEN);

    // populate reserved field
    uint322Buf(&buffer[PBASE + OFFSET_RSVD], 0);

    // populate checksum
    appendChecksum(buffer, SG_MSG_LEN_FLIGHT);

    return true;
}
