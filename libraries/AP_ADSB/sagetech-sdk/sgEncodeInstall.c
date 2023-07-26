/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sgEncodeInstall.c
 * @author Jacob.Garrison
 *
 * @date   Feb 23, 2021
 *
 * This file receives a populated installation message struct and
 * converts it into a installation message buffer.
 */

#include <ctype.h>
#include <string.h>
#include <stdbool.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_PAYLOAD_LEN_INSTALL SG_MSG_LEN_INSTALL - 5 /// the payload length.

#define PBASE 4             /// the payload offset.
#define OFFSET_ICAO 0       /// the icao address offset in the payload.
#define OFFSET_REG 3        /// the registration offset in the payload.
#define OFFSET_RSVD1 10     /// the first reserved field offset in the payload.
#define OFFSET_COM0 12      /// the COM port 0 offset in the payload.
#define OFFSET_COM1 13      /// the COM port 1 offset in the payload.
#define OFFSET_IP 14        /// the IP address offset in the payload.
#define OFFSET_MASK 18      /// the net mask offset in the payload.
#define OFFSET_PORT 22      /// the port number offset in the payload.
#define OFFSET_GPS 24       /// the GPS integrity offset in the payload.
#define OFFSET_EMIT_SET 25  /// the emitter category offset in the payload.
#define OFFSET_EMIT_TYPE 26 /// the emitter type offset in the payload.
#define OFFSET_SIZE 27      /// the aircraft size offset in the payload.
#define OFFSET_SPEED 28     /// the maximum airspeed offset in the payload.
#define OFFSET_ENCODER 29   /// the altitude-encoder-offset offset in the payload.
#define OFFSET_RSVD2 31     /// the second reserved field offset in the payload.
#define OFFSET_CONFIG 33    /// the configuration offset in the payload.
#define OFFSET_RSVD3 34     /// the third reserved field offset in the payload.

#define REG_LEN 7 /// the registration field length.
/*
 * Documented in the header file.
 */
bool sgEncodeInstall(uint8_t *buffer, sg_install_t *stl, uint8_t msgId)
{
    // populate header
    buffer[0] = SG_MSG_START_BYTE;
    buffer[1] = SG_MSG_TYPE_HOST_INSTALL;
    buffer[2] = msgId;
    buffer[3] = SG_PAYLOAD_LEN_INSTALL;

    // populate icao address
    icao2Buf(&buffer[PBASE + OFFSET_ICAO], stl->icao);

    // populate aircraft registration
    charArray2Buf(&buffer[PBASE + OFFSET_REG], stl->reg, REG_LEN);

    // populate reserved fields
    uint162Buf(&buffer[PBASE + OFFSET_RSVD1], 0);

    // populate COM port 0, correct enum offset
    buffer[PBASE + OFFSET_COM0] = stl->com0;

    // populate COM port 1, correct enum offset
    buffer[PBASE + OFFSET_COM1] = stl->com1;

    // populate IP address
    uint322Buf(&buffer[PBASE + OFFSET_IP], stl->eth.ipAddress);

    // populate net mask
    uint322Buf(&buffer[PBASE + OFFSET_MASK], stl->eth.subnetMask);

    // populate port number
    uint162Buf(&buffer[PBASE + OFFSET_PORT], stl->eth.portNumber);

    // populate gps integrity
    buffer[PBASE + OFFSET_GPS] = stl->sil << 4 |
                                 stl->sda;

    // populate emitter category set and type
    uint8_t emitSet;
    uint8_t emitType;
    if (stl->emitter < SG_EMIT_OFFSET_B) // group A
    {
        emitSet = SG_EMIT_GROUP_A;
        emitType = stl->emitter - SG_EMIT_OFFSET_A;
    }
    else if (stl->emitter < SG_EMIT_OFFSET_C) // group B
    {
        emitSet = SG_EMIT_GROUP_B;
        emitType = stl->emitter - SG_EMIT_OFFSET_B;
    }
    else if (stl->emitter < SG_EMIT_OFFSET_D) // group C
    {
        emitSet = SG_EMIT_GROUP_C;
        emitType = stl->emitter - SG_EMIT_OFFSET_C;
    }
    else // group D
    {
        emitSet = SG_EMIT_GROUP_D;
        emitType = stl->emitter - SG_EMIT_OFFSET_D;
    }
    buffer[PBASE + OFFSET_EMIT_SET] = emitSet;
    buffer[PBASE + OFFSET_EMIT_TYPE] = emitType;

    // populate aircraft size
    buffer[PBASE + OFFSET_SIZE] = stl->size;

    // populate max airspeed
    buffer[PBASE + OFFSET_SPEED] = stl->maxSpeed;

    // populate altitude encoder offset
    uint162Buf(&buffer[PBASE + OFFSET_ENCODER], stl->altOffset);

    // populate reserved fields
    uint162Buf(&buffer[PBASE + OFFSET_RSVD2], 0);

    // populate install configuration
    buffer[PBASE + OFFSET_CONFIG] = stl->wowConnected << 7 |
                                    stl->heater << 6 |
                                    stl->airspeedTrue << 5 |
                                    stl->hdgTrueNorth << 4 |
                                    stl->altRes100 << 3 |
                                    stl->antenna;

    // populate reserved fields
    uint162Buf(&buffer[PBASE + OFFSET_RSVD3], 0);

    // populate checksum
    appendChecksum(buffer, SG_MSG_LEN_INSTALL);

    return true;
}
