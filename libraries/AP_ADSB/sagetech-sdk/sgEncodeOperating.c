/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sgEncodeOperating.c
 * @author Jacob.Garrison
 *
 * @date   Feb 15, 2021
 *
 * This file receives a populated operating message struct and
 * converts it into a operating message buffer.
 */

#include <stdbool.h>
#include <stdlib.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_PAYLOAD_LEN_OPMSG SG_MSG_LEN_OPMSG - 5 /// the payload length.

#define PBASE 4            /// the payload offset.
#define OFFSET_SQUAWK 0    /// the squawk code offset in the payload.
#define OFFSET_CONFIG 2    /// the mode/config offset in the payload.
#define OFFSET_EMRG_ID 3   /// the emergency flag offset in the payload.
#define OFFSET_ALT 4       /// the altitude offset in the payload.
#define OFFSET_RATE 6      /// the climb rate offset in the payload.
#define OFFSET_HEADING 8   /// the heading offset in the payload.
#define OFFSET_AIRSPEED 10 /// the airspeed offset in the payload.

/*
 * Documented in the header file.
 */
bool sgEncodeOperating(uint8_t *buffer, sg_operating_t *op, uint8_t msgId)
{
    // populate header
    buffer[0] = SG_MSG_START_BYTE;
    buffer[1] = SG_MSG_TYPE_HOST_OPMSG;
    buffer[2] = msgId;
    buffer[3] = SG_PAYLOAD_LEN_OPMSG;

    // populate Squawk code
    uint162Buf(&buffer[PBASE + OFFSET_SQUAWK], op->squawk);

    // populate Mode/Config
    buffer[PBASE + OFFSET_CONFIG] = op->milEmergency << 5 |
                                    op->enableXBit << 4 |
                                    op->enableSqt << 3 |
                                    op->savePowerUp << 2 |
                                    op->opMode;

    // populate Emergency/Ident
    buffer[PBASE + OFFSET_EMRG_ID] = op->identOn << 3 |
                                     op->emergcType;

    // populate Altitude
    uint16_t altCode = 0;
    if (op->altUseIntrnl)
    {
        altCode = 0x8000;
    }
    else if (op->altHostAvlbl)
    {
        // 100 foot encoding conversion
        altCode = (op->altitude + 1200) / 100;

        if (op->altRes25)
        {
            altCode *= 4;
        }

        // 'Host altitude available' flag
        altCode += 0x4000;
    }
    uint162Buf(&buffer[PBASE + OFFSET_ALT], altCode);

    // populate Altitude Rate
    int16_t rate = op->climbRate / 64;
    if (!op->climbValid)
    {
        rate = 0x8000;
    }
    uint162Buf(&buffer[PBASE + OFFSET_RATE], rate);

    // populate Heading
    //    conversion: heading * ( pow(2, 15) / 360 )
    uint16_t heading = op->heading * 32768 / 360;
    if (op->headingValid)
    {
        heading += 0x8000;
    }
    uint162Buf(&buffer[PBASE + OFFSET_HEADING], heading);

    // populate Airspeed
    uint16_t airspeed = op->airspd;
    if (op->airspdValid)
    {
        airspeed += 0x8000;
    }
    uint162Buf(&buffer[PBASE + OFFSET_AIRSPEED], airspeed);

    // populate checksum
    appendChecksum(buffer, SG_MSG_LEN_OPMSG);

    return true;
}
