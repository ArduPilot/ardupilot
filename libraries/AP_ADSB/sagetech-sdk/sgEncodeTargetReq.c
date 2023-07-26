/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sgEncodeTargetReq.c
 * @author Jacob.Garrison
 *
 * @date Feb 19, 2021
 *
 * This file receives a populated target request struct and
 * converts it into a target request message buffer.
 */

#include <stdbool.h>
#include <stdlib.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_PAYLOAD_LEN_TARGETREQ SG_MSG_LEN_TARGETREQ - 5 /// the payload length.

#define PBASE 4 /// the payload offset.

#define OFFSET_REQ_TYPE 0    /// the adsb reporting type and transmit port offset
#define OFFSET_MAX_TARGETS 1 /// the maximum number of targets offset
#define OFFSET_ICAO 3        /// the requested target icao offset
#define OFFSET_REPORTS 6     /// the requested report type offset
/*
 * Documented in the header file.
 */
bool sgEncodeTargetReq(uint8_t *buffer, sg_targetreq_t *tgt, uint8_t msgId)
{

    // populate header
    buffer[0] = SG_MSG_START_BYTE;
    buffer[1] = SG_MSG_TYPE_HOST_TARGETREQ;
    buffer[2] = msgId;
    buffer[3] = SG_PAYLOAD_LEN_TARGETREQ;

    // populate Request Type
    buffer[PBASE + OFFSET_REQ_TYPE] = tgt->transmitPort << 6 |
                                      tgt->reqType;

    // populate Max Targets
    uint162Buf(&buffer[PBASE + OFFSET_MAX_TARGETS], tgt->maxTargets);

    // populate Requested ICAO
    icao2Buf(&buffer[PBASE + OFFSET_ICAO], tgt->icao);

    // populated Requested Reports
    buffer[PBASE + OFFSET_REPORTS] = tgt->ownship << 7 |
                                     tgt->commA << 6 |
                                     tgt->military << 5 |
                                     tgt->tisb << 4 |
                                     tgt->airRefVel << 3 |
                                     tgt->targetState << 2 |
                                     tgt->modeStatus << 1 |
                                     tgt->stateVector;

    // populate checksum
    appendChecksum(buffer, SG_MSG_LEN_TARGETREQ);

    return true;
}
