/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sgDecodeAck.c
 * @author jimb
 *
 * @date   Feb 10, 2021
 *
 * This file receives a raw Acknowledge message buffer and
 * parses the payload into a data struct.
 */

#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_ACK_XPNDR_FAIL 0x01
#define SG_ACK_SYSTEM_FAIL 0x02
#define SG_ACK_CRYPTO_FAIL 0x04
#define SG_ACK_WOW 0x08
#define SG_ACK_MAINT 0x10
#define SG_ACK_ALT_SOURCE 0x20
#define SG_ACK_OP_MODE 0xC0

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t type;
    uint8_t id;
    uint8_t payloadLen;
    uint8_t ackType;
    uint8_t ackId;
    uint8_t state;
    uint8_t alt[3];
    uint8_t checksum;
} ack_t;

/*
 * Documented in the header file.
 */
bool sgDecodeAck(uint8_t *buffer, sg_ack_t *ack)
{
    ack_t sgAck;
    memcpy(&sgAck, buffer, sizeof(ack_t));

    ack->ackType = sgAck.ackType;
    ack->ackId = sgAck.ackId;
    ack->failXpdr = (sgAck.state & SG_ACK_XPNDR_FAIL) > 0;
    ack->failSystem = (sgAck.state & SG_ACK_SYSTEM_FAIL) > 0;
    ack->failCrypto = (sgAck.state & SG_ACK_CRYPTO_FAIL) > 0;
    ack->wow = (sgAck.state & SG_ACK_WOW) > 0;
    ack->maint = (sgAck.state & SG_ACK_MAINT) > 0;
    ack->isHostAlt = (sgAck.state & SG_ACK_ALT_SOURCE) > 0;
    ack->opMode = (sgAck.state & SG_ACK_OP_MODE) >> 6;

    int32_t int24 = toInt32(sgAck.alt);

    // Bitmask altitude field to determine if alt = invalid
    if ((int24 & 0x00FFFFFF) == 0x00800000)
    {
        ack->alt = 0;
        ack->altValid = false;
    }
    else
    {
        ack->alt = int24;
        ack->altValid = true;
    }

    return true;
}
