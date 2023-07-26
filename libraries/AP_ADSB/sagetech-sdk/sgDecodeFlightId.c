/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file sgDecodeFlightId.c
 * @author Jacob.Garrison
 *
 * @date Mar 10, 2021
 *
 */

#include <string.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_ID_LEN 8 // The number of bytes in the flight id field

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t type;
    uint8_t id;
    uint8_t payloadLen;
    char flightId[SG_ID_LEN];
    uint8_t rsvd[4];
    uint8_t checksum;
} flightid_t;

/*
 * Documented in the header file.
 */
bool sgDecodeFlightId(uint8_t *buffer, sg_flightid_t *id)
{
    flightid_t sgId;
    memcpy(&sgId, buffer, sizeof(flightid_t));

    strcpy(id->flightId, sgId.flightId);
    memset(&id->flightId[SG_ID_LEN], '\0', 1); // Ensure flight id is null-terminated

    return true;
}
