/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file sgEncodeGPS.c
 * @author Jacob.Garrison
 *
 * @date Mar 1, 2021
 *
 * This file receives a populated GPS message struct and
 * converts it into a GPS message buffer.
 */

#include <stdbool.h>
#include <stdlib.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_PAYLOAD_LEN_GPS SG_MSG_LEN_GPS - 5 /// the payload length.
#define _UNUSED(x) ((void)(x))

#define PBASE 4            /// the payload offset.
#define OFFSET_LONGITUDE 0 /// the longitude offset in the payload.
#define OFFSET_LATITUDE 11 /// the latitude offset in the payload.
#define OFFSET_SPEED 21    /// the ground speed offset in the payload.
#define OFFSET_TRACK 27    /// the ground track offset in the payload.
#define OFFSET_STATUS 35   /// the hemisphere/data status offset in the payload.
#define OFFSET_TIME 36     /// the time of fix offset in the payload.
#define OFFSET_HEIGHT 46   /// the GNSS height offset in the payload.
#define OFFSET_HPL 50      /// the horizontal protection limit offset in the payload.
#define OFFSET_HFOM 54     /// the horizontal figure of merit offset in the payload.
#define OFFSET_VFOM 58     /// the vertical figure of merit offset in the payload.
#define OFFSET_NACV 62     /// the navigation accuracy for velocity offset in the payload.

#define LEN_LNG 11  /// bytes in the longitude field
#define LEN_LAT 10  /// bytes in the latitude field
#define LEN_SPD 6   /// bytes in the speed over ground field
#define LEN_TRK 8   /// bytes in the ground track field
#define LEN_TIME 10 /// bytes in the time of fix field

/*
 * Documented in the header file.
 */
bool sgEncodeGPS(uint8_t *buffer, sg_gps_t *gps, uint8_t msgId)
{
    // populate header
    buffer[0] = SG_MSG_START_BYTE;
    buffer[1] = SG_MSG_TYPE_HOST_GPS;
    buffer[2] = msgId;
    buffer[3] = SG_PAYLOAD_LEN_GPS;

    // populate longitude
    charArray2Buf(&buffer[PBASE + OFFSET_LONGITUDE], gps->longitude, LEN_LNG);

    // populate latitude
    charArray2Buf(&buffer[PBASE + OFFSET_LATITUDE], gps->latitude, LEN_LAT);

    // populate ground speed
    charArray2Buf(&buffer[PBASE + OFFSET_SPEED], gps->grdSpeed, LEN_SPD);

    // populate ground track
    charArray2Buf(&buffer[PBASE + OFFSET_TRACK], gps->grdTrack, LEN_TRK);

    // populate hemisphere/data status
    buffer[PBASE + OFFSET_STATUS] = !gps->gpsValid << 7 |
                                    gps->fdeFail << 6 |
                                    gps->lngEast << 1 |
                                    gps->latNorth;

    // populate time of fix
    charArray2Buf(&buffer[PBASE + OFFSET_TIME], gps->timeOfFix, LEN_TIME);

    // populate gnss height
    float2Buf(&buffer[PBASE + OFFSET_HEIGHT], gps->height);

    // populate HPL
    float2Buf(&buffer[PBASE + OFFSET_HPL], gps->hpl);

    // populate HFOM
    float2Buf(&buffer[PBASE + OFFSET_HFOM], gps->hfom);

    // populate VFOM
    float2Buf(&buffer[PBASE + OFFSET_VFOM], gps->vfom);

    // populate NACv
    buffer[PBASE + OFFSET_NACV] = gps->nacv << 4;

    // populate checksum
    appendChecksum(buffer, SG_MSG_LEN_GPS);

    return true;
}
