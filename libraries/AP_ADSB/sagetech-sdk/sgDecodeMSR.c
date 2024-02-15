/**
 * @copyright Copyright (c) 2020 Sagetech, Inc. All rights reserved.
 *
 * @file sgDecodeMSR.c
 * @author jim.billmeyer
 *
 * @date Mar 17, 2021
 */

#include "sg.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sgUtil.h"

#define MS_PARAM_TOA (1 << 3)
#define MS_PARAM_ADSBVER (1 << 2)
#define MS_PARAM_CALLSIGN (1 << 1)
#define MS_PARAM_CATEMITTER (1 << 0)

#define MS_PARAM_AVLEN (1 << 7)
#define MS_PARAM_PRIORITY (1 << 6)
#define MS_PARAM_CAPCODES (1 << 5)
#define MS_PARAM_OPMODE (1 << 4)
#define MS_PARAM_NACP (1 << 3)
#define MS_PARAM_NACV (1 << 2)
#define MS_PARAM_SDA (1 << 1)
#define MS_PARAM_GVA (1 << 0)

#define MS_PARAM_NIC (1 << 7)
#define MS_PARAM_HEADING (1 << 6)
#define MS_PARAM_VRATE (1 << 5)

#define MS_CAP_B2LOW (1 << 3)
#define MS_CAP_UAT (1 << 1)
#define MS_CAP_TCR ((1 << 2) | (1 << 3))
#define MS_CAP_TSR (1 << 4)
#define MS_CAP_ARV (1 << 5)
#define MS_CAP_ADSB (1 << 6)
#define MS_CAP_TCAS (1 << 7)

#define MS_OP_GPS_LATFMT (1 << 7)
#define MS_OP_GPS_LONFMT (1 << 6)
#define MS_OP_TCAS_RA (1 << 5)
#define MS_OP_IDENT (1 << 4)
#define MS_OP_SINGLE_ANT (1 << 2)

/// the payload offset.
#define PBASE 4

/*
 * Documented in the header file.
 */
bool sgDecodeMSR(uint8_t *buffer, sg_msr_t *msr)
{
    memset(msr, 0, sizeof(sg_msr_t));

    uint8_t fields[3];
    memcpy(fields, &buffer[PBASE + 0], 3);

    if (buffer[PBASE + 1] == 0x6E && buffer[PBASE + 2] == 0x60)
    {
        msr->type = msrTypeV0;
    }
    else if (buffer[PBASE + 1] == 0x7E && buffer[PBASE + 2] == 0xE0)
    {
        msr->type = msrTypeV1Airborne;
    }
    else if (buffer[PBASE + 1] == 0xFE && buffer[PBASE + 2] == 0xE0)
    {
        msr->type = msrTypeV1Surface;
    }
    else if (buffer[PBASE + 1] == 0x7F && buffer[PBASE + 2] == 0xE0)
    {
        msr->type = msrTypeV2Airborne;
    }
    else if (buffer[PBASE + 1] == 0xFF && buffer[PBASE + 2] == 0xE0)
    {
        msr->type = msrTypeV2Surface;
    }

    msr->flags = buffer[PBASE + 3];
    msr->addr = toInt32(&buffer[PBASE + 4]) & 0xFFFFFF;
    msr->addrType = buffer[PBASE + 7] & 0xFF;

    uint8_t ofs = 8;

    if (fields[0] & MS_PARAM_TOA)
    {
        msr->toa = toTOA(&buffer[PBASE + ofs]);
        ofs += 2;
    }

    if (fields[0] & MS_PARAM_ADSBVER)
    {
        msr->version = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[0] & MS_PARAM_CALLSIGN)
    {
        memset(msr->callsign, 0, 9);
        memcpy(msr->callsign, &buffer[PBASE + ofs], 8);
        ofs += 8;
    }

    if (fields[0] & MS_PARAM_CATEMITTER)
    {
        msr->emitter = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[1] & MS_PARAM_AVLEN)
    {
        msr->size = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[1] & MS_PARAM_PRIORITY)
    {
        msr->priority = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[1] & MS_PARAM_CAPCODES)
    {
        uint8_t cap = buffer[PBASE + ofs + 0];
        msr->capability.b2low = cap & MS_CAP_B2LOW;

        cap = buffer[PBASE + ofs + 1];
        msr->capability.uat = cap & MS_CAP_UAT;
        msr->capability.tcr = (cap & MS_CAP_TCR) >> 2;
        msr->capability.tsr = cap & MS_CAP_TSR;
        msr->capability.arv = cap & MS_CAP_ARV;
        msr->capability.adsb = cap & MS_CAP_ADSB;
        msr->capability.tcas = cap & MS_CAP_TCAS;

        ofs += 3;
    }

    if (fields[1] & MS_PARAM_OPMODE)
    {
        uint8_t op = buffer[PBASE + ofs + 0];
        msr->opMode.gpsLatFmt = (op & MS_OP_GPS_LATFMT) == 0;
        msr->opMode.gpsLonFmt = (op & MS_OP_GPS_LONFMT) == 0;
        msr->opMode.tcasRA = op & MS_OP_TCAS_RA;
        msr->opMode.ident = op & MS_OP_IDENT;
        msr->opMode.singleAnt = op & MS_OP_SINGLE_ANT;

        op = buffer[PBASE + ofs + 1];
        msr->opMode.gpsLatOfs = op >> 5;
        msr->opMode.gpsLonOfs = op & 0x17;

        ofs += 2;
    }

    if (fields[1] & MS_PARAM_NACP)
    {
        msr->svQuality.nacp = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[1] & MS_PARAM_NACV)
    {
        msr->svQuality.nacv = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[1] & MS_PARAM_SDA)
    {
        uint8_t sda = buffer[PBASE + ofs];
        msr->svQuality.sda = (sda & 0x18) >> 3;
        msr->svQuality.silSupp = (sda & 0x04);
        msr->svQuality.sil = (sda & 0x03);
        ofs++;
    }

    if (fields[1] & MS_PARAM_GVA)
    {
        msr->svQuality.gva = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[2] & MS_PARAM_NIC)
    {
        msr->svQuality.nicBaro = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[2] & MS_PARAM_HEADING)
    {
        msr->trackHeading = buffer[PBASE + ofs];
        ofs++;
    }

    if (fields[2] & MS_PARAM_VRATE)
    {
        msr->vrateType = buffer[PBASE + ofs];
        ofs++;
    }

    return true;
}
