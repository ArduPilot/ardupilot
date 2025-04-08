/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sgDecodeSVR.c
 * @author jimb
 *
 * @date   Feb 10, 2021
 *
 * This file receives a raw ADS-B target state vector report message buffer and
 * parses the payload into a data struct.
 */

#include <string.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "sg.h"
#include "sgUtil.h"
#include "target.h"

// airborne surface
// -------- --------
#define SV_PARAM_TOA_EPOS (1 << 3) // x
#define SV_PARAM_TOA_POS (1 << 2)  // x        x
#define SV_PARAM_TOA_VEL (1 << 1)  // x        x
#define SV_PARAM_LATLON (1 << 0)   // x        x

#define SV_PARAM_GEOALT (1 << 7)    // x
#define SV_PARAM_VEL (1 << 6)       // x
#define SV_PARAM_SURF_GS (1 << 5)   //          x
#define SV_PARAM_SURF_HEAD (1 << 4) //          x
#define SV_PARAM_BAROALT (1 << 3)   // x
#define SV_PARAM_VRATE (1 << 2)     // x
#define SV_PARAM_NIC (1 << 1)       // x        x
#define SV_PARAM_ESTLAT (1 << 0)    // x

#define SV_PARAM_ESTLON (1 << 7) // x
#define SV_PARAM_ESTNVEL (1 << 6)
#define SV_PARAM_ESTEVAL (1 << 5)
#define SV_PARAM_SURV (1 << 4)   // x        x
#define SV_PARAM_REPORT (1 << 3) // x        x

/// the payload offset.
#define PBASE 4

/*
 * Documented in the header file.
 */
bool sgDecodeSVR(uint8_t *buffer, sg_svr_t *svr)
{
    //   memset(svr, 0, sizeof(sg_svr_t));

    uint8_t fields[3];
    memcpy(&fields, &buffer[PBASE + 0], 3);

    svr->type = buffer[PBASE + 0] == 0x1F ? svrAirborne : svrSurface;
    svr->flags = buffer[PBASE + 3];
    svr->eflags = buffer[PBASE + 4];
    svr->addr = toInt32(&buffer[PBASE + 5]) & 0xFFFFFF;
    svr->addrType = buffer[PBASE + 8] & 0xFF;

    uint8_t ofs = 9;

    if (fields[0] & SV_PARAM_TOA_EPOS)
    {
        svr->toaEst = toTOA(&buffer[PBASE + ofs]);
        ofs += 2;
    }
    if (fields[0] & SV_PARAM_TOA_POS)
    {
        svr->toaPosition = toTOA(&buffer[PBASE + ofs]);
        ofs += 2;
    }
    if (fields[0] & SV_PARAM_TOA_VEL)
    {
        svr->toaSpeed = toTOA(&buffer[PBASE + ofs]);
        ofs += 2;
    }

    if (fields[0] & SV_PARAM_LATLON)
    {
        if (svr->validity.position)
        {
            svr->lat = toLatLon(&buffer[PBASE + ofs + 0]);
            svr->lon = toLatLon(&buffer[PBASE + ofs + 3]);
        }
        else
        {
            svr->lat = 0.0;
            svr->lon = 0.0;
        }

        ofs += 6;
    }

    if (svr->type == svrAirborne)
    {
        if (fields[1] & SV_PARAM_GEOALT)
        {
            if (svr->validity.geoAlt)
            {
                svr->airborne.geoAlt = toAlt(&buffer[PBASE + ofs]);
            }
            else
            {
                svr->airborne.geoAlt = 0;
            }

            ofs += 3;
        }

        if (fields[1] & SV_PARAM_VEL)
        {
            if (svr->validity.airSpeed)
            {
                int16_t nvel = toVel(&buffer[PBASE + ofs + 0]);
                int16_t evel = toVel(&buffer[PBASE + ofs + 2]);

                svr->airborne.heading = toHeading2((double)nvel, (double)evel);
                svr->airborne.speed = sqrt(nvel * nvel + evel * evel);
                svr->airborne.velEW = evel;
                svr->airborne.velNS = nvel;
            }
            else
            {
                svr->airborne.heading = 0;
                svr->airborne.speed = 0;
                svr->airborne.velEW = 0;
                svr->airborne.velNS = 0;
            }

            ofs += 4;
        }

        if (fields[1] & SV_PARAM_BAROALT)
        {
            if (svr->validity.baroAlt)
            {
                svr->airborne.baroAlt = toAlt(&buffer[PBASE + ofs]);
            }
            else
            {
                svr->airborne.baroAlt = 0;
            }

            ofs += 3;
        }

        if (fields[1] & SV_PARAM_VRATE)
        {
            if (svr->validity.baroVRate || svr->validity.geoVRate)
            {
                svr->airborne.vrate = toInt16(&buffer[PBASE + ofs]);
            }
            else
            {
                svr->airborne.vrate = 0;
            }

            ofs += 2;
        }
    }
    else
    {
        if (fields[1] & SV_PARAM_SURF_GS)
        {
            if (svr->validity.surfSpeed)
            {
                svr->surface.speed = toGS(&buffer[PBASE + ofs]);
            }
            else
            {
                svr->surface.speed = 0;
            }

            ofs += 1;
        }

        if (fields[1] & SV_PARAM_SURF_HEAD)
        {
            if (svr->validity.surfHeading)
            {
                svr->surface.heading = toHeading(&buffer[PBASE + ofs]);
            }
            else
            {
                svr->surface.heading = 0;
            }

            ofs += 1;
        }
    }

    if (fields[1] & SV_PARAM_NIC)
    {
        svr->nic = buffer[PBASE + ofs];

        ofs += 1;
    }

    if (fields[1] & SV_PARAM_ESTLAT)
    {
        if (svr->evalidity.estPosition)
        {
            svr->airborne.estLat = toLatLon(&buffer[PBASE + ofs]);
        }
        else
        {
            svr->airborne.estLat = 0;
        }

        ofs += 3;
    }

    if (fields[2] & SV_PARAM_ESTLON)
    {
        if (svr->evalidity.estPosition)
        {
            svr->airborne.estLon = toLatLon(&buffer[PBASE + ofs]);
        }
        else
        {
            svr->airborne.estLon = 0;
        }

        ofs += 3;
    }

    if (fields[2] & SV_PARAM_SURV)
    {
        svr->survStatus = buffer[PBASE + ofs];
        ofs += 1;
    }

    if (fields[2] & SV_PARAM_REPORT)
    {
        svr->mode = buffer[PBASE + ofs];
    }

    return true;
}
