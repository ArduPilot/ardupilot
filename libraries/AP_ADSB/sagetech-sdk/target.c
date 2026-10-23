/**
 * @copyright Copyright (c) 2020 SoftSolutions, Inc. All rights reserved.
 *
 * @File:   target.c
 * @Author: jim billmeyer
 *
 * @date    December 11, 2020, 12:50 AM
 */

#include <stdlib.h>
#include "target.h"

static ownship_t ownship;
static target_t targets[XPNDR_ADSB_TARGETS] = {
    {
        0,
    },
};

/*
 * Documented in the header file.
 */
target_t *targetList(void)
{
    return targets;
}

/*
 * Documented in the header file.
 */
ownship_t *targetOwnship(void)
{
    return &ownship;
}

/*
 * Documented in the header file.
 */
target_t *targetFind(uint32_t icao)
{
    for (uint16_t i = 0; i < XPNDR_ADSB_TARGETS; i++)
    {
        if (icao == targets[i].icao)
        {
            // clear strike counter and set find flag while preserving the used bit.
            targets[i].flag = TARGET_FLAG_FOUND | TARGET_FLAG_USED;
            return &targets[i];
        }
    }

    return 0;
}

/*
 * Documented in the header file.
 */
void targetPurge(void)
{
    for (uint16_t i = 0; i < XPNDR_ADSB_TARGETS; i++)
    {
        // the the found flag was not set increment the strike counter.
        if ((targets[i].flag & TARGET_FLAG_USED) && ((targets[i].flag & TARGET_FLAG_FOUND) == 0))
        {
            uint8_t strikes = (targets[i].flag & 0xFE) >> 2;
            strikes++;

            if (strikes > 5)
            {
                memset(&targets[i], 0, sizeof(target_t));
            }
            else
            {
                // set the strike counter and clear the found flag.
                targets[i].flag = strikes << 2 | TARGET_FLAG_USED;
            }
        }
        else
        {
            // clear the found flag so the target find function can set it
            // to signal that the icao address is still in range.
            targets[i].flag = targets[i].flag & (TARGET_FLAG_STRIKE_MASK | TARGET_FLAG_USED);
        }
    }
}

/*
 * Documented in the header file.
 */
void targetAdd(target_t *target)
{
    for (uint16_t i = 0; i < XPNDR_ADSB_TARGETS; i++)
    {
        if ((targets[i].flag & TARGET_FLAG_USED) == 0x0)
        {
            memcpy(&targets[i], target, sizeof(target_t));
            targets[i].flag = TARGET_FLAG_USED;
            break;
        }
    }
}

/*
 * Documented in the header file.
 */
targetclimb_t targetClimb(int16_t vrate)
{
    if (abs(vrate) < 500)
    {
        return trafLevel;
    }
    else if (vrate > 0)
    {
        return trafClimb;
    }
    else
    {
        return trafDescend;
    }
}

/*
 * Documented in the header file.
 */
targetalert_t targetAlert(double dist,
                          uint16_t alt,
                          int16_t nvel,
                          int16_t evel)
{
    if (alt <= 3000)
    {
        if (dist <= 3.0)
        {
            return trafResolution;
        }
        else if (dist <= 6.0)
        {
            return trafAdvisory;
        }
    }

    return trafTraffic;
}
