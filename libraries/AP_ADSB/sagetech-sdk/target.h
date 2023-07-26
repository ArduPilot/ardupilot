/**
 * @copyright Copyright (c) 2020 Sagetech, Inc. All rights reserved.
 *
 * @File:   target.h
 * @Author: jim billmeyer
 *
 * @date    December 11, 2020, 12:49 AM
 */

#ifndef TARGET_H
#define TARGET_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define XPNDR_ADSB_TARGETS 400 // change this to the max number of
                               // target supported in the system.

typedef enum
{
    trafLevel,
    trafClimb,
    trafDescend
} targetclimb_t;

typedef enum
{
    trafTraffic,
    trafAdvisory,
    trafResolution
} targetalert_t;

// bit 0    - target found flag.
// bit 1    - target slot in use.
// bits 2-7 - the strike counter.
#define TARGET_FLAG_FOUND 0x01
#define TARGET_FLAG_USED 0x02
#define TARGET_FLAG_STRIKE_MASK 0xFC

typedef struct __attribute__((packed))
{
    uint32_t icao;
    bool airborne;
    float bearing;
    uint8_t distance;
    int8_t altDiff;
    int16_t nvel;
    int16_t evel;
    targetclimb_t climb;
    targetalert_t alert;
#ifdef TARGET_SVR
    msg_svr_t svr;
#endif
    uint8_t flag; // used internally to purge stale targets.
} target_t;

typedef struct
{
    uint32_t icao;
    bool airborne;
    float lat;
    float lon;
    int32_t alt;
    int16_t heading;
    uint16_t speed;
} ownship_t;

/**
 * Gets the target list.
 *
 * @return The array of traffic targets.
 */
target_t *targetList(void);

/**
 * Gets the ownship target information.
 *
 * @return The ownship target info.
 */
ownship_t *targetOwnship(void);

/**
 * Find a target based on its icao number.
 *
 * @param icao The target's icao number
 *
 * @return A pointer to the target element or null if not found.
 */
target_t *targetFind(uint32_t icao);

/**
 * Purge the traffic target list of stale traffic.
 *
 * The traffic gets purged if a find has not been done based
 * on a strike counter.
 */
void targetPurge(void);

/**
 * Adds a target to the traffic target list.
 *
 * @param target The target to add.
 */
void targetAdd(target_t *target);

/**
 * Gets the target climb flag based on the vertical rate.
 *
 * @param vrate The current vertical rate of climb for the target.
 *
 * @return The level, climb or descend flag.
 */
targetclimb_t targetClimb(int16_t vrate);

/**
 * Gets the traffic alert flag.
 *
 * @param dist The distance of the target to the ownship.
 * @param alt  The altitude difference between the target and ownship.
 * @param nvel The NS speed vector of the target.
 * @param evel The EW speed vector of the target.
 *
 * @return The traffic flag based on the parameters.
 */
targetalert_t targetAlert(double dist,
                          uint16_t alt,
                          int16_t nvel,
                          int16_t evel);

#endif /* TARGET_H */
