/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file toHeading2.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *
 */

#include <math.h>

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
int16_t toHeading2(double y, double x)
{
    int16_t heading = (toDeg(atan2(y, x)) + 0.5);
    heading = 360 - heading + 90; // atan is ccw 0 degrees at x = 1 and y = 0.

    if (heading > 360)
    {
        heading -= 360;
    }
    else
    {
        while (heading < 0)
        {
            heading += 360;
        }
    }

    return heading;
}
