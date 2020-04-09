#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "OrionPublicPacket.h"

#ifndef PI
# define PI              3.1415926535897932384626433832795f
#endif

#ifndef PIf
# define PIf             3.1415926535897932384626433832795f
#endif

#ifndef PId
# define PId             3.1415926535897932384626433832795
#endif

#define ISQRT_3		    0.5773503f
#define SQRT_3          1.7320508f

#ifndef SQR
# define SQR(x)          ((x) * (x))
#endif

#define SGN(x)          (((x) < 0) ? -1 : 1)

#ifndef MIN
# define MIN(x,y)       (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
# define MAX(x,y)       (((x) > (y)) ? (x) : (y))
#endif

// Leap seconds as of 12/31/2016
#define LEAP_SECONDS    18

/*!
 * Bound a value to be >= a minimum and <= a maximum
 * \param min is the minimum value allowed
 * \param value is the variable to bound
 * \param max is the maximum value allowed
 * \return value, bound to be between min and max
 */
#define BOUND(min,value,max)    MAX(min, MIN(value, max))

#define SATURATE(value, max)  BOUND(-(max), (value), (max))

#define rad2deg(rad)  ((rad)*180.0/PId)
#define rad2degf(rad) ((rad)*180.0f/PIf)
#define deg2rad(deg)  ((deg)*PId/180.0)
#define deg2radf(deg) ((deg)*PIf/180.0f)

#define radians deg2rad
#define radiansf deg2radf
#define degrees rad2deg
#define degreesf rad2degf

#endif // CONSTANTS_H
