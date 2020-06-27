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


#ifndef ISQRT_3
    #define ISQRT_3		    0.5773503f
#endif
#ifndef SQRT_3
    #define SQRT_3          1.7320508f
#endif

#ifndef SQR
# define SQR(x)          ((x) * (x))
#endif

#ifndef SGN
    #define SGN(x)          (((x) < 0) ? -1 : 1)
#endif

#ifndef MIN
# define MIN(x,y)       (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
# define MAX(x,y)       (((x) > (y)) ? (x) : (y))
#endif

// Leap seconds as of 12/31/2016
#ifndef LEAP_SECONDS
    #define LEAP_SECONDS    18
#endif

/*!
 * Bound a value to be >= a minimum and <= a maximum
 * \param min is the minimum value allowed
 * \param value is the variable to bound
 * \param max is the maximum value allowed
 * \return value, bound to be between min and max
 */
#define BOUND(min,value,max)    MAX(min, MIN(value, max))

#define SATURATE(value, max)  BOUND(-(max), (value), (max))

#ifndef rad2deg
    #define rad2deg(rad)  ((rad)*180.0/PId)
#endif
#ifndef rad2degf
    #define rad2degf(rad) ((rad)*180.0f/PIf)
#endif
#ifndef deg2rad
    #define deg2rad(deg)  ((deg)*PId/180.0)
#endif
#ifndef deg2radf
    #define deg2radf(deg) ((deg)*PIf/180.0f)
#endif

#ifndef radians
    #define radians deg2rad
#endif
#ifndef radiansf
    #define radiansf deg2radf
#endif
#ifndef degrees
    #define degrees rad2deg
#endif
#ifndef degreesf
    #define degreesf rad2degf
#endif

#endif // CONSTANTS_H
