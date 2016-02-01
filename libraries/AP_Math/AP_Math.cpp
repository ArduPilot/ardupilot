#include "AP_Math.h"
#include <float.h>


#if ROTATION_COMBINATION_SUPPORT
// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
// Note that this relies the set of rotations being complete. The
// optional 'found' parameter is for the test suite to ensure that it is.
enum Rotation rotation_combination(enum Rotation r1, enum Rotation r2, bool *found)
{
    Vector3f tv1, tv2;
    enum Rotation r;
    tv1(1,2,3);
    tv1.rotate(r1);
    tv1.rotate(r2);

    for (r=ROTATION_NONE; r<ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        Vector3f diff;
        tv2(1,2,3);
        tv2.rotate(r);
        diff = tv1 - tv2;
        if (diff.length() < 1.0e-6f) {
            // we found a match
            if (found) {
                *found = true;
            }
            return r;
        }
    }

    // we found no matching rotation. Someone has edited the
    // rotations list and broken its completeness property ...
    if (found) {
        *found = false;
    }
    return ROTATION_NONE;
}
#endif

