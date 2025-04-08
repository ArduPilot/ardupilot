/* SpeedToFly class by Samuel Tabor, 2021.

Calculates the optimal speed to fly given drag polar, expected climb rate in next thermal and
horizontal and vertical air movement between thermals.
*/
#include "SpeedToFly.h"

void SpeedToFly::update(float Wx, float Wz, float Wexp, float CLmin, float CLmax)
{
    // The solution to the speed-to-fly problem does not have a closed form solution. A Newton
    // method with some additional checks will converge to an acceptable level within 3-4 iterations.
    // However, to keep the computation constant per function call we just do a single iteration using
    // the previous approximation as a starting point.
    // This gives good accuracy as the inputs don't change rapidly. It would also be possible to store 
    // the inputs and converge the solution over 3-4 function calls, but this real-time iteration
    // approach gives better accuracy in tests as well as simpler code.

    Wz *= -1.0f; // Sink defined positive.

    float sqrtfk   = sqrtf(_polarParams.K);
    float minSink = (sqrtfk/sqrtf(CLmax)*(_polarParams.CD0 + _polarParams.B*CLmax*CLmax))/CLmax;

    if (!is_positive(minSink+Wz+Wexp)) {
        // Special case. If lift is greater than min sink speed, fly at min sink
        // speed.
        _CL_estimate = CLmax;
        return;
    }

    float CD0 = _polarParams.CD0;
    float B   = _polarParams.B;
    float Wxp =  Wx/sqrtfk;
    float WZ  = (Wz + Wexp)/sqrtfk;

    // Guess starting location.
    float CL  = _CL_estimate>0 ? _CL_estimate : 0.5f*(CLmax+CLmin);

    float t0 = powf(CL,1.5f);
    float t1 = CD0 + B*CL*CL + t0*WZ;
    float t2 = 1.5f*sqrtf(CL)*WZ + 2.0f*B*CL;
    
    float Jd  = (1.5f*sqrtf(CL)*Wxp + 1.0f)/t1 - (t2*(CL + t0*Wxp))/(t1*t1);
    
    float Jdd = 2.0f*t2*t2*(CL + t0*Wxp)/powf(t1,3) - (2.0f*t2*(1.5f*sqrtf(CL)*Wxp + 1.0f))/(t1*t1) - ((2.0f*B + 0.75f*WZ/sqrtf(CL))*(CL + t0*Wxp))/(t1*t1) + 0.75f*Wxp/(sqrtf(CL)*t1);

    // Check we're heading to a maximum, not a minimum!!
    if (is_positive(Jdd)) {
        // Alternate mode, go uphill.
        CL = CL + 0.1 * (Jd>0.0f ? 1.0f : -1.0f);
    } else {
        // Newton should work.
        CL = CL - Jd/Jdd;
    }

    _CL_estimate = CL;
    _CL_estimate = constrain_float(_CL_estimate, CLmin, CLmax);
}
