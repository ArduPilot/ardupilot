#include "quaternion.h"
#include "mathutilities.h"
#include <math.h>

#ifndef SQRT_2
#define SQRT_2 1.4142135623730950488016887242097f
#endif

/*!
 * Convert a quaternion to a DCM
 * \param quat is the quaternion to convert
 * \param dcm receives the direction cosine matrix
 */
void quaternionToDCM(const float quat[NQUATERNION], DCM_t* dcm)
{
    // squares of the quaternion elements
    float q0sq = SQR(quat[Q0]);
    float q1sq = SQR(quat[Q1]);
    float q2sq = SQR(quat[Q2]);
    float q3sq = SQR(quat[Q3]);

    // This form taken from Groves
    dcmSet(dcm, 0, 0, q0sq + q1sq - q2sq - q3sq);
    dcmSet(dcm, 0, 1, 2*(quat[Q1]*quat[Q2] - quat[Q3]*quat[Q0]));
    dcmSet(dcm, 0, 2, 2*(quat[Q1]*quat[Q3] + quat[Q2]*quat[Q0]));

    dcmSet(dcm, 1, 0, 2*(quat[Q1]*quat[Q2] + quat[Q3]*quat[Q0]));
    dcmSet(dcm, 1, 1, q0sq - q1sq + q2sq - q3sq);
    dcmSet(dcm, 1, 2, 2*(quat[Q2]*quat[Q3] - quat[Q1]*quat[Q0]));

    dcmSet(dcm, 2, 0, 2*(quat[Q1]*quat[Q3] - quat[Q2]*quat[Q0]));
    dcmSet(dcm, 2, 1, 2*(quat[Q2]*quat[Q3] + quat[Q1]*quat[Q0]));
    dcmSet(dcm, 2, 2, q0sq - q1sq - q2sq + q3sq);

}// quaternionToDCM


/*!
 * Convert a DCM to quaternion
 * \param dcm is the direction cosine matrix to convert
 * \param quat receives the quaternion
 */
void dcmToQuaternion(const DCM_t* dcm, float quat[NQUATERNION])
{
    const float* dcmdata = dcm->data;
    float mult;
    int imax, i;

    // The diagonal terms, quat is a temporary here. Notice the fabs. This
    // protects agains sqrt of negative. If the argument would have been
    // negative then the fabs changes the sign of all terms of the quat. This is
    // allowable as inverting all 4 elements does not change the rotation.
    //
    // This code is derived from:
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    quat[Q0] = fabsf(1 + dcmdata[T11] + dcmdata[T22] + dcmdata[T33]);
    quat[Q1] = fabsf(1 + dcmdata[T11] - dcmdata[T22] - dcmdata[T33]);
    quat[Q2] = fabsf(1 - dcmdata[T11] + dcmdata[T22] - dcmdata[T33]);
    quat[Q3] = fabsf(1 - dcmdata[T11] - dcmdata[T22] + dcmdata[T33]);

    // Find the largest one
    imax = 0;
    for(i = 1; i < 4; i++)
    {
        if(quat[i] > quat[imax])
            imax = i;
    }

    // The largest term
    quat[imax] = 0.5f*sqrtf(quat[imax]);

    // Multiplier on all the other elements. It is this division here that leads
    // to singularities if we don't select the largest quaternion element
    mult = 0.25f/quat[imax];

    // Compute the remaining terms from the off-diagonal elements. The equation
    // is a function of which term we computed from the diagonal element.
    switch(imax)
    {
    default:
    case Q0:
        quat[Q1] = mult*(dcmdata[T32] - dcmdata[T23]);
        quat[Q2] = mult*(dcmdata[T13] - dcmdata[T31]);
        quat[Q3] = mult*(dcmdata[T21] - dcmdata[T12]);
        break;
    case Q1:
        quat[Q0] = mult*(dcmdata[T32] - dcmdata[T23]);
        quat[Q2] = mult*(dcmdata[T21] + dcmdata[T12]);
        quat[Q3] = mult*(dcmdata[T13] + dcmdata[T31]);
        break;
    case Q2:
        quat[Q0] = mult*(dcmdata[T13] - dcmdata[T31]);
        quat[Q1] = mult*(dcmdata[T21] + dcmdata[T12]);
        quat[Q3] = mult*(dcmdata[T32] + dcmdata[T23]);
        break;
    case Q3:
        quat[Q0] = mult*(dcmdata[T21] - dcmdata[T12]);
        quat[Q1] = mult*(dcmdata[T13] + dcmdata[T31]);
        quat[Q2] = mult*(dcmdata[T32] + dcmdata[T23]);
        break;
    }

    // Finally, we would like the leading element to be positive
    if(quat[Q0] < 0.0f)
    {
        // Same rotation if all signs reversed.
        quat[Q0] *= -1;
        quat[Q1] *= -1;
        quat[Q2] *= -1;
        quat[Q3] *= -1;
    }

}// dcmToQuaternion


/*!
 * Fill out the quaternion from an Euler roll angle
 * \param quat is filled out according to the angle
 * \param roll is the Euler roll angle in radians
 */
void setQuaternionBasedOnRoll(float quat[NQUATERNION], float roll)
{
    quat[Q0] = (SQRT_2*0.5f)*sqrtf(1 + cosf(roll));
    quat[Q1] = 0.5f*sinf(roll)/quat[Q0];
    quat[Q2] = 0.0f;
    quat[Q3] = 0.0f;
}

/*!
 * Fill out the quaternion from an Euler pitch angle
 * \param quat is filled out according to the angle
 * \param pitch is the Euler Pitch angle in radians
 */
void setQuaternionBasedOnPitch(float quat[NQUATERNION], float pitch)
{
    quat[Q0] = (SQRT_2*0.5f)*sqrtf(1 + cosf(pitch));
    quat[Q1] = 0.0f;
    quat[Q2] = 0.5f*sinf(pitch)/quat[Q0];
    quat[Q3] = 0.0f;
}

/*!
 * Fill out the quaternion from an Euler yaw angle
 * \param quat is filled out according to the angle
 * \param yaw is the Euler Yaw angle in radians
 */
void setQuaternionBasedOnYaw(float quat[NQUATERNION], float yaw)
{
    quat[Q0] = (SQRT_2*0.5f)*sqrtf(1 + cosf(yaw));
    quat[Q1] = 0.0f;
    quat[Q2] = 0.0f;
    quat[Q3] = 0.5f*sinf(yaw)/quat[Q0];
}


/*!
 * Fill out the quaternion from an Euler, yaw, then pitch, then roll rotation
 * \param quat is filled out according to the angles.
 * \param yaw is the Euler yaw angle in radians
 * \param pitch is the Euler pitch angle in radians
 * \param roll is the Euler roll angle in radians
 */
void setQuaternionBasedOnEuler(float quat[NQUATERNION], float yaw, float pitch, float roll)
{
    stackAllocateDCM(dcm);

    setDCMBasedOnEuler(&dcm, yaw, pitch, roll);

    dcmToQuaternion(&dcm, quat);
}


/*!
 * Compute the Euler yaw angle of a quaternion.
 * \param quat is the body to reference quaternion.
 * \return the Euler yaw angle in radians, from -PI to PI.
 */
float quaternionYaw(const float quat[NQUATERNION])
{
    return atan2f(2*(quat[Q1]*quat[Q2] + quat[Q3]*quat[Q0]), SQR(quat[Q0]) + SQR(quat[Q1]) - SQR(quat[Q2]) - SQR(quat[Q3]));

}// quaternionYaw


/*!
 * Compute the Euler pitch rotation of a quaternion.
 * \param quat is the body to reference quaternion.
 * \return the Euler pitch angle in radians, from -PI/2 to PI/2.
 */
float quaternionPitch(const float quat[NQUATERNION])
{
    return asinf(quaternionSinPitch(quat));

}// quaternionPitch


/*!
 * Compute the cosine of the Euler pitch angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the cosine of the Euler pitch angle.
 */
float quaternionCosPitch(const float quat[NQUATERNION])
{
    float sinp = quaternionSinPitch(quat);

    // pitch goes from -90 to 90, so cosine of pitch must be positive.
    return sqrtf(1.0f - sinp*sinp);

}// quaternionCosPitch


/*!
 * Compute the sin of the Euler pitch angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the sin of the Euler pitch angle.
 */
float quaternionSinPitch(const float quat[NQUATERNION])
{
    return SATURATE(-2*(quat[Q1]*quat[Q3] - quat[Q2]*quat[Q0]), 1.0f);
}


/*!
 * Compute the Euler roll rotation.
 * \param quat is the body to reference quaternion.
 * \return the Euler roll angle in radians, from -PI to PI.
 */
float quaternionRoll(const float quat[NQUATERNION])
{
    return atan2f(2*(quat[Q2]*quat[Q3] + quat[Q1]*quat[Q0]), SQR(quat[Q0]) - SQR(quat[Q1]) - SQR(quat[Q2]) + SQR(quat[Q3]));

}// roll


/*!
 * Compute the cosine of the Euler roll angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the cosine of the Euler roll angle.
 */
float quaternionCosRoll(const float quat[NQUATERNION])
{
    return cosf(quaternionRoll(quat));
}


/*!
 * Compute the sin of the Euler roll angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the sin of the Euler roll angle.
 */
float quaternionSinRoll(const float quat[NQUATERNION])
{
    return sinf(quaternionRoll(quat));
}


/*!
 * Use a quaternion to rotate a vector. The input and output
 * vector can be the same vector
 * \param quat is the quaternion.
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void quaternionApplyRotation(const float quat[NQUATERNION], const float input[], float output[])
{
    float zero, one, two;

    stackAllocateDCM(rotation);

    quaternionToDCM(quat, &rotation);

    // Doing it this way makes it possible for input and output to be the same vector
    zero = rotationdata[0]*input[0] + rotationdata[1]*input[1] + rotationdata[2]*input[2];
    one  = rotationdata[3]*input[0] + rotationdata[4]*input[1] + rotationdata[5]*input[2];
    two  = rotationdata[6]*input[0] + rotationdata[7]*input[1] + rotationdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// quaternionApplyRotation


/*!
 * Use a quaternion to rotate a vector, in the reverse direction. The input and output
 * vector can be the same vector.
 * \param quat is the quaternion.
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void quaternionApplyReverseRotation(const float quat[NQUATERNION], const float input[], float output[])
{
    float zero, one, two;

    stackAllocateDCM(rotation);

    quaternionToDCM(quat, &rotation);

    // Doing it this way makes it possible for input and output to be the same vector
    zero = rotationdata[0]*input[0] + rotationdata[3]*input[1] + rotationdata[6]*input[2];
    one  = rotationdata[1]*input[0] + rotationdata[4]*input[1] + rotationdata[7]*input[2];
    two  = rotationdata[2]*input[0] + rotationdata[5]*input[1] + rotationdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// quaternionApplyReverseRotation


/*!
 * Compute the length of a quaternion which should be 1.0
 * \param quat is the quaternion whose length is computed
 * \return the length of the quaternion
 */
float quaternionLength(const float quat[NQUATERNION])
{
    return sqrtf(SQR(quat[Q0]) + SQR(quat[Q1]) + SQR(quat[Q2]) + SQR(quat[Q3]));
}


/*!
 * Test quaternion operations
 * \return TRUE if test passed
 */
BOOL testQuaternion(void)
{
    float error = 0.0f;
    float quat[NQUATERNION];
    stackAllocateDCM(dcm);

    matrixSetIdentityf(&dcm);
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += testForIdentityf(&dcm);

    setDCMBasedOnYaw(&dcm, deg2radf(13.5f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(13.5f) - quaternionYaw(quat));

    setDCMBasedOnPitch(&dcm, deg2radf(-27.5f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(-27.5f) - quaternionPitch(quat));

    setDCMBasedOnRoll(&dcm, deg2radf(160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(160.0f) - quaternionRoll(quat));

    setDCMBasedOnYaw(&dcm, deg2radf(160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(160.0f) - quaternionYaw(quat));

    setDCMBasedOnPitch(&dcm, deg2radf(85.5f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(85.5f) - quaternionPitch(quat));

    setQuaternionBasedOnYaw(quat, deg2radf(13.5f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(13.5f) - dcmYaw(&dcm));

    setQuaternionBasedOnPitch(quat, deg2radf(-27.5f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-27.5f) - dcmPitch(&dcm));

    setQuaternionBasedOnRoll(quat, deg2radf(-160.0f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-160.0f) - dcmRoll(&dcm));

    setQuaternionBasedOnYaw(quat, deg2radf(-160.0f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-160.0f) - dcmYaw(&dcm));

    setQuaternionBasedOnPitch(quat, deg2radf(-85.5f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-85.5f) - dcmPitch(&dcm));

    setDCMBasedOnEuler(&dcm, deg2radf(-13.5f), deg2radf(27.5f), deg2radf(-160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(-13.5f) - quaternionYaw(quat));
    error += fabsf(deg2radf(27.5f) - quaternionPitch(quat));
    error += fabsf(deg2radf(-160.0f) - quaternionRoll(quat));

    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-13.5f) - dcmYaw(&dcm));
    error += fabsf(deg2radf(27.5f) - dcmPitch(&dcm));
    error += fabsf(deg2radf(-160.0f) - dcmRoll(&dcm));

    setDCMBasedOnEuler(&dcm, deg2radf(-85.5f), deg2radf(75.0f), deg2radf(-160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(-85.5f) - quaternionYaw(quat));
    error += fabsf(deg2radf(75.0f) - quaternionPitch(quat));
    error += fabsf(deg2radf(-160.0f) - quaternionRoll(quat));

    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-85.5f) - dcmYaw(&dcm));
    error += fabsf(deg2radf(75.0f) - dcmPitch(&dcm));
    error += fabsf(deg2radf(-160.0f) - dcmRoll(&dcm));


    if(error < 0.001f)
        return TRUE;
    else
        return FALSE;
}

