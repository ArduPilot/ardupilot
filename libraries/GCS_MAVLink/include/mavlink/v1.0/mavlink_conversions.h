#ifndef  _MAVLINK_CONVERSIONS_H_
#define  _MAVLINK_CONVERSIONS_H_

/* enable math defines on Windows */
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>

#ifndef M_PI_2
    #define M_PI_2 ((float)asin(1))
#endif

/**
 * @file mavlink_conversions.h
 *
 * These conversion functions follow the NASA rotation standards definition file
 * available online.
 *
 * Their intent is to lower the barrier for MAVLink adopters to use gimbal-lock free
 * (both rotation matrices, sometimes called DCM, and quaternions are gimbal-lock free)
 * rotation representations. Euler angles (roll, pitch, yaw) will be phased out of the
 * protocol as widely as possible.
 *
 * @author James Goppert
 */

MAVLINK_HELPER void mavlink_quaternion_to_dcm(const float quaternion[4], float dcm[3][3])
{
    double a = quaternion[0];
    double b = quaternion[1];
    double c = quaternion[2];
    double d = quaternion[3];
    double aSq = a * a;
    double bSq = b * b;
    double cSq = c * c;
    double dSq = d * d;
    dcm[0][0] = aSq + bSq - cSq - dSq;
    dcm[0][1] = 2.0 * (b * c - a * d);
    dcm[0][2] = 2.0 * (a * c + b * d);
    dcm[1][0] = 2.0 * (b * c + a * d);
    dcm[1][1] = aSq - bSq + cSq - dSq;
    dcm[1][2] = 2.0 * (c * d - a * b);
    dcm[2][0] = 2.0 * (b * d - a * c);
    dcm[2][1] = 2.0 * (a * b + c * d);
    dcm[2][2] = aSq - bSq - cSq + dSq;
}

MAVLINK_HELPER void mavlink_dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw)
{
    float phi, theta, psi;
    theta = asin(-dcm[2][0]);

    if (fabsf(theta - (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = (atan2f(dcm[1][2] - dcm[0][1],
                dcm[0][2] + dcm[1][1]) + phi);

    } else if (fabsf(theta + (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = atan2f(dcm[1][2] - dcm[0][1],
                  dcm[0][2] + dcm[1][1] - phi);

    } else {
        phi = atan2f(dcm[2][1], dcm[2][2]);
        psi = atan2f(dcm[1][0], dcm[0][0]);
    }

    *roll = phi;
    *pitch = theta;
    *yaw = psi;
}

MAVLINK_HELPER void mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw)
{
    float dcm[3][3];
    mavlink_quaternion_to_dcm(quaternion, dcm);
    mavlink_dcm_to_euler(dcm, roll, pitch, yaw);
}

MAVLINK_HELPER void mavlink_euler_to_quaternion(float roll, float pitch, float yaw, float quaternion[4])
{
    double cosPhi_2 = cos((double)roll / 2.0);
    double sinPhi_2 = sin((double)roll / 2.0);
    double cosTheta_2 = cos((double)pitch / 2.0);
    double sinTheta_2 = sin((double)pitch / 2.0);
    double cosPsi_2 = cos((double)yaw / 2.0);
    double sinPsi_2 = sin((double)yaw / 2.0);
    quaternion[0] = (cosPhi_2 * cosTheta_2 * cosPsi_2 +
            sinPhi_2 * sinTheta_2 * sinPsi_2);
    quaternion[1] = (sinPhi_2 * cosTheta_2 * cosPsi_2 -
            cosPhi_2 * sinTheta_2 * sinPsi_2);
    quaternion[2] = (cosPhi_2 * sinTheta_2 * cosPsi_2 +
            sinPhi_2 * cosTheta_2 * sinPsi_2);
    quaternion[3] = (cosPhi_2 * cosTheta_2 * sinPsi_2 -
            sinPhi_2 * sinTheta_2 * cosPsi_2);
}

MAVLINK_HELPER void mavlink_dcm_to_quaternion(const float dcm[3][3], float quaternion[4])
{
    quaternion[0] = (0.5 * sqrt(1.0 +
            (double)(dcm[0][0] + dcm[1][1] + dcm[2][2])));
    quaternion[1] = (0.5 * sqrt(1.0 +
            (double)(dcm[0][0] - dcm[1][1] - dcm[2][2])));
    quaternion[2] = (0.5 * sqrt(1.0 +
            (double)(-dcm[0][0] + dcm[1][1] - dcm[2][2])));
    quaternion[3] = (0.5 * sqrt(1.0 +
            (double)(-dcm[0][0] - dcm[1][1] + dcm[2][2])));
}

MAVLINK_HELPER void mavlink_euler_to_dcm(float roll, float pitch, float yaw, float dcm[3][3])
{
    double cosPhi = cos(roll);
    double sinPhi = sin(roll);
    double cosThe = cos(pitch);
    double sinThe = sin(pitch);
    double cosPsi = cos(yaw);
    double sinPsi = sin(yaw);

    dcm[0][0] = cosThe * cosPsi;
    dcm[0][1] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
    dcm[0][2] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

    dcm[1][0] = cosThe * sinPsi;
    dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
    dcm[1][2] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

    dcm[2][0] = -sinThe;
    dcm[2][1] = sinPhi * cosThe;
    dcm[2][2] = cosPhi * cosThe;
}

#endif
