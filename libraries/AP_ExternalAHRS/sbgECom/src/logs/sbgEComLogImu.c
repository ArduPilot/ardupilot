#include "sbgEComLogImu.h"

//----------------------------------------------------------------------//
//- Private constant definitions                                       -//
//----------------------------------------------------------------------//

/*!
 * Standard gyroscope scale factor.
 */
#define SBG_ECOM_LOG_IMU_GYRO_SCALE_STD                     (67108864.0f)

/*!
 * High range gyroscope scale factor derived from 10000 degrees per second maximum range.
 *
 * Calculation: (2^31 - 1) / (10000 * π / 180)
 */
#define SBG_ECOM_LOG_IMU_GYRO_SCALE_HIGH                    (12304174.0f)

/*!
 * Maximum value for the standard scale factor, in radians per second.
 *
 * Approximately equivalent to 1833 °/s.
 */
#define SBG_ECOM_LOG_IMU_GYRO_SCALE_STD_MAX_RAD             ((float)INT32_MAX / SBG_ECOM_LOG_IMU_GYRO_SCALE_STD)

/*!
 * Standard accelerometer scale factor.
 */
#define SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD                    (1048576.0f)

/*!
 * Standard temperature scale factor.
 */
#define SBG_ECOM_LOG_IMU_TEMP_SCALE_STD                     (256.0f)

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogImuShortReadFromStream(SbgEComLogImuShort *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pStreamBuffer);
    assert(pLogData);

    pLogData->timeStamp         = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->status            = sbgStreamBufferReadUint16LE(pStreamBuffer);

    pLogData->deltaVelocity[0]  = sbgStreamBufferReadInt32LE(pStreamBuffer);
    pLogData->deltaVelocity[1]  = sbgStreamBufferReadInt32LE(pStreamBuffer);
    pLogData->deltaVelocity[2]  = sbgStreamBufferReadInt32LE(pStreamBuffer);

    pLogData->deltaAngle[0]     = sbgStreamBufferReadInt32LE(pStreamBuffer);
    pLogData->deltaAngle[1]     = sbgStreamBufferReadInt32LE(pStreamBuffer);
    pLogData->deltaAngle[2]     = sbgStreamBufferReadInt32LE(pStreamBuffer);

    pLogData->temperature       = sbgStreamBufferReadInt16LE(pStreamBuffer);

    return sbgStreamBufferGetLastError(pStreamBuffer);
}

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

float sbgEComLogImuShortGetDeltaAngle(const SbgEComLogImuShort *pImuShort, size_t idx)
{
    float                                scaleFactor = SBG_ECOM_LOG_IMU_GYRO_SCALE_STD;

    assert(pImuShort);
    assert(idx < 3);

    if (pImuShort->status & SBG_ECOM_IMU_GYROS_USE_HIGH_SCALE)
    {
        scaleFactor = SBG_ECOM_LOG_IMU_GYRO_SCALE_HIGH;
    }

    return pImuShort->deltaAngle[idx] / scaleFactor;
}

float sbgEComLogImuShortGetDeltaVelocity(const SbgEComLogImuShort *pImuShort, size_t idx)
{
    assert(pImuShort);
    assert(idx < 3);

    return pImuShort->deltaVelocity[idx] / SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD;
}

float sbgEComLogImuShortGetTemperature(const SbgEComLogImuShort *pImuShort)
{
    assert(pImuShort);

    return pImuShort->temperature / SBG_ECOM_LOG_IMU_TEMP_SCALE_STD;
}
