// Local headers
#include "sbgErrorCodes.h"

//----------------------------------------------------------------------//
//- Macros definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * Expand a case block that converts and assigns an error code to a string variable.
 *
 * \param[in]   errorCode                       Error code.
 * \param[out]  errorString                     String representation of the error code.
 */
#define SBG_CASE_ERROR(errorCode, errorString)  \
case errorCode:                                 \
    errorString = SBG_QUOTE(errorCode);         \
    break

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SBG_COMMON_LIB_API const char *sbgErrorCodeToString(SbgErrorCode errorCode)
{
    const char                          *errorStr = NULL;

    switch (errorCode)
    {
    SBG_CASE_ERROR(SBG_NO_ERROR, errorStr);
    SBG_CASE_ERROR(SBG_ERROR, errorStr);
    SBG_CASE_ERROR(SBG_NULL_POINTER, errorStr);
    SBG_CASE_ERROR(SBG_INVALID_CRC, errorStr);
    SBG_CASE_ERROR(SBG_INVALID_FRAME, errorStr);
    SBG_CASE_ERROR(SBG_TIME_OUT, errorStr);
    SBG_CASE_ERROR(SBG_WRITE_ERROR, errorStr);
    SBG_CASE_ERROR(SBG_READ_ERROR, errorStr);
    SBG_CASE_ERROR(SBG_BUFFER_OVERFLOW, errorStr);
    SBG_CASE_ERROR(SBG_INVALID_PARAMETER, errorStr);
    SBG_CASE_ERROR(SBG_NOT_READY, errorStr);
    SBG_CASE_ERROR(SBG_MALLOC_FAILED, errorStr);
    SBG_CASE_ERROR(SBG_CALIB_MAG_NOT_ENOUGH_POINTS, errorStr);
    SBG_CASE_ERROR(SBG_CALIB_MAG_INVALID_TAKE, errorStr);
    SBG_CASE_ERROR(SBG_CALIB_MAG_SATURATION, errorStr);
    SBG_CASE_ERROR(SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE, errorStr);
    SBG_CASE_ERROR(SBG_DEVICE_NOT_FOUND, errorStr);
    SBG_CASE_ERROR(SBG_OPERATION_CANCELLED, errorStr);
    SBG_CASE_ERROR(SBG_NOT_CONTINUOUS_FRAME, errorStr);
    SBG_CASE_ERROR(SBG_INCOMPATIBLE_HARDWARE, errorStr);
    SBG_CASE_ERROR(SBG_INVALID_VERSION, errorStr);
    }

    return errorStr;
}
