#include <AP_HAL/AP_HAL.h>

// sbgCommonLib headers
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Include specific header for WIN32 and UNIX platforms               -//
//----------------------------------------------------------------------//
#ifdef WIN32
    #include <windows.h>
#elif defined(__APPLE__)
    #include <mach/mach_time.h>
#else
    #include <unistd.h>
#endif

//----------------------------------------------------------------------//
//- Global singleton for the log callback                              -//
//----------------------------------------------------------------------//

/*!
 * Unique singleton used to log error messages.
 */
SbgCommonLibOnLogFunc   gLogCallback = NULL;

extern const AP_HAL::HAL& hal;

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

SBG_COMMON_LIB_API void sbgSleep(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

SBG_COMMON_LIB_API void sbgCommonLibSetLogCallback(SbgCommonLibOnLogFunc logCallback)
{
    gLogCallback = logCallback;
}

SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...)
{
    char        errorMsg[SBG_CONFIG_LOG_MAX_SIZE];
    va_list     args;

    assert(pFileName);
    assert(pFunctionName);
    assert(pCategory);
    assert(pFormat);

    //
    // Initialize the list of variable arguments on the latest function argument
    //
    va_start(args, pFormat);

    //
    // Generate the error message string
    //
    vsnprintf(errorMsg, sizeof(errorMsg), pFormat, args);

    //
    // Close the list of variable arguments
    //
    va_end(args);

    //
    // Check if there is a valid logger callback if not use a default output
    //
    if (gLogCallback)
    {
        gLogCallback(pFileName, pFunctionName, line, pCategory, logType, errorCode, errorMsg);
    }
}
