#include "OrionComm.h"


BOOL OrionCommOpen(int *pArgc, char ***pArgv)
{
    // If there are at least two arguments, and the first looks like a serial port or IP
    if (*pArgc >= 2)
    {
        // Serial port...?
        if (((*pArgv)[1][0] == '/') || ((*pArgv)[1][0] == '\\'))
        {
            // Decrement the number of arguments and push the pointer up one arg
            (*pArgc)--;
            (*pArgv) = &(*pArgv)[1];

            // Try opening the specified serial port
            return OrionCommOpenSerial((*pArgv)[0]);
        }
        // IP address...?
        else if (OrionCommIpStringValid((*pArgv)[1]))
        {
            // Decrement the number of arguments and push the pointer up one arg
            (*pArgc)--;
            (*pArgv) = &(*pArgv)[1];

            // Try connecting to a gimbal at this IP
            return OrionCommOpenNetworkIp((*pArgv)[0]);
        }
    }

    // If we haven't connected any other way, try using network broadcast
    return OrionCommOpenNetwork();

}// OrionCommOpen
