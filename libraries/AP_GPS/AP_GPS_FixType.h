#pragma once

// a AP_GPS-library-independent enumeration which lists the commonly
// accepted set of GPS Fix Types which GPSs report.  This header can
// be used even if AP_GPS is not compiled in.

// this is not enum-class as many places in the code want to check for
// "a fix at least this good" using "<".

enum class AP_GPS_FixType {
    NO_GPS = 0,           ///< No GPS connected/detected
    NONE = 1,             ///< Receiving valid GPS messages but no lock
    FIX_2D = 2,           ///< Receiving valid messages and 2D lock
    FIX_3D = 3,           ///< Receiving valid messages and 3D lock
    DGPS = 4,             ///< Receiving valid messages and 3D lock with differential improvements
    RTK_FLOAT = 5,        ///< Receiving valid messages and 3D RTK Float
    RTK_FIXED = 6,        ///< Receiving valid messages and 3D RTK Fixed
};
