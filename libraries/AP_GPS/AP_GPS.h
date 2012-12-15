// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_GPS.h
/// @brief	Catch-all header that defines all supported GPS classes.

#include "AP_GPS_NMEA.h"
#include "AP_GPS_SIRF.h"
#include "AP_GPS_406.h"
#include "AP_GPS_UBLOX.h"
#include "AP_GPS_MTK.h"
#include "AP_GPS_MTK19.h"
#include "AP_GPS_None.h"
#include "AP_GPS_Auto.h"
#include "AP_GPS_HIL.h"
#include "AP_GPS_Shim.h"        // obsoletes AP_GPS_HIL, use in preference

