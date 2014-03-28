// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Auto-detecting pseudo-GPS driver
//

#ifndef __AP_GPS_AUTO_H__
#define __AP_GPS_AUTO_H__

#include <AP_HAL.h>
#include "GPS.h"

class AP_GPS_Auto : public GPS
{
public:
    /// Constructor
    ///
    /// @note The stream is expected to be set up and configured for the
    ///       correct bitrate before ::init is called.
    ///
    /// @param	port	UARTDriver connected to the GPS module.
    /// @param	ptr		Pointer to a GPS * that will be fixed up by ::init
    ///					when the GPS type has been detected.
    ///
    AP_GPS_Auto(GPS **gps);

    /// Dummy init routine, does nothing
    void init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting, DataFlash_Class *DataFlash);

    /// Detect and initialise the attached GPS unit.  Updates the
    /// pointer passed into the constructor when a GPS is detected.
    ///
    bool read(void);

private:
    /// global GPS driver pointer, updated by auto-detection
    ///
    GPS **     _gps;

    /// low-level auto-detect routine
    ///
    GPS *                           _detect(void);

    static const prog_char          _ublox_set_binary[];
    static const prog_char          _mtk_set_binary[];
    static const prog_char          _sirf_set_binary[];

    void _send_progstr(const prog_char *pstr, uint8_t size);
    void _update_progstr(void);

// maximum number of pending progstrings
#define PROGSTR_QUEUE_SIZE 3

    struct progstr_queue {
        const prog_char *pstr;
        uint8_t ofs, size;
    };

    struct progstr_state {
        uint8_t queue_size;
        uint8_t idx, next_idx;
        struct progstr_queue queue[PROGSTR_QUEUE_SIZE];
    };

    struct detect_state {
        uint32_t detect_started_ms;
        uint32_t last_baud_change_ms;
        uint8_t last_baud;
        struct progstr_state progstr_state;
        AP_GPS_UBLOX::detect_state ublox_detect_state;
        AP_GPS_MTK::detect_state mtk_detect_state;
        AP_GPS_MTK19::detect_state mtk19_detect_state;
        AP_GPS_SIRF::detect_state sirf_detect_state;
        AP_GPS_NMEA::detect_state nmea_detect_state;
    } *state;
};
#endif
