/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
//

/// @file	AP_GPS_NMEA.h
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// NMEA mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_NMEA_ENABLED
/// NMEA parser
///
class AP_GPS_NMEA : public AP_GPS_Backend
{
    friend class AP_GPS_NMEA_Test;

public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read() override;

	static bool _detect(struct NMEA_detect_state &state, uint8_t data);

    const char *name() const override { return "NMEA"; }

    // driver specific health, returns true if the driver is healthy
    bool is_healthy(void) const override;

    // get lag in seconds
    bool get_lag(float &lag_sec) const override;

#if HAL_LOGGING_ENABLED
    void Write_AP_Logger_Log_Startup_messages() const override;
#endif

private:
    /// Coding for the GPS sentences that the parser handles
    enum _sentence_types : uint16_t {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
        _GPS_SENTENCE_HDT = 128,
        _GPS_SENTENCE_PHD = 138, // extension for AllyStar GPS modules
        _GPS_SENTENCE_THS = 160, // True heading with quality indicator, available on Trimble MB-Two
        _GPS_SENTENCE_KSXT = 170, // extension for Unicore, 21 fields
        _GPS_SENTENCE_AGRICA = 193, // extension for Unicore, 65 fields
        _GPS_SENTENCE_VERSIONA = 270, // extension for Unicore, version, 10 fields
        _GPS_SENTENCE_UNIHEADINGA = 290, // extension for Unicore, uniheadinga, 20 fields
        _GPS_SENTENCE_OTHER = 0
    };

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the NMEA input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);

    /// Parses the @p as a NMEA-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a NMEA-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of NMEA messages
    bool _have_new_message(void);

#if AP_GPS_NMEA_UNICORE_ENABLED
    /*
      parse an AGRICA field
     */
    void parse_agrica_field(uint16_t term_number, const char *term);

    // parse VERSIONA field
    void parse_versiona_field(uint16_t term_number, const char *term);

#if GPS_MOVING_BASELINE
    // parse UNIHEADINGA field
    void parse_uniheadinga_field(uint16_t term_number, const char *term);
#endif
#endif


    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    uint32_t _crc32;                                            ///< CRC for unicore messages
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[30];                                                     ///< buffer for the current term within the current sentence
    uint16_t _sentence_type;                                     ///< the sentence type currently being processed
    bool _is_unicore;                                           ///< true if in a unicore '#' sentence
    uint16_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _sentence_done;                                        ///< set when a sentence has been fully decoded

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    float   _new_gps_yaw;                                        ///< yaw parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms;
    uint32_t _last_GGA_ms;
    uint32_t _last_VTG_ms;
    uint32_t _last_yaw_ms;
    uint32_t _last_vvelocity_ms;
    uint32_t _last_vaccuracy_ms;
    uint32_t _last_3D_velocity_ms;
    uint32_t _last_KSXT_pos_ms;
    uint32_t _last_AGRICA_ms;
    uint32_t _last_fix_ms;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];

    /*
      the $PHD message is an extension from AllyStar that gives
      vertical velocity and more accuracy estimates. It is designed as
      a mapping from ublox UBX protocol messages to NMEA. So class 1,
      message 12 is a mapping to NMEA of the NAV-VELNED UBX message
      and contains the same fields. Class 1 message 26 is called
      "NAV-PVERR", but does not correspond to a UBX message

      example:
        $PHD,01,12,TIIITTITT,,245808000,0,0,0,0,0,10260304,0,0*27
        $PHD,01,26,TTTTTTT,,245808000,877,864,1451,11,11,17*17
     */
    struct {
        uint8_t msg_class;
        uint8_t msg_id;
        uint32_t itow;
        int32_t fields[8];
    } _phd;

    /*
      The KSXT message is an extension from Unicore that gives 3D velocity and yaw
      example: $KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,*2D
     */
    struct {
        double fields[21];
    } _ksxt;

#if AP_GPS_NMEA_UNICORE_ENABLED
    /*
      unicore AGRICA message parsing
     */
    struct {
        uint32_t start_byte;
        uint8_t rtk_status;
        uint8_t heading_status;
        Vector3f vel_NED;
        Vector3f vel_stddev;
        double lat, lng;
        float alt;
        uint32_t itow;
        float undulation;
        Vector3f pos_stddev;
    } _agrica;

    // unicore VERSIONA parsing
    struct {
        char type[10];
        char version[20];
        char build_date[13];
    } _versiona;
    bool _have_unicore_versiona;

#if GPS_MOVING_BASELINE
    // unicore UNIHEADINGA parsing
    struct {
        float baseline_length;
        float heading;
        float pitch;
        float heading_sd;
    } _uniheadinga;
#endif
#endif // AP_GPS_NMEA_UNICORE_ENABLED
    bool _expect_agrica;

    // last time we sent type specific config strings
    uint32_t last_config_ms;

    // send type specific config strings
    void send_config(void);
};

#if AP_GPS_NMEA_UNICORE_ENABLED && !defined(NMEA_UNICORE_SETUP)
// we don't know what port the GPS may be using, so configure all 3. We need to get it sending
// one message to allow the NMEA detector to run
#define NMEA_UNICORE_SETUP "CONFIG COM1 230400 8 n 1\r\nCONFIG COM2 230400 8 n 1\r\nCONFIG COM3 230400 8 n 1\r\nGPGGA 0.2\r\n"
#endif

#endif // AP_GPS_NMEA_ENABLED

