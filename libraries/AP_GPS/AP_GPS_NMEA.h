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

private:
    /// Coding for the GPS sentences that the parser handles
    enum _sentence_types : uint8_t {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
        _GPS_SENTENCE_HDT = 128,
        _GPS_SENTENCE_PHD = 138, // extension for AllyStar GPS modules
        _GPS_SENTENCE_THS = 160, // True heading with quality indicator, available on Trimble MB-Two
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

    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[15];                                                     ///< buffer for the current term within the current sentence
    uint8_t _sentence_type;                                     ///< the sentence type currently being processed
    uint8_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _gps_data_good;                                        ///< set when the sentence indicates data is good
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
    uint32_t _last_HDT_THS_ms;
    uint32_t _last_PHD_12_ms;
    uint32_t _last_PHD_26_ms;
    uint32_t _last_fix_ms;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _MTK_init_string[];                  ///< init string for MediaTek units
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
};

#define AP_GPS_NMEA_HEMISPHERE_INIT_STRING \
        "$JATT,NMEAHE,0\r\n" /* Prefix of GP on the HDT message */      \
        "$JASC,GPGGA,5\r\n" /* GGA at 5Hz */                            \
        "$JASC,GPRMC,5\r\n" /* RMC at 5Hz */                            \
        "$JASC,GPVTG,5\r\n" /* VTG at 5Hz */                            \
        "$JASC,GPHDT,5\r\n" /* HDT at 5Hz */                            \
        "$JMODE,SBASR,YES\r\n" /* Enable SBAS */
