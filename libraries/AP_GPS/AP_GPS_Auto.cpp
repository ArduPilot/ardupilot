// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_GPS_Auto.cpp
/// @brief	Simple GPS auto-detection logic.

#include <FastSerial.h>
#include <AP_Common.h>

#include "AP_GPS.h"             // includes AP_GPS_Auto.h

// Define this to add NMEA to the auto-detection cycle.
//
// Note that there is a potential race where NMEA data may overlap with
// the commands that switch a GPS out of NMEA mode that can cause
// the GPS to switch to binary mode at the same time that this code
// detects it as being in NMEA mode.
//
//#define WITH_NMEA_MODE	1

static unsigned int baudrates[] = {38400U, 57600U, 9600U, 4800U};

const prog_char AP_GPS_Auto::_mtk_set_binary[]   PROGMEM = MTK_SET_BINARY;
const prog_char AP_GPS_Auto::_sirf_set_binary[]  PROGMEM = SIRF_SET_BINARY;


AP_GPS_Auto::AP_GPS_Auto(FastSerial *s, GPS **gps)  :
    GPS(s),
    _fs(s),
    _gps(gps)
{
}

// Do nothing at init time - it may be too early to try detecting the GPS
//
void
AP_GPS_Auto::init(enum GPS_Engine_Setting nav_setting)
{
    idleTimeout = 1200;
    if (callback == NULL) callback = delay;
    _nav_setting = nav_setting;
}


// Called the first time that a client tries to kick the GPS to update.
//
// We detect the real GPS, then update the pointer we have been called through
// and return.
//
/// @todo	This routine spends a long time trying to detect a GPS.  That's not strictly
///			desirable; it might be a good idea to rethink the logic here to make it
///			more asynchronous, so that other parts of the system can get a chance
///			to run while GPS detection is in progress.
///
bool
AP_GPS_Auto::read(void)
{
    GPS         *gps;
    uint8_t i;
    uint32_t then;

    // Loop through possible baudrates trying to detect a GPS at one of them.
    //
    // Note that we need to have a FastSerial rather than a Stream here because
    // Stream has no idea of line speeds.  FastSerial is quite OK with us calling
    // ::begin any number of times.
    //
    for (i = 0; i < (sizeof(baudrates) / sizeof(baudrates[0])); i++) {

        // ensure the serial port has a large enough buffer for any protocol
        _fs->begin(baudrates[i], 256, 16);
        if (NULL != (gps = _detect())) {

            // configure the detected GPS and give it a chance to listen to its device
            gps->init(_nav_setting);
            then = millis();
            while ((millis() - then) < 1200) {
                // if we get a successful update from the GPS, we are done
                gps->new_data = false;
                gps->update();
                if (gps->new_data) {
                    Serial.println_P(PSTR("OK"));
                    *_gps = gps;
                    return true;
                }
            }
            // GPS driver failed to parse any data from GPS,
            // delete the driver and continue the process.
            Serial.println_P(PSTR("failed, retrying"));
            delete gps;
        }
    }
    return false;
}

//
// Perform one iteration of the auto-detection process.
//
GPS *
AP_GPS_Auto::_detect(void)
{
    uint32_t then;
    uint8_t fingerprint[4];
    uint8_t tries;
    uint16_t charcount;
    GPS         *gps;

    //
    // Loop attempting to detect a recognized GPS
    //
    Serial.print('G');
    gps = NULL;

    for (tries = 0; tries < 2; tries++) {
        //
        // Empty the serial buffer and wait for 50ms of quiet.
        //
        // XXX We can detect babble by counting incoming characters, but
        //     what would we do about it?
        //
        charcount = 0;
        _port->flush();
        then = millis();
        do {
            if (_port->available()) {
                then = millis();
                _port->read();
                charcount++;
            }
        } while ((millis() - then) < 50 && charcount < 5000);

        if (tries == 0) {
            // write configuration strings to put the GPS into the preferred
            // mode
            _write_progstr_block(_fs, _mtk_set_binary, sizeof(_mtk_set_binary));
            _write_progstr_block(_fs, AP_GPS_UBLOX::_ublox_set_binary, AP_GPS_UBLOX::_ublox_set_binary_size);
            _write_progstr_block(_fs, _sirf_set_binary, sizeof(_sirf_set_binary));

            // ensure its all been written
            while (_fs->tx_pending()) {
                callback(10);
            }

            // give the GPS time to react to the settings
            callback(100);
        }

        //
        // Collect four characters to fingerprint a device
        //
        // If we take more than 1200ms to receive four characters, abort.
        // This will normally only be the case where there is no GPS attached.
        //
        while (_port->available() < 4) {
            callback(1);
            if ((millis() - then) > 1200) {
                Serial.print('!');
                return NULL;
            }
        }
        fingerprint[0] = _port->read();
        fingerprint[1] = _port->read();
        fingerprint[2] = _port->read();
        fingerprint[3] = _port->read();

        //
        // ublox or MTK in DIYD binary mode (whose smart idea was
        // it to make the MTK look sort-of like it was talking UBX?)
        //
        if ((0xb5 == fingerprint[0]) &&
            (0x62 == fingerprint[1]) &&
            (0x01 == fingerprint[2])) {

            // message 5 is MTK pretending to talk UBX
            if (0x05 == fingerprint[3]) {
                gps = new AP_GPS_MTK(_port);
                Serial.print_P(PSTR(" MTK1.4 "));
                break;
            }

            // any other message is ublox
            gps = new AP_GPS_UBLOX(_port);
            Serial.print_P(PSTR(" ublox "));
            break;
        }

        // new style 3DR UBlox (April 2012)x
        if (0xb5 == fingerprint[0] &&
            0x62 == fingerprint[1] &&
            0x0d == fingerprint[2] &&
            0x01 == fingerprint[3]) {
            // new style Ublox
            gps = new AP_GPS_UBLOX(_port);
            Serial.print_P(PSTR(" ublox "));
            break;
        }

        //
        // MTK v1.6
        //
        if ((0xd0 == fingerprint[0]) &&
            (0xdd == fingerprint[1]) &&
            (0x20 == fingerprint[2])) {
            gps = new AP_GPS_MTK16(_port);
            Serial.print_P(PSTR(" MTK1.6 "));
            break;
        }

        //
        // SIRF in binary mode
        //
        if ((0xa0 == fingerprint[0]) &&
            (0xa2 == fingerprint[1])) {
            gps = new AP_GPS_SIRF(_port);
            Serial.print_P(PSTR(" SiRF "));
            break;
        }

#if WITH_NMEA_MODE
        //
        // Something talking NMEA
        //
        if (('$' == fingerprint[0]) &&
            (('G' == fingerprint[1]) || ('P' == fingerprint[1]))) {

            // XXX this may be a bit presumptive, might want to give the GPS a couple of
            //     iterations around the loop to react to init strings?
            gps = new AP_GPS_NMEA(_port);
            break;
        }
#endif
        Serial.printf("?");
    }
    return(gps);
}

