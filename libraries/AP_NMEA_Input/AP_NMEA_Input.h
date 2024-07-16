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


   Author: Peter Barker

 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>

class AP_NMEA_Input {

public:

    /* Do not allow copies */
    AP_NMEA_Input(const AP_NMEA_Input &other) = delete;
    AP_NMEA_Input &operator=(const AP_NMEA_Input&) = delete;

    // See if we can read in some data
    void update();

    bool init(AP_SerialManager::SerialProtocol prot, uint8_t prot_instance);
    void init(AP_HAL::UARTDriver *_uart) { nmea_input_uart = _uart; }

protected:

    AP_NMEA_Input() {}

    // called when the sentence type has been determined; should
    // return true if this sentence should be fully decoded
    virtual bool start_sentence_type(const char *term_type) = 0;
    // called as every term is decoded in a handled sentence.
    // Sentence may yet be invalid - handle_decode_success will be
    // called if the term should *actually* be used.  Should return
    // false if the value in this term makes the sentence invalid.
    virtual bool handle_term(uint8_t term_number, const char *term) = 0;
    // called when a complete sentence has been decoded and passed checksum:
    virtual void handle_decode_success() = 0;

private:

    // pointer to serial uart
    AP_HAL::UARTDriver *nmea_input_uart = nullptr;

    // try and decode NMEA message
    bool decode(char c);

    // decode each term
    bool decode_latest_term();

    // convert from char to hex value for checksum
    int16_t char_to_hex(char a);

    char _nmea_input_term[15];            // buffer for the current term within the current sentence
    uint8_t _term_offset;      // offset within the _term buffer where the next character should be placed
    uint8_t _nmea_input_term_number;      // term index within the current sentence
    uint8_t _checksum;         // checksum accumulator
    bool _term_is_checksum;    // current term is the checksum
    bool _sentence_valid;      // is current sentence valid so far
    bool _sentence_done;       // true if this sentence has already been decoded
};
