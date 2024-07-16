#include "AP_NMEA_Input.h"

bool AP_NMEA_Input::init(AP_SerialManager::SerialProtocol prot, uint8_t prot_instance)
{
    const AP_SerialManager& serial_manager = AP::serialmanager();
    auto *_nmea_input_uart = serial_manager.find_serial(prot, prot_instance);
    if (_nmea_input_uart == nullptr) {
        return false;
    }

    _nmea_input_uart->begin(serial_manager.find_baudrate(prot, prot_instance));

    init(_nmea_input_uart);

    return true;
}

void AP_NMEA_Input::update()
{
    if (nmea_input_uart == nullptr) {
        return;
    }

    // read any available lines from the windvane
    int16_t nbytes = nmea_input_uart->available();
    while (nbytes-- > 0) {
        char c = nmea_input_uart->read();
        if (decode(c)) {
            handle_decode_success();
        }
    }
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_NMEA_Input::decode(char c)
{
    switch (c) {
    case ',':
        // end of a term, add to checksum
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        if (_sentence_done) {
            return false;
        }

        // null terminate and decode latest term
        _nmea_input_term[_term_offset] = 0;
        bool valid_sentence = decode_latest_term();

        // move onto next term
        _nmea_input_term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;
    }

    case '$': // sentence begin
        _sentence_valid = false;
        _nmea_input_term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        _sentence_done = false;
        return false;
    }

    // ordinary characters are added to term
    if (_term_offset < sizeof(_nmea_input_term) - 1) {
        _nmea_input_term[_term_offset++] = c;
    }
    if (!_term_is_checksum) {
        _checksum ^= c;
    }

    return false;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_NMEA_Input::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        _sentence_done = true;
        uint8_t checksum = 16 * char_to_hex(_nmea_input_term[0]) + char_to_hex(_nmea_input_term[1]);
        return ((checksum == _checksum) && _sentence_valid);
    }

    // the first term determines the sentence type
    if (_nmea_input_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        if (_nmea_input_term[0] < 'A' || _nmea_input_term[0] > 'Z' ||
            _nmea_input_term[1] < 'A' || _nmea_input_term[1] > 'Z') {
             // unknown ID (we are actually expecting II)
            return false;
        }
        const char *term_type = &_nmea_input_term[2];
        if (start_sentence_type(term_type)) {
            _sentence_valid = true;
        }
        return false;
    }

    // if this is not the sentence we want then wait for another
    if (!_sentence_valid) {
        return false;
    }

    if (!handle_term(_nmea_input_term_number, _nmea_input_term)) {
        _sentence_valid = false;
    }

    return false;
}

// return the numeric value of an ascii hex character
int16_t AP_NMEA_Input::char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}
