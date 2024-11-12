#include "MMLPlayer.h"

#include <ctype.h>
#include <math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>

#if HAL_CANMANAGER_ENABLED
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_CANManager/AP_CANManager.h>
#endif

extern const AP_HAL::HAL& hal;

void MMLPlayer::update()
{
    // Check if note is over
    if (_playing && AP_HAL::micros()-_note_start_us > _note_duration_us) {
        next_action();
    }
}

void MMLPlayer::prepare_to_play_string(const char* string)
{
    stop();

    _string = string;
    _next = 0;
    _tempo = 120;
    _default_note_length = 4;
    _note_mode = MODE_NORMAL;
    _octave = 4;
    _volume = 255;
    _silence_duration = 0;
    _repeat = false;

    _playing = true;
    _note_duration_us = 0;
}

void MMLPlayer::play(const char* string)
{
    prepare_to_play_string(string);
    next_action();
}

void MMLPlayer::stop()
{
    _playing = false;
    hal.util->toneAlarm_set_buzzer_tone(0,0,0);
}

void MMLPlayer::start_silence(float duration)
{
    _note_start_us = AP_HAL::micros();
    _note_duration_us = duration*1e6;
    hal.util->toneAlarm_set_buzzer_tone(0, 0, 0);
}

void MMLPlayer::start_note(float duration, float frequency, float volume)
{
    _note_start_us = AP_HAL::micros();
    _note_duration_us = duration*1e6;
    hal.util->toneAlarm_set_buzzer_tone(frequency, volume, _note_duration_us/1000U);

#if HAL_ENABLE_DRONECAN_DRIVERS
    // support CAN buzzers too
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    uavcan_equipment_indication_BeepCommand msg;

    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *uavcan = AP_DroneCAN::get_dronecan(i);
        if (uavcan != nullptr &&
            (AP::notify().get_buzzer_types() & uint8_t(AP_Notify::BuzzerType::UAVCAN))) {
            msg.frequency = frequency;
            msg.duration = _note_duration_us*1.0e-6;
            uavcan->buzzer.broadcast(msg);
        }
    }
#endif
}

char MMLPlayer::next_char()
{
    while (_string[_next] != '\0' && isspace(_string[_next])) {
        _next++;
    }

    return toupper(_string[_next]);
}

uint8_t MMLPlayer::next_number()
{
    uint8_t ret = 0;
    while (isdigit(next_char())) {
        ret = (ret*10) + (next_char() - '0');
        _next++;
    }
    return ret;
}

size_t MMLPlayer::next_dots()
{
    size_t ret = 0;
    while (next_char() == '.') {
        ret++;
        _next++;
    }
    return ret;
}

float MMLPlayer::rest_duration(uint32_t rest_length, uint8_t dots) const
{
    float whole_note_period = 240.0f / _tempo;
    if (rest_length == 0) {
        rest_length = 1;
    }

    float rest_period = whole_note_period/rest_length;
    float dot_extension = rest_period * 0.5f;

    while (dots--) {
        rest_period += dot_extension;
        dot_extension *= 0.5f;
    }

    return rest_period;
}

void MMLPlayer::next_action()
{
    if (_silence_duration > 0) {
        start_silence(_silence_duration);
        _silence_duration = 0;
        return;
    }

    uint8_t note = 0;
    uint8_t note_length;

    while (note == 0) {
        char c = next_char();
        if (c == '\0') {
            if (_repeat) {
                // don't "play" here, as we may have been called from
                // there, and it turns out infinite recursion on
                // invalid strings is suboptimal.  The next call to
                // update() will push things out as appropriate.
                prepare_to_play_string(_string);
            } else {
                stop();
            }
            return;
        }

        _next++;

        switch (c) {
        case 'V': {
            _volume = next_number();
            break;
        }
        case 'L': {
            _default_note_length = next_number();
            if (_default_note_length == 0) {
                stop();
                return;
            }
            break;
        }
        case 'O':
            _octave = next_number();
            if (_octave > 6) {
                _octave = 6;
            }
            break;
        case '<':
            if (_octave > 0) {
                _octave--;
            }
            break;
        case '>':
            if (_octave < 6) {
                _octave++;
            }
            break;
        case 'M':
            c = next_char();
            if (c == '\0') {
                stop();
                return;
            }
            _next++;
            switch (c) {
            case 'N':
                _note_mode = MODE_NORMAL;
                break;
            case 'L':
                _note_mode = MODE_LEGATO;
                break;
            case 'S':
                _note_mode = MODE_STACCATO;
                break;
            case 'F':
                _repeat = false;
                break;
            case 'B':
                _repeat = true;
                break;
            default:
                stop();
                return;
            }
            break;
        case 'R':
        case 'P': {
            uint8_t num = next_number();
            uint8_t dots = next_dots();
            start_silence(rest_duration(num, dots));
            return;
        }
        case 'T':
            _tempo = next_number();
            if (_tempo < 32) {
                stop();
                return;
            }
            break;
        case 'N':
            note = next_number();
            note_length = _default_note_length;
            if (note > 84) {
                stop();
                return;
            }
            if (note == 0) {
                uint8_t num = next_number();
                uint8_t dots = next_dots();
                start_silence(rest_duration(num, dots));
                return;
            }
            break;
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        case 'G': {
            static const uint8_t note_tab[] = {9,11,0,2,4,5,7};
            note = note_tab[c-'A'] + (_octave*12) + 1;

            c = next_char();

            switch (c) {
            case '#':
            case '+':
                if (note < 84) {
                    note++;
                }
                _next++;
                break;
            case '-':
                if (note > 1) {
                    note--;
                }
                _next++;
                break;
            default:
                break;
            }
            note_length = next_number();
            if (note_length == 0) {
                note_length = _default_note_length;
            }
            break;
        }
        default:
            stop();
            return;
        }
    }

    // Avoid division by zero
    if (_tempo == 0 || note_length == 0) {
        stop();
        return;
    }

    float note_period = 240.0f / (float)_tempo / (float)note_length;

    switch (_note_mode) {
    case MODE_NORMAL:
        _silence_duration = note_period/8;
        break;
    case MODE_STACCATO:
        _silence_duration = note_period/4;
        break;
    case MODE_LEGATO:
        _silence_duration = 0;
        break;
    }
    note_period -= _silence_duration;

    float dot_extension = note_period * 0.5f;
    uint8_t dots = next_dots();
    while (dots--) {
        note_period += dot_extension;
        dot_extension *= 0.5f;
    }

    float note_frequency = 880.0f * expf(logf(2.0f) * ((int)note - 46) / 12.0f);
    float note_volume = _volume/255.0f;
    note_volume *= AP::notify().get_buzz_volume() * 0.01;
    note_volume = constrain_float(note_volume, 0, 1);

    note_frequency = constrain_float(note_frequency, 10, 22000);

    start_note(note_period, note_frequency, note_volume);
}
