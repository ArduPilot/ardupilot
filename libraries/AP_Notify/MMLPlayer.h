#pragma once

#include <stdint.h>
#include <stdlib.h>

class MMLPlayer {
public:
    void update();
    void play(const char* string);
    void stop();

private:

    // initialise ready to play string
    void prepare_to_play_string(const char* string);

    bool _playing;

    uint32_t _note_duration_us;
    uint32_t _note_start_us;
    const char* _string;
    uint8_t _tempo;
    uint8_t _default_note_length;
    uint8_t _volume;
    size_t _next;
    uint8_t _octave;
    float _silence_duration;
    bool _repeat;
    enum node_mode_t {
        MODE_NORMAL,
        MODE_LEGATO,
        MODE_STACCATO
    } _note_mode;

    void start_silence(float duration);
    void start_note(float duration, float frequency, float volume);
    char next_char();
    uint8_t next_number();
    size_t next_dots();
    float rest_duration(uint32_t rest_length, uint8_t dots);

    // Called when the MML player should start the next action
    void next_action();
};










