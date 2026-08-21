#include "AP_Networking_PPP_SoftFlow.h"

// Both codewords use only XON/XOFF, which PPP escapes when configured with
// the corresponding ACCM bits. They are complements (Hamming distance eight)
// and form a comma-free code: no unaligned window across two adjacent
// codewords is itself a valid codeword. This lets a complete retry safely
// follow a partially transmitted command.
static constexpr uint8_t start_codeword[AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH] {
    0x11, 0x11, 0x13, 0x13, 0x11, 0x13, 0x13, 0x11,
};
static constexpr uint8_t stop_codeword[AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH] {
    0x13, 0x13, 0x11, 0x11, 0x13, 0x11, 0x11, 0x13,
};

const uint8_t *AP_Networking_PPP_SoftFlow::codeword(Command command)
{
    switch (command) {
    case Command::START:
        return start_codeword;
    case Command::STOP:
        return stop_codeword;
    case Command::NONE:
        return nullptr;
    }
    return nullptr;
}

void AP_Networking_PPP_SoftFlow::reset()
{
    _match_length = 0;
    _match_window = 0;
    _command_repetitions = 0;
    _receive_stopped = false;
    _transmit_stopped = false;
    _last_match_ms = 0;
    _last_stop_received_ms = 0;
    _last_command_sent_ms = 0;
}

bool AP_Networking_PPP_SoftFlow::process_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte != XON && byte != XOFF) {
        _match_length = 0;
        _match_window = 0;
        return false;
    }

    if (_match_length != 0 && now_ms - _last_match_ms > CODEWORD_TIMEOUT_MS) {
        _match_length = 0;
        _match_window = 0;
    }
    _last_match_ms = now_ms;

    _match_window = uint8_t((_match_window << 1) | (byte == XOFF));
    if (_match_length < CODEWORD_LENGTH) {
        _match_length++;
    }
    if (_match_length == CODEWORD_LENGTH) {
        // START is 00110110 and STOP is its complement 11001001.
        if (_match_window == 0xC9) {
            _transmit_stopped = true;
            _last_stop_received_ms = now_ms;
            _match_length = 0;
            _match_window = 0;
        } else if (_match_window == 0x36) {
            _transmit_stopped = false;
            _match_length = 0;
            _match_window = 0;
        }
    }
    return true;
}

void AP_Networking_PPP_SoftFlow::update_receive_space(uint32_t space, uint32_t size, uint32_t now_ms)
{
    if (size == 0) {
        return;
    }

    if (!_receive_stopped && space <= size / 4U) {
        _receive_stopped = true;
        _command_repetitions = COMMAND_REPETITIONS;
        _last_command_sent_ms = now_ms - STOP_REFRESH_MS;
    } else if (_receive_stopped && space >= (size * 3U) / 4U) {
        _receive_stopped = false;
        _command_repetitions = COMMAND_REPETITIONS;
        _last_command_sent_ms = now_ms - STOP_REFRESH_MS;
    }
}

bool AP_Networking_PPP_SoftFlow::transmit_paused(uint32_t now_ms)
{
    if (_transmit_stopped && now_ms - _last_stop_received_ms > STOP_LEASE_MS) {
        _transmit_stopped = false;
    }
    return _transmit_stopped;
}

AP_Networking_PPP_SoftFlow::Command AP_Networking_PPP_SoftFlow::pending_command(uint32_t now_ms)
{
    if (_command_repetitions != 0 ||
        (_receive_stopped && now_ms - _last_command_sent_ms >= STOP_REFRESH_MS)) {
        return _receive_stopped ? Command::STOP : Command::START;
    }
    return Command::NONE;
}

void AP_Networking_PPP_SoftFlow::command_sent(Command command, uint32_t now_ms)
{
    const Command current = _receive_stopped ? Command::STOP : Command::START;
    if (command != current) {
        return;
    }
    if (_command_repetitions != 0) {
        _command_repetitions--;
    }
    _last_command_sent_ms = now_ms;
}
