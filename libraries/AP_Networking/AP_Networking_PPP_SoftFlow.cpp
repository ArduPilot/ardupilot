#include "AP_Networking_PPP_SoftFlow.h"

#include <string.h>

// Both codewords use only XON/XOFF, which PPP escapes when configured with
// the corresponding ACCM bits. They are complements (Hamming distance eight)
// and form a comma-free code: no unaligned window across two adjacent
// codewords is itself a valid codeword. A complete retry of a command can
// safely follow a partial one. A partial command followed by the other
// command can be misread, which the repeats of each command recover.
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
    _candidate_length = 0;
    _command_repetitions = 0;
    _receive_stopped = false;
    _transmit_stopped = false;
    _last_match_ms = 0;
    _last_stop_received_ms = 0;
    _last_command_sent_ms = 0;
    _stop_start_ms = 0;
    _stop_rx_base = 0;
    _stop_rx_base_valid = false;
}

uint8_t AP_Networking_PPP_SoftFlow::process_byte(uint8_t byte, uint32_t now_ms, uint8_t *output)
{
    uint8_t output_length = flush(now_ms, output);

    if (byte != XON && byte != XOFF) {
        memcpy(&output[output_length], _candidate, _candidate_length);
        output_length += _candidate_length;
        _candidate_length = 0;
        output[output_length++] = byte;
        return output_length;
    }

    _last_match_ms = now_ms;
    _candidate[_candidate_length++] = byte;

    if (_candidate_length == CODEWORD_LENGTH) {
        if (memcmp(_candidate, stop_codeword, CODEWORD_LENGTH) == 0) {
            _transmit_stopped = true;
            _last_stop_received_ms = now_ms;
            _candidate_length = 0;
        } else if (memcmp(_candidate, start_codeword, CODEWORD_LENGTH) == 0) {
            _transmit_stopped = false;
            _candidate_length = 0;
        } else {
            // Preserve a non-command byte while retaining the following
            // seven bytes as candidates for a possibly unaligned command.
            output[output_length++] = _candidate[0];
            memmove(_candidate, &_candidate[1], CODEWORD_LENGTH - 1U);
            _candidate_length--;
        }
    }
    return output_length;
}

uint8_t AP_Networking_PPP_SoftFlow::flush(uint32_t now_ms, uint8_t *output)
{
    if (_candidate_length == 0 ||
        now_ms - _last_match_ms <= CODEWORD_TIMEOUT_MS) {
        return 0;
    }

    const uint8_t output_length = _candidate_length;
    memcpy(output, _candidate, output_length);
    _candidate_length = 0;
    return output_length;
}

void AP_Networking_PPP_SoftFlow::update_receive_space(uint32_t space, uint32_t size, uint32_t now_ms)
{
    if (size == 0) {
        return;
    }

    if (!_receive_stopped && space <= size / 4U) {
        _receive_stopped = true;
        _stop_start_ms = now_ms;
        _stop_rx_base_valid = false;
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

bool AP_Networking_PPP_SoftFlow::stop_ignored(uint32_t rx_total, uint32_t limit, uint32_t now_ms)
{
    if (!_receive_stopped || now_ms - _stop_start_ms < STOP_SETTLE_MS) {
        return false;
    }
    if (!_stop_rx_base_valid || _transmit_stopped) {
        // a peer we have stopped flushes its queue to send its own STOP
        _stop_rx_base = rx_total;
        _stop_rx_base_valid = true;
        return false;
    }
    return rx_total - _stop_rx_base > limit;
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
