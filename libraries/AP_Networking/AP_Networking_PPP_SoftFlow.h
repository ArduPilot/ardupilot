#pragma once

#include <stdint.h>

class AP_Networking_PPP_SoftFlow
{
public:
    AP_Networking_PPP_SoftFlow()
    {
        reset();
    }

    enum class Command : uint8_t {
        NONE,
        START,
        STOP,
    };

    static constexpr uint8_t CODEWORD_LENGTH = 8;
    static constexpr uint32_t STOP_REFRESH_MS = 20;
    static constexpr uint32_t STOP_LEASE_MS = 120;
    static constexpr uint32_t CODEWORD_TIMEOUT_MS = 20;

    // Return true when byte is an out-of-band flow-control byte.
    bool process_byte(uint8_t byte, uint32_t now_ms);

    // Update the receive window advertised to the peer.
    void update_receive_space(uint32_t space, uint32_t size, uint32_t now_ms);

    // Return true while a current STOP lease prevents ordinary transmission.
    bool transmit_paused(uint32_t now_ms);

    // Obtain the next priority command to transmit, if any.
    Command pending_command(uint32_t now_ms);
    void command_sent(Command command, uint32_t now_ms);

    void reset();

    static const uint8_t *codeword(Command command);

private:
    static constexpr uint8_t XON = 0x11;
    static constexpr uint8_t XOFF = 0x13;
    static constexpr uint8_t COMMAND_REPETITIONS = 3;

    uint8_t _match_length;
    uint8_t _match_window;
    uint8_t _command_repetitions;
    bool _receive_stopped;
    bool _transmit_stopped;
    uint32_t _last_match_ms;
    uint32_t _last_stop_received_ms;
    uint32_t _last_command_sent_ms;
};
