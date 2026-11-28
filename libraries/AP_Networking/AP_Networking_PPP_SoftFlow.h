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
    // a peer honouring STOP is quiet by this time after it was requested
    static constexpr uint32_t STOP_SETTLE_MS = 200;

    // Process one byte, returning ordinary stream bytes in output. Potential
    // codeword bytes are held until they match a command or can be released.
    uint8_t process_byte(uint8_t byte, uint32_t now_ms, uint8_t *output);
    uint8_t flush(uint32_t now_ms, uint8_t *output);

    // Update the receive window advertised to the peer.
    void update_receive_space(uint32_t space, uint32_t size, uint32_t now_ms);

    // Return true while a current STOP lease prevents ordinary transmission.
    bool transmit_paused(uint32_t now_ms);

    // Return true while we are holding the peer stopped.
    bool receive_stopped() const { return _receive_stopped; }

    // Return true if the peer kept sending through our STOP. rx_total is
    // a running count of all bytes that have arrived from the peer, and
    // limit is how many of them a peer honouring STOP may still send.
    bool stop_ignored(uint32_t rx_total, uint32_t limit, uint32_t now_ms);

    // Obtain the next priority command to transmit, if any.
    Command pending_command(uint32_t now_ms);
    void command_sent(Command command, uint32_t now_ms);

    void reset();

    static const uint8_t *codeword(Command command);

private:
    static constexpr uint8_t XON = 0x11;
    static constexpr uint8_t XOFF = 0x13;
    static constexpr uint8_t COMMAND_REPETITIONS = 3;

    uint8_t _candidate[CODEWORD_LENGTH];
    uint8_t _candidate_length;
    uint8_t _command_repetitions;
    bool _receive_stopped;
    bool _transmit_stopped;
    uint32_t _last_match_ms;
    uint32_t _last_stop_received_ms;
    uint32_t _last_command_sent_ms;
    uint32_t _stop_start_ms;
    uint32_t _stop_rx_base;
    bool _stop_rx_base_valid;
};
