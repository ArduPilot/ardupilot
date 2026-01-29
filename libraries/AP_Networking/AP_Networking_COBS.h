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

#pragma once

#include <stdint.h>
#include <stddef.h>

/*
  COBS (Consistent Overhead Byte Stuffing) encoding/decoding

  COBS is a data encoding scheme that eliminates zero bytes from data.
  This makes it useful for serial protocols that use zero bytes as frame
  delimiters.

  The encoding works by replacing zero bytes with a count of the number
  of non-zero bytes until the next zero (or end of packet). The count
  is stored in a single byte, limiting runs to 254 non-zero bytes (0xFF
  indicates 254 non-zero bytes followed by a zero).

  This implementation provides:
  - Static encode/decode functions for full buffers
  - Streaming decoder for byte-by-byte processing (useful for UART RX)
 */

class AP_Networking_COBS
{
public:
    // Encode: input buffer -> COBS encoded output buffer
    // Returns: encoded length, or 0 on error
    // Errors: NULL pointers, output buffer too small
    static size_t encode(const uint8_t *input, size_t input_len,
                         uint8_t *output, size_t output_max);

    // Decode: COBS encoded buffer -> decoded output buffer
    // Returns: decoded length, or 0 on error
    // Errors: NULL pointers, output buffer too small, invalid COBS encoding
    static size_t decode(const uint8_t *input, size_t input_len,
                         uint8_t *output, size_t output_max);

    // Streaming decoder for byte-by-byte processing
    // Useful for UART RX where bytes arrive one at a time
    class Decoder
    {
    public:
        enum class State {
            IDLE,      // Waiting for start of packet (next byte is code byte)
            IN_PACKET, // Receiving packet data
            RESYNC     // Discarding bytes until next 0x00 delimiter
        };

        Decoder() : state(State::IDLE), code_byte(0), code_remaining(0),
            frame_len(0), frame_buffer(nullptr), frame_max_len(0) {}

        // Process one byte, returns true if frame complete
        // After returning true, call get_frame() to retrieve the decoded frame
        bool process_byte(uint8_t byte);

        // Signal end of input - completes frame if we've finished the last code block
        // Returns true if frame is now complete
        bool process_end_of_input();

        // Get decoded frame (only valid after process_byte returns true)
        // Returns true on success, false on error
        // On success, *len contains the decoded frame length
        bool get_frame(uint8_t *output, size_t *len, size_t max_len);

        // Reset decoder to IDLE state
        void reset();

        // Force resync - discards bytes until next 0x00 delimiter
        // Use after detecting corruption (CRC error) to recover faster
        void resync();

        // Get current decoder state
        State get_state() const
        {
            return state;
        }

    private:
        State state;
        uint8_t code_byte;
        uint8_t code_remaining;
        size_t frame_len;
        uint8_t *frame_buffer;
        size_t frame_max_len;

        // Internal buffer for decoded frame (max Ethernet frame size + CRC)
        uint8_t internal_buffer[1522 + 4];
    };
};

