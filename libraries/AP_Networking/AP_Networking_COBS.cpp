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

#include "AP_Networking_COBS.h"
#include <AP_Common/AP_Common.h>

/*
  COBS encode: input buffer -> COBS encoded output buffer
  Returns: encoded length, or 0 on error
 */
size_t AP_Networking_COBS::encode(const uint8_t *input, size_t input_len,
                                  uint8_t *output, size_t output_max)
{
    if (input == nullptr || output == nullptr) {
        return 0;
    }

    if (input_len == 0) {
        // Empty input produces empty output
        return 0;
    }

    const uint8_t *src_read_ptr = input;
    const uint8_t *src_end_ptr = src_read_ptr + input_len;
    uint8_t *dst_buf_start_ptr = output;
    uint8_t *dst_buf_end_ptr = dst_buf_start_ptr + output_max;
    uint8_t *dst_code_write_ptr = output;
    uint8_t *dst_write_ptr = dst_code_write_ptr + 1;
    uint8_t search_len = 1;

    // Iterate over the source bytes
    while (src_read_ptr < src_end_ptr) {
        // Check for running out of output buffer space
        if (dst_write_ptr >= dst_buf_end_ptr) {
            return 0; // Output buffer overflow
        }

        uint8_t src_byte = *src_read_ptr++;

        if (src_byte == 0) {
            // We found a zero byte
            *dst_code_write_ptr = search_len;
            dst_code_write_ptr = dst_write_ptr++;
            search_len = 1;
        } else {
            // Copy the non-zero byte to the destination buffer
            *dst_write_ptr++ = src_byte;
            search_len++;

            if (search_len == 0xFF) {
                // We have a long string of non-zero bytes, so we need
                // to write out a length code of 0xFF (254 bytes + zero)
                *dst_code_write_ptr = search_len;
                dst_code_write_ptr = dst_write_ptr++;
                search_len = 1;
            }
        }
    }

    // Finalise the remaining output - write the last code byte
    if (dst_code_write_ptr >= dst_buf_end_ptr) {
        return 0; // Output buffer overflow
    }

    *dst_code_write_ptr = search_len;

    // Calculate and return the output length
    return (size_t)(dst_write_ptr - dst_buf_start_ptr);
}

/*
  COBS decode: COBS encoded buffer -> decoded output buffer
  Returns: decoded length, or 0 on error
 */
size_t AP_Networking_COBS::decode(const uint8_t *input, size_t input_len,
                                  uint8_t *output, size_t output_max)
{
    if (input == nullptr || output == nullptr) {
        return 0;
    }

    if (input_len == 0) {
        // Empty input produces empty output
        return 0;
    }

    const uint8_t *src_read_ptr = input;
    const uint8_t *src_end_ptr = src_read_ptr + input_len;
    uint8_t *dst_buf_start_ptr = output;
    uint8_t *dst_buf_end_ptr = dst_buf_start_ptr + output_max;
    uint8_t *dst_write_ptr = output;

    while (src_read_ptr < src_end_ptr) {
        uint8_t len_code = *src_read_ptr++;

        if (len_code == 0) {
            // Zero byte in input is invalid COBS encoding
            return 0;
        }

        len_code--; // len_code now represents number of non-zero bytes

        // Check length code against remaining input bytes
        size_t remaining_bytes = (size_t)(src_end_ptr - src_read_ptr);
        if (len_code > remaining_bytes) {
            return 0; // Input too short
        }

        // Check length code against remaining output buffer space
        remaining_bytes = (size_t)(dst_buf_end_ptr - dst_write_ptr);
        if (len_code > remaining_bytes) {
            return 0; // Output buffer overflow
        }

        // Copy len_code non-zero bytes
        for (uint8_t i = 0; i < len_code; i++) {
            uint8_t src_byte = *src_read_ptr++;
            if (src_byte == 0) {
                // Zero byte in non-zero section is invalid
                return 0;
            }
            *dst_write_ptr++ = src_byte;
        }

        // Check if we've reached the end of input
        if (src_read_ptr >= src_end_ptr) {
            break;
        }

        // Add a zero to the end (unless len_code was 0xFE, meaning 254 bytes)
        if (len_code != 0xFE) {
            if (dst_write_ptr >= dst_buf_end_ptr) {
                return 0; // Output buffer overflow
            }
            *dst_write_ptr++ = 0;
        }
    }

    return (size_t)(dst_write_ptr - dst_buf_start_ptr);
}

/*
  Streaming decoder: process one byte
  Returns true when frame is complete
 */
bool AP_Networking_COBS::Decoder::process_byte(uint8_t byte)
{
    // Initialize internal buffer on first use
    if (frame_buffer == nullptr) {
        frame_buffer = internal_buffer;
        frame_max_len = ARRAY_SIZE(internal_buffer);
        frame_len = 0;
    }

    // RESYNC state: discard bytes until we see a 0x00 delimiter
    if (state == State::RESYNC) {
        if (byte == 0) {
            // Found delimiter, ready for next frame
            state = State::IDLE;
            frame_len = 0;
        }
        // Stay in RESYNC, discard this byte
        return false;
    }

    if (state == State::IDLE) {
        // Start of new packet - first byte is the code byte
        if (byte == 0) {
            // Zero as first byte means empty frame or consecutive delimiters - ignore
            return false;
        }
        code_byte = byte;
        code_remaining = code_byte - 1; // code_remaining is number of non-zero bytes
        state = State::IN_PACKET;
        frame_len = 0;  // Reset frame length for new packet
        return false;
    }

    // We're in IN_PACKET state
    // Check for frame delimiter (0x00). We require delimiter to mark end-of-frame.
    if (byte == 0) {
        // Frame delimiter received
        if (code_remaining == 0) {
            // We've finished the current code block; complete the frame
            state = State::IDLE;
            return true;
        } else {
            // Zero byte mid-block is invalid - need to resync
            state = State::IDLE;  // This 0x00 IS the delimiter, next byte is code
            frame_len = 0;
            return false;
        }
    }

    // Non-zero byte
    if (code_remaining > 0) {
        // We're reading non-zero bytes for the current code block
        if (frame_len >= frame_max_len) {
            // Buffer overflow - enter resync to find next delimiter
            state = State::RESYNC;
            frame_len = 0;
            return false;
        }
        frame_buffer[frame_len++] = byte;
        code_remaining--;
    } else {
        // code_remaining == 0, we've finished reading the current code block
        // Add zero byte (unless code_byte was 0xFF, meaning 254 bytes)
        if (code_byte != 0xFF) {
            if (frame_len >= frame_max_len) {
                // Buffer overflow - enter resync
                state = State::RESYNC;
                frame_len = 0;
                return false;
            }
            frame_buffer[frame_len++] = 0;
        }

        // This byte is the next code byte starting a new block
        code_byte = byte;
        // Note: code_byte can't be 0 here (checked above)
        code_remaining = code_byte - 1;
        // Do not mark frame complete here; we require the delimiter (0x00)
    }

    return false;
}

/*
  Signal end of input - completes frame if we've finished the last code block
 */
bool AP_Networking_COBS::Decoder::process_end_of_input()
{
    if (state != State::IN_PACKET) {
        return false;
    }

    // If we've finished reading the last code block (code_remaining == 0),
    // the frame is complete
    if (code_remaining == 0) {
        state = State::IDLE;
        return true;
    }

    return false;
}

/*
  Get decoded frame after process_byte returns true
 */
bool AP_Networking_COBS::Decoder::get_frame(uint8_t *output, size_t *len, size_t max_len)
{
    if (state != State::IDLE || frame_buffer == nullptr || frame_len == 0) {
        return false;
    }

    if (output == nullptr || len == nullptr) {
        return false;
    }

    if (frame_len > max_len) {
        return false;
    }

    // Copy frame to output buffer
    for (size_t i = 0; i < frame_len; i++) {
        output[i] = frame_buffer[i];
    }

    *len = frame_len;
    return true;
}

/*
  Reset decoder to IDLE state
 */
void AP_Networking_COBS::Decoder::reset()
{
    state = State::IDLE;
    code_byte = 0;
    code_remaining = 0;
    frame_len = 0;
    // Keep frame_buffer pointing to internal_buffer for reuse
}

/*
  Force decoder into RESYNC state - discards bytes until next delimiter
  Use this after detecting corruption (e.g., CRC error) to recover faster
 */
void AP_Networking_COBS::Decoder::resync()
{
    state = State::RESYNC;
    code_byte = 0;
    code_remaining = 0;
    frame_len = 0;
}

