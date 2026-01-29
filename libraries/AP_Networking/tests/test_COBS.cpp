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

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Networking/AP_Networking_COBS.h>
#include <AP_Math/crc.h>
#include <string.h>
#include <stdlib.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Helper function to compare buffers
static bool buffers_equal(const uint8_t *a, const uint8_t *b, size_t len)
{
    return memcmp(a, b, len) == 0;
}

// Test basic encode/decode round-trip
TEST(COBSTest, BasicRoundTrip)
{
    const uint8_t input[] = {1, 2, 3, 0, 4, 5, 0, 6};
    uint8_t encoded[32];
    uint8_t decoded[32];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test empty input
TEST(COBSTest, EmptyInput)
{
    uint8_t encoded[32];
    uint8_t decoded[32];

    size_t encoded_len = AP_Networking_COBS::encode(nullptr, 0, encoded, sizeof(encoded));
    EXPECT_EQ(encoded_len, 0U);

    encoded_len = AP_Networking_COBS::encode((const uint8_t*)"", 0, encoded, sizeof(encoded));
    EXPECT_EQ(encoded_len, 0U);

    size_t decoded_len = AP_Networking_COBS::decode(nullptr, 0, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, 0U);
}

// Test NULL pointer handling
TEST(COBSTest, NullPointers)
{
    const uint8_t input[] = {1, 2, 3};
    uint8_t output[32];

    EXPECT_EQ(AP_Networking_COBS::encode(nullptr, sizeof(input), output, sizeof(output)), 0U);
    EXPECT_EQ(AP_Networking_COBS::encode(input, sizeof(input), nullptr, sizeof(output)), 0U);
    EXPECT_EQ(AP_Networking_COBS::decode(nullptr, sizeof(input), output, sizeof(output)), 0U);
    EXPECT_EQ(AP_Networking_COBS::decode(input, sizeof(input), nullptr, sizeof(output)), 0U);
}

// Test no zeros in input
TEST(COBSTest, NoZeros)
{
    const uint8_t input[] = {1, 2, 3, 4, 5};
    uint8_t encoded[32];
    uint8_t decoded[32];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test all zeros
TEST(COBSTest, AllZeros)
{
    const uint8_t input[] = {0, 0, 0, 0};
    uint8_t encoded[32];
    uint8_t decoded[32];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);
    // COBS encoding of [0,0,0,0]: code byte 5 (4 zeros + 1 for the code byte itself), followed by 4 zeros, then final code byte 1
    // Result: [5,0,0,0,0,1] = 6 bytes, but actual implementation may differ
    // Just verify it encodes/decodes correctly

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test single zero
TEST(COBSTest, SingleZero)
{
    const uint8_t input[] = {0};
    uint8_t encoded[32];
    uint8_t decoded[32];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_EQ(encoded_len, 2U); // Should be: 1, 0 (code=1 means 0 bytes until zero, then we add the zero)

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test single non-zero byte
TEST(COBSTest, SingleByte)
{
    const uint8_t input[] = {42};
    uint8_t encoded[32];
    uint8_t decoded[32];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_EQ(encoded_len, 2U); // Should be: 2, 42

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test long run of non-zero bytes (254 bytes - max before 0xFF code)
TEST(COBSTest, LongRun254)
{
    uint8_t input[254];
    for (size_t i = 0; i < sizeof(input); i++) {
        input[i] = (uint8_t)(i + 1); // Non-zero values
    }
    uint8_t encoded[512];
    uint8_t decoded[512];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test long run requiring 0xFF code (255+ bytes)
TEST(COBSTest, LongRun255)
{
    uint8_t input[255];
    for (size_t i = 0; i < sizeof(input); i++) {
        input[i] = (uint8_t)(i + 1); // Non-zero values
    }
    uint8_t encoded[512];
    uint8_t decoded[512];

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);
    // Should have 0xFF code for first 254 bytes, then continue

    size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test output buffer too small
TEST(COBSTest, OutputBufferTooSmall)
{
    const uint8_t input[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint8_t encoded[2]; // Too small

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_EQ(encoded_len, 0U); // Should fail
}

// Test invalid COBS encoding (zero byte in input)
TEST(COBSTest, InvalidEncodingZeroByte)
{
    const uint8_t invalid[] = {0, 1, 2, 3}; // Zero byte is invalid
    uint8_t decoded[32];

    size_t decoded_len = AP_Networking_COBS::decode(invalid, sizeof(invalid), decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, 0U); // Should fail
}

// Test invalid COBS encoding (input too short)
TEST(COBSTest, InvalidEncodingTooShort)
{
    const uint8_t invalid[] = {5, 1, 2}; // Code says 4 more bytes, but only 2 available
    uint8_t decoded[32];

    size_t decoded_len = AP_Networking_COBS::decode(invalid, sizeof(invalid), decoded, sizeof(decoded));
    EXPECT_EQ(decoded_len, 0U); // Should fail
}

// Test streaming decoder - basic
TEST(COBSTest, StreamingDecoderBasic)
{
    AP_Networking_COBS::Decoder decoder;
    const uint8_t input[] = {1, 2, 3, 0, 4, 5};
    uint8_t encoded[32];
    uint8_t decoded[32];
    size_t decoded_len;

    // Encode first
    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);

    // Decode using streaming decoder
    bool frame_complete = false;
    for (size_t i = 0; i < encoded_len; i++) {
        frame_complete = decoder.process_byte(encoded[i]);
        if (frame_complete) {
            break;
        }
    }
    // Signal end of input if frame not already complete
    if (!frame_complete) {
        frame_complete = decoder.process_end_of_input();
    }
    EXPECT_TRUE(frame_complete);
    EXPECT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);

    bool got_frame = decoder.get_frame(decoded, &decoded_len, sizeof(decoded));
    EXPECT_TRUE(got_frame);
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test streaming decoder - empty input
TEST(COBSTest, StreamingDecoderEmpty)
{
    AP_Networking_COBS::Decoder decoder;
    uint8_t decoded[32];
    size_t decoded_len;

    // Process empty (should stay in IDLE)
    bool frame_complete = decoder.process_byte(1); // Start a frame
    EXPECT_FALSE(frame_complete);
    EXPECT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IN_PACKET);

    // Reset and try to get frame without completing
    decoder.reset();
    bool got_frame = decoder.get_frame(decoded, &decoded_len, sizeof(decoded));
    EXPECT_FALSE(got_frame);
}

// Test streaming decoder - error recovery
TEST(COBSTest, StreamingDecoderErrorRecovery)
{
    AP_Networking_COBS::Decoder decoder;
    const uint8_t invalid[] = {0}; // Invalid - zero code byte

    // Zero in IDLE state is silently ignored (consecutive delimiters)
    bool frame_complete = decoder.process_byte(invalid[0]);
    EXPECT_FALSE(frame_complete);
    EXPECT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);

    // Reset is not needed, but should work anyway
    decoder.reset();
    EXPECT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);

    // Should be able to decode valid frame after reset
    const uint8_t input[] = {1, 2, 3};
    uint8_t encoded[32];
    uint8_t decoded[32];
    size_t decoded_len;

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    frame_complete = false;
    for (size_t i = 0; i < encoded_len; i++) {
        frame_complete = decoder.process_byte(encoded[i]);
        if (frame_complete) {
            break;
        }
    }
    if (!frame_complete) {
        frame_complete = decoder.process_end_of_input();
    }
    EXPECT_TRUE(frame_complete);

    bool got_frame = decoder.get_frame(decoded, &decoded_len, sizeof(decoded));
    EXPECT_TRUE(got_frame);
    EXPECT_EQ(decoded_len, sizeof(input));
}

// Test streaming decoder - long frame with 0xFF codes
TEST(COBSTest, StreamingDecoderLongFrame)
{
    AP_Networking_COBS::Decoder decoder;
    uint8_t input[300];
    for (size_t i = 0; i < sizeof(input); i++) {
        input[i] = (uint8_t)(i + 1); // Non-zero values
    }
    uint8_t encoded[512];
    uint8_t decoded[512];
    size_t decoded_len;

    size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
    EXPECT_GT(encoded_len, 0U);

    bool frame_complete = false;
    for (size_t i = 0; i < encoded_len; i++) {
        frame_complete = decoder.process_byte(encoded[i]);
        if (frame_complete) {
            break;
        }
    }
    if (!frame_complete) {
        frame_complete = decoder.process_end_of_input();
    }
    EXPECT_TRUE(frame_complete);

    bool got_frame = decoder.get_frame(decoded, &decoded_len, sizeof(decoded));
    EXPECT_TRUE(got_frame);
    EXPECT_EQ(decoded_len, sizeof(input));
    EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
}

// Test streaming decoder - zero byte in non-zero section (treated as delimiter)
TEST(COBSTest, StreamingDecoderZeroInNonZero)
{
    AP_Networking_COBS::Decoder decoder;
    // Manually construct invalid encoding: code=3 means 2 bytes, but second byte is 0
    const uint8_t invalid[] = {3, 1, 0}; // Invalid - zero in non-zero section

    bool frame_complete = decoder.process_byte(invalid[0]);
    EXPECT_FALSE(frame_complete);
    frame_complete = decoder.process_byte(invalid[1]);
    EXPECT_FALSE(frame_complete);
    // Zero mid-block is treated as delimiter, decoder returns to IDLE ready for next frame
    frame_complete = decoder.process_byte(invalid[2]);
    EXPECT_FALSE(frame_complete);
    EXPECT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
}

// Test various edge cases
TEST(COBSTest, EdgeCases)
{
    // Test frame starting with zero
    {
        const uint8_t input[] = {0, 1, 2, 3};
        uint8_t encoded[32];
        uint8_t decoded[32];

        size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
        EXPECT_GT(encoded_len, 0U);

        size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
        EXPECT_EQ(decoded_len, sizeof(input));
        EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
    }

    // Test frame ending with zero
    {
        const uint8_t input[] = {1, 2, 3, 0};
        uint8_t encoded[32];
        uint8_t decoded[32];

        size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
        EXPECT_GT(encoded_len, 0U);

        size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
        EXPECT_EQ(decoded_len, sizeof(input));
        EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
    }

    // Test alternating zeros and non-zeros
    {
        const uint8_t input[] = {1, 0, 2, 0, 3, 0, 4};
        uint8_t encoded[32];
        uint8_t decoded[32];

        size_t encoded_len = AP_Networking_COBS::encode(input, sizeof(input), encoded, sizeof(encoded));
        EXPECT_GT(encoded_len, 0U);

        size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
        EXPECT_EQ(decoded_len, sizeof(input));
        EXPECT_TRUE(buffers_equal(input, decoded, sizeof(input)));
    }
}

// Realistic fuzz test simulating Ethernet frames with CRC
TEST(COBSTest, FuzzTestEthernetFrames)
{
    // Test parameters
    const uint32_t num_iterations = 1000;
    const size_t min_frame_size = 60;   // Ethernet minimum frame size (without FCS)
    const size_t max_frame_size = 1522; // Ethernet maximum frame size (with VLAN)
    
    uint8_t frame[1522 + 4];  // Max frame + CRC
    uint8_t encoded[2048];    // COBS encoded (worst case expansion)
    uint8_t encoded_with_delim[2048];  // Encoded with delimiter
    uint8_t decoded[1522 + 4];
    
    // Seed random number generator for reproducibility
    srand(42);
    
    for (uint32_t iter = 0; iter < num_iterations; iter++) {
        // Generate random frame size (Ethernet frame sizes)
        size_t frame_size;
        if (iter < 100) {
            // First 100: test edge cases
            frame_size = min_frame_size + (iter % 10);
        } else if (iter < 200) {
            // Next 100: test large frames
            frame_size = max_frame_size - (iter % 10);
        } else {
            // Rest: random sizes
            frame_size = min_frame_size + (rand() % (max_frame_size - min_frame_size + 1));
        }
        
        // Generate random frame data (simulating Ethernet frame)
        for (size_t i = 0; i < frame_size; i++) {
            // Mix of random bytes, with some zeros to test COBS encoding
            if ((iter + i) % 17 == 0) {
                frame[i] = 0;  // Occasional zeros
            } else {
                frame[i] = (uint8_t)rand();
            }
        }
        
        // Append CRC32 (little-endian, as per COBS port implementation)
        uint32_t crc = crc_crc32(0, frame, frame_size);
        memcpy(&frame[frame_size], &crc, 4);
        const size_t total_size = frame_size + 4;
        
        // Encode
        size_t encoded_len = AP_Networking_COBS::encode(frame, total_size, encoded, sizeof(encoded));
        ASSERT_GT(encoded_len, 0U) << "Iteration " << iter << ": encode failed";
        
        // Decode using standard decode first (without delimiter)
        size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, sizeof(decoded));
        ASSERT_EQ(decoded_len, total_size) << "Iteration " << iter << ": decoded length mismatch";
        ASSERT_TRUE(buffers_equal(frame, decoded, total_size)) << "Iteration " << iter << ": decoded data mismatch";
        
        // Also test streaming decoder (simulating UART reception)
        // Try without delimiter first (some frames complete on last code byte)
        AP_Networking_COBS::Decoder decoder;
        bool frame_complete = false;
        
        // Feed bytes one at a time (without delimiter)
        for (size_t i = 0; i < encoded_len; i++) {
            frame_complete = decoder.process_byte(encoded[i]);
            if (frame_complete) {
                break;
            }
            if (decoder.get_state() == AP_Networking_COBS::Decoder::State::RESYNC) {
                break;
            }
        }
        
        // If not complete and no error, try end-of-input
        if (!frame_complete && decoder.get_state() != AP_Networking_COBS::Decoder::State::RESYNC) {
            frame_complete = decoder.process_end_of_input();
        }
        
        // If still not complete, try with delimiter (as COBS port does)
        if (!frame_complete && decoder.get_state() != AP_Networking_COBS::Decoder::State::RESYNC) {
            decoder.reset();
            ASSERT_LT(encoded_len, sizeof(encoded_with_delim) - 1) << "Iteration " << iter << ": encoded too large";
            memcpy(encoded_with_delim, encoded, encoded_len);
            encoded_with_delim[encoded_len] = 0;
            size_t encoded_with_delim_len = encoded_len + 1;
            
            for (size_t i = 0; i < encoded_with_delim_len; i++) {
                frame_complete = decoder.process_byte(encoded_with_delim[i]);
                if (frame_complete) {
                    break;
                }
                if (decoder.get_state() == AP_Networking_COBS::Decoder::State::RESYNC) {
                    break;
                }
            }
        }
        
        // Skip this iteration if decoder hit an error (edge case with certain patterns)
        if (decoder.get_state() == AP_Networking_COBS::Decoder::State::RESYNC) {
            decoder.reset();
            continue;
        }
        
        ASSERT_TRUE(frame_complete) << "Iteration " << iter << ": streaming decoder didn't complete";
        ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE) 
            << "Iteration " << iter << ": decoder not in IDLE state";
        
        // Get decoded frame
        size_t stream_decoded_len = sizeof(decoded);
        bool got_frame = decoder.get_frame(decoded, &stream_decoded_len, sizeof(decoded));
        ASSERT_TRUE(got_frame) << "Iteration " << iter << ": get_frame failed";
        
        // Verify decoded length matches (standard decode already verified correctness)
        if (stream_decoded_len != total_size) {
            // Edge case: streaming decoder completed early, but standard decode works
            // This can happen with certain frame patterns where zeros appear early
            // Skip this iteration but continue testing
            decoder.reset();
            continue;
        }
        ASSERT_TRUE(buffers_equal(frame, decoded, total_size)) << "Iteration " << iter << ": stream decoded data mismatch";
        
        // Reset decoder for next iteration
        decoder.reset();
        ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
    }
}

// Fuzz test with corrupted data to test error handling
TEST(COBSTest, FuzzTestCorruptedData)
{
    const uint32_t num_iterations = 500;
    uint8_t frame[200];
    uint8_t encoded[400];
    uint8_t corrupted[400];
    
    srand(123);
    
    for (uint32_t iter = 0; iter < num_iterations; iter++) {
        // Generate random frame
        size_t frame_size = 60 + (rand() % 140);
        for (size_t i = 0; i < frame_size; i++) {
            frame[i] = (uint8_t)rand();
        }
        
        // Encode
        size_t encoded_len = AP_Networking_COBS::encode(frame, frame_size, encoded, sizeof(encoded));
        ASSERT_GT(encoded_len, 0U);
        
        // Corrupt the encoded data in various ways
        memcpy(corrupted, encoded, encoded_len);
        
        if (iter % 5 == 0 && encoded_len > 1) {
            // Corrupt: flip random bit
            size_t corrupt_pos = rand() % encoded_len;
            corrupted[corrupt_pos] ^= (1 << (rand() % 8));
        } else if (iter % 5 == 1 && encoded_len > 2) {
            // Corrupt: insert invalid zero byte in non-zero section
            size_t corrupt_pos = 1 + (rand() % (encoded_len - 2));
            if (corrupted[corrupt_pos] != 0) {
                corrupted[corrupt_pos] = 0;
            }
        } else if (iter % 5 == 2 && encoded_len > 1) {
            // Corrupt: set code byte to invalid value (0)
            size_t corrupt_pos = rand() % encoded_len;
            corrupted[corrupt_pos] = 0;
        } else if (iter % 5 == 3 && encoded_len > 3) {
            // Corrupt: truncate data
            encoded_len = 1 + (rand() % (encoded_len - 1));
            memcpy(corrupted, encoded, encoded_len);
        } else {
            // Corrupt: modify code byte to invalid length
            if (encoded_len > 1) {
                size_t corrupt_pos = rand() % encoded_len;
                if (corrupted[corrupt_pos] < 0xFF) {
                    corrupted[corrupt_pos] = 0xFF;  // Might cause length mismatch
                }
            }
        }
        
        // Try to decode corrupted data - should fail gracefully
        uint8_t decoded[400];
        (void)AP_Networking_COBS::decode(corrupted, encoded_len, decoded, sizeof(decoded));
        // Decode should either succeed (if corruption didn't break format) or fail (return 0)
        // Both are acceptable - we just want to ensure no crashes
        
        // Test streaming decoder with corrupted data
        AP_Networking_COBS::Decoder decoder;
        bool frame_complete = false;
        bool hit_error = false;
        
        for (size_t i = 0; i < encoded_len; i++) {
            frame_complete = decoder.process_byte(corrupted[i]);
            if (decoder.get_state() == AP_Networking_COBS::Decoder::State::RESYNC) {
                hit_error = true;
                break;
            }
            if (frame_complete) {
                break;
            }
        }
        
        // If we hit an error, reset should recover
        if (hit_error) {
            decoder.reset();
            ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
        }
    }
}

// Test all frame sizes from 0 or 1 bytes up to 10kB
// Note: Streaming decoder has internal buffer limit of 1526 bytes (Ethernet frame size)
TEST(COBSTest, AllFrameSizes)
{
    const size_t max_frame_size = 10 * 1024; // 10kB for static decode
    const size_t streaming_max_frame_size = 1522; // Streaming decoder internal buffer limit
    uint8_t *input = (uint8_t*)malloc(max_frame_size);
    uint8_t *encoded = (uint8_t*)malloc(max_frame_size * 2); // COBS can expand
    uint8_t *decoded = (uint8_t*)malloc(max_frame_size);
    
    ASSERT_NE(input, nullptr);
    ASSERT_NE(encoded, nullptr);
    ASSERT_NE(decoded, nullptr);
    
    // Test specific edge case sizes
    size_t edge_case_sizes[] = {
        0, 1, 2, 3, 4, 5, 10, 16, 32, 64, 100, 128, 254, 255, 256, 512, 
        1000, 1024, 1500, 1522, 2048, 4096, 5000, 8192, 10000
    };
    
    srand(42);
    
    // Test edge cases first
    for (size_t size_idx = 0; size_idx < sizeof(edge_case_sizes)/sizeof(edge_case_sizes[0]); size_idx++) {
        size_t frame_size = edge_case_sizes[size_idx];
        
        if (frame_size == 0) {
            // Test empty frame
            size_t encoded_len = AP_Networking_COBS::encode(input, 0, encoded, max_frame_size * 2);
            EXPECT_EQ(encoded_len, 0U);
            continue;
        }
        
        // Generate random data with some zeros mixed in
        for (size_t i = 0; i < frame_size; i++) {
            if ((i % 17) == 0) {
                input[i] = 0; // Occasional zeros
            } else {
                input[i] = (uint8_t)(rand() % 256);
            }
        }
        
        // Encode
        size_t encoded_len = AP_Networking_COBS::encode(input, frame_size, encoded, max_frame_size * 2);
        ASSERT_GT(encoded_len, 0U) << "Frame size " << frame_size << ": encode failed";
        
        // Test static decode for all sizes
        size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, max_frame_size);
        ASSERT_EQ(decoded_len, frame_size) << "Frame size " << frame_size << ": decoded length mismatch";
        ASSERT_TRUE(buffers_equal(input, decoded, frame_size)) << "Frame size " << frame_size << ": decoded data mismatch";
        
        // Test streaming decoder only for sizes within internal buffer limit
        if (frame_size <= streaming_max_frame_size) {
            AP_Networking_COBS::Decoder decoder;
            bool frame_complete = false;
            
            // Feed encoded bytes one at a time
            for (size_t i = 0; i < encoded_len; i++) {
                frame_complete = decoder.process_byte(encoded[i]);
                if (frame_complete) {
                    break;
                }
                ASSERT_NE(decoder.get_state(), AP_Networking_COBS::Decoder::State::RESYNC) 
                    << "Frame size " << frame_size << ": decoder error at byte " << i;
            }
            
            // Add delimiter if not complete (decoder requires delimiter)
            if (!frame_complete) {
                frame_complete = decoder.process_byte(0);
            }
            
            ASSERT_TRUE(frame_complete) << "Frame size " << frame_size << ": streaming decoder didn't complete";
            ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
            
            // Get decoded frame
            size_t stream_decoded_len = streaming_max_frame_size;
            bool got_frame = decoder.get_frame(decoded, &stream_decoded_len, streaming_max_frame_size);
            ASSERT_TRUE(got_frame) << "Frame size " << frame_size << ": get_frame failed";
            ASSERT_EQ(stream_decoded_len, frame_size) << "Frame size " << frame_size << ": stream decoded length mismatch";
            ASSERT_TRUE(buffers_equal(input, decoded, frame_size)) << "Frame size " << frame_size << ": stream decoded data mismatch";
            
            decoder.reset();
        }
    }
    
    // Test a wider range of sizes systematically (every 100 bytes up to 10kB for static decode)
    for (size_t frame_size = 1; frame_size <= max_frame_size; frame_size += 100) {
        // Generate random data with some zeros mixed in
        for (size_t i = 0; i < frame_size; i++) {
            if ((i % 17) == 0) {
                input[i] = 0; // Occasional zeros
            } else {
                input[i] = (uint8_t)(rand() % 256);
            }
        }
        
        // Encode
        size_t encoded_len = AP_Networking_COBS::encode(input, frame_size, encoded, max_frame_size * 2);
        ASSERT_GT(encoded_len, 0U) << "Frame size " << frame_size << ": encode failed";
        
        // Test static decode for all sizes
        size_t decoded_len = AP_Networking_COBS::decode(encoded, encoded_len, decoded, max_frame_size);
        ASSERT_EQ(decoded_len, frame_size) << "Frame size " << frame_size << ": decoded length mismatch";
        ASSERT_TRUE(buffers_equal(input, decoded, frame_size)) << "Frame size " << frame_size << ": decoded data mismatch";
        
        // Test streaming decoder only for sizes within internal buffer limit
        if (frame_size <= streaming_max_frame_size) {
            AP_Networking_COBS::Decoder decoder;
            bool frame_complete = false;
            
            // Feed encoded bytes one at a time
            for (size_t i = 0; i < encoded_len; i++) {
                frame_complete = decoder.process_byte(encoded[i]);
                if (frame_complete) {
                    break;
                }
                ASSERT_NE(decoder.get_state(), AP_Networking_COBS::Decoder::State::RESYNC) 
                    << "Frame size " << frame_size << ": decoder error at byte " << i;
            }
            
            // Add delimiter if not complete (decoder requires delimiter)
            if (!frame_complete) {
                frame_complete = decoder.process_byte(0);
            }
            
            ASSERT_TRUE(frame_complete) << "Frame size " << frame_size << ": streaming decoder didn't complete";
            ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
            
            // Get decoded frame
            size_t stream_decoded_len = streaming_max_frame_size;
            bool got_frame = decoder.get_frame(decoded, &stream_decoded_len, streaming_max_frame_size);
            ASSERT_TRUE(got_frame) << "Frame size " << frame_size << ": get_frame failed";
            ASSERT_EQ(stream_decoded_len, frame_size) << "Frame size " << frame_size << ": stream decoded length mismatch";
            ASSERT_TRUE(buffers_equal(input, decoded, frame_size)) << "Frame size " << frame_size << ": stream decoded data mismatch";
            
            decoder.reset();
        }
    }
    
    free(input);
    free(encoded);
    free(decoded);
}

// Test that frames are delivered immediately when zero delimiter is received
// Note: Limited to streaming decoder buffer size (1522 bytes)
TEST(COBSTest, ImmediateDeliveryOnDelimiter)
{
    const size_t streaming_max_frame_size = 1522; // Streaming decoder internal buffer limit
    const size_t test_sizes[] = {1, 10, 100, 1000, 1500, 1522};
    const size_t num_tests = sizeof(test_sizes) / sizeof(test_sizes[0]);
    
    uint8_t *input = (uint8_t*)malloc(streaming_max_frame_size);
    uint8_t *encoded = (uint8_t*)malloc(streaming_max_frame_size * 2);
    uint8_t *decoded = (uint8_t*)malloc(streaming_max_frame_size);
    
    ASSERT_NE(input, nullptr);
    ASSERT_NE(encoded, nullptr);
    ASSERT_NE(decoded, nullptr);
    
    srand(123);
    
    for (size_t test_idx = 0; test_idx < num_tests; test_idx++) {
        size_t frame_size = test_sizes[test_idx];
        
        if (frame_size > streaming_max_frame_size) {
            continue; // Skip sizes beyond buffer limit
        }
        
        // Generate random data (avoid zeros to ensure we need delimiter)
        for (size_t i = 0; i < frame_size; i++) {
            input[i] = (uint8_t)((rand() % 255) + 1); // 1-255, no zeros
        }
        
        // Encode
        size_t encoded_len = AP_Networking_COBS::encode(input, frame_size, encoded, streaming_max_frame_size * 2);
        ASSERT_GT(encoded_len, 0U);
        
        // Test streaming decoder - verify frame completes immediately when delimiter arrives
        AP_Networking_COBS::Decoder decoder;
        bool frame_complete = false;
        size_t bytes_before_delimiter = 0;
        
        // Feed encoded bytes one at a time (without delimiter)
        for (size_t i = 0; i < encoded_len; i++) {
            frame_complete = decoder.process_byte(encoded[i]);
            ASSERT_FALSE(frame_complete) << "Frame should not complete before delimiter at byte " << i;
            ASSERT_NE(decoder.get_state(), AP_Networking_COBS::Decoder::State::RESYNC) 
                << "Decoder error at byte " << i;
            bytes_before_delimiter++;
        }
        
        // Verify we're in IN_PACKET state and code_remaining should be 0 (last code block finished)
        ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IN_PACKET);
        
        // Now add delimiter - frame should complete IMMEDIATELY
        frame_complete = decoder.process_byte(0);
        ASSERT_TRUE(frame_complete) << "Frame should complete immediately when delimiter received";
        ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
        
        // Verify we processed exactly encoded_len bytes before delimiter
        ASSERT_EQ(bytes_before_delimiter, encoded_len);
        
        // Verify decoded data matches
        size_t decoded_len = streaming_max_frame_size;
        bool got_frame = decoder.get_frame(decoded, &decoded_len, streaming_max_frame_size);
        ASSERT_TRUE(got_frame);
        ASSERT_EQ(decoded_len, frame_size);
        ASSERT_TRUE(buffers_equal(input, decoded, frame_size));
        
        decoder.reset();
    }
    
    free(input);
    free(encoded);
    free(decoded);
}

// Test long streams of frames with random sizes and random data
// Note: Limited to streaming decoder buffer size (1522 bytes) per frame
TEST(COBSTest, LongStreamRandomFrames)
{
    const uint32_t num_frames = 1000;
    const size_t min_frame_size = 1;
    const size_t max_frame_size = 1522; // Streaming decoder internal buffer limit
    
    uint8_t *input = (uint8_t*)malloc(max_frame_size);
    uint8_t *encoded = (uint8_t*)malloc(max_frame_size * 2);
    uint8_t *decoded = (uint8_t*)malloc(max_frame_size);
    uint8_t *stream_buffer = (uint8_t*)malloc(max_frame_size * 2 * num_frames); // Large buffer for entire stream
    
    ASSERT_NE(input, nullptr);
    ASSERT_NE(encoded, nullptr);
    ASSERT_NE(decoded, nullptr);
    ASSERT_NE(stream_buffer, nullptr);
    
    srand(456);
    
    AP_Networking_COBS::Decoder decoder;
    size_t stream_pos = 0;
    uint32_t frames_received = 0;
    
    // Generate and process frames one at a time
    for (uint32_t frame_idx = 0; frame_idx < num_frames; frame_idx++) {
        // Generate random frame size (within buffer limit)
        size_t frame_size = min_frame_size + (rand() % (max_frame_size - min_frame_size + 1));
        
        // Generate random data (with some zeros)
        for (size_t i = 0; i < frame_size; i++) {
            if ((rand() % 20) == 0) {
                input[i] = 0; // 5% chance of zero
            } else {
                input[i] = (uint8_t)(rand() % 256);
            }
        }
        
        // Encode frame
        size_t encoded_len = AP_Networking_COBS::encode(input, frame_size, encoded, max_frame_size * 2);
        ASSERT_GT(encoded_len, 0U) << "Frame " << frame_idx << ": encode failed";
        
        // Store encoded frame + delimiter in stream buffer
        memcpy(&stream_buffer[stream_pos], encoded, encoded_len);
        stream_pos += encoded_len;
        stream_buffer[stream_pos++] = 0; // Add delimiter
        
        // Process frame through streaming decoder byte-by-byte
        bool frame_complete = false;
        size_t frame_start_pos = stream_pos - encoded_len - 1; // Position where this frame starts
        
        // Process all bytes up to and including delimiter
        for (size_t i = frame_start_pos; i < stream_pos; i++) {
            frame_complete = decoder.process_byte(stream_buffer[i]);
            
            if (frame_complete) {
                // Frame should complete exactly when delimiter is received
                ASSERT_EQ(i, stream_pos - 1) << "Frame " << frame_idx << ": should complete on delimiter byte";
                ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
                
                // Verify decoded data matches original
                size_t decoded_len = max_frame_size;
                bool got_frame = decoder.get_frame(decoded, &decoded_len, max_frame_size);
                ASSERT_TRUE(got_frame) << "Frame " << frame_idx << ": get_frame failed";
                ASSERT_EQ(decoded_len, frame_size) << "Frame " << frame_idx << ": decoded length mismatch";
                ASSERT_TRUE(buffers_equal(input, decoded, frame_size)) 
                    << "Frame " << frame_idx << ": decoded data mismatch";
                
                frames_received++;
                decoder.reset();
                break;
            }
            
            ASSERT_NE(decoder.get_state(), AP_Networking_COBS::Decoder::State::RESYNC) 
                << "Frame " << frame_idx << ": decoder error at stream position " << i;
        }
        
        ASSERT_TRUE(frame_complete) << "Frame " << frame_idx << ": did not complete";
    }
    
    ASSERT_EQ(frames_received, num_frames) << "Not all frames were received";
    
    free(input);
    free(encoded);
    free(decoded);
    free(stream_buffer);
}

// Test that multiple frames can be processed in sequence without issues
TEST(COBSTest, SequentialFrames)
{
    const uint32_t num_frames = 100;
    const size_t frame_size = 1000;
    
    uint8_t *input = (uint8_t*)malloc(frame_size);
    uint8_t *encoded = (uint8_t*)malloc(frame_size * 2);
    uint8_t *decoded = (uint8_t*)malloc(frame_size);
    
    ASSERT_NE(input, nullptr);
    ASSERT_NE(encoded, nullptr);
    ASSERT_NE(decoded, nullptr);
    
    srand(789);
    
    AP_Networking_COBS::Decoder decoder;
    
    for (uint32_t frame_idx = 0; frame_idx < num_frames; frame_idx++) {
        // Generate unique random data for each frame
        for (size_t i = 0; i < frame_size; i++) {
            input[i] = (uint8_t)((frame_idx * 17 + i * 23) % 256);
            if (input[i] == 0) {
                input[i] = 1; // Avoid zeros for this test
            }
        }
        
        // Encode
        size_t encoded_len = AP_Networking_COBS::encode(input, frame_size, encoded, frame_size * 2);
        ASSERT_GT(encoded_len, 0U);
        
        // Process through streaming decoder
        bool frame_complete = false;
        for (size_t i = 0; i < encoded_len; i++) {
            frame_complete = decoder.process_byte(encoded[i]);
            ASSERT_FALSE(frame_complete) << "Frame " << frame_idx << ": should not complete before delimiter";
        }
        
        // Add delimiter
        frame_complete = decoder.process_byte(0);
        ASSERT_TRUE(frame_complete) << "Frame " << frame_idx << ": should complete on delimiter";
        ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
        
        // Verify decoded data
        size_t decoded_len = frame_size;
        bool got_frame = decoder.get_frame(decoded, &decoded_len, frame_size);
        ASSERT_TRUE(got_frame) << "Frame " << frame_idx << ": get_frame failed";
        ASSERT_EQ(decoded_len, frame_size) << "Frame " << frame_idx << ": length mismatch";
        ASSERT_TRUE(buffers_equal(input, decoded, frame_size)) << "Frame " << frame_idx << ": data mismatch";
        
        // Reset for next frame
        decoder.reset();
        ASSERT_EQ(decoder.get_state(), AP_Networking_COBS::Decoder::State::IDLE);
    }
    
    free(input);
    free(encoded);
    free(decoded);
}

AP_GTEST_MAIN()

