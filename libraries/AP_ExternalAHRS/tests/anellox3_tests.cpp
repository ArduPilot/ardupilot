#include <stdio.h>
#include <string.h>
#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_AnelloX3.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace std;

// using the endian conversion function from array of uint8_t's
TEST(AnelloX3Test, EndianConv16)
{
    uint8_t payload[2] = {0x3F,0xAB}; // assume this is little-endian repr
    uint16_t conv_payload = le16toh_ptr(payload);
    EXPECT_NE(conv_payload, 0x3FAB);
    EXPECT_EQ(conv_payload, 0xAB3F);
}


// grabbing from the middle of an array 
TEST(AnelloX3Test, EdianConv16_SelectFromLargerArray)
{
    uint8_t payload[4] = {0x21,0x5C,0x3F,0xAB}; // grabbing starting from second val
    uint16_t conv_payload = le16toh_ptr(payload+1);
    EXPECT_NE(conv_payload, 0x5C3F);
    EXPECT_EQ(conv_payload, 0x3F5C);
}

// iterating over two consecutive 16-bit chunks
TEST(AnelloX3Test, EdianConv16_SequentialSelection)
{
    uint8_t payload[4] = {0x21,0x5C,0x3F,0xAB}; // grabbing starting from second val

    // first 16 bits
    uint16_t conv_payload = le16toh_ptr(payload);
    EXPECT_NE(conv_payload, 0x215C);
    EXPECT_EQ(conv_payload, 0x5C21);

    // second 16 bits -- overwrite previous value
    conv_payload = le16toh_ptr(payload+2);
    EXPECT_NE(conv_payload, 0x5C21);
    EXPECT_NE(conv_payload, 0x3FAB);
    EXPECT_EQ(conv_payload, 0xAB3F);

}


// iterating over two consecutive 16-bit chunks
TEST(AnelloX3Test, UnpackAsLoop)
{
    enum Chunks { CHUNK1, CHUNK2 };
    int payload_length = 4;
    uint8_t payload[4] = {0x21,0x5C,0x3F,0xAB};

    // outputs from parsing
    uint16_t first_chunk, second_chunk;

    int i = 0;
    enum Chunks chunk = CHUNK1;
    while (i < payload_length)
    {
        switch (chunk)
        {
            case CHUNK1:
                // unpack the first chunk
                first_chunk = le16toh_ptr(payload + i);

                // advance parse state and pointer val
                chunk = CHUNK2;
                i += 2;
                break;
            case CHUNK2:
                // unpack the second chunk
                second_chunk = le16toh_ptr(payload + i);

                // done parsing
                i += 2;
                break;
            default:
                cout << "We shouldn't end up here..." << endl;
        }
    }

    // check the output values
    EXPECT_NE(first_chunk, 0x215C);
    EXPECT_EQ(first_chunk, 0x5C21);
    EXPECT_NE(second_chunk, 0x3FAB);
    EXPECT_EQ(second_chunk, 0xAB3F);
}

TEST(AnelloX3Test, PayloadParse)
{
}

AP_GTEST_MAIN()
