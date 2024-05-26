/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/marshal/bit_stream.hpp>
#include <uavcan/transport/transfer_buffer.hpp>


TEST(BitStream, ToString)
{
    {
        uavcan::StaticTransferBuffer<8> buf;
        uavcan::BitStream bs(buf);
        ASSERT_EQ("", bs.toString());
        ASSERT_EQ("", bs.toString());
    }
    {
        const uint8_t data[] = {0xad};   // 10101101
        uavcan::StaticTransferBuffer<8> buf;
        uavcan::BitStream bs(buf);
        ASSERT_EQ(1, bs.write(data, 8)); // all 8
        ASSERT_EQ("10101101", bs.toString());
    }
    {
        const uint8_t data[] = {0xad, 0xbe};  // 10101101 10111110
        uavcan::StaticTransferBuffer<8> buf;
        uavcan::BitStream bs(buf);
        ASSERT_EQ(1, bs.write(data, 16));     // all 16
        ASSERT_EQ("10101101 10111110", bs.toString());
    }
    {
        const uint8_t data[] = {0xad, 0xbe, 0xfc};  // 10101101 10111110 11111100
        uavcan::StaticTransferBuffer<8> buf;
        uavcan::BitStream bs(buf);
        ASSERT_EQ(1, bs.write(data, 20));           // 10101101 10111110 1111
        ASSERT_EQ("10101101 10111110 11110000", bs.toString());
    }
}


TEST(BitStream, BitOrderSimple)
{
    /*
     * a = 1010
     * b = 1011
     * c = 1100
     * d = 1101
     * e = 1110
     * f = 1111
     */
    uavcan::StaticTransferBuffer<32> buf;
    {   // Write
        const uint8_t data[] = {0xad, 0xbe};            // adbe
        uavcan::BitStream bs(buf);
        ASSERT_EQ(1, bs.write(data, 12));               // adb0
        ASSERT_EQ("10101101 10110000", bs.toString());  // adb0
    }
    {   // Read
        uavcan::BitStream bs(buf);
        ASSERT_EQ("10101101 10110000", bs.toString());  // Same data
        uint8_t data[] = {0xFF, 0xFF};                  // Uninitialized
        ASSERT_EQ(1, bs.read(data, 12));
        ASSERT_EQ(0xad, data[0]);
        ASSERT_EQ(0xb0, data[1]);
    }
}


TEST(BitStream, BitOrderComplex)
{
    static const std::string REFERENCE =
        "10101101 10111111 11101111 01010110 11011111 01000100 10001101 00010101 10011110 00100110 10101111 00110111 10111100 00000100";

    uavcan::StaticTransferBuffer<32> buf;
    {   // Write
        const uint8_t data1[] = {0xad, 0xbe};              // 10101101 10111110
        const uint8_t data2[] = {0xfc};                    // 11111100
        const uint8_t data3[] = {0xde, 0xad, 0xbe, 0xef};  // 11011110 10101101 10111110 11101111
        const uint8_t data4[] = {0x12, 0x34, 0x56, 0x78,   // 00010010 00110100 01010110 01111000
                                 0x9a, 0xbc, 0xde, 0xf0};  // 10011010 10111100 11011110 11110000

        uavcan::BitStream bs(buf);
        ASSERT_EQ(1, bs.write(data1, 11));  // 10101101 101
        std::cout << bs.toString() << std::endl;
        ASSERT_EQ(1, bs.write(data2, 6));   //             11111 1
        std::cout << bs.toString() << std::endl;
        ASSERT_EQ(1, bs.write(data3, 25));  //                    1101111 01010110 11011111 01
        std::cout << bs.toString() << std::endl;
        ASSERT_EQ(1, bs.write(data4, 64));  // all 64, total 42 + 64 = 106
        std::cout << bs.toString() << std::endl;
        ASSERT_EQ(1, bs.write(data4, 4));   // 0001
        std::cout << bs.toString() << std::endl;

        std::cout << "Reference:\n" << REFERENCE << std::endl;

        ASSERT_EQ(REFERENCE, bs.toString());
    }
    {   // Read back in the same order
        uint8_t data[8];
        std::fill(data, data + sizeof(data), 0xA5);    // Filling with garbage
        uavcan::BitStream bs(buf);
        ASSERT_EQ(REFERENCE, bs.toString());

        ASSERT_EQ(1, bs.read(data, 11));     // 10101101 10100000
        ASSERT_EQ(0xad, data[0]);
        ASSERT_EQ(0xa0, data[1]);

        ASSERT_EQ(1, bs.read(data, 6));      // 11111100
        ASSERT_EQ(0xfc, data[0]);

        ASSERT_EQ(1, bs.read(data, 25));     // 11011110 10101101 10111110 10000000
        ASSERT_EQ(0xde, data[0]);
        ASSERT_EQ(0xad, data[1]);
        ASSERT_EQ(0xbe, data[2]);
        ASSERT_EQ(0x80, data[3]);

        ASSERT_EQ(1, bs.read(data, 64));     // Data - see above
        ASSERT_EQ(0x12, data[0]);
        ASSERT_EQ(0x34, data[1]);
        ASSERT_EQ(0x56, data[2]);
        ASSERT_EQ(0x78, data[3]);
        ASSERT_EQ(0x9a, data[4]);
        ASSERT_EQ(0xbc, data[5]);
        ASSERT_EQ(0xde, data[6]);
        ASSERT_EQ(0xf0, data[7]);
    }
}


TEST(BitStream, BitByBit)
{
    static const int NUM_BYTES = 1024;
    uavcan::StaticTransferBuffer<NUM_BYTES> buf;
    uavcan::BitStream bs_wr(buf);

    std::string binary_string;
    unsigned counter = 0;
    for (int byte = 0; byte < NUM_BYTES; byte++)
    {
        for (int bit = 0; bit < 8; bit++, counter++)
        {
            const bool value = counter % 3 == 0;
            binary_string.push_back(value ? '1' : '0');
            const uint8_t data[] = { uint8_t(value << 7) };
            ASSERT_EQ(1, bs_wr.write(data, 1));
        }
        binary_string.push_back(' ');
    }
    binary_string.erase(binary_string.length() - 1, 1);

    /*
     * Currently we have no free buffer space, so next write() must fail
     */
    const uint8_t dummy_data_wr[] = { 0xFF };
    ASSERT_EQ(0, bs_wr.write(dummy_data_wr, 1));

    /*
     * Bitstream content validation
     */
//    std::cout << bs.toString() << std::endl;
//    std::cout << "Reference:\n" << binary_string << std::endl;
    ASSERT_EQ(binary_string, bs_wr.toString());

    /*
     * Read back
     */
    uavcan::BitStream bs_rd(buf);
    counter = 0;
    for (int byte = 0; byte < NUM_BYTES; byte++)
    {
        for (int bit = 0; bit < 8; bit++, counter++)
        {
            const bool value = counter % 3 == 0;
            uint8_t data[1];
            ASSERT_EQ(1, bs_rd.read(data, 1));
            if (value)
            {
                ASSERT_EQ(0x80, data[0]);
            }
            else
            {
                ASSERT_EQ(0, data[0]);
            }
        }
    }

    /*
     * Making sure that reading out of buffer range will fail with error
     */
    uint8_t dummy_data_rd[] = { 0xFF };
    ASSERT_EQ(0, bs_wr.read(dummy_data_rd, 1));
    ASSERT_EQ(0xFF, dummy_data_rd[0]);
}
