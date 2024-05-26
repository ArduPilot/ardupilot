/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>

#include <uavcan/helpers/ostream.hpp>
#include <uavcan/transport/transfer_buffer.hpp>

#include <uavcan/Timestamp.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>
#include <uavcan/protocol/Panic.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>
#include <uavcan/protocol/DataTypeKind.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

#include <root_ns_a/Deep.hpp>
#include <root_ns_a/UnionTest.hpp>
#include <root_ns_a/UnionTest4.hpp>

template <typename T>
static bool validateYaml(const T& obj, const std::string& reference)
{
    uavcan::OStream::instance() << "Validating YAML:\n" << obj << "\n" << uavcan::OStream::endl;

    std::ostringstream os;
    os << obj;
    if (os.str() == reference)
    {
        return true;
    }
    else
    {
        std::cout << "INVALID YAML:\n"
                  << "EXPECTED:\n"
                  << "===\n"
                  << reference
                  << "\n===\n"
                  << "ACTUAL:\n"
                  << "\n===\n"
                  << os.str()
                  << "\n===\n" << std::endl;
        return false;
    }
}

TEST(Dsdl, Streaming)
{
    EXPECT_TRUE(validateYaml(uavcan::protocol::GetNodeInfo::Response(),
                             "status: \n"
                             "  uptime_sec: 0\n"
                             "  health: 0\n"
                             "  mode: 0\n"
                             "  sub_mode: 0\n"
                             "  vendor_specific_status_code: 0\n"
                             "software_version: \n"
                             "  major: 0\n"
                             "  minor: 0\n"
                             "  optional_field_flags: 0\n"
                             "  vcs_commit: 0\n"
                             "  image_crc: 0\n"
                             "hardware_version: \n"
                             "  major: 0\n"
                             "  minor: 0\n"
                             "  unique_id: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
                             "  certificate_of_authenticity: \"\"\n"
                             "name: \"\""));

    root_ns_a::Deep ps;
    ps.a.resize(1);
    EXPECT_TRUE(validateYaml(ps,
                             "c: 0\n"
                             "str: \"\"\n"
                             "a: \n"
                             "  - \n"
                             "    scalar: 0\n"
                             "    vector: \n"
                             "      - \n"
                             "        vector: [0, 0]\n"
                             "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
                             "      - \n"
                             "        vector: [0, 0]\n"
                             "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
                             "b: \n"
                             "  - \n"
                             "    vector: [0, 0]\n"
                             "    bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
                             "  - \n"
                             "    vector: [0, 0]\n"
                             "    bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"));
}


template <typename T>
static bool encodeDecodeValidate(const T& obj, const std::string& reference_bit_string)
{
    uavcan::StaticTransferBuffer<256> buf;

    {
        uavcan::BitStream bits(buf);
        uavcan::ScalarCodec codec(bits);
        /*
         * Coding
         */
        if (0 > T::encode(obj, codec))
        {
            std::cout << "Failed to encode" << std::endl;
            return false;
        }

        /*
         * Validating the encoded bitstream
         */
        const std::string result = bits.toString();
        if (result != reference_bit_string)
        {
            std::cout << "ENCODED VALUE DOESN'T MATCH THE REFERENCE:\nEXPECTED:\n"
                      << reference_bit_string << "\nACTUAL:\n"
                      << result << std::endl;
            return false;
        }
    }

    /*
     * Decoding back and comparing
     */
    uavcan::BitStream bits(buf);
    uavcan::ScalarCodec codec(bits);

    T decoded;

    if (0 > T::decode(decoded, codec))
    {
        std::cout << "Failed to decode" << std::endl;
        return false;
    }

    if (!decoded.isClose(obj))
    {
        std::cout << "DECODED OBJECT DOESN'T MATCH THE REFERENCE:\nEXPECTED:\n"
                  << obj << "\nACTUAL:\n"
                  << decoded << std::endl;
        return false;
    }
    return true;
}


TEST(Dsdl, Union)
{
    using root_ns_a::UnionTest;
    using root_ns_a::NestedInUnion;

    ASSERT_EQ(3, UnionTest::MinBitLen);
    ASSERT_EQ(16, UnionTest::MaxBitLen);
    ASSERT_EQ(13, NestedInUnion::MinBitLen);
    ASSERT_EQ(13, NestedInUnion::MaxBitLen);

    UnionTest s;

    EXPECT_TRUE(validateYaml(s, "z: "));
    encodeDecodeValidate(s, "00000000");

    s.to<UnionTest::Tag::a>() = 16;
    EXPECT_TRUE(validateYaml(s, "a: 16"));
    EXPECT_TRUE(encodeDecodeValidate(s, "00110000"));

    s.to<UnionTest::Tag::b>() = 31;
    EXPECT_TRUE(validateYaml(s, "b: 31"));
    EXPECT_TRUE(encodeDecodeValidate(s, "01011111"));

    s.to<UnionTest::Tag::c>() = 256;
    EXPECT_TRUE(validateYaml(s, "c: 256"));
    EXPECT_TRUE(encodeDecodeValidate(s, "01100000 00000001"));

    s.to<UnionTest::Tag::d>().push_back(true);
    s.to<UnionTest::Tag::d>().push_back(false);
    s.to<UnionTest::Tag::d>().push_back(true);
    s.to<UnionTest::Tag::d>().push_back(true);
    s.to<UnionTest::Tag::d>().push_back(false);
    s.to<UnionTest::Tag::d>().push_back(false);
    s.to<UnionTest::Tag::d>().push_back(true);
    s.to<UnionTest::Tag::d>().push_back(true);
    s.to<UnionTest::Tag::d>().push_back(true);
    ASSERT_EQ(9, s.to<UnionTest::Tag::d>().size());
    EXPECT_TRUE(validateYaml(s, "d: [1, 0, 1, 1, 0, 0, 1, 1, 1]"));
    EXPECT_TRUE(encodeDecodeValidate(s, "10010011 01100111"));

    s.to<UnionTest::Tag::e>().array[0] = 0;
    s.to<UnionTest::Tag::e>().array[1] = 1;
    s.to<UnionTest::Tag::e>().array[2] = 2;
    s.to<UnionTest::Tag::e>().array[3] = 3;
    EXPECT_TRUE(validateYaml(s, "e: \n  array: [0, 1, 2, 3]"));
    EXPECT_TRUE(encodeDecodeValidate(s, "10100000 11011000"));
}


TEST(Dsdl, UnionTagWidth)
{
    using root_ns_a::UnionTest4;

    ASSERT_EQ(2, UnionTest4::MinBitLen);
    ASSERT_EQ(8, UnionTest4::MaxBitLen);

    UnionTest4 s;

    {
        uavcan::StaticTransferBuffer<100> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, UnionTest4::encode(s, sc_wr));
        ASSERT_EQ("00000000", bs_wr.toString());
    }

    {
        uavcan::StaticTransferBuffer<100> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        s.to<UnionTest4::Tag::third>() = 1U << 5U;   // 32, 0b100000

        ASSERT_EQ(1, UnionTest4::encode(s, sc_wr));
        ASSERT_EQ("10100000", bs_wr.toString());
    }
}


TEST(Dsdl, ParamGetSetRequestUnion)
{
    uavcan::protocol::param::GetSet::Request req;

    req.index = 8191;
    req.name = "123"; // 49, 50, 51 // 00110001, 00110010, 00110011
    EXPECT_TRUE(encodeDecodeValidate(req, "11111111 11111000 00110001 00110010 00110011"));

    req.value.to<uavcan::protocol::param::Value::Tag::string_value>() = "abc"; // 01100001, 01100010, 01100011
    EXPECT_TRUE(encodeDecodeValidate(req,
                                     "11111111 11111100 "               // Index, Union tag
                                     "00000011 "                        // Array length
                                     "01100001 01100010 01100011 "      // Payload
                                     "00110001 00110010 00110011"));    // Name

    EXPECT_TRUE(validateYaml(req,
                             "index: 8191\n"
                             "value: \n"
                             "  string_value: \"abc\"\n"
                             "name: \"123\""));

    req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = 1;
    EXPECT_TRUE(encodeDecodeValidate(req,
                                     "11111111 11111001 "               // Index, Union tag
                                     "00000001 00000000 00000000 00000000 00000000 00000000 00000000 00000000 " // Payload
                                     "00110001 00110010 00110011"));    // Name
}


TEST(Dsdl, ParamGetSetResponseUnion)
{
    uavcan::protocol::param::GetSet::Response res;

    res.value.to<uavcan::protocol::param::Value::Tag::string_value>() = "abc";
    res.default_value.to<uavcan::protocol::param::Value::Tag::string_value>(); // Empty
    res.name = "123";
    EXPECT_TRUE(encodeDecodeValidate(res,
                                     "00000100 "                        // Value union tag
                                     "00000011 "                        // Value array length
                                     "01100001 01100010 01100011 "      // Value array payload
                                     "00000100 "                        // Default union tag
                                     "00000000 "                        // Default array length
                                     "00000000 "                        // Max value tag
                                     "00000000 "                        // Min value tag
                                     "00110001 00110010 00110011"));    // Name

    res.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = true;
    res.default_value.to<uavcan::protocol::param::Value::Tag::boolean_value>(); // False
    res.name = "123";
    EXPECT_TRUE(encodeDecodeValidate(res,
                                     "00000011 "                        // Value union tag
                                     "00000001 "                        // Value
                                     "00000011 "                        // Default union tag
                                     "00000000 "                        // Default value
                                     "00000000 "                        // Max value tag
                                     "00000000 "                        // Min value tag
                                     "00110001 00110010 00110011"));    // Name
}
