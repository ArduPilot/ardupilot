/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/data_type.hpp>


TEST(DataTypeSignatureCRC, Correctness)
{
    uavcan::DataTypeSignatureCRC crc;

    ASSERT_EQ(0xFFFFFFFFFFFFFFFF ^ 0xFFFFFFFFFFFFFFFF, crc.get());

    crc.add('1');
    crc.add('2');
    crc.add('3');
    crc.add(reinterpret_cast<const uint8_t*>("456789"), 6);

    ASSERT_EQ(0x62EC59E3F1A4F00A, crc.get());
}


TEST(DataTypeSignatureCRC, Extension)
{
    uavcan::DataTypeSignatureCRC crc1;

    crc1.add('1');
    crc1.add('2');
    crc1.add('3');

    uavcan::DataTypeSignatureCRC crc2 = uavcan::DataTypeSignatureCRC::extend(crc1.get());

    crc2.add(reinterpret_cast<const uint8_t*>("456789"), 6);

    ASSERT_EQ(0x62EC59E3F1A4F00A, crc2.get());
}


TEST(DataTypeSignature, Correctness)
{
    using uavcan::DataTypeSignature;
    using uavcan::DataTypeSignatureCRC;

    DataTypeSignature signature;
    ASSERT_EQ(0, signature.get());

    /*
     * First extension
     */
    signature.extend(DataTypeSignature(0x123456789abcdef0));

    DataTypeSignatureCRC crc;
    crc.add(0xF0);
    crc.add(0xDE);
    crc.add(0xBC);
    crc.add(0x9A);
    crc.add(0x78);
    crc.add(0x56);
    crc.add(0x34);
    crc.add(0x12);
    for (int i = 0; i < 8; i++)
    {
        crc.add(0);
    }

    ASSERT_EQ(crc.get(), signature.get());

    const uint64_t old_signature = signature.get();

    /*
     * Second extension
     */
    signature.extend(DataTypeSignature(0xfedcba9876543210));
    crc.add(0x10);
    crc.add(0x32);
    crc.add(0x54);
    crc.add(0x76);
    crc.add(0x98);
    crc.add(0xba);
    crc.add(0xdc);
    crc.add(0xfe);
    for (int i = 0; i < 64; i += 8)
    {
        crc.add((old_signature >> i) & 0xFF);
    }

    ASSERT_EQ(crc.get(), signature.get());

    /*
     * Comparison
     */
    ASSERT_TRUE(signature == DataTypeSignature(signature.get()));
    ASSERT_FALSE(signature == DataTypeSignature());
    ASSERT_FALSE(signature != DataTypeSignature(signature.get()));
    ASSERT_TRUE(signature != DataTypeSignature());
}


TEST(DataTypeDescriptor, ToString)
{
    uavcan::DataTypeDescriptor desc;
    ASSERT_EQ(":65535s:0000000000000000", desc.toString());

    desc = uavcan::DataTypeDescriptor(uavcan::DataTypeKindMessage, 123,
                                      uavcan::DataTypeSignature(0xdeadbeef1234), "Bar");
    ASSERT_EQ("Bar:123m:0000deadbeef1234", desc.toString());

    // Max length - 80 chars
    desc = uavcan::DataTypeDescriptor(uavcan::DataTypeKindMessage, 1023, uavcan::DataTypeSignature(0xdeadbeef12345678),
              "sirius_cybernetics_corporation.marvin.model_a.LongDataTypeName123456789abcdefghi");
    ASSERT_EQ("sirius_cybernetics_corporation.marvin.model_a.LongDataTypeName123456789abcdefghi:1023m:deadbeef12345678",
              desc.toString());
}


TEST(DataTypeDescriptor, Match)
{
    uavcan::DataTypeDescriptor desc(uavcan::DataTypeKindMessage, 123,
                                    uavcan::DataTypeSignature(0xdeadbeef1234), "namespace.TypeName");
    ASSERT_TRUE(desc.match(uavcan::DataTypeKindMessage, "namespace.TypeName"));
    ASSERT_FALSE(desc.match(uavcan::DataTypeKindMessage, "boo"));
    ASSERT_FALSE(desc.match(uavcan::DataTypeKindService, "namespace.TypeName"));
}


TEST(DataTypeID, Basic)
{
    uavcan::DataTypeID id;

    ASSERT_EQ(0xFFFF, id.get());
    ASSERT_FALSE(id.isValidForDataTypeKind(uavcan::DataTypeKindMessage));
    ASSERT_FALSE(id.isValidForDataTypeKind(uavcan::DataTypeKindService));

    id = 123;
    uavcan::DataTypeID id2 = 255;

    ASSERT_EQ(123, id.get());
    ASSERT_EQ(255, id2.get());

    ASSERT_TRUE(id.isValidForDataTypeKind(uavcan::DataTypeKindMessage));
    ASSERT_TRUE(id.isValidForDataTypeKind(uavcan::DataTypeKindService));
    ASSERT_TRUE(id2.isValidForDataTypeKind(uavcan::DataTypeKindMessage));
    ASSERT_TRUE(id2.isValidForDataTypeKind(uavcan::DataTypeKindService));

    ASSERT_TRUE(id < id2);
    ASSERT_TRUE(id <= id2);
    ASSERT_TRUE(id2 > id);
    ASSERT_TRUE(id2 >= id);
    ASSERT_TRUE(id != id2);

    id = id2;
    ASSERT_FALSE(id < id2);
    ASSERT_TRUE(id <= id2);
    ASSERT_FALSE(id2 > id);
    ASSERT_TRUE(id2 >= id);
    ASSERT_TRUE(id == id2);

    id = 1024;
    ASSERT_TRUE(id.isValidForDataTypeKind(uavcan::DataTypeKindMessage));
    ASSERT_FALSE(id.isValidForDataTypeKind(uavcan::DataTypeKindService));
}
