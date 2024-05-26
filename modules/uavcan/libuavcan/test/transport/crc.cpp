/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/transport/crc.hpp>

/*
   import crcmod
   crc = crcmod.predefined.Crc('crc-ccitt-false')
   crc.update('123')
   crc.hexdigest()
   '5BCE'
   crc.update('456789')
   crc.hexdigest()
   '29B1'
 */

TEST(TransferCRC, Correctness)
{
    uavcan::TransferCRC crc;

    ASSERT_EQ(0xFFFF, crc.get());

    crc.add('1');
    crc.add('2');
    crc.add('3');
    ASSERT_EQ(0x5BCE, crc.get());

    crc.add(reinterpret_cast<const uint8_t*>("456789"), 6);
    ASSERT_EQ(0x29B1, crc.get());
}
