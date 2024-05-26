/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/Timestamp.hpp>

/*
 * Objective of this test is to make sure that the generated types are being linked correcly.
 * This test requests the address of some static const member variables to make sure that there
 * will be no duplicated symbol linking errors.
 */
TEST(DsdlConst1, Timestamp)
{
    using uavcan::Timestamp;

    std::cout << &Timestamp::UNKNOWN << std::endl;
}
