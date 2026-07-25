/*
 * Copyright (c) 2017 UAVCAN Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Contributors: https://github.com/UAVCAN/libcanard/contributors
 */

#include <gtest/gtest.h>
#include "canard.h"

static bool shouldAcceptTransferMock(const CanardInstance*,
                                     uint64_t*,
                                     uint16_t,
                                     CanardTransferType,
                                     uint8_t)
{
    return false;
}

static void onTransferReceptionMock(CanardInstance*,
                                    CanardRxTransfer*)
{
}


TEST(Init, UserReference)
{
    std::uint8_t memory_arena[1024];

    ::CanardInstance ins;
    canardInit(&ins,
               memory_arena,
               sizeof(memory_arena),
               &onTransferReceptionMock,
               &shouldAcceptTransferMock,
               reinterpret_cast<void*>(12345U));

    ASSERT_TRUE(12345 == reinterpret_cast<intptr_t>(canardGetUserReference(&ins)));
}
