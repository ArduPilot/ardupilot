// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "FullBuffer.hpp"

class FullBufferCallback : public FullBuffer
{
public:

    FullBufferCallback()
    {
        ucdr_set_on_full_buffer_callback(&writer, on_full_buffer, buffer);
        ucdr_set_on_full_buffer_callback(&reader, on_full_buffer, buffer);
    }

protected:

    static bool on_full_buffer(
            ucdrBuffer* ub,
            void* args)
    {
        uint8_t* buffer =  static_cast<uint8_t*>(args);

        EXPECT_EQ(buffer, ub->init);
        EXPECT_EQ(buffer + BUFFER_LENGTH, ub->final); //only satisfied if BUFFER_LENGTH is alignment to 8.

        return true;
    }

};

TEST_F(FullBufferCallback, Block_8_callback)
{
    fill_buffer_except(7);
    try_block_8();
}

TEST_F(FullBufferCallback, Block_4_callback)
{
    fill_buffer_except(3);
    try_block_4();
}

TEST_F(FullBufferCallback, Block_2_callback)
{
    fill_buffer_except(1);
    try_block_2();
}

TEST_F(FullBufferCallback, Block_1_callback)
{
    fill_buffer_except(0);
    try_block_1();
}

# define SUCCESSFUL_SERIALIZATION   3
# define ARRAY_SERIALIZATION        (SUCCESSFUL_SERIALIZATION + 1)
TEST_F(FullBufferCallback, ArrayBlock_8_callback)
{
    fill_buffer_except(8 * SUCCESSFUL_SERIALIZATION + 7);
    try_array_block_8(ARRAY_SERIALIZATION);
}

TEST_F(FullBufferCallback, ArrayBlock_4_callback)
{
    fill_buffer_except(4 * SUCCESSFUL_SERIALIZATION + 3);
    try_array_block_4(ARRAY_SERIALIZATION);
}

TEST_F(FullBufferCallback, ArrayBlock_2_callback)
{
    fill_buffer_except(2 * SUCCESSFUL_SERIALIZATION + 1);
    try_array_block_2(ARRAY_SERIALIZATION);
}

TEST_F(FullBufferCallback, ArrayBlock_1_callback)
{
    fill_buffer_except(SUCCESSFUL_SERIALIZATION);
    try_array_block_1(ARRAY_SERIALIZATION);
}

