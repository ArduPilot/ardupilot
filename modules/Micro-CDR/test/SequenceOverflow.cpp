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

#include "serialization/SequenceSerialization.hpp"

#define SEQUENCE_SIZE_OVERFLOW      ARRAY_CAPACITY + 1

class SequenceOverflow : public SequenceSerialization
{
public:

    SequenceOverflow()
    {
        set_sequence_size(SEQUENCE_SIZE_OVERFLOW);
    }

    ~SequenceOverflow()
    {
        EXPECT_TRUE(reader.error);

        // To satisfy the base destructor
        reader.error = false;
        reader.iterator = writer.iterator;
    }

private:
};

TEST_F(SequenceOverflow, Block1)
{
    uint8_t input[SEQUENCE_SIZE_OVERFLOW] = {0};
    uint8_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_sequence_uint8_t(&writer, input, SEQUENCE_SIZE_OVERFLOW));
    EXPECT_FALSE(ucdr_deserialize_sequence_uint8_t(&reader, output, ARRAY_CAPACITY, &output_size));
}

TEST_F(SequenceOverflow, Block2)
{
    uint16_t input[SEQUENCE_SIZE_OVERFLOW] = {0};
    uint16_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_sequence_uint16_t(&writer, input, SEQUENCE_SIZE_OVERFLOW));
    EXPECT_FALSE(ucdr_deserialize_sequence_uint16_t(&reader, output, ARRAY_CAPACITY, &output_size));
}

TEST_F(SequenceOverflow, Block4)
{
    uint32_t input[SEQUENCE_SIZE_OVERFLOW] = {0};
    uint32_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_sequence_uint32_t(&writer, input, SEQUENCE_SIZE_OVERFLOW));
    EXPECT_FALSE(ucdr_deserialize_sequence_uint32_t(&reader, output, ARRAY_CAPACITY, &output_size));
}

TEST_F(SequenceOverflow, Block8)
{
    uint64_t input[SEQUENCE_SIZE_OVERFLOW] = {0};
    uint64_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_sequence_uint64_t(&writer, input, SEQUENCE_SIZE_OVERFLOW));
    EXPECT_FALSE(ucdr_deserialize_sequence_uint64_t(&reader, output, ARRAY_CAPACITY, &output_size));
}

