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

#include "FastCDRSerialization.hpp"

#include <cmath>

using MyTypes = ::testing::Types<char, uint8_t, uint16_t, uint32_t, uint64_t, int8_t, int16_t, int32_t, int64_t, float,
                double>;
TYPED_TEST_SUITE(FastCDRSerialization, MyTypes);

TYPED_TEST(FastCDRSerialization, micro_to_fast) {

    for (size_t i = 1; i < this->byte_buffer.size(); i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer({i, this->byte_buffer.size() - i});
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam fill = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        EXPECT_TRUE(this->ucdr_serialize(cdr, fill++));
                    }
                });

        this->prepare_fast_cdr([](Cdr& cdr)
                {
                    TypeParam out;
                    TypeParam compare = 0;
                    for (size_t j = 0; j < ELEMENT_NUMBER; j++)
                    {
                        cdr.deserialize(out);
                        ASSERT_EQ(compare++, out);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, micro_to_fast_multiple_slots) {

    for (size_t i = 1; i < 10; i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer(i);
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam fill = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        bool out = (this->ucdr_serialize(cdr, fill++));
                        ASSERT_TRUE(out);
                    }
                });

        this->prepare_fast_cdr([](Cdr& cdr)
                {
                    TypeParam out;
                    TypeParam compare = 0;
                    for (size_t j = 0; j < ELEMENT_NUMBER; j++)
                    {
                        cdr.deserialize(out);
                        ASSERT_EQ(compare++, out);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, fast_to_micro) {

    this->prepare_fast_cdr([](Cdr& cdr)
            {
                TypeParam value = 0;
                for (size_t i = 0; i < ELEMENT_NUMBER; i++)
                {
                    cdr.serialize(value++);
                }
            });

    for (size_t i = 1; i < this->byte_buffer.size(); i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer({i, this->byte_buffer.size() - i});
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam out = 0;
                    TypeParam compare = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        EXPECT_TRUE(this->ucdr_deserialize(cdr, &out));
                        EXPECT_EQ(out, compare++);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, fast_to_micro_multiple_slots) {

    this->prepare_fast_cdr([](Cdr& cdr)
            {
                TypeParam value = 0;
                for (size_t i = 0; i < ELEMENT_NUMBER; i++)
                {
                    cdr.serialize(value++);
                }
            });

    for (size_t i = 1; i < 10; i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer(i);
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam out = 0;
                    TypeParam compare = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        EXPECT_TRUE(this->ucdr_deserialize(cdr, &out));
                        EXPECT_EQ(out, compare++);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, micro_to_fast_as_array) {

    for (size_t i = 1; i < this->byte_buffer.size(); i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer({i, this->byte_buffer.size() - i});
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam out[ELEMENT_NUMBER] = {};
                    TypeParam fill = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        out[k] = fill++;
                    }
                    EXPECT_TRUE(this->ucdr_serialize_array(cdr, out, ELEMENT_NUMBER));
                });

        this->prepare_fast_cdr([](Cdr& cdr)
                {
                    TypeParam out;
                    TypeParam compare = 0;
                    for (size_t j = 0; j < ELEMENT_NUMBER; j++)
                    {
                        cdr.deserialize(out);
                        ASSERT_EQ(compare++, out);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, micro_to_fast_multiple_slots_as_array) {

    for (size_t i = 1; i < 10; i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer(i);
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam out[ELEMENT_NUMBER] = {};
                    TypeParam fill = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        out[k] = fill++;
                    }
                    EXPECT_TRUE(this->ucdr_serialize_array(cdr, out, ELEMENT_NUMBER));
                });

        this->prepare_fast_cdr([](Cdr& cdr)
                {
                    TypeParam out;
                    TypeParam compare = 0;
                    for (size_t j = 0; j < ELEMENT_NUMBER; j++)
                    {
                        cdr.deserialize(out);
                        ASSERT_EQ(compare++, out);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, fast_to_micro_as_array) {
    this->prepare_fast_cdr([](Cdr& cdr)
            {
                TypeParam value = 0;
                for (size_t i = 0; i < ELEMENT_NUMBER; i++)
                {
                    cdr.serialize(value++);
                }
            });

    for (size_t i = 1; i < this->byte_buffer.size(); i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer({i, this->byte_buffer.size() - i});
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam out[ELEMENT_NUMBER] = {};
                    EXPECT_TRUE(this->ucdr_deserialize_array(cdr, out, ELEMENT_NUMBER));

                    TypeParam compare = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        EXPECT_EQ(out[k], compare++);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, fast_to_micro_multiple_slots_as_array) {

    this->prepare_fast_cdr([](Cdr& cdr)
            {
                TypeParam value = 0;
                for (size_t i = 0; i < ELEMENT_NUMBER; i++)
                {
                    cdr.serialize(value++);
                }
            });

    for (size_t i = 1; i < 10; i++)
    {
        typename TestFixture::partial_buffer split_buffers = this->split_buffer(i);
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    TypeParam out[ELEMENT_NUMBER] = {};
                    EXPECT_TRUE(this->ucdr_deserialize_array(cdr, out, ELEMENT_NUMBER));

                    TypeParam compare = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER; k++)
                    {
                        EXPECT_EQ(out[k], compare++);
                    }
                });
    }
}

TYPED_TEST(FastCDRSerialization, multitypes) {
    for (size_t padding = 0; padding < 32; padding++)
    {
        size_t padded_elements = static_cast<size_t>(std::ceil(static_cast<float>(padding) / sizeof(TypeParam)));

        typename TestFixture::partial_buffer split_buffers = this->split_buffer(2);
        this->prepare_micro_cdr(split_buffers, [&](ucdrBuffer& cdr)
                {
                    uint8_t pad = 'a';
                    for (size_t j = 0; j < padding; j++)
                    {
                        EXPECT_TRUE(this->ucdr_serialize(cdr, pad));
                    }

                    TypeParam fill = 0;
                    for (size_t k = 0; k < ELEMENT_NUMBER - padded_elements; k++)
                    {
                        EXPECT_TRUE(this->ucdr_serialize(cdr, fill++));
                    }
                });

        this->prepare_fast_cdr([&](Cdr& cdr)
                {
                    uint8_t pad;
                    for (size_t j = 0; j < padding; j++)
                    {
                        pad = 'b';
                        cdr.deserialize(pad);
                        EXPECT_EQ(pad, 'a');
                    }

                    TypeParam out;
                    TypeParam compare = 0;
                    for (size_t j = 0; j < ELEMENT_NUMBER - padded_elements; j++)
                    {
                        cdr.deserialize(out);
                        ASSERT_EQ(compare++, out);
                    }
                });
    }
}

TEST(FastCDRSerialization, multibuffer_fragmentation_serialization) {
    uint8_t buffer[100] = {};

    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, buffer, 10);
    ucdr_set_on_full_buffer_callback(&ucdr_buffer,
            [](struct ucdrBuffer* internal_buffer, void* args) -> bool
            {
                uint8_t* b = reinterpret_cast<uint8_t*>(args);

                ucdrBuffer temp_buffer;
                ucdr_init_buffer_origin(&temp_buffer, b, 90, internal_buffer->offset);
                ucdr_set_on_full_buffer_callback(&temp_buffer, internal_buffer->on_full_buffer, internal_buffer->args);
                *internal_buffer = temp_buffer;
                return false;
            }, static_cast<void*>(&buffer[10]));

    for (size_t i = 0; i < 9; i++)
    {
        ASSERT_TRUE(ucdr_serialize_uint8_t(&ucdr_buffer, 'a'));
    }

    ASSERT_GT(ucdr_buffer.final, ucdr_buffer.iterator);
    ASSERT_GT(ucdr_buffer_alignment(&ucdr_buffer, sizeof(double)),
            static_cast<size_t>(ucdr_buffer.final - ucdr_buffer.iterator));

    ASSERT_TRUE(ucdr_serialize_double(&ucdr_buffer, 3.1416));

    FastBuffer cdrbuffer(reinterpret_cast<char*>(buffer), sizeof(buffer));
    Cdr cdr_ser(cdrbuffer);

    uint8_t out_byte;
    for (size_t i = 0; i < 9; i++)
    {
        out_byte = 0;
        cdr_ser.deserialize(out_byte);
        ASSERT_EQ(out_byte, 'a');
    }

    double out_double = 0;
    cdr_ser.deserialize(out_double);
    ASSERT_EQ(out_double, 3.1416);
}

TEST(FastCDRSerialization, multibuffer_fragmentation_serialization_array) {
    uint8_t buffer[100] = {};

    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, buffer, 10);
    ucdr_set_on_full_buffer_callback(&ucdr_buffer,
            [](struct ucdrBuffer* internal_buffer, void* args) -> bool
            {
                uint8_t* b = reinterpret_cast<uint8_t*>(args);

                ucdrBuffer temp_buffer;
                ucdr_init_buffer_origin(&temp_buffer, b, 90, internal_buffer->offset);
                ucdr_set_on_full_buffer_callback(&temp_buffer, internal_buffer->on_full_buffer, internal_buffer->args);
                *internal_buffer = temp_buffer;
                return false;
            }, static_cast<void*>(&buffer[10]));

    uint8_t in_bytes[9] = {'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};
    ASSERT_TRUE(ucdr_serialize_array_uint8_t(&ucdr_buffer, in_bytes, 9));

    ASSERT_GT(ucdr_buffer.final, ucdr_buffer.iterator);
    ASSERT_GT(ucdr_buffer_alignment(&ucdr_buffer, sizeof(double)),
            static_cast<size_t>(ucdr_buffer.final - ucdr_buffer.iterator));

    double in_double = 3.1416;
    ASSERT_TRUE(ucdr_serialize_array_double(&ucdr_buffer, &in_double, 1));

    FastBuffer cdrbuffer(reinterpret_cast<char*>(buffer), sizeof(buffer));
    Cdr cdr_ser(cdrbuffer);

    uint8_t out_byte;
    for (size_t i = 0; i < 9; i++)
    {
        out_byte = 0;
        cdr_ser.deserialize(out_byte);
        ASSERT_EQ(out_byte, 'a');
    }

    double out_double = 0;
    cdr_ser.deserialize(out_double);
    ASSERT_EQ(out_double, 3.1416);
}

TEST(FastCDRSerialization, multibuffer_fragmentation_deserialization) {
    uint8_t buffer[100] = {};

    FastBuffer cdrbuffer(reinterpret_cast<char*>(buffer), sizeof(buffer));
    Cdr cdr_ser(cdrbuffer);

    uint8_t in_byte = 'a';
    for (size_t i = 0; i < 9; i++)
    {
        cdr_ser.serialize(in_byte);
    }

    double in_double = 3.1416;
    cdr_ser.serialize(in_double);

    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, buffer, 10);
    ucdr_set_on_full_buffer_callback(&ucdr_buffer,
            [](struct ucdrBuffer* internal_buffer, void* args) -> bool
            {
                uint8_t* b = reinterpret_cast<uint8_t*>(args);

                ucdrBuffer temp_buffer;
                ucdr_init_buffer_origin(&temp_buffer, b, 90, internal_buffer->offset);
                ucdr_set_on_full_buffer_callback(&temp_buffer, internal_buffer->on_full_buffer, internal_buffer->args);
                *internal_buffer = temp_buffer;
                return false;
            }, static_cast<void*>(&buffer[10]));

    uint8_t out_byte;
    for (size_t i = 0; i < 9; i++)
    {
        out_byte = 0;
        ASSERT_TRUE(ucdr_deserialize_uint8_t(&ucdr_buffer, &out_byte));
        ASSERT_EQ(out_byte, 'a');
    }

    ASSERT_GT(ucdr_buffer.final, ucdr_buffer.iterator);
    ASSERT_GT(ucdr_buffer_alignment(&ucdr_buffer, sizeof(double)),
            static_cast<size_t>(ucdr_buffer.final - ucdr_buffer.iterator));

    double out_double = 0;
    ASSERT_TRUE(ucdr_deserialize_double(&ucdr_buffer, &out_double));
    ASSERT_EQ(out_double, 3.1416);
}

TEST(FastCDRSerialization, multibuffer_fragmentation_deserialization_array) {
    uint8_t buffer[100] = {};

    FastBuffer cdrbuffer(reinterpret_cast<char*>(buffer), sizeof(buffer));
    Cdr cdr_ser(cdrbuffer);

    uint8_t in_byte = 'a';
    for (size_t i = 0; i < 9; i++)
    {
        cdr_ser.serialize(in_byte);
    }

    double in_double = 3.1416;
    cdr_ser.serialize(in_double);

    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, buffer, 10);
    ucdr_set_on_full_buffer_callback(&ucdr_buffer,
            [](struct ucdrBuffer* internal_buffer, void* args) -> bool
            {
                uint8_t* b = reinterpret_cast<uint8_t*>(args);

                ucdrBuffer temp_buffer;
                ucdr_init_buffer_origin(&temp_buffer, b, 90, internal_buffer->offset);
                ucdr_set_on_full_buffer_callback(&temp_buffer, internal_buffer->on_full_buffer, internal_buffer->args);
                *internal_buffer = temp_buffer;
                return false;
            }, static_cast<void*>(&buffer[10]));

    uint8_t out_byte[9] = {};
    ASSERT_TRUE(ucdr_deserialize_array_uint8_t(&ucdr_buffer, out_byte, 9));
    for (size_t i = 0; i < 9; i++)
    {
        ASSERT_EQ(out_byte[i], 'a');
    }

    ASSERT_GT(ucdr_buffer.final, ucdr_buffer.iterator);
    ASSERT_GT(ucdr_buffer_alignment(&ucdr_buffer, sizeof(double)),
            static_cast<size_t>(ucdr_buffer.final - ucdr_buffer.iterator));

    double out_double = 0;
    ASSERT_TRUE(ucdr_deserialize_array_double(&ucdr_buffer, &out_double, 1));
    ASSERT_EQ(out_double, 3.1416);
}

