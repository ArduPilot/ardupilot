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

#ifndef _FASTCDR_SERIALIZATION_HPP_
#define _FASTCDR_SERIALIZATION_HPP_

#include <gtest/gtest.h>
#include <ucdr/microcdr.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>

#include <functional>
#include <queue>

using namespace eprosima::fastcdr;

#define ELEMENT_NUMBER 50

template <typename T>
class FastCDRSerialization : public ::testing::Test
{
public:

    constexpr static size_t buffer_size = ELEMENT_NUMBER * sizeof(T);
    std::array<uint8_t, buffer_size> byte_buffer;
    using partial_buffer = std::queue<std::pair<char*, size_t>>;

    partial_buffer split_buffer(
            size_t slots)
    {
        partial_buffer result;
        size_t slot_size = byte_buffer.size() / slots;
        size_t rest = byte_buffer.size() % slots;
        for (size_t i = 0; i < slots; ++i)
        {
            result.push(std::make_pair(reinterpret_cast<char*>(byte_buffer.data()) + i * slot_size, slot_size));
        }
        result.back().second += rest;
        return result;
    }

    partial_buffer split_buffer(
            std::vector<size_t> slots_sizes)
    {
        partial_buffer result;
        size_t offset = 0;
        for (size_t slot_size : slots_sizes)
        {
            if (offset + slot_size > byte_buffer.size())
            {
                break;
            }
            result.push(std::make_pair(reinterpret_cast<char*>(byte_buffer.data()) + offset, slot_size));
            offset += slot_size;
        }
        return result;
    }

    void prepare_fast_cdr(
            std::function<void(Cdr&)> cb)
    {
        FastBuffer cdrbuffer(reinterpret_cast<char*>(byte_buffer.data()), byte_buffer.size());
        Cdr cdr_ser(cdrbuffer);
        cb(cdr_ser);
    }

    void prepare_micro_cdr(
            partial_buffer buffers,
            std::function<void(ucdrBuffer&)> cb)
    {
        ASSERT_GT(buffers.size(), 0UL);

        ucdrBuffer ucdr_buffer;
        ucdr_init_buffer(&ucdr_buffer, reinterpret_cast<uint8_t*>(buffers.front().first), buffers.front().second);
        buffers.pop();
        ucdr_set_on_full_buffer_callback(&ucdr_buffer,
                [](struct ucdrBuffer* buffer, void* args) -> bool
                {
                    partial_buffer& b = *reinterpret_cast<partial_buffer*>(args);

                    if (0 == b.size())
                    {
                        return true;
                    }
                    ucdrBuffer temp_buffer;
                    ucdr_init_buffer_origin(&temp_buffer, reinterpret_cast<uint8_t*>(b.front().first), b.front().second,
                    buffer->offset);

                    ucdr_set_on_full_buffer_callback(&temp_buffer, buffer->on_full_buffer, buffer->args);
                    b.pop();

                    *buffer = temp_buffer;
                    return false;
                }, static_cast<void*>(&buffers));

        cb(ucdr_buffer);
    }

    #define GENERATE_SERIALIZERS(TYPE) \
    bool ucdr_serialize_array(ucdrBuffer & cdr, TYPE * d, size_t size){ return ucdr_serialize_array_ ## TYPE(&cdr, d, \
                                                                                       size); } \
    bool ucdr_deserialize_array(ucdrBuffer & cdr, TYPE * d, size_t size){ return ucdr_deserialize_array_ ## TYPE(&cdr, \
                                                                                         d, \
                                                                                         size); } \
    bool ucdr_serialize(ucdrBuffer & cdr, TYPE d){ return ucdr_serialize_ ## TYPE(&cdr, d); } \
    bool ucdr_deserialize(ucdrBuffer & cdr, TYPE * d){ return ucdr_deserialize_ ## TYPE(&cdr, d); }

    GENERATE_SERIALIZERS(char)
    GENERATE_SERIALIZERS(bool)
    GENERATE_SERIALIZERS(uint8_t)
    GENERATE_SERIALIZERS(uint16_t)
    GENERATE_SERIALIZERS(uint32_t)
    GENERATE_SERIALIZERS(uint64_t)
    GENERATE_SERIALIZERS(int8_t)
    GENERATE_SERIALIZERS(int16_t)
    GENERATE_SERIALIZERS(int32_t)
    GENERATE_SERIALIZERS(int64_t)
    GENERATE_SERIALIZERS(float)
    GENERATE_SERIALIZERS(double)
};

#endif //_FASTCDR_SERIALIZATION_HPP_
