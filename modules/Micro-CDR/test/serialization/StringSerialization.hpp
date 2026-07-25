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

#ifndef _STRING_SERIALIZATION_HPP_
#define _STRING_SERIALIZATION_HPP_

#include "BasicSerialization.hpp"

#define MAX_STRING_LENGTH 64

class StringSerialization : public BasicSerialization
{
public:

    StringSerialization()
    {
    }

    virtual ~StringSerialization()
    {
    }

    void string_serialization()
    {
        char input[MAX_STRING_LENGTH] = "This is a message test";
        char output[MAX_STRING_LENGTH] = {0};

        EXPECT_TRUE(ucdr_serialize_string(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_string(&reader, output, MAX_STRING_LENGTH));

        EXPECT_STREQ(input, output);
    }

};

#endif //_STRING_SERIALIZATION_HPP_
