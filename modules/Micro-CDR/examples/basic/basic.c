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

#include <ucdr/microcdr.h>
#include <stdio.h>

#define BUFFER_LENGTH 256

int main()
{
    // Data buffer
    uint8_t buffer[BUFFER_LENGTH];

    // Structs for handle the buffer.
    ucdrBuffer writer;
    ucdrBuffer reader;

    // Initialize the MicroBuffers for working with an user-managed buffer.
    ucdr_init_buffer(&writer, buffer, BUFFER_LENGTH);
    ucdr_init_buffer(&reader, buffer, BUFFER_LENGTH);

    // Serialize data
    char input[16] = "Hello MicroCDR!"; //16 characters
    ucdr_serialize_array_char(&writer, input, 16);

    // Deserialize data
    char output[16];
    ucdr_deserialize_array_char(&reader, output, 16);

    printf("Input: %s\n", input);
    printf("Output: %s\n", output);

    return 0;
}
