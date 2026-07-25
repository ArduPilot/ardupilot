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
#include <string.h>
#include <inttypes.h>

#define BUFFER_LENGTH 12
#define SLOTS 12
#define STRING_MAX 128

bool on_full_buffer(
        ucdrBuffer* ub,
        void* args)
{
    uint8_t* buffer =  (uint8_t*) args;

    // This value correspong with the ub->error, and will be returned by this function to indicates
    // if the serialization must continue or must stop because of an error.
    bool error = true;

    // Leave the odd slots empty.
    uint32_t next_slot = 2 + (uint32_t)(ub->init - buffer) / BUFFER_LENGTH;
    if (next_slot < SLOTS)
    {
        // Modify the internal buffer
        ub->init = buffer + BUFFER_LENGTH * next_slot;
        ub->iterator = ub->init;
        ub->final = ub->init + BUFFER_LENGTH;

        printf("  Extend buffer to slot %u\n", next_slot);

        // As we want to continue the serialization without errors, we return false.
        error = false;
    }

    return error;
}

int main()
{
    // Data buffer
    uint8_t buffer[SLOTS * BUFFER_LENGTH];

    // Structs for handle the buffer.
    ucdrBuffer writer;
    ucdrBuffer reader;

    // Initialize the MicroBuffers for working with an user-managed buffer.
    ucdr_init_buffer(&writer, buffer, BUFFER_LENGTH);
    ucdr_init_buffer(&reader, buffer, BUFFER_LENGTH);

    // Add a full buffer behavior to the writer and the reader
    ucdr_set_on_full_buffer_callback(&writer, on_full_buffer, buffer);
    ucdr_set_on_full_buffer_callback(&reader, on_full_buffer, buffer);

    // Serialize data
    printf("Serializing...\n");
    char input[STRING_MAX] = "Hello MicroCDR! this message is fragmented";
    ucdr_serialize_string(&writer, input);
    printf("\n");

    // Deserialize data
    printf("Deserializing...\n");
    char output[STRING_MAX];
    ucdr_deserialize_string(&reader, output, STRING_MAX);
    printf("\n");

    printf("Input: %s\n", input);
    printf("Output: %s\n", output);

    return 0;
}
