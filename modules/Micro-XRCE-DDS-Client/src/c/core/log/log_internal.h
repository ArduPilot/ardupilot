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

#ifndef _SRC_C_CORE_LOG_LOG_INTERNAL_H_
#define _SRC_C_CORE_LOG_LOG_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <stdint.h>
#include <stddef.h>

#define UXR_SEND 1
#define UXR_ERROR_SEND ~1
#define UXR_RECV 2
#define UXR_ERROR_RECV ~2

#ifdef UXR_MESSAGE_LOGS
#define UXR_MESSAGE_LOGS_AVAILABLE 1
#else
#define UXR_MESSAGE_LOGS_AVAILABLE 0
#endif // ifdef UXR_MESSAGE_LOGS

#ifdef UXR_SERIALIZATION_LOGS
#define UXR_SERIALIZATION_LOGS_AVAILABLE 1
#else
#define UXR_SERIALIZATION_LOGS_AVAILABLE 0
#endif // ifdef UXR_SERIALIZATION_LOGS

void uxr_print_message(
        int direction,
        uint8_t* buffer,
        size_t size,
        const uint8_t* client_key);
void uxr_print_serialization(
        int direction,
        const uint8_t* buffer,
        size_t size);

#if defined(UXR_MESSAGE_LOGS) || defined(UXR_SERIALIZATION_LOGS)
#define UXR_DEBUG_PRINT_MESSAGE(direction, buffer, size, client_key) \
    do \
    { \
        if (UXR_MESSAGE_LOGS_AVAILABLE)uxr_print_message(direction, buffer, size, client_key); \
        if (UXR_SERIALIZATION_LOGS_AVAILABLE)uxr_print_serialization(direction, buffer, size); \
    } while (0)
#else
#define UXR_DEBUG_PRINT_MESSAGE(direction, buffer, size, client_key) do {} while (0)
#endif // if defined(UXR_MESSAGE_LOGS) || defined(UXR_SERIALIZATION_LOGS)

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_LOG_LOG_INTERNAL_H_
