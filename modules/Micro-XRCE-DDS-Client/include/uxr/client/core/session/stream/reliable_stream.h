// Copyright 2017-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef UXR__CLIENT__CORE__SESSION__STREAM__RELIABLE_STREAM_H_
#define UXR__CLIENT__CORE__SESSION__STREAM__RELIABLE_STREAM_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <stdint.h>
#include <stddef.h>

typedef struct uxrReliableStream
{
    uint8_t* buffer;
    size_t size;
    uint16_t history;

} uxrReliableStream;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR__CLIENT__CORE__SESSION__STREAM__RELIABLE_STREAM_H_
