// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef SRC_C_UTIL_TIME_H_
#define SRC_C_UTIL_TIME_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/visibility.h>
#include <stdint.h>

static inline int64_t uxr_convert_to_nanos(
        int32_t sec,
        uint32_t nsec)
{
    return ((int64_t)sec * 1000000000) + nsec;
}

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // SRC_C_UTIL_TIME_H_
