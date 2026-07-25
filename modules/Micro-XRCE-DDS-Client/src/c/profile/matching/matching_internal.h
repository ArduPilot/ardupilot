// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef UXR_CLIENT_PROFILE_MATCHING_INTERNAL_H_
#define UXR_CLIENT_PROFILE_MATCHING_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/config.h>

#include <uxr/client/visibility.h>
#include <uxr/client/core/session/object_id.h>
#include <uxr/client/core/type/xrce_types.h>

#include <stdbool.h>

typedef uint32_t hash_int_t;
#define UXR_MATCHING_HASH_SIZE sizeof(hash_int_t)

bool uxr_generate_hash_from_xml(
        const char* xml,
        uxrObjectId id,
        char* hash);

bool uxr_generate_hash_from_ref(
        const char* ref,
        char* hash);

bool uxr_generate_hash_from_strings(
        char* hash,
        size_t number_strings,
        ...);

bool uxr_match_endpoint_qosbinary(
        const OBJK_Endpoint_QosBinary* entity_1,
        const OBJK_Endpoint_QosBinary* entity_2);



#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_PROFILE_MATCHING_INTERNAL_H_
