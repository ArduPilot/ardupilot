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

#ifndef UXR_CLIENT_PROFILE_SHARED_MEMORY_INTERNAL_H_
#define UXR_CLIENT_PROFILE_SHARED_MEMORY_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/config.h>

#include <uxr/client/visibility.h>
#include <uxr/client/core/session/session.h>
#include <uxr/client/profile/multithread/multithread.h>

#ifdef UCLIENT_PROFILE_SHARED_MEMORY

#define UXR_PREPARE_SHARED_MEMORY(a, b, c, d, e) uxr_prepare_shared_memory(a, b, c, d, e)
#define UXR_HANDLE_SHARED_MEMORY() uxr_handle_shared_memory()
#define UXR_ADD_SHARED_MEMORY_ENTITY_XML(a, b, c) uxr_add_shared_memory_entity_xml(a, b, c)
#define UXR_ADD_SHARED_MEMORY_ENTITY_BIN(a, b, c) uxr_add_shared_memory_entity_bin(a, b, (void*) c)
#define UXR_CLEAN_SHARED_MEMORY() uxr_clean_shared_memory()

#else // UCLIENT_PROFILE_SHARED_MEMORY

#define UXR_PREPARE_SHARED_MEMORY(a, b, c, d, e)
#define UXR_HANDLE_SHARED_MEMORY()
#define UXR_ADD_SHARED_MEMORY_ENTITY_XML(a, b, c)
#define UXR_ADD_SHARED_MEMORY_ENTITY_BIN(a, b, c)
#define UXR_CLEAN_SHARED_MEMORY()

#endif // UCLIENT_PROFILE_SHARED_MEMORY

void uxr_prepare_shared_memory(
        uxrSession* session,
        uxrObjectId entity_id,
        ucdrBuffer* ub,
        uint16_t data_size,
        uint16_t request_id);

void uxr_handle_shared_memory();

void uxr_add_shared_memory_entity_xml(
        uxrSession* session,
        uxrObjectId entity_id,
        const char* xml);

void uxr_add_shared_memory_entity_bin(
        uxrSession* session,
        uxrObjectId entity_id,
        const void* entity);

void uxr_clean_shared_memory();

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_PROFILE_SHARED_MEMORY_INTERNAL_H_
