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

#ifndef UXR_CLIENT_PROFILE_MULTITHREAD_H_
#define UXR_CLIENT_PROFILE_MULTITHREAD_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/config.h>
#include <uxr/client/visibility.h>
#include <uxr/client/core/session/stream/stream_id.h>

struct uxrSession;

#ifdef UCLIENT_PROFILE_MULTITHREAD

#ifdef WIN32
#elif defined(PLATFORM_NAME_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#elif defined(UCLIENT_PLATFORM_ZEPHYR)
#elif defined(UCLIENT_PLATFORM_POSIX)
#include <pthread.h>
#endif // ifdef WIN32

// Micro XRCE-DDS Client mutex implementation

typedef struct uxrMutex
{
#ifdef WIN32
#elif defined(PLATFORM_NAME_FREERTOS)
    SemaphoreHandle_t impl;
    StaticSemaphore_t xMutexBuffer;
#elif defined(UCLIENT_PLATFORM_ZEPHYR)
    struct k_mutex impl;
#elif defined(UCLIENT_PLATFORM_POSIX)
    pthread_mutex_t impl;
#endif // ifdef WIN32
} uxrMutex;

UXRDLLAPI uxrMutex* uxr_get_stream_mutex_from_id(
        struct uxrSession* session,
        uxrStreamId stream_id);

UXRDLLAPI void uxr_init_lock(
        uxrMutex* mutex);

UXRDLLAPI void uxr_lock(
        uxrMutex* mutex);

UXRDLLAPI void uxr_unlock(
        uxrMutex* mutex);

// Conditional defines

#define UXR_INIT_LOCK(X) uxr_init_lock(X)
#define UXR_LOCK(X) uxr_lock(X)
#define UXR_UNLOCK(X) uxr_unlock(X)

#define UXR_INIT_LOCK_SESSION uxr_init_lock(&session->mutex)
#define UXR_LOCK_SESSION(session) uxr_lock(&session->mutex)
#define UXR_UNLOCK_SESSION(session) uxr_unlock(&session->mutex)

#define UXR_LOCK_TRANSPORT(comm) uxr_lock(&comm->mutex)
#define UXR_UNLOCK_TRANSPORT(comm) uxr_unlock(&comm->mutex)

#define UXR_LOCK_STREAM_ID(session, stream_id) { \
        uxrMutex* stream_mutex = uxr_get_stream_mutex_from_id(session, stream_id); \
        if (stream_mutex != NULL){ \
            uxr_lock(stream_mutex); \
        } \
}

#define UXR_UNLOCK_STREAM_ID(session, stream_id){ \
        uxrMutex* stream_mutex = uxr_get_stream_mutex_from_id(session, stream_id); \
        if (stream_mutex != NULL){ \
            uxr_unlock(stream_mutex); \
        } \
}

#define UXR_LOCK_ALL_INPUT_STREAMS(session) \
    for (uint8_t i = 0; i < session->streams.input_best_effort_size; ++i){ \
        uxr_lock(&session->streams.input_best_effort[i].mutex); } \
    for (uint8_t i = 0; i < session->streams.input_reliable_size; ++i){ \
        uxr_lock(&session->streams.input_reliable[i].mutex); \
    }

#define UXR_UNLOCK_ALL_INPUT_STREAMS(session) \
    for (uint8_t i = 0; i < session->streams.input_best_effort_size; ++i){ \
        uxr_unlock( &session->streams.input_best_effort[i].mutex); \
    } \
    for (uint8_t i = 0; i < session->streams.input_reliable_size; ++i){  \
        uxr_unlock( &session->streams.input_reliable[i].mutex); \
    }


#else // UCLIENT_PROFILE_MULTITHREAD

#define UXR_INIT_LOCK(X)
#define UXR_LOCK(X)
#define UXR_UNLOCK(X)

#define UXR_INIT_LOCK_SESSION
#define UXR_LOCK_SESSION(session)
#define UXR_UNLOCK_SESSION(session)

#define UXR_LOCK_TRANSPORT(comm)
#define UXR_UNLOCK_TRANSPORT(comm)

#define UXR_LOCK_STREAM_ID(session, stream_id)
#define UXR_UNLOCK_STREAM_ID(session, stream_id)

#define UXR_LOCK_ALL_INPUT_STREAMS(session)
#define UXR_UNLOCK_ALL_INPUT_STREAMS(session)

#endif // UCLIENT_PROFILE_MULTITHREAD

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_PROFILE_MULTITHREAD_H_
