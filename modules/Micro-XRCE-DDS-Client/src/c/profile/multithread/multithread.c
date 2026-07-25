#include <uxr/client/profile/multithread/multithread.h>

#include <uxr/client/core/session/session.h>
#include "../../core/session/stream/stream_storage_internal.h"

//==================================================================
//                             PRIVATE
//==================================================================

void uxr_init_lock(
        uxrMutex* mutex)
{
#if defined(PLATFORM_NAME_FREERTOS)
    mutex->impl = xSemaphoreCreateRecursiveMutexStatic( &mutex->xMutexBuffer );
#elif defined(UCLIENT_PLATFORM_ZEPHYR)
    k_mutex_init(&mutex->impl);
#elif defined(UCLIENT_PLATFORM_POSIX)
    pthread_mutexattr_t mat;
    pthread_mutexattr_init(&mat);
    pthread_mutexattr_settype(&mat, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&mutex->impl, &mat);
#else
#error XRCE multithreading not supported for this platform.
#endif /* if defined(PLATFORM_NAME_FREERTOS) */
}

void uxr_lock(
        uxrMutex* mutex)
{
#if defined(PLATFORM_NAME_FREERTOS)
    xSemaphoreTakeRecursive(mutex->impl, portMAX_DELAY);
#elif defined(UCLIENT_PLATFORM_ZEPHYR)
    k_mutex_lock(&mutex->impl, K_FOREVER);
#elif defined(UCLIENT_PLATFORM_POSIX)
    pthread_mutex_lock(&mutex->impl);
#else
#error XRCE multithreading not supported for this platform.
#endif /* if defined(PLATFORM_NAME_FREERTOS) */
}

void uxr_unlock(
        uxrMutex* mutex)
{
#if defined(PLATFORM_NAME_FREERTOS)
    xSemaphoreGiveRecursive(mutex->impl);
#elif defined(UCLIENT_PLATFORM_ZEPHYR)
    k_mutex_unlock(&mutex->impl);
#elif defined(UCLIENT_PLATFORM_POSIX)
    pthread_mutex_unlock(&mutex->impl);
#else
#error XRCE multithreading not supported for this platform.
#endif /* if defined(PLATFORM_NAME_FREERTOS) */
}

uxrMutex* uxr_get_stream_mutex_from_id(
        struct uxrSession* session,
        uxrStreamId stream_id)
{
    uxrMutex* mutex = NULL;

    if (stream_id.type == UXR_BEST_EFFORT_STREAM && stream_id.direction == UXR_OUTPUT_STREAM)
    {
        uxrOutputBestEffortStream* stream = uxr_get_output_best_effort_stream(&session->streams, stream_id.index);
        mutex = (stream == NULL) ? NULL : &stream->mutex;
    }
    else if (stream_id.type == UXR_BEST_EFFORT_STREAM && stream_id.direction == UXR_INPUT_STREAM)
    {
        uxrInputBestEffortStream* stream = uxr_get_input_best_effort_stream(&session->streams, stream_id.index);
        mutex = (stream == NULL) ? NULL : &stream->mutex;
    }
    else if (stream_id.type == UXR_RELIABLE_STREAM && stream_id.direction == UXR_OUTPUT_STREAM)
    {
        uxrOutputReliableStream* stream = uxr_get_output_reliable_stream(&session->streams, stream_id.index);
        mutex = (stream == NULL) ? NULL : &stream->mutex;
    }
    else if (stream_id.type == UXR_RELIABLE_STREAM && stream_id.direction == UXR_INPUT_STREAM)
    {
        uxrInputReliableStream* stream = uxr_get_input_reliable_stream(&session->streams, stream_id.index);
        mutex = (stream == NULL) ? NULL : &stream->mutex;
    }

    return mutex;
}
