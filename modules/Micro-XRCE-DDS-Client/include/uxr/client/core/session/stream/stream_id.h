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

/**
 * @file
 */

#ifndef UXR_CLIENT_CORE_SESSION_STREAM_STREAM_ID_H_
#define UXR_CLIENT_CORE_SESSION_STREAM_STREAM_ID_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/visibility.h>
#include <stdint.h>

/** \addtogroup general_utils General utilities
 *  Utility functions. The declaration of these functions can be found in uxr/client/core/session/stream_id.h and uxr/client/core/session/object_id.h.
 *  @{
 */

/**
 * The enum that identifies the kind of stream.
 * According to the XRCE standard, a stream represents an independent flow of messages within a session.
 * The XRCE standard defines 3 different kinds of streams:
 * * **none** streams do not provide neither non-out-of-order delivery nor guarantee for the delivery.
 * * **best-effort** streams provide non-out-of-order delivery but they do not provide guarantee for delivery.
 * * **reliable** streams provide both non-out-of-order delivery and guarantee for delivery.
 *
 * The streams are identified by an octet. Each session is able to handle
 * 1 none stream (id 0x00),
 * 127 best-effort streams (id 0x01 - 0x7F),
 * and 128 reliable streams (id 0x80 - 0xFF),
 * for each direction: output (Client to Agent) and input (Agent to Client).
 */
typedef enum uxrStreamType
{
    /** Identifies a none stream. */
    UXR_NONE_STREAM,
    /** Identifies a best-effort stream. */
    UXR_BEST_EFFORT_STREAM,
    /** Identifies a reliable stream. */
    UXR_RELIABLE_STREAM,
    /** Identifies a shared memory stream. */
    UXR_SHARED_MEMORY_STREAM

} uxrStreamType;

/**
 * The enum that identifies the direction of a stream.
 * There are two different directions: output (Client to Agent) and input (Agent to Client).
 */
typedef enum uxrStreamDirection
{
    /** Indicates the input direction of the stream. */
    UXR_INPUT_STREAM,
    /** Indicates the output direction of the stream. */
    UXR_OUTPUT_STREAM

} uxrStreamDirection;

/**
 * @nosubgrouping
 */
typedef struct uxrStreamId
{
    uint8_t raw;
    uint8_t index;
    uint8_t type;
    uint8_t direction;

} uxrStreamId;

/**
 * @brief Creates a stream identifier.
 *        This function does not create a new stream, but only creates the identifier to be used in the Client API.
 * @param index     The identifier of the stream.
 *                  Its value corresponds to the creation order of each kind of stream.
 * @param type      The uxrStreamType of the stream.
 * @param direction The uxrStreamDirection of the stream.
 * @return A identifier of a stream.
 */
UXRDLLAPI uxrStreamId uxr_stream_id(
        uint8_t index,
        uxrStreamType type,
        uxrStreamDirection direction);

/**
 * @brief Creates a stream identifier.
 *        This function does not create a new stream, but only creates the identifier to be used in the Client API.
 * @param stream_id_raw The identifier of the stream.
 *                      0 corresponds to the UXR_NONE_STREAM;
 *                      from 1 to 127 correspond to a UXR_BEST_EFFORT_STREAM;
 *                      from 128 to 255 correspond to a UXR_RELIABLE_STREAM;
 * @param direction     The uxrStreamDirection of the stream.
 * @return A identifier of a stream.
 */
UXRDLLAPI uxrStreamId uxr_stream_id_from_raw(
        uint8_t stream_id_raw,
        uxrStreamDirection direction);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_SESSION_STREAM_STREAM_ID_H
