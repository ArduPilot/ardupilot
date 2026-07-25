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

#ifndef UXR_CLIENT_CORE_SESSION_COMMON_CREATE_ENTITIES_H_
#define UXR_CLIENT_CORE_SESSION_COMMON_CREATE_ENTITIES_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/session.h>

/** \addtogroup create_common Create entities common profile
 *  These functions are enabled when either PROFILE_CREATE_ENTITIES_XML or PROFILE_CREATE_ENTITIES_REF are activated as CMake arguments. The declaration of these functions can be found in uxr/client/profile/session/common_create_entities.h.
 *  @{
 */

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE DELETE submessage.
 *        The submessage will be sent when `uxr_flash_output_stream` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will delete an XRCE entity.
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id     The identifier of the XRCE entity.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_delete_entity(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_SESSION_COMMON_CREATE_ENTITIES_H_
