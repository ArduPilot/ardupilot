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

#ifndef UXR_CLIENT_CORE_SESSION_CREATE_ENTITIES_REF_H_
#define UXR_CLIENT_CORE_SESSION_CREATE_ENTITIES_REF_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/common_create_entities.h>

/** \addtogroup create_ref Create entities by reference
 *  These functions are enabled when PROFILE_CREATE_ENTITIES_REF is activated as a CMake argument. The declaration of these functions can be found in uxr/client/profile/session/create_entities_ref.h.
 *  @{
 */

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Participant payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Participant according to
 *        the reference provided in the CREATE submessage.
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id     The identifier of the XRCE Participant.
 * @param domain_id     The identifier of the Domain to which the XRCE Participant belongs.
 * @param ref           The reference of the XRCE Participant.
 * @param mode          The set of flags that determines the entity creation mode.
 *                      The Creation Mode Table describes the entities creation behaviour according to the
 *                      `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_participant_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t domain_id,
        const char* ref,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Topic payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Topic according to
 *        the reference provided in the CREATE submessage.
 * @param session           A uxrSession structure previously initialized.
 * @param stream_id         The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id         The identifier of the XRCE Topic.
 * @param participant_id    The identifier of the associated XRCE Participant.
 * @param ref               The reference of the XRCE Topic.
 * @param mode              The set of flags that determines the entity creation mode.
 *                          The Creation Mode Table describes the entities creation behaviour according to the
 *                          `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_topic_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* ref,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE DataWriter payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE DataWriter according to
 *        the reference provided in the CREATE submessage.
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id     The identifier of the XRCE DataWriter.
 * @param publisher_id  The identifier of the associated XRCE Publisher.
 * @param ref           The reference of the XRCE DataWriter.
 * @param mode          The set of flags that determines the entity creation mode.
 *                      The Creation Mode Table describes the entities creation behaviour according to the
 *                      `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_datawriter_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId publisher_id,
        const char* ref,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE DataReader payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE DataReader according to
 *        the reference provided in the CREATE submessage.
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id     The identifier of the XRCE DataReader.
 * @param subscriber_id The identifier of the associated XRCE Subscriber.
 * @param ref           The reference of the XRCE DataReader.
 * @param mode          The set of flags that determines the entity creation mode.
 *                      The Creation Mode Table describes the entities creation behaviour according to the
 *                      `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_datareader_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId subscriber_id,
        const char* ref,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Requester payload.
 *        The submessage will be sent when `uxr_flag_output_streams` or `uxr_run_session` functions are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Requester according to
 *        the reference provided in the CREATE submessage.
 *
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE Requester.
 * @param participant_id        The identifier of the associated XRCE Participant.
 * @param ref                   The reference of the XRCE Requester.
 * @param mode                  The set of flags that determines the entitiy creation mode.
 *                              the Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_requester_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* ref,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Replier payload.
 *        The submessage will be sent when `uxr_flag_output_streams` or `uxr_run_session` functions are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Replier according to
 *        the reference provided in the CREATE submessage.
 *
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE Requester.
 * @param participant_id        The identifier of the associated XRCE Participant.
 * @param ref                   The reference of the XRCE Replier.
 * @param mode                  The set of flags that determines the entitiy creation mode.
 *                              the Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_replier_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* ref,
        uint8_t mode);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_SESSION_CREATE_ENTITIES_REF_H_
