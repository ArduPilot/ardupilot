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

#ifndef UXR_CLIENT_CORE_SESSION_WRITE_ACCESS_H_
#define UXR_CLIENT_CORE_SESSION_WRITE_ACCESS_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/session.h>

/** \addtogroup write Write access profile
 *  The Write Access is used by the Client to handle the write operation on the Agent. The declaration of these functions can be found in uxr/client/profile/session/write_access.h.
 *  @{
 */

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE WRITE_DATA submessage.
 *        As a consequence, an XRCE request is generated associated to the WRITE_DATA submessage.
 *
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the WRITE_DATA submessage will be buffered.
 * @param requester_id  The identifier of the XRCE Requester that will write the request into the DDS GDS.
 * @param buffer        The pointer to the request data.
 * @param len           The length of the request data.
 * @return A `request_id` that identifies the XRCE request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
uint16_t uxr_buffer_request(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId requester_id,
        uint8_t* buffer,
        size_t len);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE WRITE_DATA submessage.
 *        As a consequence, an XRCE request is generated associated to the WRITE_DATA submessage.
 *
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the WRITE_DATA submessage will be buffered.
 * @param replier_id    The identifier of the XRCE Replier that will write the reply into the DDS GDS.
 * @param sample_id     The `SampleIdentity` that identifies the request.
 *                      It will be read by the Requester to filter and identify the reply.
 * @param buffer        The pointer to the reply data.
 * @param len           The length of the reply data.
 * @return A `request_id` that identifies the XRCE request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
uint16_t uxr_buffer_reply(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId replier_id,
        SampleIdentity* sample_id,
        uint8_t* buffer,
        size_t len);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE WRITE_DATA submessage.
 *        As a consequence, an XRCE request is generated associated to the WRITE_DATA submessage.
 *
 * @param session       A uxrSession structure previously initialized.
 * @param stream_id     The output stream identifier where the WRITE_DATA submessage will be buffered.
 * @param datawriter_id The identifier of the XRCE Datawriter that will write the topic into the DDS GDS.
 * @param buffer        The pointer to the topic data.
 * @param len           The length of the topic data.
 * @return A `request_id` that identifies the XRCE request made by the Publisher.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
uint16_t uxr_buffer_topic(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId datawriter_id,
        uint8_t* buffer,
        size_t len);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE WRITE_DATA submessage.
 *        The submessage will be sent when `uxr_flash_output_stream` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will write a topic into the DDS Global-Data-Space.
 * @param session           A uxrSession structure previously initialized.
 * @param stream_id         The output stream identifier where the WRITE_DATA submessage will be buffered.
 * @param entity_id         The identifier of the XRCE DataWriter that will write the topic into the DDS GDS.
 * @param ub                The ucdrBuffer structure used for serializing the topic.
 * @param len               The size of the topic in bytes.
 * @return A `request_id` that identifies the XRCE request made by the Entity.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 *  */
UXRDLLAPI uint16_t uxr_prepare_output_stream(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId entity_id,
        struct ucdrBuffer* ub,
        uint32_t len);


/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE WRITE_DATA submessage.
 *        The submessage will be sent when `uxr_flash_output_stream` or `uxr_run_session` function are called.
 *        This function handles the buffer flush by means of  `uxrOnBuffersFull` callback.
 *        As a result of the reception of this submessage, the Agent will write a topic into the DDS Global-Data-Space.
 * @param session              A uxrSession structure previously initialized.
 * @param stream_id            The output stream identifier where the WRITE_DATA submessage will be buffered.
 * @param datawriter_id        The identifier of the XRCE DataWriter that will write the topic into the DDS GDS.
 * @param ub                   The ucdrBuffer structure used for serializing the topic.
 * @param data_size            The size of the topic in bytes.
 * @param flush_callback       Callback that is call by the library when user should flush output buffers.
 * @param flush_callback_args  Arguments passed to flush callback.
 * @return A `request_id` that identifies the XRCE request made by the Entity.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 *  */

UXRDLLAPI uint16_t uxr_prepare_output_stream_fragmented(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId datawriter_id,
        ucdrBuffer* ub,
        size_t data_size,
        uxrOnBuffersFull flush_callback,
        void* flush_callback_args);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_SESSION_WRITE_ACCESS_H_
