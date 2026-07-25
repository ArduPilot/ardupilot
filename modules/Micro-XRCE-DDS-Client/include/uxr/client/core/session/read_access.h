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

#ifndef UXR_CLIENT_CORE_SESSION_READ_ACCESS_H_
#define UXR_CLIENT_CORE_SESSION_READ_ACCESS_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/session.h>

#define UXR_MAX_SAMPLES_UNLIMITED           0xFFFF
#define UXR_MAX_ELAPSED_TIME_UNLIMITED      0x0000
#define UXR_MAX_BYTES_PER_SECOND_UNLIMITED  0x0000

/** \addtogroup read Read access
 *  The Read Access is used by the Client to handle the read operation on the Agent. The declaration of these functions can be found in uxr/client/profile/session/read_access.h.
 *  @{
 */

/** @struct uxrDeliveryControl
 *  @brief A structure used for controlling the delivery of topic from the Agent to the Client.
 *
 *  @var uxrDeliveryControl::max_samples
 *  The maximum number of topics that the Agent shall send to the Client.
 *
 *  @var uxrDeliveryControl::max_elapsed_time
 *  The maximum amount of time in seconds that shall be spent by the Agent delivering the topic.
 *
 *  @var uxrDeliveryControl::max_bytes_per_second
 *  The maximum transfer rate, in bytes per second, that the Agent shall use.
 *
 *  @var uxrDeliveryControl::min_pace_period
 *  The minimum elapsed time, in milliseconds, between two topics deliveries.
 */

typedef struct uxrDeliveryControl
{
    uint16_t max_samples;
    uint16_t max_elapsed_time;
    uint16_t max_bytes_per_second;
    uint16_t min_pace_period;

} uxrDeliveryControl;

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE READ_DATA submessage.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will start to read a topic from the
 *        DDS Global-Data-Space.
 *        Each time the Agent reads a message from the topic, it will be sent to the Client according to the uxrDeliveryControl set
 *        by the Client.
 *        Each time the Client receives a topic message from the Agent, it will call the `on_topic_callback` set by the user.
 *        This callback provides the `request_id` returned by this function in order to identify the topic.
 *        In case of error at the Agent side, it will send an XRCE STATUS submessage with the error code.
 *        When the STATUS submessage is received by the Client, it will call the `on_status_callback` with the
 *        `request_id` returned by this function.
 * @param session           A uxrSession structure previously initialized.
 * @param stream_id         The output stream identifier where the READ_DATA submessage will be buffered.
 * @param datareader_id     The identifier of the XRCE DataReader that will read the topics from the DDS GDS.
 * @param data_stream_id    The identifier of the input stream through which the data will be received.
 * @param delivery_control  An optional parameter that is used for controlling the delivery of topics from the Agent.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_request_data(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId datareader_id,
        uxrStreamId data_stream_id,
        const uxrDeliveryControl* const delivery_control);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE READ_DATA submessage.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will cancel any previous read operation.
 * @param session           A uxrSession structure previously initialized.
 * @param stream_id         The output stream identifier where the READ_DATA submessage will be buffered.
 * @param datareader_id     The identifier of the XRCE DataReader that will read the topics from the DDS GDS.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_cancel_data(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId datareader_id);

/** @}*/

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_SESSION_READ_ACCESS_H_
