// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef UXR_CLIENT_CORE_SESSION_CREATE_ENTITIES_BIN_H_
#define UXR_CLIENT_CORE_SESSION_CREATE_ENTITIES_BIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/common_create_entities.h>
#include <uxr/client/core/type/xrce_types.h>

//==================================================================
//                              PUBLIC
//==================================================================

/**
 * The enum that identifies the durability of the QoS of the DDS entity.
 */

typedef enum uxrQoSDurability
{
    UXR_DURABILITY_TRANSIENT_LOCAL = 0,
    UXR_DURABILITY_TRANSIENT,
    UXR_DURABILITY_VOLATILE,
    UXR_DURABILITY_PERSISTENT
} uxrQoSDurability;

typedef enum uxrQoSReliability
{
    UXR_RELIABILITY_RELIABLE = 0,
    UXR_RELIABILITY_BEST_EFFORT,
} uxrQoSReliability;


// uxrQosHistory
typedef enum uxrQoSHistory
{
    UXR_HISTORY_KEEP_LAST = 0,
    UXR_HISTORY_KEEP_ALL
} uxrQoSHistory;

typedef struct uxrQoS_t
{
    uxrQoSDurability durability;
    uxrQoSReliability reliability;
    uxrQoSHistory history;
    uint16_t depth;
} uxrQoS_t;

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Participant payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Participant according to
 *        the binary provided in the CREATE submessage.
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE Participant.
 * @param domain_id             The identifier of the Domain to which the XRCE Participant belongs.
 * @param participant_name      The XRCE Participant name. Can be NULL.
 * @param mode                  The set of flags that determines the entity creation mode.
 *                              The Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_participant_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t domain_id,
        const char* participant_name,
        uint8_t mode);
/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Topic payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Topic according to
 *        the binary provided in the CREATE submessage.
 * @param session           A uxrSession structure previously initialized.
 * @param stream_id         The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id         The identifier of the XRCE Topic.
 * @param participant_id    The identifier of the associated XRCE Participant.
 * @param topic_name        The XRCE Topic name.
 * @param type_name         The XRCE Topic type.
 * @param mode              The set of flags that determines the entity creation mode.
 *                          The Creation Mode Table describes the entities creation behaviour according to the
 *                          `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_topic_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* topic_name,
        const char* type_name,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Subscriber payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Publisher according to
 *        the binary provided in the CREATE submessage.
 * @param session           A uxrSession structure previously initialized.
 * @param mode              The set of flags that determines the entity creation mode.
 *                          The Creation Mode Table describes the entities creation behaviour according to the
 *                          `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_publisher_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Subscriber payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Subscriber according to
 *        the binary provided in the CREATE submessage.
 * @param session           A uxrSession structure previously initialized.
 * @param stream_id         The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id         The identifier of the XRCE Subscriber.
 * @param participant_id    The identifier of the associated XRCE Participant.
 * @param mode              The set of flags that determines the entity creation mode.
 *                          The Creation Mode Table describes the entities creation behaviour according to the
 *                          `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_subscriber_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE DataWriter payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE DataWriter according to
 *        the binary provided in the CREATE submessage.
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE DataWriter.
 * @param publisher_id          The identifier of the associated XRCE Publisher.
 * @param topic_id              The identifier of the associated XRCE Topic.
 * @param qos                   QoS definition.
 * @param mode                  The set of flags that determines the entity creation mode.
 *                              The Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_datawriter_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId publisher_id,
        uxrObjectId topic_id,
        uxrQoS_t qos,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE DataReader payload.
 *        The submessage will be sent when `uxr_flash_output_streams` or `uxr_run_session` function are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE DataReader according to
 *        the binary provided in the CREATE submessage.
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE DataReader.
 * @param subscriber_id         The identifier of the associated XRCE Subscriber.
 * @param topic_id              The identifier of the associated XRCE Topic.
 * @param qos                   QoS definition.
 * @param mode                  The set of flags that determines the entity creation mode.
 *                              The Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_datareader_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId subscriber_id,
        uxrObjectId topic_id,
        uxrQoS_t qos,
        uint8_t mode);
/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Requester payload.
 *        The submessage will be sent when `uxr_flag_output_streams` or `uxr_run_session` functions are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Requester according to
 *        the binary provided in the CREATE submessage.
 *
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE Requester.
 * @param participant_id        The identifier of the associated XRCE Participant.
 * @param service_name          Requester service name.
 * @param request_type          Requester request type.
 * @param reply_type            Requester reply type.
 * @param request_topic_name    Requester request topic name.
 * @param reply_topic_name      Requester reply topic name.
 * @param qos                   QoS definition.
 * @param mode                  The set of flags that determines the entitiy creation mode.
 *                              the Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_requester_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* service_name,
        const char* request_type,
        const char* reply_type,
        const char* request_topic_name,
        const char* reply_topic_name,
        uxrQoS_t qos,
        uint8_t mode);

/**
 * @brief Buffers into the stream identified by `stream_id` an XRCE CREATE submessage with an XRCE Replier payload.
 *        The submessage will be sent when `uxr_flag_output_streams` or `uxr_run_session` functions are called.
 *        As a result of the reception of this submessage, the Agent will create an XRCE Replier according to
 *        the binary provided in the CREATE submessage.
 *
 * @param session               A uxrSession structure previously initialized.
 * @param stream_id             The output stream identifier where the CREATE submessage will be buffered.
 * @param object_id             The identifier of the XRCE Requester.
 * @param participant_id        The identifier of the associated XRCE Participant.
 * @param service_name          Replier service name.
 * @param request_type          Replier request type.
 * @param reply_type            Replier reply type.
 * @param request_topic_name    Replier request topic name.
 * @param reply_topic_name      Replier reply topic name.
 * @param qos                   QoS definition.
 * @param mode                  The set of flags that determines the entitiy creation mode.
 *                              the Creation Mode Table describes the entities creation behaviour according to the
 *                              `UXR_REUSE` and `UXR_REPLACE` flags.
 * @return A `request_id` that identifies the request made by the Client.
 *         This could be used in the `uxr_run_session_until_one_status` or `uxr_run_session_until_all_status` functions.
 */
UXRDLLAPI uint16_t uxr_buffer_create_replier_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* service_name,
        const char* request_type,
        const char* reply_type,
        const char* request_topic_name,
        const char* reply_topic_name,
        uxrQoS_t qos,
        uint8_t mode);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_SESSION_CREATE_ENTITIES_BIN_H_
