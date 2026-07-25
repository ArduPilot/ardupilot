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

#ifndef _UXR_CLIENT_CORE_SESSION_OBJECT_ID_H_
#define _UXR_CLIENT_CORE_SESSION_OBJECT_ID_H_

/**
 * @file
 */

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/visibility.h>
#include <stdint.h>

#define UXR_INVALID_ID     0x00
#define UXR_PARTICIPANT_ID 0x01
#define UXR_TOPIC_ID       0x02
#define UXR_PUBLISHER_ID   0x03
#define UXR_SUBSCRIBER_ID  0x04
#define UXR_DATAWRITER_ID  0x05
#define UXR_DATAREADER_ID  0x06
#define UXR_REQUESTER_ID   0x07
#define UXR_REPLIER_ID     0x08
#define UXR_OTHER_ID       0x0F

typedef struct uxrObjectId
{
    uint16_t id;
    uint8_t type;

} uxrObjectId;

/**
 * @brief This function creates an identifier to reference an entity.
 * @ingroup     general_utils
 * @param id	Identifier of the object, different for each type. There can be several objects with the same ID, provided they have different types.
 * @param type	The type of the entity. It can be: UXR_PARTICIPANT_ID, UXR_TOPIC_ID, UXR_PUBLISHER_ID, UXR_SUBSCRIBER_ID, UXR_DATAWRITER_ID, UXR_DATAREADER_ID, UXR_REQUESTER_ID, or UXR_REPLIER_ID.
 * @return	Generated entity identifier.
 */
UXRDLLAPI uxrObjectId uxr_object_id(
        uint16_t id,
        uint8_t type);

UXRDLLAPI uxrObjectId uxr_object_id_from_raw(
        const uint8_t* raw);

UXRDLLAPI void uxr_object_id_to_raw(
        uxrObjectId object_id,
        uint8_t* raw);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_UXR_CLIENT_CORE_SESSION_OBJECT_ID_H_
