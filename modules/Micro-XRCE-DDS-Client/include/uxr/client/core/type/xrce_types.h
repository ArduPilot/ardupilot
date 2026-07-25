// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef UXR_CLIENT_CORE_TYPE_XRCETYPES_H_
#define UXR_CLIENT_CORE_TYPE_XRCETYPES_H_

#ifdef __cplusplus
extern "C"
{
#define INLINE_STRUCT(x) (x)
#else
#define INLINE_STRUCT(x)  x
#endif // ifdef __cplusplus

#include <uxr/client/defines.h>
#include <uxr/client/config.h>

#include <ucdr/microcdr.h>
#include <stdint.h>
#include <stdbool.h>

// TODO (julibert): move this configuration to CMake flags.
#define UXR_STRING_SIZE_MAX                512
#define UXR_SAMPLE_DATA_SIZE_MAX           512
#define UXR_STRING_SEQUENCE_MAX            8
#define UXR_BINARY_SEQUENCE_MAX            512
#define UXR_BINARY_SEQUENCE_SMALL_MAX      8
#define UXR_SAMPLE_SEQUENCE_MAX            8
#define UXR_SAMPLE_DATA_SEQUENCE_MAX       8
#define UXR_SAMPLE_DELTA_SEQUENCE_MAX      8
#define UXR_PACKED_SAMPLES_SEQUENCE_MAX    8
#define UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX 4

#ifdef UCLIENT_PROFILE_SHARED_MEMORY
#define PROFILE_SHARED_MEMORY_SEQ_COUNT 1
#else
#define PROFILE_SHARED_MEMORY_SEQ_COUNT 0
#endif // ifdef UCLIENT_PROFILE_SHARED_MEMORY

#ifdef UCLIENT_HARD_LIVELINESS_CHECK
#define HARD_LIVELINESS_CHECK_SEQ_COUNT 1
#else
#define HARD_LIVELINESS_CHECK_SEQ_COUNT 0
#endif // ifdef UCLIENT_HARD_LIVELINESS_CHECK

#if  (PROFILE_SHARED_MEMORY_SEQ_COUNT + HARD_LIVELINESS_CHECK_SEQ_COUNT) == 0
#define UXR_PROPERTY_SEQUENCE_MAX          1
#else
#define UXR_PROPERTY_SEQUENCE_MAX          PROFILE_SHARED_MEMORY_SEQ_COUNT + HARD_LIVELINESS_CHECK_SEQ_COUNT
#endif // if  (PROFILE_SHARED_MEMORY_SEQ_COUNT + HARD_LIVELINESS_CHECK_SEQ_COUNT) == 0

typedef struct Time_t
{
    int32_t seconds;
    uint32_t nanoseconds;

} Time_t;

typedef struct BinarySequence_t
{
    uint32_t size;
    uint8_t data[UXR_BINARY_SEQUENCE_MAX];

} BinarySequence_t;

typedef struct BinarySequenceSmall_t
{
    uint32_t size;
    uint8_t data[UXR_BINARY_SEQUENCE_SMALL_MAX];

} BinarySequenceSmall_t;

typedef struct StringSequence_t
{
    uint32_t size;
    char* data[UXR_STRING_SEQUENCE_MAX];

} StringSequence_t;

typedef struct ClientKey
{
    uint8_t data[4];

} ClientKey;


#define DDS_XRCE_CLIENT_INVALID COMPOUND_LITERAL(ClientKey){{0x00, 0x00, 0x00, 0x00}}

typedef uint8_t ObjectKind;
#define DDS_XRCE_OBJK_INVALID 0x00
#define DDS_XRCE_OBJK_PARTICIPANT 0x01
#define DDS_XRCE_OBJK_TOPIC 0x02
#define DDS_XRCE_OBJK_PUBLISHER 0x03
#define DDS_XRCE_OBJK_SUBSCRIBER 0x04
#define DDS_XRCE_OBJK_DATAWRITER 0x05
#define DDS_XRCE_OBJK_DATAREADER 0x06
#define DDS_XRCE_OBJK_REQUESTER 0x07
#define DDS_XRCE_OBJK_REPLIER 0x08
#define DDS_XRCE_OBJK_TYPE 0x0A
#define DDS_XRCE_OBJK_QOSPROFILE 0x0B
#define DDS_XRCE_OBJK_APPLICATION 0x0C
#define DDS_XRCE_OBJK_AGENT 0x0D
#define DDS_XRCE_OBJK_CLIENT 0x0E
#define DDS_XRCE_OBJK_OTHER 0x0F


typedef struct ObjectId
{
    uint8_t data[2];

} ObjectId;


typedef struct ObjectPrefix
{
    uint8_t data[2];

} ObjectPrefix;
#define DDS_XRCE_OBJECTID_INVALID COMPOUND_LITERAL(ObjectId){{0x00, 0x00}}
#define DDS_XRCE_OBJECTID_AGENT COMPOUND_LITERAL(ObjectId){{0xFF, 0xFD}}
#define DDS_XRCE_OBJECTID_CLIENT COMPOUND_LITERAL(ObjectId){{0xFF, 0xFE}}
#define DDS_XRCE_OBJECTID_SESSION COMPOUND_LITERAL(ObjectId){{0xFF, 0xFF}}


typedef struct XrceCookie
{
    uint8_t data[4];

} XrceCookie;
#define DDS_XRCE_XRCE_COOKIE COMPOUND_LITERAL(XrceCookie){{0x58, 0x52, 0x43, 0x45}}


typedef struct XrceVersion
{
    uint8_t data[2];

} XrceVersion;
#define DDS_XRCE_XRCE_VERSION_MAJOR 0x01
#define DDS_XRCE_XRCE_VERSION_MINOR 0x00
#define DDS_XRCE_XRCE_VERSION COMPOUND_LITERAL(XrceVersion){{DDS_XRCE_XRCE_VERSION_MAJOR, DDS_XRCE_XRCE_VERSION_MINOR}}


typedef struct XrceVendorId
{
    uint8_t data[2];

} XrceVendorId;
#define DDS_XRCE_XRCE_VENDOR_INVALID {0x00, 0x00}

typedef enum TransportLocatorFormat
{
    ADDRESS_FORMAT_SMALL = 0,
    ADDRESS_FORMAT_MEDIUM = 1,
    ADDRESS_FORMAT_LARGE = 2,
    ADDRESS_FORMAT_STRING = 3

} TransportLocatorFormat;


typedef struct TransportLocatorSmall
{
    uint8_t address[2];
    uint8_t locator_port;

} TransportLocatorSmall;


typedef struct TransportLocatorMedium
{
    uint8_t address[4];
    uint16_t locator_port;

} TransportLocatorMedium;


typedef struct TransportLocatorLarge
{
    uint8_t address[16];
    uint32_t locator_port;

} TransportLocatorLarge;


typedef struct TransportLocatorString
{
    char* value;

} TransportLocatorString;


typedef union TransportLocatorU
{
    TransportLocatorSmall small_locator;
    TransportLocatorMedium medium_locator;
    TransportLocatorLarge large_locator;
    TransportLocatorString string_locator;

} TransportLocatorU;


typedef struct TransportLocator
{
    uint8_t format;
    TransportLocatorU _;

} TransportLocator;


typedef struct TransportLocatorSeq
{
    uint32_t size;
    TransportLocator data[UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX];

} TransportLocatorSeq;


typedef struct Property
{
    char* name;
    char* value;

} Property;


typedef struct PropertySeq
{
    uint32_t size;
    Property data[UXR_PROPERTY_SEQUENCE_MAX];

} PropertySeq;


typedef struct CLIENT_Representation
{
    XrceCookie xrce_cookie;
    XrceVersion xrce_version;
    XrceVendorId xrce_vendor_id;
    ClientKey client_key;
    uint8_t session_id;
    bool optional_properties;
    PropertySeq properties;
    uint16_t mtu;

} CLIENT_Representation;


typedef struct AGENT_Representation
{
    XrceCookie xrce_cookie;
    XrceVersion xrce_version;
    XrceVendorId xrce_vendor_id;
    bool optional_properties;
    PropertySeq properties;

} AGENT_Representation;

typedef uint8_t RepresentationFormat;
#define DDS_XRCE_REPRESENTATION_BY_REFERENCE 0x01
#define DDS_XRCE_REPRESENTATION_AS_XML_STRING 0x02
#define DDS_XRCE_REPRESENTATION_IN_BINARY 0x03
#define DDS_XRCE_REFERENCE_MAX_LEN 128


typedef union OBJK_Representation3FormatsU
{
    char* object_reference;
    char* xml_string_represenatation;
    BinarySequence_t binary_representation;

} OBJK_Representation3FormatsU;


typedef struct OBJK_Representation3Formats
{
    uint8_t format;
    OBJK_Representation3FormatsU _;

} OBJK_Representation3Formats;


typedef union OBJK_RepresentationRefAndXMLFormatsU
{
    char* object_name;
    char* xml_string_represenatation;

} OBJK_RepresentationRefAndXMLFormatsU;


typedef struct OBJK_RepresentationRefAndXMLFormats
{
    uint8_t format;
    OBJK_RepresentationRefAndXMLFormatsU _;

} OBJK_RepresentationRefAndXMLFormats;


typedef union OBJK_RepresentationBinAndXMLFormatsU
{
    BinarySequence_t binary_representation;
    char* string_represenatation;

} OBJK_RepresentationBinAndXMLFormatsU;


typedef struct OBJK_RepresentationBinAndXMLFormats
{
    uint8_t format;
    OBJK_RepresentationBinAndXMLFormatsU _;

} OBJK_RepresentationBinAndXMLFormats;


typedef struct OBJK_RepresentationRefAndXML_Base
{
    OBJK_RepresentationRefAndXMLFormats representation;

} OBJK_RepresentationRefAndXML_Base;


typedef struct OBJK_RepresentationBinAndXML_Base
{
    OBJK_RepresentationBinAndXMLFormats representation;

} OBJK_RepresentationBinAndXML_Base;


typedef struct OBJK_Representation3_Base
{
    OBJK_Representation3Formats representation;

} OBJK_Representation3_Base;


typedef struct OBJK_QOSPROFILE_Representation
{
    OBJK_RepresentationRefAndXML_Base base;

} OBJK_QOSPROFILE_Representation;


typedef struct OBJK_TYPE_Representation
{
    OBJK_RepresentationRefAndXML_Base base;

} OBJK_TYPE_Representation;


typedef struct OBJK_DOMAIN_Representation
{
    OBJK_RepresentationRefAndXML_Base base;

} OBJK_DOMAIN_Representation;


typedef struct OBJK_APPLICATION_Representation
{
    OBJK_RepresentationRefAndXML_Base base;

} OBJK_APPLICATION_Representation;


typedef struct OBJK_PUBLISHER_Representation
{
    OBJK_RepresentationBinAndXML_Base base;
    ObjectId participant_id;

} OBJK_PUBLISHER_Representation;


typedef struct OBJK_SUBSCRIBER_Representation
{
    OBJK_RepresentationBinAndXML_Base base;
    ObjectId participant_id;

} OBJK_SUBSCRIBER_Representation;


typedef struct DATAWRITER_Representation
{
    OBJK_Representation3_Base base;
    ObjectId publisher_id;

} DATAWRITER_Representation;


typedef struct DATAREADER_Representation
{
    OBJK_Representation3_Base base;
    ObjectId subscriber_id;

} DATAREADER_Representation;


typedef struct OBJK_PARTICIPANT_Representation
{
    OBJK_Representation3_Base base;
    int16_t domain_id;

} OBJK_PARTICIPANT_Representation;


typedef struct OBJK_TOPIC_Representation
{
    OBJK_Representation3_Base base;
    ObjectId participant_id;

} OBJK_TOPIC_Representation;


typedef struct OBJK_REQUESTER_Representation
{
    OBJK_Representation3_Base base;
    ObjectId participant_id;

} OBJK_REQUESTER_Representation;


typedef struct OBJK_REPLIER_Representation
{
    OBJK_Representation3_Base base;
    ObjectId participant_id;

} OBJK_REPLIER_Representation;


typedef struct OBJK_DomainParticipant_Binary
{
    bool optional_domain_reference;
    char* domain_reference;
    bool optional_qos_profile_reference;
    char* qos_profile_reference;

} OBJK_DomainParticipant_Binary;


typedef struct OBJK_Topic_Binary
{
    char* topic_name;
    bool optional_type_reference;
    char* type_reference;
    bool optional_type_name;
    char* type_name;

} OBJK_Topic_Binary;


typedef struct OBJK_Publisher_Binary_Qos
{
    bool optional_partitions;
    StringSequence_t partitions;
    bool optional_group_data;
    BinarySequence_t group_data;

} OBJK_Publisher_Binary_Qos;


typedef struct OBJK_Publisher_Binary
{
    bool optional_publisher_name;
    char* publisher_name;
    bool optional_qos;
    OBJK_Publisher_Binary_Qos qos;

} OBJK_Publisher_Binary;


typedef struct OBJK_Subscriber_Binary_Qos
{
    bool optional_partitions;
    StringSequence_t partitions;
    bool optional_group_data;
    BinarySequence_t group_data;

} OBJK_Subscriber_Binary_Qos;


typedef struct OBJK_Subscriber_Binary
{
    bool optional_subscriber_name;
    char* subscriber_name;
    bool optional_qos;
    OBJK_Subscriber_Binary_Qos qos;

} OBJK_Subscriber_Binary;


typedef enum EndpointQosFlags
{
    is_reliable = 0x01 << 0,
    is_history_keep_last = 0x01 << 1,
    is_ownership_exclusive = 0x01 << 2,
    is_durability_transient_local = 0x01 << 3,
    is_durability_transient = 0x01 << 4,
    is_durability_persistent = 0x01 << 5

} EndpointQosFlags;


typedef struct OBJK_Endpoint_QosBinary
{
    uint16_t qos_flags;
    bool optional_history_depth;
    uint16_t history_depth;
    bool optional_deadline_msec;
    uint32_t deadline_msec;
    bool optional_lifespan_msec;
    uint32_t lifespan_msec;
    bool optional_user_data;
    BinarySequenceSmall_t user_data;

} OBJK_Endpoint_QosBinary;


typedef struct OBJK_DataWriter_Binary_Qos
{
    OBJK_Endpoint_QosBinary base;
    bool optional_ownership_strength;
    uint64_t ownership_strength;

} OBJK_DataWriter_Binary_Qos;


typedef struct OBJK_DataReader_Binary_Qos
{
    OBJK_Endpoint_QosBinary base;
    bool optional_timebasedfilter_msec;
    uint64_t timebasedfilter_msec;
    bool optional_contentbased_filter;
    char* contentbased_filter;

} OBJK_DataReader_Binary_Qos;


typedef struct OBJK_DataReader_Binary
{
    ObjectId topic_id;
    bool optional_qos;
    OBJK_DataReader_Binary_Qos qos;

} OBJK_DataReader_Binary;


typedef struct OBJK_DataWriter_Binary
{
    ObjectId topic_id;
    bool optional_qos;
    OBJK_DataWriter_Binary_Qos qos;

} OBJK_DataWriter_Binary;

typedef struct OBJK_Requester_Binary
{
    char* service_name;
    char* request_type;
    char* reply_type;
    bool optional_request_topic_name;
    char* request_topic_name;
    bool optional_reply_topic_name;
    char* reply_topic_name;

} OBJK_Requester_Binary;

typedef struct OBJK_Replier_Binary
{
    char* service_name;
    char* request_type;
    char* reply_type;
    bool optional_request_topic_name;
    char* request_topic_name;
    bool optional_reply_topic_name;
    char* reply_topic_name;

} OBJK_Replier_Binary;

typedef union ObjectVariantU
{
    AGENT_Representation agent;
    CLIENT_Representation client;
    OBJK_APPLICATION_Representation application;
    OBJK_PARTICIPANT_Representation participant;
    OBJK_QOSPROFILE_Representation qos_profile;
    OBJK_TYPE_Representation type;
    OBJK_TOPIC_Representation topic;
    OBJK_PUBLISHER_Representation publisher;
    OBJK_SUBSCRIBER_Representation subscriber;
    OBJK_REQUESTER_Representation requester;
    OBJK_REPLIER_Representation replier;
    DATAWRITER_Representation data_writer;
    DATAREADER_Representation data_reader;

} ObjectVariantU;


typedef struct ObjectVariant
{
    uint8_t kind;
    ObjectVariantU _;

} ObjectVariant;


typedef struct CreationMode
{
    bool reuse;
    bool replace;

} CreationMode;


typedef struct RequestId
{
    uint8_t data[2];

} RequestId;


typedef struct ResultStatus
{
    uint8_t status;
    uint8_t implementation_status;

} ResultStatus;


typedef struct BaseObjectRequest
{
    RequestId request_id;
    ObjectId object_id;

} BaseObjectRequest;

typedef BaseObjectRequest RelatedObjectRequest;

typedef enum InfoMask
{
    INFO_CONFIGURATION = 0x01 << 0,
    INFO_ACTIVITY = 0x01 << 1

} InfoMask;


typedef struct AGENT_ActivityInfo
{
    int16_t availability;
    TransportLocatorSeq address_seq;

} AGENT_ActivityInfo;


typedef struct DATAREADER_ActivityInfo
{
    int16_t highest_acked_num;

} DATAREADER_ActivityInfo;


typedef struct DATAWRITER_ActivityInfo
{
    int16_t stream_seq_num;
    uint64_t sample_seq_num;

} DATAWRITER_ActivityInfo;


typedef union ActivityInfoVariantU
{
    AGENT_ActivityInfo agent;
    DATAWRITER_ActivityInfo data_writer;
    DATAREADER_ActivityInfo data_reader;

} ActivityInfoVariantU;


typedef struct ActivityInfoVariant
{
    uint8_t kind;
    ActivityInfoVariantU _;

} ActivityInfoVariant;


typedef struct ObjectInfo
{
    bool optional_config;
    ObjectVariant config;
    bool optional_activity;
    ActivityInfoVariant activity;

} ObjectInfo;


typedef struct BaseObjectReply
{
    BaseObjectRequest related_request;
    ResultStatus result;

} BaseObjectReply;


typedef enum DataFormat
{
    FORMAT_DATA = 0x00,
    FORMAT_SAMPLE = 0x02,
    FORMAT_DATA_SEQ = 0x08,
    FORMAT_SAMPLE_SEQ = 0x0A,
    FORMAT_PACKED_SAMPLES = 0x0E,
    FORMAT_MASK = 0x0E

} DataFormat;


typedef struct DataDeliveryControl
{
    uint16_t max_samples;
    uint16_t max_elapsed_time;
    uint16_t max_bytes_per_seconds;
    uint16_t min_pace_period;

} DataDeliveryControl;


typedef struct ReadSpecification
{
    uint8_t preferred_stream_id;
    uint8_t data_format;
    bool optional_content_filter_expression;
    char* content_filter_expression;
    bool optional_delivery_control;
    DataDeliveryControl delivery_control;

} ReadSpecification;


typedef enum SampleInfoFlags
{
    INSTANCE_STATE_UNREGISTERD = 0x01 << 0,
    INSTANCE_STATE_DISPOSED = 0x01 << 1,
    VIEW_STATE_NEW = 0x01 << 2,
    SAMPLE_STATE_READ = 0x01 << 3

} SampleInfoFlags;


typedef enum SampleInfoFormat
{
    FORMAT_EMPTY = 0x00,
    FORMAT_SEQNUM = 0x01,
    FORMAT_TIMESTAMP = 0x02,
    FORMAT_SEQN_TIMS = 0x03

} SampleInfoFormat;


typedef struct SeqNumberAndTimestamp
{
    uint32_t sequence_number;
    uint32_t session_time_offset;

} SeqNumberAndTimestamp;


typedef union SampleInfoDetailU
{
    uint32_t sequence_number;
    uint32_t session_time_offset;
    SeqNumberAndTimestamp seqnum_n_timestamp;

} SampleInfoDetailU;


typedef struct SampleInfoDetail
{
    uint32_t format;
    SampleInfoDetailU _;

} SampleInfoDetail;


typedef struct SampleInfo
{
    uint8_t state;
    SampleInfoDetail detail;

} SampleInfo;

typedef uint16_t DeciSecond;


typedef struct SampleInfoDelta
{
    uint8_t state;
    uint8_t seq_number_delta;
    uint16_t timestamp_delta;

} SampleInfoDelta;


typedef struct SampleData
{
    uint32_t size;
    uint8_t data[UXR_SAMPLE_DATA_SIZE_MAX];

} SampleData;


typedef struct SampleDataSeq
{
    uint32_t size;
    SampleData data[UXR_SAMPLE_DATA_SEQUENCE_MAX];

} SampleDataSeq;


typedef struct Sample
{
    SampleInfo info;
    SampleData data;

} Sample;


typedef struct SampleSeq
{
    uint32_t size;
    Sample data[UXR_SAMPLE_SEQUENCE_MAX];

} SampleSeq;


typedef struct SampleDelta
{
    SampleInfoDelta info_delta;
    SampleData data;

} SampleDelta;


typedef struct SampleDeltaSequence
{
    uint32_t size;
    SampleDelta data[UXR_SAMPLE_DELTA_SEQUENCE_MAX];

} SampleDeltaSequence;


typedef struct PackedSamples
{
    SampleInfo info_base;
    SampleDeltaSequence sample_delta_seq;

} PackedSamples;


typedef struct SamplePackedSeq
{
    uint32_t size;
    PackedSamples data[UXR_PACKED_SAMPLES_SEQUENCE_MAX];

} SamplePackedSeq;


typedef union DataRepresentationU
{
    SampleData data;
    Sample sample;
    SampleDataSeq data_seq;
    SampleSeq sample_seq;
    PackedSamples packed_samples;

} DataRepresentationU;


typedef struct DataRepresentation
{
    uint8_t format;
    DataRepresentationU _;

} DataRepresentation;


typedef struct CREATE_CLIENT_Payload
{
    CLIENT_Representation client_representation;

} CREATE_CLIENT_Payload;


typedef struct CREATE_Payload
{
    BaseObjectRequest base;
    ObjectVariant object_representation;

} CREATE_Payload;


typedef struct GET_INFO_Payload
{
    BaseObjectRequest base;
    uint32_t info_mask;

} GET_INFO_Payload;


typedef struct DELETE_Payload
{
    BaseObjectRequest base;

} DELETE_Payload;


typedef struct STATUS_AGENT_Payload
{
    ResultStatus result;
    AGENT_Representation agent_info;

} STATUS_AGENT_Payload;


typedef struct STATUS_Payload
{
    BaseObjectReply base;

} STATUS_Payload;


typedef struct INFO_Payload
{
    BaseObjectReply base;
    ObjectInfo object_info;

} INFO_Payload;


typedef struct READ_DATA_Payload
{
    BaseObjectRequest base;
    ReadSpecification read_specification;

} READ_DATA_Payload;


typedef struct WRITE_DATA_Payload_Data
{
    BaseObjectRequest base;

} WRITE_DATA_Payload_Data;


typedef struct WRITE_DATA_Payload_Sample
{
    BaseObjectRequest base;
    Sample sample;

} WRITE_DATA_Payload_Sample;


typedef struct WRITE_DATA_Payload_DataSeq
{
    BaseObjectRequest base;
    SampleDataSeq data_seq;

} WRITE_DATA_Payload_DataSeq;


typedef struct WRITE_DATA_Payload_SampleSeq
{
    BaseObjectRequest base;
    SampleSeq sample_seq;

} WRITE_DATA_Payload_SampleSeq;


typedef struct WRITE_DATA_Payload_PackedSamples
{
    BaseObjectRequest base;
    PackedSamples packed_samples;

} WRITE_DATA_Payload_PackedSamples;


typedef struct DATA_Payload_Data
{
    BaseObjectRequest base;

} DATA_Payload_Data;


typedef struct DATA_Payload_Sample
{
    BaseObjectRequest base;
    Sample sample;

} DATA_Payload_Sample;


typedef struct DATA_Payload_DataSeq
{
    BaseObjectRequest base;
    SampleDataSeq data_seq;

} DATA_Payload_DataSeq;


typedef struct DATA_Payload_SampleSeq
{
    BaseObjectRequest base;
    SampleSeq sample_seq;

} DATA_Payload_SampleSeq;


typedef struct DATA_Payload_PackedSamples
{
    BaseObjectRequest base;
    PackedSamples packed_samples;

} DATA_Payload_PackedSamples;


typedef struct ACKNACK_Payload
{
    uint16_t first_unacked_seq_num;
    uint8_t nack_bitmap[2];
    uint8_t stream_id;

} ACKNACK_Payload;

typedef struct HEARTBEAT_Payload
{
    uint16_t first_unacked_seq_nr;
    uint16_t last_unacked_seq_nr;
    uint8_t stream_id;

} HEARTBEAT_Payload;

typedef struct TIMESTAMP_Payload
{
    Time_t transmit_timestamp;

} TIMESTAMP_Payload;

typedef struct TIMESTAMP_REPLY_Payload
{
    Time_t transmit_timestamp;
    Time_t receive_timestamp;
    Time_t originate_timestamp;

} TIMESTAMP_REPLY_Payload;

typedef struct GuidPrefix
{
    uint8_t data[12];

} GuidPrefix_t;

typedef struct EntityId_t
{
    uint8_t entityKey[3];
    uint8_t entityKind;

} EntityId_t;

typedef struct GUID_t
{
    GuidPrefix_t guidPrefix;
    EntityId_t entityId;

} GUID_t;

typedef struct SequenceNumber_t
{
    int32_t high;
    uint32_t low;

} SequenceNumber_t;

typedef struct SampleIdentity
{
    GUID_t writer_guid;
    SequenceNumber_t sequence_number;

} SampleIdentity;

#ifdef PERFORMANCE_TESTING
typedef struct PERFORMANCE_Payload
{
    uint32_t epoch_time_lsb;
    uint32_t epoch_time_msb;
    uint8_t* buf;
    uint16_t len;

} PERFORMANCE_Payload;
#endif // ifdef PERFORMANCE_TESTING

bool uxr_serialize_Time_t(
        ucdrBuffer* buffer,
        const Time_t* input);
bool uxr_deserialize_Time_t(
        ucdrBuffer* buffer,
        Time_t* output);

bool uxr_serialize_BinarySequence_t(
        ucdrBuffer* buffer,
        const BinarySequence_t* input);
bool uxr_deserialize_BinarySequence_t(
        ucdrBuffer* buffer,
        BinarySequence_t* output);

bool uxr_serialize_StringSequence_t(
        ucdrBuffer* buffer,
        const StringSequence_t* input);
bool uxr_deserialize_StringSequence_t(
        ucdrBuffer* buffer,
        StringSequence_t* output);

bool uxr_serialize_ClientKey(
        ucdrBuffer* buffer,
        const ClientKey* input);
bool uxr_deserialize_ClientKey(
        ucdrBuffer* buffer,
        ClientKey* output);

bool uxr_serialize_ObjectId(
        ucdrBuffer* buffer,
        const ObjectId* input);
bool uxr_deserialize_ObjectId(
        ucdrBuffer* buffer,
        ObjectId* output);

bool uxr_serialize_ObjectPrefix(
        ucdrBuffer* buffer,
        const ObjectPrefix* input);
bool uxr_deserialize_ObjectPrefix(
        ucdrBuffer* buffer,
        ObjectPrefix* output);

bool uxr_serialize_XrceCookie(
        ucdrBuffer* buffer,
        const XrceCookie* input);
bool uxr_deserialize_XrceCookie(
        ucdrBuffer* buffer,
        XrceCookie* output);

bool uxr_serialize_XrceVersion(
        ucdrBuffer* buffer,
        const XrceVersion* input);
bool uxr_deserialize_XrceVersion(
        ucdrBuffer* buffer,
        XrceVersion* output);

bool uxr_serialize_XrceVendorId(
        ucdrBuffer* buffer,
        const XrceVendorId* input);
bool uxr_deserialize_XrceVendorId(
        ucdrBuffer* buffer,
        XrceVendorId* output);

bool uxr_serialize_TransportLocatorSmall(
        ucdrBuffer* buffer,
        const TransportLocatorSmall* input);
bool uxr_deserialize_TransportLocatorSmall(
        ucdrBuffer* buffer,
        TransportLocatorSmall* output);

bool uxr_serialize_TransportLocatorMedium(
        ucdrBuffer* buffer,
        const TransportLocatorMedium* input);
bool uxr_deserialize_TransportLocatorMedium(
        ucdrBuffer* buffer,
        TransportLocatorMedium* output);

bool uxr_serialize_TransportLocatorLarge(
        ucdrBuffer* buffer,
        const TransportLocatorLarge* input);
bool uxr_deserialize_TransportLocatorLarge(
        ucdrBuffer* buffer,
        TransportLocatorLarge* output);

bool uxr_serialize_TransportLocatorString(
        ucdrBuffer* buffer,
        const TransportLocatorString* input);
bool uxr_deserialize_TransportLocatorString(
        ucdrBuffer* buffer,
        TransportLocatorString* output);

bool uxr_serialize_TransportLocator(
        ucdrBuffer* buffer,
        const TransportLocator* input);
bool uxr_deserialize_TransportLocator(
        ucdrBuffer* buffer,
        TransportLocator* output);

bool uxr_serialize_TransportLocatorSeq(
        ucdrBuffer* buffer,
        const TransportLocatorSeq* input);
bool uxr_deserialize_TransportLocatorSeq(
        ucdrBuffer* buffer,
        TransportLocatorSeq* output);

bool uxr_serialize_Property(
        ucdrBuffer* buffer,
        const Property* input);
bool uxr_deserialize_Property(
        ucdrBuffer* buffer,
        Property* output);

bool uxr_serialize_PropertySeq(
        ucdrBuffer* buffer,
        const PropertySeq* input);
bool uxr_deserialize_PropertySeq(
        ucdrBuffer* buffer,
        PropertySeq* output);

bool uxr_serialize_CLIENT_Representation(
        ucdrBuffer* buffer,
        const CLIENT_Representation* input);
bool uxr_deserialize_CLIENT_Representation(
        ucdrBuffer* buffer,
        CLIENT_Representation* output);

bool uxr_serialize_AGENT_Representation(
        ucdrBuffer* buffer,
        const AGENT_Representation* input);
bool uxr_deserialize_AGENT_Representation(
        ucdrBuffer* buffer,
        AGENT_Representation* output);

bool uxr_serialize_OBJK_Representation3Formats(
        ucdrBuffer* buffer,
        const OBJK_Representation3Formats* input);
bool uxr_deserialize_OBJK_Representation3Formats(
        ucdrBuffer* buffer,
        OBJK_Representation3Formats* output);

bool uxr_serialize_OBJK_RepresentationRefAndXMLFormats(
        ucdrBuffer* buffer,
        const OBJK_RepresentationRefAndXMLFormats* input);
bool uxr_deserialize_OBJK_RepresentationRefAndXMLFormats(
        ucdrBuffer* buffer,
        OBJK_RepresentationRefAndXMLFormats* output);

bool uxr_serialize_OBJK_RepresentationBinAndXMLFormats(
        ucdrBuffer* buffer,
        const OBJK_RepresentationBinAndXMLFormats* input);
bool uxr_deserialize_OBJK_RepresentationBinAndXMLFormats(
        ucdrBuffer* buffer,
        OBJK_RepresentationBinAndXMLFormats* output);

bool uxr_serialize_OBJK_RepresentationRefAndXML_Base(
        ucdrBuffer* buffer,
        const OBJK_RepresentationRefAndXML_Base* input);
bool uxr_deserialize_OBJK_RepresentationRefAndXML_Base(
        ucdrBuffer* buffer,
        OBJK_RepresentationRefAndXML_Base* output);

bool uxr_serialize_OBJK_RepresentationBinAndXML_Base(
        ucdrBuffer* buffer,
        const OBJK_RepresentationBinAndXML_Base* input);
bool uxr_deserialize_OBJK_RepresentationBinAndXML_Base(
        ucdrBuffer* buffer,
        OBJK_RepresentationBinAndXML_Base* output);

bool uxr_serialize_OBJK_Representation3_Base(
        ucdrBuffer* buffer,
        const OBJK_Representation3_Base* input);
bool uxr_deserialize_OBJK_Representation3_Base(
        ucdrBuffer* buffer,
        OBJK_Representation3_Base* output);

bool uxr_serialize_OBJK_QOSPROFILE_Representation(
        ucdrBuffer* buffer,
        const OBJK_QOSPROFILE_Representation* input);
bool uxr_deserialize_OBJK_QOSPROFILE_Representation(
        ucdrBuffer* buffer,
        OBJK_QOSPROFILE_Representation* output);

bool uxr_serialize_OBJK_TYPE_Representation(
        ucdrBuffer* buffer,
        const OBJK_TYPE_Representation* input);
bool uxr_deserialize_OBJK_TYPE_Representation(
        ucdrBuffer* buffer,
        OBJK_TYPE_Representation* output);

bool uxr_serialize_OBJK_DOMAIN_Representation(
        ucdrBuffer* buffer,
        const OBJK_DOMAIN_Representation* input);
bool uxr_deserialize_OBJK_DOMAIN_Representation(
        ucdrBuffer* buffer,
        OBJK_DOMAIN_Representation* output);

bool uxr_serialize_OBJK_APPLICATION_Representation(
        ucdrBuffer* buffer,
        const OBJK_APPLICATION_Representation* input);
bool uxr_deserialize_OBJK_APPLICATION_Representation(
        ucdrBuffer* buffer,
        OBJK_APPLICATION_Representation* output);

bool uxr_serialize_OBJK_PUBLISHER_Representation(
        ucdrBuffer* buffer,
        const OBJK_PUBLISHER_Representation* input);
bool uxr_deserialize_OBJK_PUBLISHER_Representation(
        ucdrBuffer* buffer,
        OBJK_PUBLISHER_Representation* output);

bool uxr_serialize_OBJK_SUBSCRIBER_Representation(
        ucdrBuffer* buffer,
        const OBJK_SUBSCRIBER_Representation* input);
bool uxr_deserialize_OBJK_SUBSCRIBER_Representation(
        ucdrBuffer* buffer,
        OBJK_SUBSCRIBER_Representation* output);

bool uxr_serialize_DATAWRITER_Representation(
        ucdrBuffer* buffer,
        const DATAWRITER_Representation* input);
bool uxr_deserialize_DATAWRITER_Representation(
        ucdrBuffer* buffer,
        DATAWRITER_Representation* output);

bool uxr_serialize_DATAREADER_Representation(
        ucdrBuffer* buffer,
        const DATAREADER_Representation* input);
bool uxr_deserialize_DATAREADER_Representation(
        ucdrBuffer* buffer,
        DATAREADER_Representation* output);

bool uxr_serialize_OBJK_PARTICIPANT_Representation(
        ucdrBuffer* buffer,
        const OBJK_PARTICIPANT_Representation* input);
bool uxr_deserialize_OBJK_PARTICIPANT_Representation(
        ucdrBuffer* buffer,
        OBJK_PARTICIPANT_Representation* output);

bool uxr_serialize_OBJK_TOPIC_Representation(
        ucdrBuffer* buffer,
        const OBJK_TOPIC_Representation* input);
bool uxr_deserialize_OBJK_TOPIC_Representation(
        ucdrBuffer* buffer,
        OBJK_TOPIC_Representation* output);

bool uxr_serialize_OBJK_REQUESTER_Representation(
        ucdrBuffer* buffer,
        const OBJK_REQUESTER_Representation* input);
bool uxr_deserialize_OBJK_REQUESTER_Representation(
        ucdrBuffer* buffer,
        OBJK_REQUESTER_Representation* output);

bool uxr_serialize_OBJK_REPLIER_Representation(
        ucdrBuffer* buffer,
        const OBJK_REPLIER_Representation* input);
bool uxr_deserialize_OBJK_REPLIER_Representation(
        ucdrBuffer* buffer,
        OBJK_REPLIER_Representation* output);

bool uxr_serialize_OBJK_DomainParticipant_Binary(
        ucdrBuffer* buffer,
        const OBJK_DomainParticipant_Binary* input);
bool uxr_deserialize_OBJK_DomainParticipant_Binary(
        ucdrBuffer* buffer,
        OBJK_DomainParticipant_Binary* output);

bool uxr_serialize_OBJK_Topic_Binary(
        ucdrBuffer* buffer,
        const OBJK_Topic_Binary* input);
bool uxr_deserialize_OBJK_Topic_Binary(
        ucdrBuffer* buffer,
        OBJK_Topic_Binary* output);

bool uxr_serialize_OBJK_Publisher_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_Publisher_Binary_Qos* input);
bool uxr_deserialize_OBJK_Publisher_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_Publisher_Binary_Qos* output);

bool uxr_serialize_OBJK_Publisher_Binary(
        ucdrBuffer* buffer,
        const OBJK_Publisher_Binary* input);
bool uxr_deserialize_OBJK_Publisher_Binary(
        ucdrBuffer* buffer,
        OBJK_Publisher_Binary* output);

bool uxr_serialize_OBJK_Subscriber_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_Subscriber_Binary_Qos* input);
bool uxr_deserialize_OBJK_Subscriber_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_Subscriber_Binary_Qos* output);

bool uxr_serialize_OBJK_Subscriber_Binary(
        ucdrBuffer* buffer,
        const OBJK_Subscriber_Binary* input);
bool uxr_deserialize_OBJK_Subscriber_Binary(
        ucdrBuffer* buffer,
        OBJK_Subscriber_Binary* output);

bool uxr_serialize_OBJK_Endpoint_QosBinary(
        ucdrBuffer* buffer,
        const OBJK_Endpoint_QosBinary* input);
bool uxr_deserialize_OBJK_Endpoint_QosBinary(
        ucdrBuffer* buffer,
        OBJK_Endpoint_QosBinary* output);

bool uxr_serialize_OBJK_DataWriter_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_DataWriter_Binary_Qos* input);
bool uxr_deserialize_OBJK_DataWriter_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_DataWriter_Binary_Qos* output);

bool uxr_serialize_OBJK_DataReader_Binary_Qos(
        ucdrBuffer* buffer,
        const OBJK_DataReader_Binary_Qos* input);
bool uxr_deserialize_OBJK_DataReader_Binary_Qos(
        ucdrBuffer* buffer,
        OBJK_DataReader_Binary_Qos* output);

bool uxr_serialize_OBJK_DataReader_Binary(
        ucdrBuffer* buffer,
        const OBJK_DataReader_Binary* input);
bool uxr_deserialize_OBJK_DataReader_Binary(
        ucdrBuffer* buffer,
        OBJK_DataReader_Binary* output);

bool uxr_serialize_OBJK_DataWriter_Binary(
        ucdrBuffer* buffer,
        const OBJK_DataWriter_Binary* input);
bool uxr_deserialize_OBJK_DataWriter_Binary(
        ucdrBuffer* buffer,
        OBJK_DataWriter_Binary* output);

bool uxr_serialize_OBJK_Requester_Binary(
        ucdrBuffer* buffer,
        const OBJK_Requester_Binary* input);
bool uxr_deserialize_OBJK_Requester_Binary(
        ucdrBuffer* buffer,
        OBJK_Requester_Binary* output);

bool uxr_serialize_OBJK_Replier_Binary(
        ucdrBuffer* buffer,
        const OBJK_Replier_Binary* input);
bool uxr_deserialize_OBJK_Replier_Binary(
        ucdrBuffer* buffer,
        OBJK_Replier_Binary* output);

bool uxr_serialize_ObjectVariant(
        ucdrBuffer* buffer,
        const ObjectVariant* input);
bool uxr_deserialize_ObjectVariant(
        ucdrBuffer* buffer,
        ObjectVariant* output);

bool uxr_serialize_CreationMode(
        ucdrBuffer* buffer,
        const CreationMode* input);
bool uxr_deserialize_CreationMode(
        ucdrBuffer* buffer,
        CreationMode* output);

bool uxr_serialize_RequestId(
        ucdrBuffer* buffer,
        const RequestId* input);
bool uxr_deserialize_RequestId(
        ucdrBuffer* buffer,
        RequestId* output);

bool uxr_serialize_ResultStatus(
        ucdrBuffer* buffer,
        const ResultStatus* input);
bool uxr_deserialize_ResultStatus(
        ucdrBuffer* buffer,
        ResultStatus* output);

bool uxr_serialize_BaseObjectRequest(
        ucdrBuffer* buffer,
        const BaseObjectRequest* input);
bool uxr_deserialize_BaseObjectRequest(
        ucdrBuffer* buffer,
        BaseObjectRequest* output);

bool uxr_serialize_AGENT_ActivityInfo(
        ucdrBuffer* buffer,
        const AGENT_ActivityInfo* input);
bool uxr_deserialize_AGENT_ActivityInfo(
        ucdrBuffer* buffer,
        AGENT_ActivityInfo* output);

bool uxr_serialize_DATAREADER_ActivityInfo(
        ucdrBuffer* buffer,
        const DATAREADER_ActivityInfo* input);
bool uxr_deserialize_DATAREADER_ActivityInfo(
        ucdrBuffer* buffer,
        DATAREADER_ActivityInfo* output);

bool uxr_serialize_DATAWRITER_ActivityInfo(
        ucdrBuffer* buffer,
        const DATAWRITER_ActivityInfo* input);
bool uxr_deserialize_DATAWRITER_ActivityInfo(
        ucdrBuffer* buffer,
        DATAWRITER_ActivityInfo* output);

bool uxr_serialize_ActivityInfoVariant(
        ucdrBuffer* buffer,
        const ActivityInfoVariant* input);
bool uxr_deserialize_ActivityInfoVariant(
        ucdrBuffer* buffer,
        ActivityInfoVariant* output);

bool uxr_serialize_ObjectInfo(
        ucdrBuffer* buffer,
        const ObjectInfo* input);
bool uxr_deserialize_ObjectInfo(
        ucdrBuffer* buffer,
        ObjectInfo* output);

bool uxr_serialize_BaseObjectReply(
        ucdrBuffer* buffer,
        const BaseObjectReply* input);
bool uxr_deserialize_BaseObjectReply(
        ucdrBuffer* buffer,
        BaseObjectReply* output);

bool uxr_serialize_DataDeliveryControl(
        ucdrBuffer* buffer,
        const DataDeliveryControl* input);
bool uxr_deserialize_DataDeliveryControl(
        ucdrBuffer* buffer,
        DataDeliveryControl* output);

bool uxr_serialize_ReadSpecification(
        ucdrBuffer* buffer,
        const ReadSpecification* input);
bool uxr_deserialize_ReadSpecification(
        ucdrBuffer* buffer,
        ReadSpecification* output);

bool uxr_serialize_SeqNumberAndTimestamp(
        ucdrBuffer* buffer,
        const SeqNumberAndTimestamp* input);
bool uxr_deserialize_SeqNumberAndTimestamp(
        ucdrBuffer* buffer,
        SeqNumberAndTimestamp* output);

bool uxr_serialize_SampleInfoDetail(
        ucdrBuffer* buffer,
        const SampleInfoDetail* input);
bool uxr_deserialize_SampleInfoDetail(
        ucdrBuffer* buffer,
        SampleInfoDetail* output);

bool uxr_serialize_SampleInfo(
        ucdrBuffer* buffer,
        const SampleInfo* input);
bool uxr_deserialize_SampleInfo(
        ucdrBuffer* buffer,
        SampleInfo* output);

bool uxr_serialize_SampleInfoDelta(
        ucdrBuffer* buffer,
        const SampleInfoDelta* input);
bool uxr_deserialize_SampleInfoDelta(
        ucdrBuffer* buffer,
        SampleInfoDelta* output);

bool uxr_serialize_SampleData(
        ucdrBuffer* buffer,
        const SampleData* input);
bool uxr_deserialize_SampleData(
        ucdrBuffer* buffer,
        SampleData* output);

bool uxr_serialize_SampleDataSeq(
        ucdrBuffer* buffer,
        const SampleDataSeq* input);
bool uxr_deserialize_SampleDataSeq(
        ucdrBuffer* buffer,
        SampleDataSeq* output);

bool uxr_serialize_Sample(
        ucdrBuffer* buffer,
        const Sample* input);
bool uxr_deserialize_Sample(
        ucdrBuffer* buffer,
        Sample* output);

bool uxr_serialize_SampleSeq(
        ucdrBuffer* buffer,
        const SampleSeq* input);
bool uxr_deserialize_SampleSeq(
        ucdrBuffer* buffer,
        SampleSeq* output);

bool uxr_serialize_SampleDelta(
        ucdrBuffer* buffer,
        const SampleDelta* input);
bool uxr_deserialize_SampleDelta(
        ucdrBuffer* buffer,
        SampleDelta* output);

bool uxr_serialize_SampleDeltaSequence(
        ucdrBuffer* buffer,
        const SampleDeltaSequence* input);
bool uxr_deserialize_SampleDeltaSequence(
        ucdrBuffer* buffer,
        SampleDeltaSequence* output);

bool uxr_serialize_PackedSamples(
        ucdrBuffer* buffer,
        const PackedSamples* input);
bool uxr_deserialize_PackedSamples(
        ucdrBuffer* buffer,
        PackedSamples* output);

bool uxr_serialize_SamplePackedSeq(
        ucdrBuffer* buffer,
        const SamplePackedSeq* input);
bool uxr_deserialize_SamplePackedSeq(
        ucdrBuffer* buffer,
        SamplePackedSeq* output);

bool uxr_serialize_DataRepresentation(
        ucdrBuffer* buffer,
        const DataRepresentation* input);
bool uxr_deserialize_DataRepresentation(
        ucdrBuffer* buffer,
        DataRepresentation* output);

bool uxr_serialize_CREATE_CLIENT_Payload(
        ucdrBuffer* buffer,
        const CREATE_CLIENT_Payload* input);
bool uxr_deserialize_CREATE_CLIENT_Payload(
        ucdrBuffer* buffer,
        CREATE_CLIENT_Payload* output);

bool uxr_serialize_CREATE_Payload(
        ucdrBuffer* buffer,
        const CREATE_Payload* input);
bool uxr_deserialize_CREATE_Payload(
        ucdrBuffer* buffer,
        CREATE_Payload* output);

bool uxr_serialize_GET_INFO_Payload(
        ucdrBuffer* buffer,
        const GET_INFO_Payload* input);
bool uxr_deserialize_GET_INFO_Payload(
        ucdrBuffer* buffer,
        GET_INFO_Payload* output);

bool uxr_serialize_DELETE_Payload(
        ucdrBuffer* buffer,
        const DELETE_Payload* input);
bool uxr_deserialize_DELETE_Payload(
        ucdrBuffer* buffer,
        DELETE_Payload* output);

bool uxr_serialize_STATUS_AGENT_Payload(
        ucdrBuffer* buffer,
        const STATUS_AGENT_Payload* input);
bool uxr_deserialize_STATUS_AGENT_Payload(
        ucdrBuffer* buffer,
        STATUS_AGENT_Payload* output);

bool uxr_serialize_STATUS_Payload(
        ucdrBuffer* buffer,
        const STATUS_Payload* input);
bool uxr_deserialize_STATUS_Payload(
        ucdrBuffer* buffer,
        STATUS_Payload* output);

bool uxr_serialize_INFO_Payload(
        ucdrBuffer* buffer,
        const INFO_Payload* input);
bool uxr_deserialize_INFO_Payload(
        ucdrBuffer* buffer,
        INFO_Payload* output);

bool uxr_serialize_READ_DATA_Payload(
        ucdrBuffer* buffer,
        const READ_DATA_Payload* input);
bool uxr_deserialize_READ_DATA_Payload(
        ucdrBuffer* buffer,
        READ_DATA_Payload* output);

bool uxr_serialize_WRITE_DATA_Payload_Data(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_Data* input);
bool uxr_deserialize_WRITE_DATA_Payload_Data(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_Data* output);

bool uxr_serialize_WRITE_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_Sample* input);
bool uxr_deserialize_WRITE_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_Sample* output);

bool uxr_serialize_WRITE_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_DataSeq* input);
bool uxr_deserialize_WRITE_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_DataSeq* output);

bool uxr_serialize_WRITE_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_SampleSeq* input);
bool uxr_deserialize_WRITE_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_SampleSeq* output);

bool uxr_serialize_WRITE_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        const WRITE_DATA_Payload_PackedSamples* input);
bool uxr_deserialize_WRITE_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        WRITE_DATA_Payload_PackedSamples* output);

bool uxr_serialize_DATA_Payload_Data(
        ucdrBuffer* buffer,
        const DATA_Payload_Data* input);
bool uxr_deserialize_DATA_Payload_Data(
        ucdrBuffer* buffer,
        DATA_Payload_Data* output);

bool uxr_serialize_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        const DATA_Payload_Sample* input);
bool uxr_deserialize_DATA_Payload_Sample(
        ucdrBuffer* buffer,
        DATA_Payload_Sample* output);

bool uxr_serialize_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        const DATA_Payload_DataSeq* input);
bool uxr_deserialize_DATA_Payload_DataSeq(
        ucdrBuffer* buffer,
        DATA_Payload_DataSeq* output);

bool uxr_serialize_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        const DATA_Payload_SampleSeq* input);
bool uxr_deserialize_DATA_Payload_SampleSeq(
        ucdrBuffer* buffer,
        DATA_Payload_SampleSeq* output);

bool uxr_serialize_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        const DATA_Payload_PackedSamples* input);
bool uxr_deserialize_DATA_Payload_PackedSamples(
        ucdrBuffer* buffer,
        DATA_Payload_PackedSamples* output);

bool uxr_serialize_ACKNACK_Payload(
        ucdrBuffer* buffer,
        const ACKNACK_Payload* input);
bool uxr_deserialize_ACKNACK_Payload(
        ucdrBuffer* buffer,
        ACKNACK_Payload* output);

bool uxr_serialize_HEARTBEAT_Payload(
        ucdrBuffer* buffer,
        const HEARTBEAT_Payload* input);
bool uxr_deserialize_HEARTBEAT_Payload(
        ucdrBuffer* buffer,
        HEARTBEAT_Payload* output);

bool uxr_serialize_TIMESTAMP_Payload(
        ucdrBuffer* buffer,
        const TIMESTAMP_Payload* input);
bool uxr_deserialize_TIMESTAMP_Payload(
        ucdrBuffer* buffer,
        TIMESTAMP_Payload* output);

bool uxr_serialize_TIMESTAMP_REPLY_Payload(
        ucdrBuffer* buffer,
        const TIMESTAMP_REPLY_Payload* input);
bool uxr_deserialize_TIMESTAMP_REPLY_Payload(
        ucdrBuffer* buffer,
        TIMESTAMP_REPLY_Payload* output);

bool uxr_serialize_GuidPrefix_t(
        ucdrBuffer* buffer,
        const GuidPrefix_t* input);
bool uxr_deserialize_GuidPrefix_t(
        ucdrBuffer* buffer,
        GuidPrefix_t* output);

bool uxr_serialize_EntityId_t(
        ucdrBuffer* buffer,
        const EntityId_t* input);
bool uxr_deserialize_EntityId_t(
        ucdrBuffer* buffer,
        EntityId_t* output);

bool uxr_serialize_GUID_t(
        ucdrBuffer* buffer,
        const GUID_t* input);
bool uxr_deserialize_GUID_t(
        ucdrBuffer* buffer,
        GUID_t* output);

bool uxr_serialize_SequenceNumber_t(
        ucdrBuffer* buffer,
        const SequenceNumber_t* input);
bool uxr_deserialize_SequenceNumber_t(
        ucdrBuffer* buffer,
        SequenceNumber_t* output);

bool uxr_serialize_SampleIdentity(
        ucdrBuffer* buffer,
        const SampleIdentity* input);
bool uxr_deserialize_SampleIdentity(
        ucdrBuffer* buffer,
        SampleIdentity* output);

#ifdef PERFORMANCE_TESTING
bool uxr_serialize_PERFORMANCE_Payload(
        ucdrBuffer* buffer,
        const PERFORMANCE_Payload* input);
bool uxr_deserialize_PERFORMANCE_Payload(
        ucdrBuffer* buffer,
        PERFORMANCE_Payload* input);
#endif // ifdef PERFORMANCE_TESTING

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // UXR_CLIENT_CORE_TYPE_XRCETYPES_H_
