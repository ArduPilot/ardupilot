#!/usr/bin/env python3
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import os
from . import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_version_h(directory, xml):
    '''generate version.h'''
    f = open(os.path.join(directory, "version.h"), mode='w')
    t.write(f,'''
/** @file
 *  @brief MAVLink comm protocol built from ${basename}.xml
 *  @see http://mavlink.org
 */
#pragma once
 
#ifndef MAVLINK_VERSION_H
#define MAVLINK_VERSION_H

#define MAVLINK_BUILD_DATE "${parse_time}"
#define MAVLINK_WIRE_PROTOCOL_VERSION "${wire_protocol_version}"
#define MAVLINK_MAX_DIALECT_PAYLOAD_SIZE ${largest_payload}
 
#endif // MAVLINK_VERSION_H
''', xml)
    f.close()

def generate_mavlink_h(directory, xml):
    '''generate mavlink.h'''
    f = open(os.path.join(directory, "mavlink.h"), mode='w')
    t.write(f,'''
/** @file
 *  @brief MAVLink comm protocol built from ${basename}.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_PRIMARY_XML_HASH ${xml_hash}

#ifndef MAVLINK_STX
#define MAVLINK_STX ${protocol_marker}
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN ${mavlink_endian}
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS ${aligned_fields_define}
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA ${crc_extra_define}
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT ${command_24bit_define}
#endif

#include "version.h"
#include "${basename}.h"

#endif // MAVLINK_H
''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, xml.basename + ".h"), mode='w')
    t.write(f, '''
/** @file
 *  @brief MAVLink comm protocol generated from ${basename}.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_${basename_upper}_H
#define MAVLINK_${basename_upper}_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_${basename_upper}.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_${basename_upper}_XML_HASH ${xml_hash}

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {${message_lengths_array}}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {${message_crcs_array}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_${basename_upper}

// ENUM DEFINITIONS

${{enum:
/** @brief ${description} */
#ifndef HAVE_ENUM_${name}
#define HAVE_ENUM_${name}
typedef enum ${name}
{
${{entry:   ${name}=${value}, /* ${description} |${{param:${description}| }} */
}}
} ${name};
#endif
}}

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION ${version}
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION ${version}
#endif

// MESSAGE DEFINITIONS
${{message:#include "./mavlink_msg_${name_lower}.h"
}}

// base include
${{include_list:#include "../${base}/${base}.h"
}}


#if MAVLINK_${basename_upper}_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {${message_info_array}}
# define MAVLINK_MESSAGE_NAMES {${message_name_array}}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_${basename_upper}_H
''', xml)

    f.close()
             

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    if m.wip:
        m.MSG_ATTRIBUTE = 'MAVLINK_WIP\n'
    else:
        m.MSG_ATTRIBUTE = ''
    f = open(os.path.join(directory, 'mavlink_msg_%s.h' % m.name_lower), mode='w')
    t.write(f, '''
#pragma once
// MESSAGE ${name} PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_${name} ${id}

${MAVPACKED_START}
typedef struct __mavlink_${name_lower}_t {
${{ordered_fields: ${type} ${name}${array_suffix}; /*< ${units} ${description}*/
}}
}${MAVPACKED_END} mavlink_${name_lower}_t;

#define MAVLINK_MSG_ID_${name}_LEN ${wire_length}
#define MAVLINK_MSG_ID_${name}_MIN_LEN ${wire_min_length}
#define MAVLINK_MSG_ID_${id}_LEN ${wire_length}
#define MAVLINK_MSG_ID_${id}_MIN_LEN ${wire_min_length}

#define MAVLINK_MSG_ID_${name}_CRC ${crc_extra}
#define MAVLINK_MSG_ID_${id}_CRC ${crc_extra}

${{array_fields:#define MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN ${array_length}
}}

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_${name} { \\
    ${id}, \\
    "${name}", \\
    ${num_fields}, \\
    { ${{fields: { "${name}", ${c_print_format}, MAVLINK_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mavlink_${name_lower}_t, ${name}) }, \\
        }} } \\
}
#else
#define MAVLINK_MESSAGE_INFO_${name} { \\
    "${name}", \\
    ${num_fields}, \\
    { ${{fields: { "${name}", ${c_print_format}, MAVLINK_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mavlink_${name_lower}_t, ${name}) }, \\
        }} } \\
}
#endif

/**
 * @brief Pack a ${name_lower} message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
${{arg_fields: * @param ${name} ${units} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
${MSG_ATTRIBUTE}static inline uint16_t mavlink_msg_${name_lower}_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                              ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_${name}_LEN];
${{scalar_fields:    _mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_${name}_LEN);
#else
    mavlink_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_${name}_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_${name};
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
}

/**
 * @brief Pack a ${name_lower} message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
${{arg_fields: * @param ${name} ${units} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_${name_lower}_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                              ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_${name}_LEN];
${{scalar_fields:    _mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_${name}_LEN);
#else
    mavlink_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_${name}_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_${name};
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN);
#endif
}

/**
 * @brief Pack a ${name_lower} message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
${{arg_fields: * @param ${name} ${units} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
${MSG_ATTRIBUTE}static inline uint16_t mavlink_msg_${name_lower}_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   ${{arg_fields:${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_${name}_LEN];
${{scalar_fields:    _mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_${name}_LEN);
#else
    mavlink_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_${name}_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_${name};
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
}

/**
 * @brief Encode a ${name_lower} struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
${MSG_ATTRIBUTE}static inline uint16_t mavlink_msg_${name_lower}_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_${name_lower}_t* ${name_lower})
{
    return mavlink_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Encode a ${name_lower} struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
${MSG_ATTRIBUTE}static inline uint16_t mavlink_msg_${name_lower}_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_${name_lower}_t* ${name_lower})
{
    return mavlink_msg_${name_lower}_pack_chan(system_id, component_id, chan, msg,${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Encode a ${name_lower} struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_${name_lower}_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_${name_lower}_t* ${name_lower})
{
    return mavlink_msg_${name_lower}_pack_status(system_id, component_id, _status, msg, ${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 *
${{arg_fields: * @param ${name} ${units} ${description}
}}
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

${MSG_ATTRIBUTE}static inline void mavlink_msg_${name_lower}_send(mavlink_channel_t chan,${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_${name}_LEN];
${{scalar_fields:    _mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, buf, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
#else
    mavlink_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, (const char *)&packet, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
#endif
}

/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
${MSG_ATTRIBUTE}static inline void mavlink_msg_${name_lower}_send_struct(mavlink_channel_t chan, const mavlink_${name_lower}_t* ${name_lower})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_${name_lower}_send(chan,${{arg_fields: ${name_lower}->${name},}});
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, (const char *)${name_lower}, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
#endif
}

#if MAVLINK_MSG_ID_${name}_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
${MSG_ATTRIBUTE}static inline void mavlink_msg_${name_lower}_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan, ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
${{scalar_fields:    _mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, buf, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
#else
    mavlink_${name_lower}_t *packet = (mavlink_${name_lower}_t *)msgbuf;
${{scalar_fields:    packet->${name} = ${putname};
}}
${{array_fields:    mav_array_memcpy(packet->${name}, ${name}, sizeof(${type})*${array_length});
}}
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, (const char *)packet, MAVLINK_MSG_ID_${name}_MIN_LEN, MAVLINK_MSG_ID_${name}_LEN, MAVLINK_MSG_ID_${name}_CRC);
#endif
}
#endif

#endif

// MESSAGE ${name} UNPACKING

${{fields:
/**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${units} ${description}
 */
${MSG_ATTRIBUTE}static inline ${return_type} mavlink_msg_${name_lower}_get_${name}(const mavlink_message_t* msg${get_arg})
{
    return _MAV_RETURN_${type}${array_tag}(msg, ${array_return_arg} ${wire_offset});
}
}}

/**
 * @brief Decode a ${name_lower} message into a struct
 *
 * @param msg The message to decode
 * @param ${name_lower} C-struct to decode the message contents into
 */
${MSG_ATTRIBUTE}static inline void mavlink_msg_${name_lower}_decode(const mavlink_message_t* msg, mavlink_${name_lower}_t* ${name_lower})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
${{ordered_fields:    ${decode_left}mavlink_msg_${name_lower}_get_${name}(msg${decode_right});
}}
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_${name}_LEN? msg->len : MAVLINK_MSG_ID_${name}_LEN;
        memset(${name_lower}, 0, MAVLINK_MSG_ID_${name}_LEN);
    memcpy(${name_lower}, _MAV_PAYLOAD(msg), len);
#endif
}
''', m)
    f.close()


def generate_testsuite_h(directory, xml):
    '''generate testsuite.h per XML file'''
    f = open(os.path.join(directory, "testsuite.h"), mode='w')
    t.write(f, '''
/** @file
 *    @brief MAVLink comm protocol testsuite generated from ${basename}.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ${basename_upper}_TESTSUITE_H
#define ${basename_upper}_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
${{include_list:static void mavlink_test_${base}(uint8_t, uint8_t, mavlink_message_t *last_msg);
}}
static void mavlink_test_${basename}(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
${{include_list:    mavlink_test_${base}(system_id, component_id, last_msg);
}}
    mavlink_test_${basename}(system_id, component_id, last_msg);
}
#endif

${{include_list:#include "../${base}/testsuite.h"
}}

${{message:
static void mavlink_test_${name_lower}(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_${name} >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_${name_lower}_t packet_in = {
        ${{ordered_fields:${c_test_value},}}
    };
    mavlink_${name_lower}_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        ${{scalar_fields:packet1.${name} = packet_in.${name};
        }}
        ${{array_fields:mav_array_memcpy(packet1.${name}, packet_in.${name}, sizeof(${type})*${array_length});
        }}
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_${name}_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_${name}_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_pack(system_id, component_id, &msg ${{arg_fields:, packet1.${name} }});
    mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg ${{arg_fields:, packet1.${name} }});
    mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_${name_lower}_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_${name_lower}_send(MAVLINK_COMM_1 ${{arg_fields:, packet1.${name} }});
    mavlink_msg_${name_lower}_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("${name}") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_${name}) != NULL);
#endif
}
}}

static void mavlink_test_${basename}(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
${{message:    mavlink_test_${name_lower}(system_id, component_id, last_msg);
}}
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ${basename_upper}_TESTSUITE_H
''', xml)

    f.close()

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil, filecmp
    hlist = {
        "0.9": [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h' ],
        "1.0": [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h', 'mavlink_conversions.h' ],
        "2.0": [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h', 'mavlink_conversions.h',
                 'mavlink_get_info.h', 'mavlink_sha256.h' ]
        }
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'C/include_v%s' % xml.wire_protocol_version)
    print("Copying fixed headers for protocol %s to %s" % (xml.wire_protocol_version, directory))
    for h in hlist[xml.wire_protocol_version]:
        src = os.path.realpath(os.path.join(srcpath, h))
        dest = os.path.realpath(os.path.join(directory, h))
        if src == dest or (os.path.exists(dest) and filecmp.cmp(src, dest)):
            continue
        shutil.copy(src, dest)

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating C implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.command_24bit:
        xml.command_24bit_define = "1"
    else:
        xml.command_24bit_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = os.path.basename(i)[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    if not xml.command_24bit:
        for msgid in range(256):
            mlen = xml.message_min_lengths.get(msgid, 0)
            xml.message_lengths_array += '%u, ' % mlen
        xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_crcs.keys()):
            xml.message_crcs_array += '{%u, %u, %u, %u, %u, %u, %u}, ' % (msgid,
                                                                      xml.message_crcs[msgid],
                                                                      xml.message_min_lengths[msgid],
                                                                      xml.message_lengths[msgid],
                                                                      xml.message_flags[msgid],
                                                                      xml.message_target_system_ofs[msgid],
                                                                      xml.message_target_component_ofs[msgid])
    else:
        for msgid in range(256):
            crc = xml.message_crcs.get(msgid, 0)
            xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_names.keys()):
            name = xml.message_names[msgid]
            xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
    else:
        for msgid in range(256):
            name = xml.message_names.get(msgid, None)
            if name is not None:
                xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
            else:
                # Several C compilers don't accept {NULL} for
                # multi-dimensional arrays and structs
                # feed the compiler a "filled" empty message
                xml.message_info_array += '{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # form message name array
    xml.message_name_array = ''
    # sort by names
    for msgid, name in sorted(xml.message_names.items(), key=lambda k_v: (k_v[1], k_v[0])):
        xml.message_name_array += '{ "%s", %u }, ' % (name, msgid)
    xml.message_name_array = xml.message_name_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = '_array'
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s->%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
            else:
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s->%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'int64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value
        if m.needs_pack:
            m.MAVPACKED_START = "MAVPACKED("
            m.MAVPACKED_END = ")"
        else:
            m.MAVPACKED_START = ""
            m.MAVPACKED_END = ""

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_mavlink_h(directory, xml)
    generate_version_h(directory, xml)
    generate_main_h(directory, xml)
    for m in xml.message:
        generate_message_h(directory, m)
    generate_testsuite_h(directory, xml)


def generate(basename, xml_list):
    '''generate complete MAVLink C implemenation'''

    for idx in range(len(xml_list)):
        xml = xml_list[idx]
        xml.xml_hash = hash(xml.basename)        
        generate_one(basename, xml)
    copy_fixed_headers(basename, xml_list[0])
