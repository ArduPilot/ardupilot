/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  support for Inertial Sense INS
 */


#include <algorithm>
#include <cstdint>

#include <stdio.h>

#ifdef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#undef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#endif
#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_InertialSense.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS_FixType.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

/*
 * Inertial Sense SDK function implementations, condensed from ISComm.c and
 * data_sets.c. Only the subset used by this driver is included.
 */

#ifndef _MIN
#  define _MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

// ===== ISComm.h: special byte constants (from ePktSpecialChars) =====

enum ePktSpecialChars
{
    PSC_NMEA_START_BYTE     = 0x24,
    PSC_NMEA_PRE_END_BYTE   = 0x0D,
    PSC_NMEA_END_BYTE       = 0x0A,
    PSC_ISB_PREAMBLE_BYTE1  = 0xEF,
    PSC_ISB_PREAMBLE_BYTE2  = 0x49,
    PSC_ISB_PREAMBLE        = (PSC_ISB_PREAMBLE_BYTE2 << 8) | PSC_ISB_PREAMBLE_BYTE1,
    UBLOX_START_BYTE1       = 0xB5,
    UBLOX_START_BYTE2       = 0x62,
    RTCM3_START_BYTE        = 0xD3,
    SPARTN_START_BYTE       = 0x73,
    SONY_START_BYTE         = 0x7F,
};

// ===== data_sets.c: devInfoPopulateMissingHardware =====

static void devInfoPopulateMissingHardware(dev_info_t *devInfo)
{
    if (devInfo->hardwareType != IS_HARDWARE_TYPE_UNKNOWN) {
        return;
    }
    int year = ((int)(devInfo->buildYear)) + 2000;
    if (year <= 2024) {
        switch (devInfo->hardwareVer[0]) {
        case 2: devInfo->hardwareType = IS_HARDWARE_TYPE_EVB;  break;
        case 3: devInfo->hardwareType = IS_HARDWARE_TYPE_UINS; break;
        case 5: devInfo->hardwareType = IS_HARDWARE_TYPE_IMX;  break;
        }
    }
}

// ===== ISComm.c: checksum helpers =====

typedef union
{
    uint16_t ck;
    struct { uint8_t a; uint8_t b; };
} checksum16_u;

static uint16_t is_comm_fletcher16(uint16_t cksum_init, const void* data, uint32_t size)
{
    checksum16_u cksum;
    cksum.ck = cksum_init;
    for (uint32_t i = 0; i < size; i++) {
        cksum.a += ((const uint8_t*)data)[i];
        cksum.b += cksum.a;
    }
    return cksum.ck;
}

#define is_comm_isb_checksum16 is_comm_fletcher16

// ===== ISComm.c: parse constants =====

#define MAX_MSG_LENGTH_ISB      PKT_BUF_SIZE
#define PKT_PARSER_TIMEOUT_MS   100

// ===== ISComm.c: buffer and parser helpers =====

static int is_comm_reset_buffer(is_comm_instance_t* c)
{
    c->parser.state = 0;
    c->rxBuf.head = c->rxBuf.tail = c->rxBuf.scan = c->rxBuf.scanPrior = c->rxBuf.start;
    c->processPkt = nullptr;
    return (int)c->rxBuf.size;
}

static inline void is_comm_reset_parser(is_comm_instance_t* c)
{
    c->parser.state = 0;
    c->rxBuf.scanPrior = c->rxBuf.scan;
    c->rxBuf.scan = c->rxBuf.head;
    c->processPkt = nullptr;
}

static inline void is_comm_to_isb_p_data(const is_comm_instance_t *comm, p_data_t *data)
{
    data->hdr.id     = comm->rxPkt.dataHdr.id;
    data->hdr.offset = comm->rxPkt.offset;
    data->hdr.size   = (uint16_t)comm->rxPkt.data.size;
    data->ptr        = comm->rxPkt.data.ptr;
}

static inline protocol_type_t reportParseError(is_comm_instance_t* c, eParseErrorType errorType)
{
    if (!c->rxErrorState) {
        c->rxErrorState = 1;
        c->rxErrorCount++;
        c->rxErrorType = errorType;
        c->rxErrorTypeCount[errorType]++;
        return _PTYPE_PARSE_ERROR;
    }
    return _PTYPE_NONE;
}

static inline protocol_type_t parseErrorResetState(is_comm_instance_t* c, eParseErrorType errorType)
{
    is_comm_reset_parser(c);
    return reportParseError(c, errorType);
}

static inline void validPacketReset(is_comm_instance_t* c, int pktSize)
{
    c->rxBuf.head += pktSize;
    c->rxPktCount++;
    c->processPkt = nullptr;
    c->rxErrorState = 0;
}


static void setParserStart(is_comm_instance_t* c, pFnProcessPkt processPkt)
{
    c->parser.state = 1;
    c->rxBuf.head   = c->rxBuf.scan;
    c->processPkt   = processPkt;
}

// ===== ISComm.c: ISB packet parser =====

static protocol_type_t processIsbPkt(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;

    switch (p->state) {
    case 0:
        if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE1) {
            p->state++;
        }
        return _PTYPE_NONE;

    case 1:
        if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE2) {
            p->state++;
            return _PTYPE_NONE;
        }
        return parseErrorResetState(c, EPARSE_INVALID_PREAMBLE);

    case 2: {
        numBytes = (int)(c->rxBuf.scan - c->rxBuf.head);
        if (numBytes < (int)(sizeof(packet_hdr_t) - 1)) {
            return _PTYPE_NONE;
        }
        p->state++;
        packet_buf_t *isbPkt = (packet_buf_t*)(c->rxBuf.head);
        p->size = (uint16_t)(sizeof(packet_hdr_t) + isbPkt->hdr.payloadSize + 2);
        if (p->size > MAX_MSG_LENGTH_ISB) {
            return parseErrorResetState(c, EPARSE_INVALID_SIZE);
        }
        return _PTYPE_NONE;
    }

    default:
        numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
        if (numBytes < (int)(p->size)) {
            return _PTYPE_NONE;
        }
        break;
    }

    p->state = 0;

    packet_buf_t *isbPkt = (packet_buf_t*)(c->rxBuf.head);
    if (isbPkt->hdr.payloadSize > MAX_MSG_LENGTH_ISB) {
        return parseErrorResetState(c, EPARSE_INVALID_SIZE);
    }
    if ((isbPkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) &&
        (isbPkt->payload.offset + isbPkt->hdr.payloadSize > MAX_MSG_LENGTH_ISB)) {
        return parseErrorResetState(c, EPARSE_INVALID_HEADER);
    }

    uint16_t payloadSize = isbPkt->hdr.payloadSize;
    uint8_t *payload     = c->rxBuf.head + sizeof(packet_hdr_t);
    int bytes_cksum      = p->size - 2;
    uint16_t calcCksum   = is_comm_isb_checksum16(0, c->rxBuf.head, (uint32_t)bytes_cksum);
    uint16_t rxCksum;
    memcpy(&rxCksum, payload + payloadSize, sizeof(uint16_t));
    if (rxCksum != calcCksum) {
        return parseErrorResetState(c, EPARSE_INVALID_CHKSUM);
    }

    validPacketReset(c, numBytes);

    packet_t *pkt          = &(c->rxPkt);
    pkt->hdr.preamble      = isbPkt->hdr.preamble;
    pkt->hdr.flags         = isbPkt->hdr.flags;
    pkt->id = pkt->hdr.id  = isbPkt->hdr.id;
    pkt->hdr.payloadSize   = payloadSize;

    if (pkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) {
        pkt->data.size    = (payloadSize < 2 ? 0u : (uint32_t)(payloadSize - 2));
        pkt->data.ptr     = (pkt->data.size ? payload + 2 : nullptr);
        memcpy(&pkt->offset, payload, sizeof(uint16_t));
        pkt->dataHdr.size = (uint16_t)pkt->data.size;
    } else {
        pkt->data.size    = payloadSize;
        pkt->data.ptr     = (payloadSize ? payload : nullptr);
        pkt->offset       = 0;
    }

    pkt->checksum = rxCksum;
    pkt->size     = p->size;
    c->ackNeeded  = 0;

    uint8_t ptype = pkt->hdr.flags & PKT_TYPE_MASK;
    switch (ptype) {
    case PKT_TYPE_SET_DATA:
    case PKT_TYPE_DATA:
        if (pkt->data.size <= MAX_DATASET_SIZE) {
            if (ptype == PKT_TYPE_SET_DATA) {
                c->ackNeeded = PKT_TYPE_ACK;
            }
            return _PTYPE_INERTIAL_SENSE_DATA;
        } else {
            c->ackNeeded = PKT_TYPE_NACK;
        }
        break;

    case PKT_TYPE_GET_DATA: {
        p_data_get_t *get = (p_data_get_t*)&(isbPkt->payload.data);
        if (get->size <= MAX_DATASET_SIZE) {
            return _PTYPE_INERTIAL_SENSE_CMD;
        }
        break;
    }

    case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
    case PKT_TYPE_STOP_DID_BROADCAST:
    case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
        return _PTYPE_INERTIAL_SENSE_CMD;

    case PKT_TYPE_ACK:
    case PKT_TYPE_NACK:
        return _PTYPE_INERTIAL_SENSE_ACK;
    }

    return parseErrorResetState(c, EPARSE_INVALID_DATATYPE);
}

// ===== ISComm.c: init and protocol registration =====

static void is_comm_init(is_comm_instance_t* c, uint8_t *buffer, int bufferSize,
                          pfnIsCommHandler pktHandler)
{
    memset(c, 0, sizeof(is_comm_instance_t));
    memset(buffer, 0, (size_t)bufferSize);

    c->rxBuf.size  = (uint32_t)bufferSize;
    c->rxBuf.start = buffer;
    c->rxBuf.end   = buffer + bufferSize;

    is_comm_reset_buffer(c);

    c->cb.protocolMask = DEFAULT_PROTO_MASK;
    c->rxPkt.data.ptr  = c->rxBuf.start;
    c->rxErrorState    = 1;
    c->cb.all          = pktHandler;
}

static void is_comm_enable_protocol(is_comm_instance_t* instance, protocol_type_t ptype)
{
    if (instance) {
        instance->cb.protocolMask |= (0x01u << ptype);
    }
}

static pfnIsCommIsbDataHandler is_comm_register_isb_handler(is_comm_instance_t* comm,
                                                              pfnIsCommIsbDataHandler cbHandler)
{
    if (!comm) {
        return nullptr;
    }
    pfnIsCommIsbDataHandler priorCb = comm->cb.isbData;
    comm->cb.isbData = cbHandler;
    comm->cb.protocolMask |= ENABLE_PROTOCOL_ISB;
    return priorCb;
}

// ===== ISComm.c: buffer management =====

static void move_buffer_32bit(void* dest, void* src, size_t size)
{
    uint32_t* dest32 = (uint32_t*)dest;
    uint32_t* src32  = (uint32_t*)src;
    size_t size32    = size >> 2;
    size_t remaining = size & 0x3;

    for (size_t i = 0; i < size32; i++) {
        dest32[i] = src32[i];
    }
    uint8_t* dest8 = (uint8_t*)(dest32 + size32);
    uint8_t* src8  = (uint8_t*)(src32  + size32);
    for (size_t i = 0; i < remaining; i++) {
        dest8[i] = src8[i];
    }
}

static int is_comm_free(is_comm_instance_t* c)
{
    is_comm_buffer_t *buf = &(c->rxBuf);
    int bytesFree = (int)(buf->end - buf->tail);

    if (bytesFree < (int)(buf->size)) {
        if (c->processPkt != nullptr) {
            if (buf->head != buf->start) {
                int shift = (int)(buf->head - buf->start);
                move_buffer_32bit(buf->start, buf->head,
                                  (size_t)(buf->tail - buf->head));
                buf->head = buf->start;
                buf->tail -= shift;
                buf->scan -= shift;
                bytesFree = (int)(buf->end - buf->tail);
            } else if (bytesFree == 0) {
                reportParseError(c, EPARSE_RXBUFFER_FLUSHED);
                return is_comm_reset_buffer(c);
            }
        } else {
            if (buf->scan >= buf->tail) {
                return is_comm_reset_buffer(c);
            }
        }
    }
    return bytesFree;
}

// ===== ISComm.c: packet encode and write to buffer =====

static void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did,
                                 uint16_t data_size, uint16_t offset, const void* data)
{
    pkt->hdr.preamble    = PSC_ISB_PREAMBLE;
    pkt->hdr.flags       = flags;
    pkt->hdr.id          = (uint8_t)did;
    pkt->hdr.payloadSize = data_size;
    pkt->offset          = offset;
    if (offset) {
        pkt->hdr.flags       |= ISB_FLAGS_PAYLOAD_W_OFFSET;
        pkt->hdr.payloadSize  = (uint16_t)(data_size + 2);
    }
    pkt->data.ptr  = (uint8_t*)data;
    pkt->data.size = data_size;
    pkt->size      = (uint16_t)(pkt->hdr.payloadSize + sizeof(packet_hdr_t) + 2);
    pkt->hdrCksum  = is_comm_isb_checksum16(0, &pkt->hdr, sizeof(pkt->hdr));
}

#define MEMCPY_INC(dst, src, size) memcpy((dst), (src), (size)); (dst) += (size);

static void memcpyIncUpdateChecksum(uint8_t **dstBuf, const uint8_t* srcBuf,
                                     int len, uint16_t *checksum)
{
    *checksum = is_comm_isb_checksum16(*checksum, srcBuf, (uint32_t)len);
    MEMCPY_INC(*dstBuf, srcBuf, (size_t)len);
}

static int is_comm_write_isb_precomp_to_buffer(uint8_t *buf, uint32_t buf_size,
                                                 is_comm_instance_t* comm, packet_t *pkt)
{
    if (pkt->size > buf_size) {
        return -1;
    }
    pkt->checksum = pkt->hdrCksum;
    MEMCPY_INC(buf, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));
    if (pkt->offset) {
        memcpyIncUpdateChecksum(&buf, (uint8_t*)&(pkt->offset), 2, &(pkt->checksum));
    }
    if (pkt->data.size) {
        memcpyIncUpdateChecksum(&buf, (uint8_t*)pkt->data.ptr,
                                (int)pkt->data.size, &(pkt->checksum));
    }
    MEMCPY_INC(buf, (uint8_t*)&(pkt->checksum), 2);
    comm->txPktCount++;
    return (int)pkt->size;
}

static int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size,
                                  is_comm_instance_t* comm, uint8_t flags,
                                  uint16_t did, uint16_t data_size, uint16_t offset,
                                  const void* data)
{
    packet_t txPkt;
    is_comm_encode_hdr(&txPkt, flags, did, data_size, offset, data);
    return is_comm_write_isb_precomp_to_buffer(buf, buf_size, comm, &txPkt);
}

static int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size,
                                    is_comm_instance_t* comm, uint32_t did,
                                    uint32_t size, uint32_t offset,
                                    uint32_t periodMultiple)
{
    p_data_get_t get;
    get.id     = (uint16_t)did;
    get.offset = (uint16_t)offset;
    get.size   = (uint16_t)size;
    get.period = (uint16_t)periodMultiple;
    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_GET_DATA, 0,
                                 sizeof(p_data_get_t), 0, &get);
}

// ===== ISComm.c: byte-level and buffer-level parsing =====

static protocol_type_t is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs);

static protocol_type_t is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs)
{
    is_comm_buffer_t *buf = &(c->rxBuf);

#if PKT_PARSER_TIMEOUT_MS
    if (c->processPkt) {
        if (timeMs > c->parser.timeMs + PKT_PARSER_TIMEOUT_MS) {
            c->rxBuf.head++;
            is_comm_reset_parser(c);
        }
    }
#endif

    while (buf->scan < buf->tail) {
        if (c->processPkt == nullptr) {
            if (*(buf->scan) == PSC_ISB_PREAMBLE_BYTE1) {
                if (c->cb.protocolMask & ENABLE_PROTOCOL_ISB) {
                    setParserStart(c, processIsbPkt);
                }
            } else {
                if (reportParseError(c, EPARSE_STREAM_UNPARSABLE)) {
                    return _PTYPE_PARSE_ERROR;
                }
            }
        } else {
            protocol_type_t ptype = c->processPkt(c);
            if (ptype != _PTYPE_NONE) {
                buf->scan++;
                return ptype;
            }
        }
        buf->scan++;
    }

#if PKT_PARSER_TIMEOUT_MS
    if (c->processPkt) {
        c->parser.timeMs = timeMs;
    }
#endif

    return _PTYPE_NONE;
}

static inline protocol_type_t is_comm_parse(is_comm_instance_t* instance)
{
    return is_comm_parse_timeout(instance, 0);
}

// ===== ISComm.c: message dispatch =====

static void parse_messages(is_comm_instance_t* comm, port_handle_t port)
{
    if (!comm) {
        return;
    }
    protocol_type_t ptype;
    while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE) {
        int notConsumed = -1;
        switch (ptype) {
        case _PTYPE_INERTIAL_SENSE_DATA:
            if (comm->cb.isbData) {
                p_data_t data;
                is_comm_to_isb_p_data(comm, &data);
                notConsumed = comm->cb.isbData(comm->cb.context, &data, port);
            }
            break;
        case _PTYPE_INERTIAL_SENSE_ACK:
        case _PTYPE_INERTIAL_SENSE_CMD:
            break;
        default:
            if (comm->cb.generic[ptype]) {
                notConsumed = comm->cb.generic[ptype](
                    comm->cb.context,
                    comm->rxPkt.data.ptr + comm->rxPkt.offset,
                    (int)comm->rxPkt.data.size,
                    port);
            }
            break;
        }
        if (comm->cb.all && notConsumed) {
            comm->cb.all(comm->cb.context, ptype, &(comm->rxPkt), port);
        }
    }
}

static void is_comm_buffer_parse_messages(uint8_t *buf, uint32_t buf_size,
                                           is_comm_instance_t* comm)
{
    int n = (int)_MIN((int)buf_size, is_comm_free(comm));
    memcpy(comm->rxBuf.tail, buf, (size_t)n);
    comm->rxBuf.tail += n;
    parse_messages(comm, nullptr);
}

// ===== End of inlined Inertial Sense SDK =====

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_InertialSense *AP_ExternalAHRS_InertialSense::instance = nullptr;

AP_ExternalAHRS_InertialSense::AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state)
{
    printf("Inertial Sense ExternalAHRS created\r\n");
    hal.scheduler->delay(1000);

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = 921600; // sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    printf("Inertial Sense ExternalAHRS created: baudrate [%" PRIu32 "], port_num [%d]\n\n", baudrate, port_num);

    if (!uart) {
        printf("Inertial Sense ExternalAHRS no UART\r\n");
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Inertial Sense ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialSense::update_thread, void), "AHRS", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Inertial Sense failed to allocate ExternalAHRS update thread");
    }

    // don't offer IMU by default, the processing can take the main loop below minimum rate
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    hal.scheduler->delay(1000);
}

int8_t AP_ExternalAHRS_InertialSense::get_port(void) const {
    if (!uart) {
        return -1;
    }
    return port_num;
}

const char* AP_ExternalAHRS_InertialSense::get_name() const {
    return "Inertial Sense";
}

bool AP_ExternalAHRS_InertialSense::healthy(void) const {
    uint32_t now = AP_HAL::millis();
    return _healthy && now - last_gps_pkt < 500 && now - last_filter_pkt < 100;
}

bool AP_ExternalAHRS_InertialSense::initialised(void) const {
    uint32_t now = AP_HAL::millis();
    return initialized && now - last_gps_pkt < 500 && now - last_filter_pkt < 100;
}

bool AP_ExternalAHRS_InertialSense::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Inertial Sense unhealthy\n");
        return false;
    }

    if(_fix_type < AP_GPS_FixType::FIX_3D) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Inertial Sense no GPS lock");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_InertialSense::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (initialised())
    {
        status.flags.initalized = true;
    }

    if (healthy())
    {
        status.flags.attitude = true;
        status.flags.vert_vel = true;
        status.flags.vert_pos = true;

        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
}

bool AP_ExternalAHRS_InertialSense::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
    velVar = vel_cov * vel_gate_scale;
    posVar = pos_cov * pos_gate_scale;
    hgtVar = hgt_cov * hgt_gate_scale;
    tasVar = 0;
    return true;
}

int AP_ExternalAHRS_InertialSense::stop_message_broadcasting()
{
    printf("Inertial Sense ExternalAHRS stop_message_broadcasting\r\n");

    // Stop all broadcasts on the device
    // int ret = is_comm_stop_broadcasts_all_ports(port);
    int size = is_comm_write_to_buf(buffer, sizeof(buffer), &comm, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, nullptr);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write stop broadcasts message\r\n");
        return -3;
    }

    // ret = is_comm_stop_broadcasts_current_port(port);
    size = is_comm_write_to_buf(buffer, sizeof(buffer), &comm, PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT, 0, 0, 0, nullptr);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write stop broadcasts message\r\n");
        return -3;
    }

    return 0;
}

int AP_ExternalAHRS_InertialSense::enable_message_broadcasting()
{
    printf("Inertial Sense ExternalAHRS enable_message_broadcasting\r\n");

    int size;

    // Ask for INS message w/ update 8ms period (4ms source period x 2)
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_INS_3, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get INS message\r\n");
        return -4;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_POS, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get GPS POS message\r\n");
        return -5;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_VEL, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get GPS VEL message\r\n");
        return -5;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_RTK_POS_MISC, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get RTK POS MISC message\r\n");
        return -5;
    }

    // Don't offer IMU by default, as it may drop the main loop below minimum rate
    // // Ask for IMU message at period of 20ms (1ms source period x 20).
    // size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_PIMU, 0, 0, imu_sample_duration);
    // if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
    // {
    //     printf("Failed to encode and write get PIMU message\r\n");
    //     return -6;
    // }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_MAGNETOMETER, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get MAG message\r\n");
        return -6;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_BAROMETER, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get BARO message\r\n");
        return -6;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_INL2_NED_SIGMA, 0, 0, 1);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get NL2_NED_SIGMA message\r\n");
        return -6;
    }

    // request a device info message
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_DEV_INFO, 0, 0, 0);
    if(uart->write(buffer, (uint16_t)size) != (size_t)size) {
        printf("Failed to encode and write get BARO message\r\n");
        return -6;
    }

    return 0;
}

int AP_ExternalAHRS_InertialSense::initialize()
{
    printf("Inertial Sense ExternalAHRS initialize\r\n");

    if (uart == nullptr) {
        return -1;
    }

    uart->begin(baudrate);

    is_comm_init(&comm, comm_buf, sizeof(comm_buf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

    instance = this;
    is_comm_register_isb_handler(&comm, &AP_ExternalAHRS_InertialSense::isbDataHandler);

    int error = 0;

    if ((error = stop_message_broadcasting()))
    {
        printf("Error: stop_message_broadcasting()\r\n");
        return error;
    }
    hal.scheduler->delay(500);

    if ((error = enable_message_broadcasting()))
    {
        printf("Error: enable_message_broadcasting\r\n");
        return error;
    }

    initialized = true;

    return 0;
}

void AP_ExternalAHRS_InertialSense::handleIns3Message(ins_3_t* ins)
{
    last_filter_pkt = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    Quaternion q(ins->qn2b[0], ins->qn2b[1], ins->qn2b[2], ins->qn2b[3]);
    state.quat = q;
    state.have_quaternion = true;

    Vector3f uvw(ins->uvw[0], ins->uvw[1], ins->uvw[2]);
    state.velocity = q * uvw;
    state.have_velocity = true;

    state.location = Location{
        (int32_t)(ins->lla[0] * 1e7),
        (int32_t)(ins->lla[1] * 1e7),
        (int32_t)(ins->msl * 100),
        Location::AltFrame::ABSOLUTE};
    state.have_location = true;
    state.last_location_update_us = AP_HAL::micros();

    switch((eGpsNavFixStatus)INS_STATUS_NAV_FIX_STATUS(ins->insStatus)) {
    case eGpsNavFixStatus::GPS_NAV_FIX_NONE:
        _fix_type = AP_GPS_FixType::NONE;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_3D:
        _fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FLOAT:
        _fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FIX:
        _fix_type = AP_GPS_FixType::RTK_FIXED;
        break;

    default:
        _fix_type = AP_GPS_FixType::NO_GPS;
    }

    if (_fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        state.origin = state.location;
        state.have_origin = true;
    }

    const uint32_t hdwStatus = ins->hdwStatus;
    const bool hardware_healthy = (hdwStatus & HDW_STATUS_ERROR_MASK) == HDW_STATUS_BIT_PASSED;

    WITH_SEMAPHORE(sem);
    const bool was_healthy = _healthy;
    _healthy = hardware_healthy;

    // Only print diagnostics on the transition to unhealthy to avoid a printf
    // storm at the INS_3 message rate while semaphores are held.
    if (!hardware_healthy && was_healthy)
    {
        const uint32_t bit_state = hdwStatus & HDW_STATUS_BIT_MASK;

        printf("InertialSense: hardware status unhealthy (0x%08" PRIX32 ")\n", hdwStatus);

        // BIT state
        if (bit_state == HDW_STATUS_BIT_RUNNING)
            printf("  BIT: built-in test still running\n");
        else if (bit_state == HDW_STATUS_BIT_FAILED)
            printf("  BIT: built-in test failed\n");
        else if (bit_state != HDW_STATUS_BIT_PASSED)
            printf("  BIT: built-in test not started (state=0x%" PRIX32 ")\n", bit_state);

        // Individual error bits covered by HDW_STATUS_ERROR_MASK
        if (hdwStatus & HDW_STATUS_FAULT_SYS_CRITICAL)     printf("  critical system fault (CPU error)\n");
        if (hdwStatus & HDW_STATUS_IMU_FAULT_REJECT_GYR)   printf("  redundant gyro divergent and excluded\n");
        if (hdwStatus & HDW_STATUS_IMU_FAULT_REJECT_ACC)   printf("  redundant accelerometer divergent and excluded\n");
        if (hdwStatus & HDW_STATUS_SATURATION_GYR)         printf("  gyro saturation detected\n");
        if (hdwStatus & HDW_STATUS_SATURATION_ACC)         printf("  accelerometer saturation detected\n");
        if (hdwStatus & HDW_STATUS_SATURATION_MAG)         printf("  magnetometer saturation detected\n");
        if (hdwStatus & HDW_STATUS_SATURATION_BARO)        printf("  barometer saturation detected\n");
        if (hdwStatus & HDW_STATUS_ERR_GPS_PPS_NOISE)      printf("  GPS PPS signal noise\n");
        if (hdwStatus & HDW_STATUS_ERR_COM_TX_LIMITED)     printf("  communications TX buffer limited\n");
        if (hdwStatus & HDW_STATUS_ERR_COM_RX_OVERRUN)     printf("  communications RX buffer overrun\n");
        if (hdwStatus & HDW_STATUS_ERR_NO_GPS_PPS)         printf("  no GPS PPS signal received\n");
        if (hdwStatus & HDW_STATUS_ERR_TEMPERATURE)        printf("  temperature outside specified range\n");
    }
}

void AP_ExternalAHRS_InertialSense::handleGpsPosMessage(gps_pos_t* pos)
{
    last_gps_pkt = AP_HAL::millis();

    AP_GPS_FixType fix_type = AP_GPS_FixType::NONE;

    int fix = pos->status & GPS_STATUS_FIX_MASK;
    switch(fix) {
    case GPS_STATUS_FIX_2D:
        fix_type = AP_GPS_FixType::FIX_2D;
        break;

    case GPS_STATUS_FIX_3D:
        fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case GPS_STATUS_FIX_DGPS:
        fix_type = AP_GPS_FixType::DGPS;
        break;

    case GPS_STATUS_FIX_RTK_FLOAT:
        fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case GPS_STATUS_FIX_RTK_FIX:
        fix_type = AP_GPS_FixType::RTK_FIXED;
        break;
    }

    int32_t latitude = (int32_t)(pos->lla[0] * 1e7);
    int32_t longitude = (int32_t)(pos->lla[1] * 1e7);
    int32_t msl_altitude = (int32_t)(pos->hMSL * 100);

    gps_data_msg.gps_week = (uint16_t)pos->week;
    gps_data_msg.ms_tow = pos->timeOfWeekMs;
    gps_data_msg.fix_type = fix_type;
    gps_data_msg.satellites_in_view = pos->satsUsed;
    gps_data_msg.horizontal_pos_accuracy = pos->hAcc;
    gps_data_msg.vertical_pos_accuracy = pos->vAcc;
    gps_data_msg.longitude = longitude;
    gps_data_msg.latitude = latitude;
    gps_data_msg.msl_altitude = msl_altitude;

    gps_data_msg.hdop = pos->hAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handleGpsVelMessage(gps_vel_t* vel)
{
    gps_data_msg.horizontal_vel_accuracy = vel->sAcc;
    gps_data_msg.ned_vel_north = vel->vel[0];
    gps_data_msg.ned_vel_east = vel->vel[1];
    gps_data_msg.ned_vel_down = vel->vel[2];

    if (gps_data_msg.fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{
            gps_data_msg.latitude,
            gps_data_msg.longitude,
            gps_data_msg.msl_altitude,
            Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    uint8_t gps_instance;
    if (AP::gps().get_first_external_instance(gps_instance)) {
        AP::gps().handle_external(gps_data_msg, gps_instance);
    }

    gps_data_msg.vdop = vel->sAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handleGpsRtkPosMiscMessage(gps_rtk_misc_t* misc)
{
    gps_data_msg.hdop = misc->hDop * 100;
    gps_data_msg.vdop = misc->vDop * 100;
}

void AP_ExternalAHRS_InertialSense::handlePimuMessage(pimu_t* pimu)
{
    last_imu_pkt = AP_HAL::millis();

    Vector3f accel;
    accel[0] = pimu->vel[0] / (pimu->dt * imu_sample_duration);
    accel[1] = pimu->vel[1] / (pimu->dt * imu_sample_duration);
    accel[2] = pimu->vel[2] / (pimu->dt * imu_sample_duration);

    Vector3f gyro;
    gyro[0] = pimu->theta[0] / (pimu->dt * imu_sample_duration);
    gyro[1] = pimu->theta[1] / (pimu->dt * imu_sample_duration);
    gyro[2] = pimu->theta[2] / (pimu->dt * imu_sample_duration);

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro = gyro;
    }

    AP_ExternalAHRS::ins_data_message_t ins {
        accel: accel,
        gyro: gyro,
        temperature: -300
    };
    AP::ins().handle_external(ins);
}

void AP_ExternalAHRS_InertialSense::handleMagnetometerMessage(magnetometer_t* _mag)
{
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f{_mag->mag[0], _mag->mag[1], _mag->mag[2]};

    // mag values have been normalized to 1, so we scale to something that can be calibrated

    // For SN510457, multiply DID_MAGNETOMETER by 0.4381111936 to scale back to the sensor raw output.
    mag.field *= 0.4381111936 * 1000;

    AP::compass().handle_external(mag);
#endif
}

void AP_ExternalAHRS_InertialSense::handleBarometerMessage(barometer_t* bar)
{
#if AP_BARO_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = bar->bar * 1e3;
    baro.temperature = bar->barTemp;

    AP::baro().handle_external(baro);
#endif
}

void AP_ExternalAHRS_InertialSense::handleInl2NedSigmaMessage(inl2_ned_sigma_t *sigmas)
{
    float pos_std = Vector3f(sigmas->StdPosNed[0], sigmas->StdPosNed[1], sigmas->StdPosNed[2]).length();
    float vel_std = Vector3f(sigmas->StdVelNed[0], sigmas->StdVelNed[1], sigmas->StdVelNed[2]).length();

    pos_cov = pos_std * pos_std;
    vel_cov = vel_std * vel_std;
    hgt_cov = pos_std * pos_std;
}

void AP_ExternalAHRS_InertialSense::handleDevInfoMessage(dev_info_t *dev_info)
{
    devInfoPopulateMissingHardware(dev_info);

    const char *hardware_type;
    switch(dev_info->hardwareType) {
    case 1:
        hardware_type = "uINS";
        break;

    case 2:
        hardware_type = "EVB";
        break;

    case 3:
        hardware_type = "IMX";
        break;

    case 4:
        hardware_type = "GPX";
        break;

    default:
        hardware_type = "unknown";
    }

    printf("Inertial Sense:\n");
    printf("    manufacturer:       %s\n", dev_info->manufacturer);
    printf("    hardware type:      %s\n", hardware_type);
    printf("    hardware version:   %d.%d.%d.%d\n", dev_info->hardwareVer[0], dev_info->hardwareVer[1], dev_info->hardwareVer[2], dev_info->hardwareVer[3]);
    printf("    firmware version:   %d.%d.%d.%d\n", dev_info->firmwareVer[0], dev_info->firmwareVer[1], dev_info->firmwareVer[2], dev_info->firmwareVer[3]);
    printf("    serial number:      %" PRIu32 "\n", dev_info->serialNumber);
}

void AP_ExternalAHRS_InertialSense::handleBitMessage(bit_t* bit)
{
    if(bit->state != BIT_STATE_DONE)
        return;

    if(bit->hdwBitStatus & HDW_BIT_FAILED_MASK) {
        printf("Inertial Sense hardware built-in-test failed!\n");
        return;
    }

    if(bit->calBitStatus & CAL_BIT_FAILED_MASK) {
        printf("Inertial Sense calibration built-in-test failed!\n");
        return;
    }

    WITH_SEMAPHORE(sem);
    _healthy = true;
}

void AP_ExternalAHRS_InertialSense::update() {
    if (!check_uart()) {
        hal.scheduler->delay_microseconds(100);
    }
}

int AP_ExternalAHRS_InertialSense::parseIsbData(void* ctx, p_data_t* data, port_handle_t port) {
    switch (data->hdr.id)
    {
    case DID_INS_3:
        handleIns3Message((ins_3_t*)data->ptr);
        break;

    case DID_GPS1_POS:
        handleGpsPosMessage((gps_pos_t*)data->ptr);
        break;

    case DID_GPS1_VEL:
        handleGpsVelMessage((gps_vel_t*)data->ptr);
        break;

    case DID_MAGNETOMETER:
        handleMagnetometerMessage((magnetometer_t *)data->ptr);
        break;

    case DID_INL2_NED_SIGMA:
        handleInl2NedSigmaMessage((inl2_ned_sigma_t *)data->ptr);
        break;

    case DID_DEV_INFO:
        handleDevInfoMessage((dev_info_t*)data->ptr);
        break;

    case DID_BIT:
        handleBitMessage((bit_t*)data->ptr);
        break;

    default:
        break;
    }

    return 0;
}

bool AP_ExternalAHRS_InertialSense::check_uart() {
    if(!initialized) {
        printf("UART not initialized!\n");
        return false;
    }

    WITH_SEMAPHORE(sem);

    if(!uart->available())
        return false;

    auto len = uart->read(buffer, MIN(uart->available(), 1024u));
    is_comm_buffer_parse_messages(buffer, len, &comm);

    return true;
}

void AP_ExternalAHRS_InertialSense::update_thread() {
    if(!initialized) {
        initialize();
    }

    while(true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

uint8_t AP_ExternalAHRS_InertialSense::num_gps_sensors(void) const {
    // TODO: query actual number of GPS units on module
    return 1;
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
