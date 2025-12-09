/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "ISComm.h"
#include "ISConstants.h"

#define MAX_MSG_LENGTH_ISB          PKT_BUF_SIZE
#define MAX_MSG_LENGTH_NMEA         200
#define MAX_MSG_LENGTH_RTCM         1023  // RTCM3 standard
#define MAX_MSG_LENGTH_UBX          1024
#define MAX_MSG_LENGTH_SONY         4090
#define PKT_PARSER_TIMEOUT_MS       100   // Set to 0 to disable timeout

typedef union 
{
    uint16_t ck;
    struct
    {
        uint8_t a; // Lower 8 bits
        uint8_t b; // Upper 8 bits
    };
} checksum16_u;

const unsigned int g_validBaudRates[IS_BAUDRATE_COUNT] = {
                          // Actual on IMX-5:
    IS_BAUDRATE_10000000, // 10000000
    IS_BAUDRATE_921600,   //   930233 (default baudrate)
    IS_BAUDRATE_460800,   //   462428
    IS_BAUDRATE_230400,   //   230547
    IS_BAUDRATE_115200,
    IS_BAUDRATE_57600,
    IS_BAUDRATE_38400,
    IS_BAUDRATE_19200,
    IS_BAUDRATE_9600 
};

/**
* Calculate 24 bit crc used in formats like RTCM3 - note that no bounds checking is done on buffer
* @param buffer the buffer to calculate the CRC for
* @param len the number of bytes to calculate the CRC for
* @return the CRC value
*/
unsigned int calculate24BitCRCQ(const unsigned char* buffer, unsigned int len)
{
    static const unsigned int TABLE_CRC24Q[] =
    {
        0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
        0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
        0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
        0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
        0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
        0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
        0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
        0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
        0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
        0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
        0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
        0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
        0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
        0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
        0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
        0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
        0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
        0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
        0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
        0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
        0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
        0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
        0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
        0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
        0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
        0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
        0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
        0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
        0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
        0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
        0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
        0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
    };

    unsigned int crc = 0;
    for (uint32_t i = 0; i != len; i++)
    {
        crc = ((crc << 8) & 0xFFFFFF) ^ TABLE_CRC24Q[(crc >> 16) ^ buffer[i]];
    }
    return crc;
}

uint16_t is_comm_fletcher16(uint16_t cksum_init, const void* data, uint32_t size)
{
    checksum16_u cksum;
    cksum.ck = cksum_init;
    for (uint32_t i=0; i<size; i++)
    {
        cksum.a += ((uint8_t*)data)[i];
        cksum.b += cksum.a;
    }
    return cksum.ck;
}

uint16_t is_comm_xor16(uint16_t cksum_init, const void* data, uint32_t size)
{    
    checksum16_u cksum;
    cksum.ck = cksum_init;
    for (uint32_t i=0; i<size; i++)
    {
        cksum.a ^= ((uint8_t*)data)[i];
        cksum.b ^= cksum.a;
    }
    return cksum.ck;
}

/**
* Retrieve the 32 bit unsigned integer value of the specified bits - note that no bounds checking is done on buffer
* @param buffer the buffer containing the bits
* @param pos the start bit position in buffer to read at
* @param len the number of bits to read
* @return the 32 bit unsigned integer value
*/
unsigned int getBitsAsUInt32(const unsigned char* buffer, unsigned int pos, unsigned int len)
{
    unsigned int bits = 0;
    for (unsigned int i = pos; i < pos + len; i++)
    {
        bits = (bits << 1) + ((buffer[i / 8] >> (7 - i % 8)) & 1U);
    }
    return bits;
}

int validateBaudRate(unsigned int baudRate)
{
#if 1   // TODO: Remove after debug.  Used to debug possible cause of GPX no Rx comms.

    // Allow arbitrary baud rates within acceptable range
    if (baudRate >= IS_BAUDRATE_STANDARD_MIN && baudRate <= IS_BAUDRATE_MAX)
    {   // Valid baud rate
        return 0;
    }
    
#else

    if (baudRate <= IS_BAUDRATE_STANDARD_MAX)
    {   // Valid baudrates for InertialSense hardware
        for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(g_validBaudRates); i++)
        {
            if (g_validBaudRates[i] == baudRate)
            {
                return 0;
            }
        }
    }
    else if (baudRate <= IS_BAUDRATE_MAX)
    {   // High speed custom baud rates
        return 0;
    }

#endif

    // Invalid baud rate
    return -1;    
}

/**
 * @brief Sets buffer to initial state
 * 
 * @param c is_comm_instance_t*
 * @return returns the size of the buffer
 */
int is_comm_reset_buffer(is_comm_instance_t* c)
{
    c->parser.state = 0;
    c->rxBuf.head = c->rxBuf.tail = c->rxBuf.scan = c->rxBuf.scanPrior = c->rxBuf.start;
    c->processPkt = NULL;

    return c->rxBuf.size;
}

void is_comm_init(is_comm_instance_t* c, uint8_t *buffer, int bufferSize, pfnIsCommHandler pktHandler)
{
    memset(c, 0, sizeof(is_comm_instance_t));

    // Clear buffer and initialize buffer pointers
    memset(buffer, 0, bufferSize);
    
    c->rxBuf.size = bufferSize;
    c->rxBuf.start = buffer;
    c->rxBuf.end = buffer + bufferSize;

    is_comm_reset_buffer(c);
    
    // Set parse enable flags
    c->cb.protocolMask = DEFAULT_PROTO_MASK;
    
    c->rxPkt.data.ptr = c->rxBuf.start;
    c->rxErrorState = 1;

    c->cb.all = pktHandler;
}

void is_comm_port_init(comm_port_t* port, pfnIsCommHandler pktHandler) {
    if (port && (portType((port_handle_t)port) & PORT_TYPE__COMM)) {
        is_comm_init(&port->comm, port->buffer, sizeof(port->buffer), pktHandler);
    }
}

is_comm_instance_t* is_comm_get_port_instance(port_handle_t port) {
    if (port && (portType((port_handle_t)port) & PORT_TYPE__COMM)) {
        return &COMM_PORT(port)->comm;
    }
    return NULL;
}

/**
 * Registering ISB handler for given comm instance
 * 
 * @return handler on success
 * @return NULL if port invalid 
 */
pfnIsCommIsbDataHandler is_comm_register_isb_handler(is_comm_instance_t* comm, pfnIsCommIsbDataHandler cbHandler) {
    if (!comm)
        return NULL;

    pfnIsCommIsbDataHandler priorCb = comm->cb.isbData;
    comm->cb.isbData = cbHandler;
    comm->cb.protocolMask |= ENABLE_PROTOCOL_ISB;
    return priorCb;
}

/**
 * Registering ISB handler for given port
 * 
 * @return handler on success
 * @return NULL if port invalid 
 */
pfnIsCommIsbDataHandler is_comm_register_port_isb_handler(port_handle_t port, pfnIsCommIsbDataHandler cbHandler)
{
    if (port)
        return is_comm_register_isb_handler(&COMM_PORT(port)->comm,  cbHandler);

    return NULL;
}

/**
 * Registers msg handler for specific comm instance and protocol type
 * 
 * @return handler on success
 * @return NULL if port invalid 
 */
pfnIsCommGenMsgHandler is_comm_register_msg_handler(is_comm_instance_t* comm, int ptype, pfnIsCommGenMsgHandler cbHandler) {
    if (!comm || (ptype < _PTYPE_FIRST_DATA) || (ptype > _PTYPE_LAST_DATA))
        return NULL;

    pfnIsCommGenMsgHandler priorCb = comm->cb.generic[ptype];
    comm->cb.generic[ptype] = cbHandler;
    comm->cb.protocolMask |= (int)(0x01) << ptype;
    return priorCb;
}

/**
 * Registers msg handler for specific port and protocol type
 * 
 * @return handler on success
 * @return NULL if port invalid 
 */
pfnIsCommGenMsgHandler is_comm_register_port_msg_handler(port_handle_t port, int ptype, pfnIsCommGenMsgHandler cbHandler) {
    if (port)
        return is_comm_register_msg_handler(&COMM_PORT(port)->comm, ptype, cbHandler);

    return NULL;
}

void is_comm_register_callbacks(is_comm_instance_t* c, is_comm_callbacks_t *callbacks) {
    if (callbacks)
        c->cb = *callbacks;
}

void is_comm_register_port_callbacks(port_handle_t port, is_comm_callbacks_t *callbacks) {
    if (port)
        is_comm_register_callbacks(&COMM_PORT(port)->comm, callbacks);
}

void is_comm_enable_protocol(is_comm_instance_t* instance, protocol_type_t ptype) {
    if (instance)
        instance->cb.protocolMask |= (0x01 << ptype);
}

void is_comm_disable_protocol(is_comm_instance_t* instance, protocol_type_t ptype) {
    if (instance)
        instance->cb.protocolMask &= ~(0x01 << ptype);
}

void is_comm_set_protocol_mask(is_comm_instance_t* instance, uint32_t protocolMask) {
    if (instance)
        instance->cb.protocolMask = protocolMask;
}

uint32_t is_comm_get_protocol_mask(is_comm_instance_t* instance) {
    uint32_t protocols = 0;
    if (instance->cb.protocolMask)
        return instance->cb.protocolMask;

    if (instance->cb.isbData) protocols |= _PTYPE_INERTIAL_SENSE_DATA;
    for (int i = _PTYPE_FIRST_DATA ; i < _PTYPE_LAST_DATA; i++) {
        if (instance->cb.generic[i]) protocols |= (0x1 << i);
    }

    return protocols;
}

void setParserStart(is_comm_instance_t* c, pFnProcessPkt processPkt)
{
    is_comm_parser_t *p = &(c->parser);
    p->state = 1;
    c->rxBuf.head = c->rxBuf.scan;
    c->processPkt = processPkt;
}

static inline protocol_type_t reportParseError(is_comm_instance_t* c, eParseErrorType errorType)
{
    if (!c->rxErrorState)
    {
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
    c->rxBuf.head = c->rxBuf.head + pktSize;
    c->rxPktCount++;
    c->processPkt = NULL;
    c->rxErrorState = 0;
}

static inline void validPacketFound(is_comm_instance_t* c, int pktSize, int dataSize, uint16_t id)
{
    // Update data pointer and info
    c->rxPkt.size      = pktSize;
    c->rxPkt.data.ptr  = c->rxBuf.head;
    c->rxPkt.data.size = dataSize;
    c->rxPkt.id        = id;
    c->rxPkt.hdr.id    = 0;
    c->rxPkt.offset    = 0;

    validPacketReset(c, pktSize);
}

static protocol_type_t processIsbPkt(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;

    switch (p->state)
    {
        case 0:
            if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE1)
            {
                p->state++;
            }
            return _PTYPE_NONE;

        case 1:
            if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE2)
            {   // Found complete preamble
                p->state++;
                return _PTYPE_NONE;
            }
            // Invalid preamble - Reset state
            return parseErrorResetState(c, EPARSE_INVALID_PREAMBLE);

        case 2:        // Wait for packet header
            numBytes = (int)(c->rxBuf.scan - c->rxBuf.head);
            if (numBytes < (int)(sizeof(packet_hdr_t)-1))
            {
                return _PTYPE_NONE;
            }
            p->state++;

            // Parse header
            packet_buf_t *isbPkt = (packet_buf_t*)(c->rxBuf.head);
            p->size = sizeof(packet_hdr_t) + isbPkt->hdr.payloadSize + 2;        // Header + payload + footer (checksum)
            if (p->size > MAX_MSG_LENGTH_ISB)
            {   // Invalid size
                return parseErrorResetState(c, EPARSE_INVALID_SIZE);
            }
            return _PTYPE_NONE;

        default:    // Wait for entire packet
            numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
            if (numBytes < (int)(p->size))
            {
                return _PTYPE_NONE;
            }
            // Found packet end
            break;
    }

    // Reset state
    p->state = 0;

    // Validate checksum
    packet_buf_t *isbPkt = (packet_buf_t*)(c->rxBuf.head);
    if (isbPkt->hdr.payloadSize > MAX_MSG_LENGTH_ISB)
        return parseErrorResetState(c, EPARSE_INVALID_SIZE);
    if ((isbPkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) && (isbPkt->payload.offset + isbPkt->hdr.payloadSize > MAX_MSG_LENGTH_ISB))
        return parseErrorResetState(c, EPARSE_INVALID_HEADER);

    uint16_t payloadSize = isbPkt->hdr.payloadSize;
    uint8_t *payload = c->rxBuf.head + sizeof(packet_hdr_t);
    checksum16_u *cksum = (checksum16_u*)(payload + payloadSize);
    int bytes_cksum = p->size - 2;
    uint16_t calcCksum = is_comm_isb_checksum16(0, c->rxBuf.head, bytes_cksum);
    if (cksum->ck != calcCksum)
    {   // Invalid checksum
        return parseErrorResetState(c, EPARSE_INVALID_CHKSUM);
    }

    /////////////////////////////////////////////////////////
    // Valid packet found - Checksum passed - Populate rxPkt
    validPacketReset(c, numBytes);

    packet_t *pkt = &(c->rxPkt);

    // Header
    pkt->hdr.preamble      = isbPkt->hdr.preamble;
    pkt->hdr.flags         = isbPkt->hdr.flags;
    pkt->id = pkt->hdr.id  = isbPkt->hdr.id;
    pkt->hdr.payloadSize   = payloadSize;

    // Payload
    if (pkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET)
    {   // Offset is first two bytes in payload
        pkt->data.size     = (payloadSize < 2 ? 0 : payloadSize - 2);
        pkt->data.ptr      = (pkt->data.size ? payload+2 : NULL);    // Data starts after offset if data size is non-zero
        pkt->offset        = *((uint16_t*)payload);
        pkt->dataHdr.size  = pkt->data.size;        // rxPkt.hdr.payloadSize and rxPkt.dataHdr.size share same memory.  Remove offset size from payload/data size.
    }
    else
    {   // No offset
        pkt->data.size     = payloadSize;
        pkt->data.ptr      = (payloadSize ? payload : NULL);
        pkt->offset        = 0;
    }

    // Footer
    pkt->checksum = cksum->ck;
    pkt->size = p->size;

    c->ackNeeded = 0;

    uint8_t ptype = pkt->hdr.flags & PKT_TYPE_MASK;
    switch (ptype)
    {
    case PKT_TYPE_SET_DATA:
    case PKT_TYPE_DATA:
        // Validate data size
        if (pkt->data.size <= MAX_DATASET_SIZE)
        {
            if (ptype==PKT_TYPE_SET_DATA)
            {   // acknowledge valid data received
                c->ackNeeded = PKT_TYPE_ACK;
            }

            return _PTYPE_INERTIAL_SENSE_DATA;
        }
        else
        {   // negative acknowledge data received
            c->ackNeeded = PKT_TYPE_NACK;
        }
        break;

    case PKT_TYPE_GET_DATA:
        {
            p_data_get_t *get = (p_data_get_t*)&(isbPkt->payload.data);
            // Validate data size
            if (get->size <= MAX_DATASET_SIZE)
            {   // Update data pointer
                return _PTYPE_INERTIAL_SENSE_CMD;
            }
        }
        break;

    case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
    case PKT_TYPE_STOP_DID_BROADCAST:
    case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
        return _PTYPE_INERTIAL_SENSE_CMD;
    case PKT_TYPE_ACK:
    case PKT_TYPE_NACK:
        return _PTYPE_INERTIAL_SENSE_ACK;
    }

    // Invalid data type
    return parseErrorResetState(c, EPARSE_INVALID_DATATYPE);
}

static protocol_type_t processNmeaPkt(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;

    switch (p->state)
    {
        case 0:    // Find start
            if (*(c->rxBuf.scan) == PSC_NMEA_START_BYTE)
            {   // Found
                p->state++;
            }
            return _PTYPE_NONE;

        case 1:    // Find byte before end
            if (*(c->rxBuf.scan) == PSC_NMEA_PRE_END_BYTE)
            {     // Found
                p->state++;
            }
            else
            {
                numBytes = (int)(c->rxBuf.scan - c->rxBuf.head);
                if (numBytes > MAX_MSG_LENGTH_NMEA)
                {   // Exceeds max length
                    return parseErrorResetState(c, EPARSE_INVALID_SIZE);
                }
            }
            return _PTYPE_NONE;

        case 3:        // Wait for end of packet
            if (*(c->rxBuf.scan) != PSC_NMEA_END_BYTE)
            {   // Invalid end
                return parseErrorResetState(c, EPARSE_MISSING_EOS_MARKER);
            }
            // Found packet end
            break;
    }

    // Reset state
    p->state = 0;

    // Validate length
    numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
    if (numBytes < 8)
    {   // Packet length too short
        return parseErrorResetState(c, EPARSE_INCOMPLETE_PACKET);
    }

    // Validate checksum
    uint8_t tmp = *(c->rxBuf.scan-1);    // Backup value
    *(c->rxBuf.scan-1) = 0;                // Null terminate hex string for strtol()
    int msgChecksum = (int)strtol((const char*)c->rxBuf.scan-3, NULL, 16);
    *(c->rxBuf.scan-1) = tmp;            // Restore value
    int calChecksum = 0;
    for (uint8_t* ptr = c->rxBuf.head + 1, *ptrEnd = c->rxBuf.scan - 4; ptr < ptrEnd; ptr++)
    {
        calChecksum ^= (int)*ptr;
    }
    if (msgChecksum != calChecksum)
    {   // Invalid checksum
        return parseErrorResetState(c, EPARSE_INVALID_CHKSUM);
    }

    /////////////////////////////////////////////////////////
    // Valid packet found - Checksum passed - Populate rxPkt
    validPacketFound(c, numBytes, numBytes, getNmeaMsgId(c->rxBuf.head, numBytes));

    return _PTYPE_NMEA;
}

enum
{
    UBX_PARSE_STATE_PREAMBLE    = 1,
    UBX_PARSE_STATE_CLASS_ID    = 2,
    UBX_PARSE_STATE_MSG_ID      = 3,
    UBX_PARSE_STATE_LENGTH_1    = 4,
    UBX_PARSE_STATE_LENGTH_2    = 5,
};

static protocol_type_t processUbloxPkt(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;

    switch (p->state)
    {
    case 0:
        if (*(c->rxBuf.scan) == UBLOX_START_BYTE1)
        {
            p->state++;
        }
        return _PTYPE_NONE;

    case 1:
        if (*(c->rxBuf.scan) == UBLOX_START_BYTE2)
        {   // Found complete preamble
            p->state++;
        }
        else
        {   // Invalid preamble - Reset state
            return parseErrorResetState(c, EPARSE_INVALID_PREAMBLE);
        }
        return _PTYPE_NONE;

    case 2:        // Wait for packet header
        if ((int)(c->rxBuf.scan - c->rxBuf.head) < (int)(sizeof(ubx_pkt_hdr_t)-1))
        {
            return _PTYPE_NONE;
        }

        // Parse header
        ubx_pkt_hdr_t *hdr = (ubx_pkt_hdr_t*)(c->rxBuf.head);
        p->size = sizeof(ubx_pkt_hdr_t) + hdr->payloadSize + 2;        // Header + payload + footer (checksum)
        p->state++;
        return _PTYPE_NONE;

    default:    // Wait for end of packet
        numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
        if (numBytes < p->size)
        {
            return _PTYPE_NONE;
        }
        // Found packet end
        break;
    }

    // Reset state
    p->state = 0;

    // Validate checksum
    uint16_t pktChecksum = *((uint16_t*)(c->rxBuf.scan - 1));
    uint8_t* cksum_start = c->rxBuf.head + 2;
    uint8_t* cksum_end   = c->rxBuf.scan - 1;
    uint32_t cksum_size  = (uint32_t)(cksum_end - cksum_start);
    checksum16_u cksum;
    cksum.ck = is_comm_fletcher16(0, cksum_start, cksum_size);
    if (pktChecksum != cksum.ck)
    {   // Invalid checksum
        return parseErrorResetState(c, EPARSE_INVALID_CHKSUM);
    }

    /////////////////////////////////////////////////////////
    // Valid packet found - Checksum passed - Populate rxPkt
    validPacketFound(c, numBytes, p->size, c->rxBuf.head[2]);

    return _PTYPE_UBLOX;
}

static protocol_type_t processRtcm3Pkt(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;

    switch (p->state)
    {
        case 0:        // Find start
            if (*(c->rxBuf.scan) == RTCM3_START_BYTE)
            {   // Found start
                p->state++;
            }
            return _PTYPE_NONE;

        case 1:        // Wait for packet header
            p->state++;
            return _PTYPE_NONE;

        case 2:
            p->size = (int)getBitsAsUInt32(c->rxBuf.head, 14, 10) + 6;        // Header + payload + footer (checksum)
            p->state++;

            // Validate packet length
            if (p->size > MAX_MSG_LENGTH_RTCM || p->size > c->rxBuf.size - 6)
            {   // Corrupt data
                return parseErrorResetState(c, EPARSE_INCOMPLETE_PACKET);
            }
            return _PTYPE_NONE;

        default:    // Wait for end of packet
            numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
            if (numBytes < p->size)
            {
                return _PTYPE_NONE;
            }

            // Found packet end
            break;
    }

    // Reset state
    p->state = 0;

    // Validate checksum - len without 3 crc bytes
    int lenWithoutCrc = (int)(p->size - 3);
    uint32_t actualCRC = calculate24BitCRCQ(c->rxBuf.head, lenWithoutCrc);
    uint32_t correctCRC = getBitsAsUInt32(c->rxBuf.head + lenWithoutCrc, 0, 24);

    if (actualCRC != correctCRC)
    {   // Invalid checksum
        return parseErrorResetState(c, EPARSE_INVALID_CHKSUM);
    }

    /////////////////////////////////////////////////////////
    // Valid packet found - Checksum passed - Populate rxPkt
    validPacketFound(c, numBytes, p->size, RTCM3_MSG_ID(c->rxBuf.head));

    return _PTYPE_RTCM3;
}

static const uint8_t u8CRC_4_TABLE[] = {
    0x00U, 0x0BU, 0x05U, 0x0EU, 0x0AU, 0x01U, 0x0FU, 0x04U,
    0x07U, 0x0CU, 0x02U, 0x09U, 0x0DU, 0x06U, 0x08U, 0x03U,
    0x0EU, 0x05U, 0x0BU, 0x00U, 0x04U, 0x0FU, 0x01U, 0x0AU,
    0x09U, 0x02U, 0x0CU, 0x07U, 0x03U, 0x08U, 0x06U, 0x0DU,
    0x0FU, 0x04U, 0x0AU, 0x01U, 0x05U, 0x0EU, 0x00U, 0x0BU,
    0x08U, 0x03U, 0x0DU, 0x06U, 0x02U, 0x09U, 0x07U, 0x0CU,
    0x01U, 0x0AU, 0x04U, 0x0FU, 0x0BU, 0x00U, 0x0EU, 0x05U,
    0x06U, 0x0DU, 0x03U, 0x08U, 0x0CU, 0x07U, 0x09U, 0x02U,
    0x0DU, 0x06U, 0x08U, 0x03U, 0x07U, 0x0CU, 0x02U, 0x09U,
    0x0AU, 0x01U, 0x0FU, 0x04U, 0x00U, 0x0BU, 0x05U, 0x0EU,
    0x03U, 0x08U, 0x06U, 0x0DU, 0x09U, 0x02U, 0x0CU, 0x07U,
    0x04U, 0x0FU, 0x01U, 0x0AU, 0x0EU, 0x05U, 0x0BU, 0x00U,
    0x02U, 0x09U, 0x07U, 0x0CU, 0x08U, 0x03U, 0x0DU, 0x06U,
    0x05U, 0x0EU, 0x00U, 0x0BU, 0x0FU, 0x04U, 0x0AU, 0x01U,
    0x0CU, 0x07U, 0x09U, 0x02U, 0x06U, 0x0DU, 0x03U, 0x08U,
    0x0BU, 0x00U, 0x0EU, 0x05U, 0x01U, 0x0AU, 0x04U, 0x0FU,
    0x09U, 0x02U, 0x0CU, 0x07U, 0x03U, 0x08U, 0x06U, 0x0DU,
    0x0EU, 0x05U, 0x0BU, 0x00U, 0x04U, 0x0FU, 0x01U, 0x0AU,
    0x07U, 0x0CU, 0x02U, 0x09U, 0x0DU, 0x06U, 0x08U, 0x03U,
    0x00U, 0x0BU, 0x05U, 0x0EU, 0x0AU, 0x01U, 0x0FU, 0x04U,
    0x06U, 0x0DU, 0x03U, 0x08U, 0x0CU, 0x07U, 0x09U, 0x02U,
    0x01U, 0x0AU, 0x04U, 0x0FU, 0x0BU, 0x00U, 0x0EU, 0x05U,
    0x08U, 0x03U, 0x0DU, 0x06U, 0x02U, 0x09U, 0x07U, 0x0CU,
    0x0FU, 0x04U, 0x0AU, 0x01U, 0x05U, 0x0EU, 0x00U, 0x0BU,
    0x04U, 0x0FU, 0x01U, 0x0AU, 0x0EU, 0x05U, 0x0BU, 0x00U,
    0x03U, 0x08U, 0x06U, 0x0DU, 0x09U, 0x02U, 0x0CU, 0x07U,
    0x0AU, 0x01U, 0x0FU, 0x04U, 0x00U, 0x0BU, 0x05U, 0x0EU,
    0x0DU, 0x06U, 0x08U, 0x03U, 0x07U, 0x0CU, 0x02U, 0x09U,
    0x0BU, 0x00U, 0x0EU, 0x05U, 0x01U, 0x0AU, 0x04U, 0x0FU,
    0x0CU, 0x07U, 0x09U, 0x02U, 0x06U, 0x0DU, 0x03U, 0x08U,
    0x05U, 0x0EU, 0x00U, 0x0BU, 0x0FU, 0x04U, 0x0AU, 0x01U,
    0x02U, 0x09U, 0x07U, 0x0CU, 0x08U, 0x03U, 0x0DU, 0x06U
};

static uint8_t computeCrc4Ccitt(const uint8_t *buf, const uint32_t numBytes)
{
    // Initialize local variables
    uint8_t tableRemainder = 0;
    uint8_t remainder = 0; // Initial remainder

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (uint32_t ctr = 0; ctr < numBytes; ctr++)
    {
        tableRemainder = buf[ctr] ^ remainder;
        remainder = u8CRC_4_TABLE[tableRemainder];
    }

    return remainder & 0x0FU;
}

static protocol_type_t processSonyByte(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;
    uint8_t checksum = 0;

    switch (p->state)
    {
        case 0:// Find start
            if (*(c->rxBuf.scan) == SONY_START_BYTE)
            {   // Found start
                p->state++;
            }
            return _PTYPE_NONE;

        case 1:        // Wait for header
            if ((int)(c->rxBuf.scan - c->rxBuf.head) < (int)(sizeof(sony_pkt_hdr_t)-1))
            {
                return _PTYPE_NONE;
            }

            // Validate header checksum
            sony_pkt_hdr_t *hdr = (sony_pkt_hdr_t *)(c->rxBuf.head);
            checksum = 0;
            for (size_t i = 0; i < 4; i++)
            {
                checksum += c->rxBuf.head[i];
            }
            if (checksum != hdr->fcsh || hdr->dataSize > MAX_MSG_LENGTH_SONY || hdr->dataSize > c->rxBuf.size)
            {   // Invalid header - Reset state
                return parseErrorResetState(c, EPARSE_INVALID_PREAMBLE);
            }

            // Valid header
            p->size = hdr->dataSize + 6;        // header(4) + FCSH/headerChecksum(1) + data(n) + FCSD/dataChecksum(1)
            p->state++;
            return _PTYPE_NONE;

        default:    // Wait for end of packet
            numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
            if (numBytes < (int)(p->size))
            {
                return _PTYPE_NONE;
            }
            // Found packet end
            break;
    }

    // Reset state
    p->state = 0;

    // Validate data checksum
    sony_pkt_hdr_t *hdr = (sony_pkt_hdr_t *)(c->rxBuf.head);
    checksum = 0;
    uint8_t *ptr = c->rxBuf.head + sizeof(sony_pkt_hdr_t);
    for (size_t i = 0; i < hdr->dataSize; i++)
    {
        checksum += ptr[i];
    }

    if (checksum != c->rxBuf.scan[0])
    {   // Invalid data checksum
        return parseErrorResetState(c, EPARSE_INVALID_CHKSUM);
    }

    /////////////////////////////////////////////////////////
    // Valid packet found - Checksum passed - Populate rxPkt
    validPacketFound(c, numBytes, p->size, c->rxBuf.head[3]);

    return _PTYPE_SONY;
}

static protocol_type_t processSpartnByte(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);

    switch (p->state)
    {
        case 0:
            if (*(c->rxBuf.scan) == SPARTN_START_BYTE)
            {
                p->state++;
            }
            return _PTYPE_NONE;

        case 1:
        case 2:
        // case 3 is below this to catch bad CRCs before any more is parsed. Can be adapted to filter messages later.
        case 4:
        case 5:
        case 6:
            p->state++;
            break;

        case 3: {
            // Check length and header CRC4
            const uint8_t dbuf[3] = { c->rxBuf.head[1], c->rxBuf.head[2], c->rxBuf.head[3] & 0xF0 };
            uint8_t calc = computeCrc4Ccitt(dbuf, 3);
            if ((c->rxBuf.head[3] & 0x0F) != calc)
            {      // Invalid header - Reset state
                return parseErrorResetState(c, EPARSE_INVALID_HEADER);
            }

            p->state++;
        } break;

        case 7:            // byte 7 (8th byte) is minimum header, but depending on what bits are set...
        case 8:
        case 9:
        case 10:
        case 11: {        // we may need to parse up to byte 11 (12th byte) to get the timestamp and encryption length
            uint16_t payloadLen = ((((uint16_t)(c->rxBuf.head[1]) & 0x01) << 9) |
                            (((uint16_t)(c->rxBuf.head[2])) << 1) |
                            ((c->rxBuf.head[3] & 0x80) >> 7)) & 0x3FF;

            // Variable length CRC {0x0, 0x1, 0x2, 0x3} = {1, 2, 3, 4}bytes - appears at end of message
            payloadLen += (((c->rxBuf.head[3] >> 4) & 0x03) + 1);

            uint8_t extendedTs = c->rxBuf.head[4] & 0x08;
            uint8_t encrypt = c->rxBuf.head[3] & 0x40;
            uint8_t *encryptPtr = NULL;

            if (extendedTs)
            {   // Timestamp is 32 bit
                if (!encrypt && p->state == 9)
                {   // Encryption is disabled, we are ready to go to payload bytes
                    p->state = -((int32_t)payloadLen);
                    break;
                }
                else if (encrypt && p->state == 11)
                {   // Encryption is ENABLED, and we have all the bytes we need to compute the length of payload
                    encryptPtr = &c->rxBuf.head[10];
                    // Don't break yet; continue to calculate encryption
                }
                else
                {   // Not ready yet
                    p->state++;
                    break;
                }
            }
            else
            {   // Timestamp is 16 bit
                if (!encrypt && p->state == 7)
                {   // Encryption is disabled, we are ready to go to payload bytes
                    p->state = -((int32_t)payloadLen);
                    break;
                }
                else if (encrypt && p->state == 9)
                {   // Encryption is ENABLED, and we have all the bytes we need to compute the length of payload
                    encryptPtr = &c->rxBuf.head[8];
                    // Don't break yet; continue to calculate encryption
                }
                else
                {   // Not ready yet
                    p->state++;
                    break;
                }
            }

            // Add encryption authentication bytes
            if (encryptPtr)
            {   // If the message contains an embedded authentication sequence, add the length
                if (((encryptPtr[1] >> 3) & 0x07) > 1)
                {
                    switch(encryptPtr[1] & 0x07)
                    {
                        case 0: payloadLen += 8; break;
                        case 1: payloadLen += 12; break;
                        case 2: payloadLen += 16; break;
                        case 3: payloadLen += 32; break;
                        case 4: payloadLen += 64; break;
                        default: break;
                    }
                }
            }
            else
            {   // Invalid data
                return parseErrorResetState(c, EPARSE_INVALID_PAYLOAD);
            }

            p->state = -((int32_t)payloadLen);

        } break;


        default:
            p->state++;

            if (p->state == 0)
            {   // Valid packet

                p->state = 0;

                /////////////////////////////////////////////////////////
                // Valid packet found - Checksum passed - Populate rxPkt
                int numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
                validPacketFound(c, numBytes, numBytes, 0);

                return _PTYPE_SPARTN;
            }
            else if (p->state > 0)
            {   // corrupt data or bad state
                return parseErrorResetState(c, EPARSE_INVALID_PAYLOAD);
            }

            break;
    }

    return _PTYPE_NONE;
}

/**
 * Move a buffer of data from src to dest.  This function is used to move data between buffers 
 * that are not aligned to 32-bit boundaries.  Equivalent to memmove() but more efficient on 
 * 32-bit processors.  Requires src and dest do not overlap such that dest will overwrite src 
 * data not yet copied (i.e. ideal use is dest < src).
 * @param dest the destination buffer
 * @param src the source buffer
 * @param size the number of bytes to move
 */
void move_buffer_32bit(void* dest, void* src, size_t size)
{
    uint32_t* dest32 = (uint32_t*)dest;
    uint32_t* src32 = (uint32_t*)src;

    size_t size32 = size>>2;
    size_t remaining = size&0x3;

    // Copy 32-bit chunks
    for (size_t i = 0; i < size32; i++)
    {
        dest32[i] = src32[i];
    }

    // Copy remaining bytes
    uint8_t* dest8 = (uint8_t*)(dest32 + size32);
    uint8_t* src8 = (uint8_t*)(src32 + size32);

    for (size_t i = 0; i < remaining; i++)
    {
        dest8[i] = src8[i];
    }
}

/**
 *            *** MAKE SURE YOU UNDERSTAND THIS FUNCTION BEFORE YOU USE IT ***
 *
 * Manages the comm_instance_t buffer pointers and returns the amount of free space in the buffer.  
 * This function should be called before adding data to the is_comm buffer and calling 
 * is_comm_parse_timeout() or related parse functions.  
 * - Reset buffer pointers to the start of the buffer if 1.) parsing is done or 2.) the buffer is full.
 * - Free up buffer space by shifting partial/incomplete packets to the beginning of the buffer. 
 * @param c the comm instance associated with the port
 * @return the number of free bytes available in the buffer
 */
int is_comm_free(is_comm_instance_t* c)
{
    is_comm_buffer_t *buf = &(c->rxBuf);

    int bytesFree = (int)(buf->end - buf->tail);

    // If the buffer contains any data
    if (bytesFree < (int)(buf->size))
    {   // Buffer contains data
        if (c->processPkt != NULL)
        {   // Currently parsing a packet
            if (buf->head != buf->start)
            {   // Data is not at the beginning of the buffer. Move current parse to the front.
                int shift = (int)(buf->head - buf->start);
                // Shift current data to start of buffer

                // memmove(buf->start, buf->head, buf->tail - buf->head);
                move_buffer_32bit(buf->start, buf->head, buf->tail - buf->head);    // Use instead of memmove() for efficiency on 32-bit processors

                buf->head = buf->start;
                buf->tail -= shift;
                buf->scan -= shift;

                // Re-calculate free byte count
                bytesFree = (int)(buf->end - buf->tail);
            }
            else if (bytesFree == 0)
            {   // The current packet if too big to parse. Clear buffer and reset pointers.
                reportParseError(c, EPARSE_RXBUFFER_FLUSHED);
                return is_comm_reset_buffer(c);
            }
        }
        else
        {   // Not currently parsing a packet
            if (buf->scan >= buf->tail) // No data left to scan in buffer. RESET pointers to start of the buffer.
                 return is_comm_reset_buffer(c);
        }
    }

    return bytesFree;
}

protocol_type_t is_comm_parse_byte_timeout(is_comm_instance_t* c, uint8_t byte, uint32_t timeMs)
{
    // Reset buffer if needed.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    is_comm_free(c);

    // Add byte to buffer
    *(c->rxBuf.tail) = byte;
    c->rxBuf.tail++;

    return is_comm_parse_timeout(c, timeMs);
}

protocol_type_t is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs)
{
    is_comm_buffer_t *buf = &(c->rxBuf);

    #if PKT_PARSER_TIMEOUT_MS 
        if (c->processPkt)
        {   // Parse in progress
            if (timeMs > c->parser.timeMs + PKT_PARSER_TIMEOUT_MS)
            {   // Parser timeout.  Increment head and reset parser.
                c->rxBuf.head++;
                is_comm_reset_parser(c);
            }
        }
    #endif

    // Search for packet
    while (buf->scan < buf->tail)
    {
        if (c->processPkt == NULL)
        {   // Scan for packet start
            switch (*(buf->scan))
            {
                case PSC_ISB_PREAMBLE_BYTE1:    if (c->cb.protocolMask & ENABLE_PROTOCOL_ISB)       { setParserStart(c, processIsbPkt); }      break;
                case PSC_NMEA_START_BYTE:       if (c->cb.protocolMask & ENABLE_PROTOCOL_NMEA)      { setParserStart(c, processNmeaPkt); }     break;
                case UBLOX_START_BYTE1:         if (c->cb.protocolMask & ENABLE_PROTOCOL_UBLOX)     { setParserStart(c, processUbloxPkt); }    break;
                case RTCM3_START_BYTE:          if (c->cb.protocolMask & ENABLE_PROTOCOL_RTCM3)     { setParserStart(c, processRtcm3Pkt); }    break;
                case SPARTN_START_BYTE:         if (c->cb.protocolMask & ENABLE_PROTOCOL_SPARTN)    { setParserStart(c, processSpartnByte); }  break;
                case SONY_START_BYTE:           if (c->cb.protocolMask & ENABLE_PROTOCOL_SONY)      { setParserStart(c, processSonyByte); }    break;
                default:                        
                    if (reportParseError(c, EPARSE_STREAM_UNPARSABLE))
                    { 
                        return _PTYPE_PARSE_ERROR; 
                    }
                    break;
            }
        }
        else
        {   // Parsing packet
            protocol_type_t ptype = c->processPkt(c);
            if (ptype != _PTYPE_NONE)
            {   // Packet found or packet error
                buf->scan++;
                return ptype;
            }
        }

        buf->scan++;
    }

#if PKT_PARSER_TIMEOUT_MS 
    if (c->processPkt)
    {   // Parsing in progress.  Record current time.
        c->parser.timeMs = timeMs;
    }
#endif

    return _PTYPE_NONE;
}

static inline void parse_messages(is_comm_instance_t* comm, port_handle_t port)
{
    if (!comm)
        return;

    // Search comm buffer for valid packets
    protocol_type_t ptype;
    while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
    {
        int notConsumed = -1;  // if 0, this message was successfully processed by a protocol-specific handler; Do not process it again with the ALL callback
        // Found valid packet
        switch (ptype)
        {
            case _PTYPE_INERTIAL_SENSE_DATA:
                if (comm->cb.isbData)
                {
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
                    notConsumed = comm->cb.generic[ptype](comm->cb.context, comm->rxPkt.data.ptr + comm->rxPkt.offset, comm->rxPkt.data.size, port);
                }
                break;
        }

        if (comm->cb.all && notConsumed)
        {
            comm->cb.all(comm->cb.context, ptype, &(comm->rxPkt), port);
        }
    }
}

void is_comm_buffer_parse_messages(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm)
{
    // Read data into comm buffer.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    int n = (int)_MIN((int)buf_size, is_comm_free(comm));

    memcpy(comm->rxBuf.tail, buf, n);

    // Update comm buffer tail pointer
    comm->rxBuf.tail += n;

    // Parse messages and call corresponding callback functions
    parse_messages(comm, NULL);
}

void is_comm_port_parse_messages(port_handle_t port)
{
    if ((port == NULL) || !(portType(port) & PORT_TYPE__COMM))
        return;

    if (COMM_PORT(port)->flags & COMM_PORT_FLAG__EXPLICIT_READ)
        return;

    is_comm_instance_t* comm = &COMM_PORT(port)->comm;

    // Read data into comm buffer.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    int bytesFree = is_comm_free(comm);

    int n = portRead(port, comm->rxBuf.tail, bytesFree);

    if (n <= 0)
    {   // No update
        return;
    }

    // Update comm buffer tail pointer
    comm->rxBuf.tail += n;

    // Parse messages and call corresponding callback functions
    parse_messages(comm, port);
}

int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, uint32_t did, uint32_t size, uint32_t offset, uint32_t periodMultiple)
{
    p_data_get_t get;

    get.id = did;
    get.offset = offset;
    get.size = size;
    get.period = periodMultiple;

    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_GET_DATA, 0, sizeof(p_data_get_t), 0, &get);
}

int is_comm_get_data(port_handle_t port, uint32_t did, uint32_t size, uint32_t offset, uint32_t periodMultiple)
{
    p_data_get_t get;

    get.id = did;
    get.offset = offset;
    get.size = size;
    get.period = periodMultiple;

    return is_comm_write(port, PKT_TYPE_GET_DATA, 0, sizeof(p_data_get_t), 0, &get);
}

void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data)
{
    // Header
    pkt->hdr.preamble = PSC_ISB_PREAMBLE;
    pkt->hdr.flags = flags;
    pkt->hdr.id = (uint8_t)did;
    pkt->hdr.payloadSize = data_size;

    // Payload
    pkt->offset = offset;
    if (offset)
    {   // Offset in payload
        pkt->hdr.flags |= ISB_FLAGS_PAYLOAD_W_OFFSET;
        pkt->hdr.payloadSize += 2;
    }
    pkt->data.ptr = (uint8_t *)data; // discard 'const'
    pkt->data.size = data_size;
    pkt->size = pkt->hdr.payloadSize + sizeof(packet_hdr_t) + 2;    // Pkt header + payload + checksum

    // Header checksum
    pkt->hdrCksum = is_comm_isb_checksum16(0, &pkt->hdr, sizeof(pkt->hdr));
}

#define MEMCPY_INC(dst, src, size)    memcpy((dst), (src), (size)); (dst) += (size);

void memcpyIncUpdateChecksum(uint8_t **dstBuf, const uint8_t* srcBuf, int len, uint16_t *checksum)
{
    *checksum = is_comm_isb_checksum16(*checksum, srcBuf, len);
    MEMCPY_INC(*dstBuf, srcBuf, len);
}

int is_comm_write_isb_precomp_to_buffer(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, packet_t *pkt)
{
    if (pkt->size > buf_size)
    {   // Packet doesn't fit in buffer
        return -1;
    }

    // Set checksum using precomputed header checksum and Write packet to buffer
    pkt->checksum = pkt->hdrCksum;
    MEMCPY_INC(buf, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));                                                   // Header
    if (pkt->offset)
    {
        memcpyIncUpdateChecksum(&buf, (uint8_t*)&(pkt->offset), 2, &(pkt->checksum));                               // Offset (optional)
    }
    if (pkt->data.size)
    {
        memcpyIncUpdateChecksum(&buf, (uint8_t*)pkt->data.ptr, pkt->data.size, &(pkt->checksum));                   // Payload
    }
    MEMCPY_INC(buf, (uint8_t*)&(pkt->checksum), 2);                                                                 // Footer (checksum)
    
    // Increment Tx count
    comm->txPktCount++;

    return pkt->size;
}

int portWriteUpdateChecksum(port_handle_t port, const uint8_t* buf, int len, uint16_t *checksum)
{
    *checksum = is_comm_isb_checksum16(*checksum, buf, len);
    return portWrite(port, buf, len);
}

int is_comm_write_isb_precomp_to_port(port_handle_t port, packet_t *pkt)
{
    if ((port == NULL) || !(portType(port) & PORT_TYPE__COMM))
    {   // can't write if we don't have a valid port, or the port isn't an ISComm
        return -2;  
    }

    if (pkt->data.size + sizeof(packet_hdr_t) + 4 > PKT_BUF_SIZE)
    {   // Packet size + offset + payload + footer is too large
        return -3;
    }

    // Set checksum using precomputed header checksum
    pkt->checksum = pkt->hdrCksum;

#ifdef GPX_1    // @Tony We want to remove this and use the single write call version for all ports.  We first need the stack usage (high water marks) 
    // implemented in the GPX RTOS status so we know if it is save to allocate a 2048 bytes buffer on the stack as a local variable in this function.
    
    // Write packet to port in multiple write calls (LEGACY).  Interruptable calls to this function that also write to the port could cause severed packets written.

    // Write packet to port
    int n = portWrite(port, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));                                           // Header
    if (pkt->offset)
    {
        n += portWriteUpdateChecksum(port, (uint8_t*)&(pkt->offset), 2, &(pkt->checksum));               // Offset (optional)
    }
    if (pkt->data.size)
    {
        n += portWriteUpdateChecksum(port, (uint8_t*)pkt->data.ptr, pkt->data.size, &(pkt->checksum));   // Payload
    }
    n += portWrite(port, (uint8_t*)&(pkt->checksum), 2);                                                            // Footer (checksum)
#else
    // Write packet to port in a single write call.  Reentrant function that prevents severed packets written to the port if this function gets interrupted and data written the same port.
    uint8_t buf[PKT_BUF_SIZE];
    uint8_t *ptr = buf;

    // Set checksum using precomputed header checksum and Write packet to buffer
    MEMCPY_INC(ptr, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));                                                   // Header
    if (pkt->offset)
    {
        memcpyIncUpdateChecksum(&ptr, (uint8_t*)&(pkt->offset), 2, &(pkt->checksum));                               // Offset (optional)
    }
    if (pkt->data.size)
    {
        memcpyIncUpdateChecksum(&ptr, (uint8_t*)pkt->data.ptr, pkt->data.size, &(pkt->checksum));                   // Payload
    }
    MEMCPY_INC(ptr, (uint8_t*)&(pkt->checksum), 2);                                                                 // Footer (checksum)

    // Write entire packet to port (all at once).  
    int n = portWrite(port, buf, (unsigned int)(ptr - buf));
#endif

    // Increment Tx count
    COMM_PORT(port)->comm.txPktCount++;

    // Check that number of bytes sent matches packet size.  Return number of bytes written on success or -1 on failure.
    return (n == pkt->size) ? n : -4;
}

int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data)
{
    packet_t txPkt;

    // Encode header and header checksum
    is_comm_encode_hdr(&txPkt, flags, did, data_size, offset, data);

    // Update checksum and write packet to buffer.  Returns number of bytes written on success or -1 on failure.
    return is_comm_write_isb_precomp_to_buffer(buf, buf_size, comm, &txPkt);
}

int is_comm_write(port_handle_t port, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data)
{
    packet_t txPkt;

    // Encode header and header checksum.  Update checksum and write packet to buffer.  Returns number of bytes written on success or -1 on failure.
    return is_comm_write_pkt(port, &txPkt, flags, did, data_size, offset, data);
}

int is_comm_write_pkt(port_handle_t port, packet_t *txPkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data)
{
    if ((port == NULL) || !(portType(port) & PORT_TYPE__COMM))
    {
        return -1;
    }

    // Encode header and header checksum
    is_comm_encode_hdr(txPkt, flags, did, data_size, offset, data);

    // Update checksum and write packet to port.  Returns number of bytes written on success or -1 on failure.
    return is_comm_write_isb_precomp_to_port(port, txPkt);
}

char copyStructPToDataP(p_data_t *data, const void *sptr, const unsigned int maxsize)
{
    if ((unsigned int)(data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)(data->ptr), (uint8_t*)sptr + data->hdr.offset, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

char copyDataPToStructP(void *sptr, const p_data_t *data, const unsigned int maxsize)
{
    if ((unsigned int)(data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + data->hdr.offset, data->ptr, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

char copyDataBufPToStructP(void *sptr, const p_data_buf_t *data, const unsigned int maxsize)
{
    if ((unsigned int)(data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + data->hdr.offset, data->buf, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

char copyDataPToDataP(p_data_t *dst, const p_data_t *src, const unsigned int maxsize)
{
    if (src->hdr.size <= maxsize)
    {
        dst->hdr = src->hdr;
        memcpy(dst->ptr, src->ptr, src->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const unsigned int maxsize)
{
    if ((unsigned int)(dataHdr->size + dataHdr->offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + dataHdr->offset, dataBuf, dataHdr->size);
        return 0;
    }
    else
    {
        return -1;
    }
}

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char is_comm_copy_to_struct(void *sptr, const is_comm_instance_t *c, const unsigned int maxsize)
{   
    const bufPtr_t *data = &(c->rxPkt.data);
    if ((data->size + c->rxPkt.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + c->rxPkt.offset, data->ptr, data->size);
        return 0;
    }
    else
    {
        return -1;
    }
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
