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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>

class AP_ExternalAHRS_InertialSense : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;

protected:
    uint8_t num_gps_sensors(void) const override;

private:
    /*
     * Inertial Sense ISB protocol types, condensed from ISComm.h and base_port.h.
     * Only the subset needed by this driver is included.  Data-set payload types
     * (ins_3_t, etc.) are forward-declared here and fully defined in the .cpp.
     */

    // Forward declarations for ISB data-set payload types (defined in .cpp).
    struct dev_info_t;
    struct ins_3_t;
    struct gps_pos_t;
    struct gps_vel_t;
    struct gps_rtk_misc_t;
    struct magnetometer_t;
    struct barometer_t;
    struct inl2_ned_sigma_t;
    struct bit_t;

    // Opaque port handle (from base_port.h).
    using port_handle_t = void*;

    enum ProtocolType
    {
        _PTYPE_NONE                 = 0,
        _PTYPE_PARSE_ERROR          = 1,
        _PTYPE_INERTIAL_SENSE_ACK   = 2,
        _PTYPE_INERTIAL_SENSE_CMD   = 3,
        _PTYPE_INERTIAL_SENSE_DATA  = 4,
        _PTYPE_NMEA                 = 5,
        _PTYPE_UBLOX                = 6,
        _PTYPE_RTCM3                = 7,
        _PTYPE_SPARTN               = 8,
        _PTYPE_SONY                 = 9,
        _PTYPE_FIRST_DATA           = _PTYPE_INERTIAL_SENSE_DATA,
        _PTYPE_LAST_DATA            = _PTYPE_SONY,
        _PTYPE_SIZE                 = _PTYPE_LAST_DATA + 1,
    };

    enum IsbPacketFlags
    {
        PKT_TYPE_INVALID                        = 0,
        PKT_TYPE_ACK                            = 1,
        PKT_TYPE_NACK                           = 2,
        PKT_TYPE_GET_DATA                       = 3,
        PKT_TYPE_DATA                           = 4,
        PKT_TYPE_SET_DATA                       = 5,
        PKT_TYPE_STOP_BROADCASTS_ALL_PORTS      = 6,
        PKT_TYPE_STOP_DID_BROADCAST             = 7,
        PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT   = 8,
        PKT_TYPE_COUNT                          = 9,
        PKT_TYPE_MAX_COUNT                      = 16,
        PKT_TYPE_MASK                           = 0x0F,
        ISB_FLAGS_MASK                          = 0xF0,
        ISB_FLAGS_EXTENDED_PAYLOAD              = 0x10,
        ISB_FLAGS_PAYLOAD_W_OFFSET              = 0x20,
    };

    enum ProtocolMask
    {
        ENABLE_PROTOCOL_ISB     = (0x00000001 << _PTYPE_INERTIAL_SENSE_DATA),
        ENABLE_PROTOCOL_NMEA    = (0x00000001 << _PTYPE_NMEA),
        ENABLE_PROTOCOL_UBLOX   = (0x00000001 << _PTYPE_UBLOX),
        ENABLE_PROTOCOL_RTCM3   = (0x00000001 << _PTYPE_RTCM3),
        ENABLE_PROTOCOL_SPARTN  = (0x00000001 << _PTYPE_SPARTN),
        ENABLE_PROTOCOL_SONY    = (0x00000001 << _PTYPE_SONY),
    };

    static constexpr uint32_t MAX_DATASET_SIZE = 1024;
    static constexpr uint32_t PKT_BUF_SIZE     = 2048;
    static constexpr uint32_t DEFAULT_PROTO_MASK = ENABLE_PROTOCOL_ISB;

    struct PACKED packet_hdr_t
    {
        uint16_t    preamble;
        uint8_t     flags;
        uint8_t     id;
        uint16_t    payloadSize;
    };

    struct PACKED p_data_hdr_t
    {
        uint8_t     id;
        uint16_t    size;
        uint16_t    offset;
    };

    struct PACKED bufPtr_t
    {
        uint8_t     *ptr;
        uint32_t    size;
    };

    struct PACKED packet_t
    {
        union
        {
            struct
            {
                packet_hdr_t    hdr;
                uint16_t        offset;
            };
            struct
            {
                uint16_t        preamble;
                uint8_t         flags;
                p_data_hdr_t    dataHdr;
            };
        };
        bufPtr_t    data;
        uint16_t    hdrCksum;
        uint16_t    checksum;
        uint16_t    size;
        uint16_t    id;
    };

    struct PACKED packet_buf_t
    {
        packet_hdr_t    hdr;
        union
        {
            uint8_t     data;
            uint16_t    offset;
        }               payload;
    };

    struct PACKED p_data_t
    {
        p_data_hdr_t    hdr;
        uint8_t         *ptr;
    };

    struct PACKED p_data_buf_t
    {
        p_data_hdr_t    hdr;
        uint8_t         buf[MAX_DATASET_SIZE];
    };

    struct PACKED p_data_get_t
    {
        uint16_t    id;
        uint16_t    size;
        uint16_t    offset;
        uint16_t    period;
    };

    enum ParseErrorType {
        EPARSE_INVALID_PREAMBLE,
        EPARSE_INVALID_SIZE,
        EPARSE_INVALID_CHKSUM,
        EPARSE_INVALID_DATATYPE,
        EPARSE_MISSING_EOS_MARKER,
        EPARSE_INCOMPLETE_PACKET,
        EPARSE_INVALID_HEADER,
        EPARSE_INVALID_PAYLOAD,
        EPARSE_RXBUFFER_FLUSHED,
        EPARSE_STREAM_UNPARSABLE,
        NUM_EPARSE_ERRORS
    };

    struct is_comm_parser_t
    {
        int16_t     state;
        uint16_t    size;
        uint32_t    timeMs;
    };

    using pFnProcessPkt = ProtocolType(*)(void*);

    using pfnIsCommPortWrite = int(*)(port_handle_t port, const uint8_t* buf, int len);
    using pfnIsCommPortRead = int(*)(port_handle_t port, uint8_t* buf, int bufLen);
    using pfnIsCommIsbDataHandler = int(*)(void* ctx, p_data_t* data, port_handle_t port);
    using pfnIsCommGenMsgHandler = int(*)(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port);
    using pfnIsCommHandler = int(*)(void* ctx, ProtocolType ptype, packet_t *pkt, port_handle_t port);

    struct is_comm_buffer_t
    {
        uint8_t*    start;
        uint8_t*    end;
        uint32_t    size;
        uint8_t*    head;
        uint8_t*    tail;
        uint8_t*    scan;
        uint8_t*    scanPrior;
    };

    struct is_comm_callbacks_t
    {
        uint32_t                    protocolMask;
        void*                       context;
        pfnIsCommHandler            all;
        pfnIsCommIsbDataHandler     isbData;
        pfnIsCommGenMsgHandler      generic[_PTYPE_SIZE];
    };

    struct is_comm_instance_t
    {
        is_comm_buffer_t    rxBuf;
        uint32_t            txPktCount;
        uint32_t            rxPktCount;
        uint32_t            rxErrorCount;
        ParseErrorType      rxErrorType;
        uint32_t            rxErrorTypeCount[NUM_EPARSE_ERRORS];
        pFnProcessPkt       processPkt;
        is_comm_parser_t    parser;
        uint32_t            ackNeeded;
        packet_t            rxPkt;
        uint8_t             rxErrorState;
        is_comm_callbacks_t cb;
    };

    static void dev_info_populate_missing_hardware(dev_info_t *devInfo);
    static uint16_t isb_fletcher16(uint16_t cksum_init, const void* data, uint32_t size);
    static int is_comm_reset_buffer(is_comm_instance_t* c);
    static void is_comm_reset_parser(is_comm_instance_t* c);
    static void is_comm_to_isb_p_data(const is_comm_instance_t *comm, p_data_t *data);
    static ProtocolType report_parse_error(is_comm_instance_t* c, ParseErrorType errorType);
    static ProtocolType parse_error_reset_state(is_comm_instance_t* c, ParseErrorType errorType);
    static void valid_packet_reset(is_comm_instance_t* c, int pktSize);
    static void set_parser_start(is_comm_instance_t* c, pFnProcessPkt processPkt);
    static ProtocolType process_isb_pkt(void* v);
    static void is_comm_init(is_comm_instance_t* c, uint8_t *buf, int bufSize, pfnIsCommHandler pktHandler);
    static void is_comm_enable_protocol(is_comm_instance_t* comm, ProtocolType ptype);
    static pfnIsCommIsbDataHandler is_comm_register_isb_handler(is_comm_instance_t* comm, pfnIsCommIsbDataHandler cbHandler);
    static int is_comm_free(is_comm_instance_t* c);
    static void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);
    static void memcpy_inc_update_checksum(uint8_t **dstBuf, const uint8_t* srcBuf, int len, uint16_t *checksum);
    static int is_comm_write_isb_precomp_to_buffer(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, packet_t *pkt);
    static int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);
    static int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, uint32_t did, uint32_t size, uint32_t offset, uint32_t periodMultiple);
    static ProtocolType is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs);
    static ProtocolType is_comm_parse(is_comm_instance_t* comm);
    static void parse_messages(is_comm_instance_t* comm, port_handle_t port);
    static void is_comm_buffer_parse_messages(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm);

    void initialize();
    void update_thread();
    bool check_uart();

    int stop_message_broadcasting();
    int enable_message_broadcasting();

    void handle_ins3_message(ins_3_t* ins);
    void handle_gps_pos_message(gps_pos_t* pos);
    void handle_gps_vel_message(gps_vel_t* vel);
    void handle_gps_rtk_pos_misc_message(gps_rtk_misc_t* misc);
    void handle_gps2_pos_message(gps_pos_t* pos);
    void handle_gps2_vel_message(gps_vel_t* vel);
    void handle_flash_config_message(const uint8_t *raw, uint16_t size);
    void handle_magnetometer_message(magnetometer_t* mag);
    void handle_barometer_message(barometer_t* bar);
    void handle_inl2_ned_sigma_message(inl2_ned_sigma_t *sigmas);
    void handle_dev_info_message(dev_info_t *dev_info);
    void handle_bit_message(bit_t* bit);
    int parse_isb_data(void* ctx, p_data_t* data, port_handle_t port);

    // callback helper
    static AP_ExternalAHRS_InertialSense *instance;
    static int isb_data_handler(void* ctx, p_data_t* data, port_handle_t port) {
        return instance->parse_isb_data(ctx, data, port);
    }
    bool initialized;
    bool _healthy;
    AP_GPS_FixType _fix_type = AP_GPS_FixType::NONE;

    uint32_t baudrate;
    int8_t port_num;
    uint8_t buffer[1024];

    uint32_t last_gps_pkt;
    uint32_t last_filter_pkt;

    char _name[40] = "Inertial Sense";

    uint8_t comm_buf[2048];
    is_comm_instance_t comm;

    float vel_cov = 0;
    float pos_cov = 0;
    float hgt_cov = 0;

    uint8_t _num_gps_sensors = 1;

    AP_ExternalAHRS::gps_data_message_t gps_data_msg;
    AP_ExternalAHRS::gps_data_message_t gps2_data_msg;

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
};

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
