#pragma once

/*

./Tools/autotest/sim_vehicle.py --gdb --debug  -v ArduCopter -A --serial5=sim:sagetech_mxs --no-configure --map

param set SERIAL5_PROTOCOL 35          # ADSB
param set ADSB_TYPE        4           # Sagetech MX Series
param set SIM_ADSB_TYPES   8           # Sagetech MX Series
param set SIM_ADSB_COUNT 5
param set SIM_ADSB_RADIUS 2000         # same as ADSB_LIST_RADIUS

 */


#include "SIM_config.h"

#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED

#include "SIM_ADSB_Device.h"

#include <AP_Common/Location.h>
#include <AP_InternalError/AP_InternalError.h>

namespace SITL {

class ADSB_Sagetech_MXS : public ADSB_Device {
public:
    using ADSB_Device::ADSB_Device;

    void update(const class Aircraft *sitl_model);

private:
    void update_serial_input();
    void update_serial_output(const Aircraft *sitl_model);
    void update_serial_output_vehicles(const Aircraft *sitl_model);

    // pack a floating-point latitude or longitude into three bytes
    // according to Sagetech definitions:
    void pack_scaled_geocoord(uint8_t buf[3], float coord);
    void pack_scaled_alt(uint8_t buf[3], float alt_m);
    uint8_t scaled_groundspeed(float speed_m_s) const;
    void pack_scaled_groundspeed(uint8_t dest[1], float speed_m_s) const;
    void pack_scaled_airspeed(uint8_t dest[2], float speed_m_s) const;
    void pack_scaled_vertical_rate(uint8_t dest[2], float speed_m_s) const;
    // pack the three lower bytes of source into dest:
    void pack_int32_into_uint8_ts(int32_t source, uint8_t dest[3]);

    void send_vehicle_message(const class ADSB_Vehicle &vehicle);
    void send_vehicle_message_state_vector(const class ADSB_Vehicle &vehicle);
    void send_vehicle_message_status_report(const class ADSB_Vehicle &vehicle);

    void update_rf_output();
    void update_rf_input();

    enum class MsgType : uint8_t {
        INSTALLATION        = 0x01,
        FLIGHTID            = 0x02,
        OPMSG               = 0x03,
        GPS                 = 0x04,
        DATAREQ             = 0x05,
        TARGETREQUEST       = 0x0B,

        ACK                 = 0x80,
        STATEVECTORREPORT   = 0x91,
        MODESTATUSREPORT    = 0x92,
    };

    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(T _msg, MsgType _msgtype, uint8_t _msgid) :
            msgtype{_msgtype},
            msgid{_msgid},
            msg{_msg}
        {
            update_checksum();
        }
        uint8_t preamble { 0xAA };
        MsgType msgtype;
        uint8_t msgid;
        uint8_t payloadlen = sizeof(T);
        T msg;
        uint8_t checksum;

        uint16_t calculate_checksum(uint16_t len) const WARN_IF_UNUSED {
            uint8_t ret = 0;
            for (uint8_t i=0; i<len; i++) {
                ret += ((const char*)this)[i];

            }
            return ret;
        }
        uint16_t calculate_checksum() const WARN_IF_UNUSED {
            return calculate_checksum(4+sizeof(T));
        }

        void update_checksum() {
            checksum = calculate_checksum();
        }
    };

    enum class OperatingMode {
        OFF,
        MAINTENANCE,
    } operating_mode;

    // 0x01 - Installation Message
    class PACKED Installation {
    public:
        Installation()
        { }
        uint8_t ICAO_address[3];
        uint8_t aircraft_registration[7];
        uint8_t reserved1[2];
        uint8_t com_port_0;
        uint8_t com_port_1;
        uint8_t ip_adress[4];
        uint8_t net_mask[4];
        uint8_t port_number[2];
        uint8_t gps_integrity;
        uint8_t emitter_category_set;
        uint8_t emitter_category;
        uint8_t aircraft_size;
        uint8_t max_airspeed;
        uint8_t altitude_encoder_offset[2];
        uint8_t reserved2[2];
        uint8_t install_configuration;
        uint8_t reserved3[2];
    };

    // 0x01 - Flight ID  Message
    class PACKED FlightIDMessage {
    public:
        FlightIDMessage()
        { }
        uint8_t flight_id[8];
        uint8_t reserved[4];
    };

    // 0x03 - Operating Message
    class PACKED Operating {
    public:
    Operating()
    { }

        uint16_t squawk;
        uint8_t config;
        uint8_t emergency_flag;
        uint16_t alt;
        uint16_t climb_rate;
        uint16_t heading;
        uint16_t airspeed;
    };

    // 0x04 - GPS Navigation data message
    class PACKED GPS {
    public:
    GPS()
    { }
        uint8_t longitude[11];
        uint8_t latitude[10];
        uint8_t speed[6];
        uint8_t ground_track[8];
        uint8_t hemisphere;
        uint8_t time_of_fix[10];
        float height;
        float hpl;
        float hfom;
        float vfom;
        uint8_t nac;
    };

    // 0x05 - Data Request Message
    class PACKED DataRequest {
    public:
        DataRequest(uint8_t _reqtype) :
            reqtype(_reqtype)
            { }
        uint8_t reqtype;
        uint8_t reserved[3];

        enum class RequestDataType {
            HEALTH = 0x8D,
        };
    };

    // 0x0B - Target Request
    class PACKED TargetRequest {
    public:
        TargetRequest()
            { }
        uint8_t reqtype;
        uint16_t number_of_participants;
        uint8_t participant_id[3];
        uint8_t requested_reports;
    };

    // 0x80 - Acknowledgement Message
    class PACKED Ack {
    public:
    Ack(MsgType _ackType, uint8_t _ackId, uint8_t _state, uint8_t _alt[3]) :
        ackType{_ackType},
        ackId{_ackId},
        state{_state}
    {
        memcpy(alt, _alt, sizeof(alt));
    }
        MsgType ackType;
        uint8_t ackId;
        uint8_t state;
        uint8_t alt[3];
    };

    // 0x91 - ADSB State Vector Report.  Note that this structure is
    // full-sized, assuming that the report structure field is indicating
    // all fields present.  Actual parsing of this packet would
    // require looking at the report structure field.
    union PACKED StateVectorReport_ReportStructure {
        struct {
            struct PACKED {
                uint8_t time_of_applicability_for_estimated_positon : 1;
                uint8_t position_time_of_applicability : 1;
                uint8_t velocity_time_of_applicability : 1;
                uint8_t lat_and_lng : 1;
                uint8_t msn : 4;
            } rs0;
            struct PACKED {
                uint8_t estimated_latitude : 1;
                uint8_t navigation_integrity_category : 1;
                uint8_t vertical_rate_geometric_barometric : 1;
                uint8_t altimiter_barometric : 1;
                uint8_t heading_on_ground : 1;
                uint8_t ground_speed_on_ground : 1;
                uint8_t north_south_east_west_vel : 1;
                uint8_t altitude_geometric : 1;
            } rs1;
            struct PACKED {
                uint8_t reserved1 : 3;
                uint8_t report_mode : 1;
                uint8_t surveillance_status : 1;
                uint8_t estimated_east_west_velocity : 1;
                uint8_t estimated_north_south_velocity : 1;
                uint8_t estimated_longitude : 1;
            } rs2;
        };
        uint8_t raw[3];
    };

    class PACKED StateVectorReport {
    public:
        StateVectorReport_ReportStructure report_structure;
        uint16_t validity_flags;
        uint8_t participant_address[3];
        uint8_t address_qualifier;
        uint8_t report_times_of_applicability[6];
        uint8_t latitude[3];
        uint8_t longitude[3];
        uint8_t geometric_altitude[3];
        uint16_t vel_n;
        uint16_t vel_e;
        uint8_t groundspeed_on_surface;
        uint8_t heading_on_surface;
        uint8_t barometric_alt[3];
        uint8_t vertical_rate[2];
        uint8_t NIC;
        uint8_t estimated_latitude[3];
        uint8_t estimated_longitude[3];
        uint16_t estimated_vel_n;
        uint16_t estimated_vel_e;
        uint8_t surveillance_status;
        uint8_t report_mode;
    };
    template <uint8_t SIZE>
    class PACKED StateVectorReportBuffer {
    public:
    uint8_t buffer[SIZE];
    };


    // 0x92 - ADSB Status Report.  Note that this structure is
    // full-sized, assuming that the report structure field is indicating
    // all fields present.  Actual parsing of this packet would
    // require looking at the report structure.
    class PACKED ModeStatusReport {
    public:
        uint32_t report_structure : 24;
        uint16_t validity_flags;
        uint8_t participant_address[3];
        uint8_t address_qualifier;
        uint8_t report_times_of_applicability[6];
        uint8_t callsign[8];
        uint8_t emitter_category;
        uint8_t av_length_and_width_code;
        uint8_t emergency_priority_status;
        uint32_t capability_class_codes : 24;
        uint16_t operational_mode;
        uint8_t sv_quality_nacp;
        uint8_t sv_quality_nacv;
        uint8_t sv_quality_sil_and_sda;
        uint8_t sv_quality_gva;
        uint8_t sv_quality_nic;
        uint8_t sv_quality_track_heading_and_horizontal_reference_direction;
        uint8_t vertical_rate_type;
        uint8_t reserved[2];
    };
    template <uint8_t SIZE>
    class PACKED ModeStatusReportBuffer {
    public:
    uint8_t buffer[SIZE];
    };

    union u {
        u() {}
        char buffer[280]; // from-autopilot
        PackedMessage<Installation> packed_installation;
        PackedMessage<FlightIDMessage> packed_flightidmessage;
        PackedMessage<Operating> packed_operating;
        PackedMessage<GPS> packed_gps;
        PackedMessage<DataRequest> packed_data_req;
        PackedMessage<TargetRequest> packed_target_req;
        PackedMessage<Ack> packed_ack;  // we probably don't get ACKs from the autopilot....
        PackedMessage<StateVectorReport> packed_state_vector_report;
        PackedMessage<ModeStatusReport> packed_status_report;

        struct PACKED {
            uint8_t start_byte;
            MsgType msgtype;
            uint8_t msgid;
            uint8_t payload_length;
        } preamble;
    } msg;
    uint8_t buflen;

    // called when there is a message in the buffer which should be
    // handled.  If they return true then an ACK is sent.
    bool handle_message(const SITL::ADSB_Sagetech_MXS::Installation&);
    bool handle_message(const SITL::ADSB_Sagetech_MXS::FlightIDMessage&);
    bool handle_message(const SITL::ADSB_Sagetech_MXS::Operating&);
    bool handle_message(const SITL::ADSB_Sagetech_MXS::GPS&);
    bool handle_message(const SITL::ADSB_Sagetech_MXS::DataRequest&);
    bool handle_message(const SITL::ADSB_Sagetech_MXS::TargetRequest&);
    // bool handle_message(const SITL::ADSB_Sagetech_MXS::StateVectorReport&);

    bool _handle_message();
    void handle_message();

    double lon_string_to_double(const uint8_t *lat);
    double lat_string_to_double(const uint8_t *lng);

    /*
     * Simulated device state and methods
     */
    struct {
        bool send;
        MsgType ackType;
        uint8_t ackId;
    } ack_info;

    uint8_t msgid;

    struct {
        struct {
            double lat;
            double lng;
        } gps;
    } info_from_vehicle;

    void assert_good_flight_id();
    char flight_id[8];
    uint32_t flight_id_set_time_ms;

    /*
      serial parsing methods and data
    */

    // packet structure:
    // 1-byte start-byte
    // 1-byte msgtype
    // 1-byte msgid (sequence number)
    // 1-byte payloadlen
    // n-bytes payload
    // 1-byte checksum (sum of all previous bytes in packet % 256

    void move_preamble_in_buffer(uint8_t search_start_pos=0);

    enum class InputState {
        WANT_START_BYTE = 56,
        WANT_PREAMBLE = 57,
        WANT_PAYLOAD = 60,
        WANT_CHECKSUM = 61,
    } input_state = InputState::WANT_START_BYTE;
    void set_input_state(InputState newstate) {
        // ::fprintf(stderr, "Moving from inputstate (%u) to (%u)\n", (uint8_t)_inputstate, (uint8_t)newstate);
        input_state = newstate;
    }

};

}  // namespace SITL

#endif  // AP_SIM_ADSB_SAGETECH_MXS_ENABLED
