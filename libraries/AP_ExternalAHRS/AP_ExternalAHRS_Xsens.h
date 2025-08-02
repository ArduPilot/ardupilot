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
  support for Xsens INS devices (MTi-680G, MTi-670G, MTi-G-710, MTi-7, MTi-8)
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_XSENS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_GPS/AP_GPS.h>

class AP_ExternalAHRS_Xsens : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_Xsens(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled  
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;

    // get name of this backend
    const char* get_name() const override { return "Xsens"; }

    // check for new data
    void update() override;

    // Return the number of GPS sensors sharing data to AP_GPS
    uint8_t num_gps_sensors(void) const override { return 1; }

private:
    // Xbus constants
    static constexpr uint8_t XBUS_PREAMBLE = 0xFA;
    static constexpr uint8_t XBUS_MASTERDEVICE = 0xFF;
    static constexpr uint8_t LENGTH_EXTENDER_BYTE = 0xFF;
    static constexpr uint8_t XBUS_CHECKSUM_SIZE = 1;
    
    // Message IDs from xbus_message_id.h
    enum XsMessageId : uint8_t {
        Wakeup = 0x3E,
        WakeupAck = 0x3F,
        ReqDid = 0x00,
        DeviceId = 0x01,
        GotoConfig = 0x30,
        GotoConfigAck = 0x31,
        GotoMeasurement = 0x10,
        GotoMeasurementAck = 0x11,
        MtData2 = 0x36,
        SetOutputConfig = 0xC0,
        OutputConfig = 0xC1,
        Reset = 0x40,
        ResetAck = 0x41,
        Error = 0x42,
        SetAlignmentRotation = 0xEC,
        AlignmentRotationAck = 0xED,
        ReqFirmwareRevision = 0x12,
        FirmwareRevision = 0x13
    };

    // XDI (Xsens Data Identifier) constants
    enum XDI : uint16_t {
        PACKET_COUNTER = 0x1020,
        SAMPLE_TIME_FINE = 0x1060,
        EULER_ANGLES = 0x2030,
        STATUS_WORD = 0xE020,
        LAT_LON = 0x5042,
        ALTITUDE_ELLIPSOID = 0x5022,
        VELOCITY_XYZ = 0xD012,
        QUATERNION = 0x2010,
        ACCELERATION = 0x4020,
        RATE_OF_TURN = 0x8020,
        MAGNETIC_FIELD = 0xC020,
        UTC_TIME = 0x1010,
        BAROMETRIC_PRESSURE = 0x3010,
        TEMPERATURE = 0x0810,
        GNSSPVTDATA = 0x7010
    };

    // Device state machine
    enum class DeviceState {
        ENTERING_CONFIG_MODE,
        WAITING_FOR_CONFIG_MODE,
        CONFIGURING_OUTPUT,
        WAITING_FOR_OUTPUT_CONFIG,
        CONFIGURING_ROTLOCAL,
        WAITING_FOR_ROTLOCAL_CONFIG,
        CONFIGURING_ROTSENSOR,
        WAITING_FOR_ROTSENSOR_CONFIG,
        ENTERING_MEASUREMENT_MODE,
        WAITING_FOR_MEASUREMENT_MODE,
        RUNNING,
        ERROR
    };

    // Data structures for parsed sensor data
    struct EulerAngles {
        float roll;
        float pitch;
        float yaw;
        EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
    };

    struct LatLon {
        double latitude;
        double longitude;
        LatLon() : latitude(0.0), longitude(0.0) {}
    };

    struct VelocityXYZ {
        double velX;
        double velY;
        double velZ;
        VelocityXYZ() : velX(0.0), velY(0.0), velZ(0.0) {}
    };

    struct QuaternionData {
        float q0;  // w component
        float q1;  // x component
        float q2;  // y component
        float q3;  // z component
        QuaternionData() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    };

    struct UtcTime {
        uint32_t nanoseconds;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t flags;
        UtcTime() : nanoseconds(0), year(0), month(0), day(0), hour(0), minute(0), second(0), flags(0) {}
    };

    struct AccelerationXYZ {
        float accX;
        float accY;
        float accZ;
        AccelerationXYZ() : accX(0.0f), accY(0.0f), accZ(0.0f) {}
    };

    struct RateOfTurnXYZ {
        float gyrX;
        float gyrY;
        float gyrZ;
        RateOfTurnXYZ() : gyrX(0.0f), gyrY(0.0f), gyrZ(0.0f) {}
    };

    struct MagneticFieldXYZ {
        float magX;
        float magY;
        float magZ;
        MagneticFieldXYZ() : magX(0.0f), magY(0.0f), magZ(0.0f) {}
    };

    struct GnssPvtData {
        uint32_t iTOW;          // GPS time of week of the navigation epoch [ms]
        uint16_t year;          // Year (UTC)
        uint8_t month;          // Month, range 1..12 (UTC)
        uint8_t day;            // Day of month, range 1..31 (UTC)
        uint8_t hour;           // Hour of day, range 0..23 (UTC)
        uint8_t min;            // Minute of hour, range 0..59 (UTC)
        uint8_t sec;            // Seconds of minute, range 0..60 (UTC)
        uint8_t valid;          // Validity flags
        uint32_t tAcc;          // Time accuracy estimate [ns]
        int32_t nano;           // Fraction of second, range -1e9 .. 1e9 [ns]
        uint8_t fixType;        // GNSSfix Type
        uint8_t flags;          // Fix status flags
        uint8_t numSv;          // Number of satellites used in Nav Solution
        uint8_t reserved;       // Reserved
        int32_t lon;            // Longitude [deg * 1e-7]
        int32_t lat;            // Latitude [deg * 1e-7]
        int32_t height;         // Height above ellipsoid [mm]
        int32_t hMSL;           // Height above mean sea level [mm]
        uint32_t hAcc;          // Horizontal accuracy estimate [mm]
        uint32_t vAcc;          // Vertical accuracy estimate [mm]
        int32_t velN;           // NED north velocity [mm/s]
        int32_t velE;           // NED east velocity [mm/s]
        int32_t velD;           // NED down velocity [mm/s]
        int32_t gSpeed;         // Ground Speed (2-D) [mm/s]
        int32_t headMot;        // Heading of motion (2-D) [deg * 1e-5]
        uint32_t sAcc;          // Speed accuracy estimate [mm/s]
        uint32_t headAcc;       // Heading accuracy estimate [deg * 1e-5]
        int32_t headVeh;        // Heading of vehicle (2-D) [deg * 1e-5]
        uint16_t gDop;          // Geometric DOP [* 0.01]
        uint16_t pDop;          // Position DOP [* 0.01]
        uint16_t tDop;          // Time DOP [* 0.01]
        uint16_t vDop;          // Vertical DOP [* 0.01]
        uint16_t hDop;          // Horizontal DOP [* 0.01]
        uint16_t nDop;          // Northing DOP [* 0.01]
        uint16_t eDop;          // Easting DOP [* 0.01]
        
        GnssPvtData() : iTOW(0), year(0), month(0), day(0), hour(0), min(0), sec(0), 
                        valid(0), tAcc(0), nano(0), fixType(0), flags(0), numSv(0), 
                        reserved(0), lon(0), lat(0), height(0), hMSL(0), hAcc(0), vAcc(0),
                        velN(0), velE(0), velD(0), gSpeed(0), headMot(0), sAcc(0), 
                        headAcc(0), headVeh(0), gDop(9999), pDop(9999), tDop(9999), 
                        vDop(9999), hDop(9999), nDop(9999), eDop(9999) {}
    };

    struct SensorData {
        bool hasPacketCounter = false;
        bool hasSampleTimeFine = false;
        bool hasEulerAngles = false;
        bool hasStatusWord = false;
        bool hasLatLon = false;
        bool hasAltitudeEllipsoid = false;
        bool hasVelocityXYZ = false;
        bool hasUtcTime = false;
        bool hasQuaternion = false;
        bool hasBarometricPressure = false;
        bool hasAcceleration = false;
        bool hasRateOfTurn = false;
        bool hasMagneticField = false;
        bool hasTemperature = false;
        bool hasGnssPvtData = false;

        uint16_t packetCounter = 0;
        uint32_t sampleTimeFine = 0;
        EulerAngles eulerAngles;
        uint32_t statusWord = 0;
        LatLon latLon;
        double altitudeEllipsoid = 0.0;
        VelocityXYZ velocityXYZ;
        UtcTime utcTime;
        QuaternionData quaternion;
        uint32_t barometricPressure = 0;
        AccelerationXYZ acceleration;
        RateOfTurnXYZ rateOfTurn;
        MagneticFieldXYZ magneticField;
        float temperature = 0.0f;
        GnssPvtData gnssPvtData;
    };

    // Member variables
    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
    int8_t port_num;
    uint32_t baudrate;
    bool port_open = false;

    DeviceState device_state = DeviceState::ENTERING_CONFIG_MODE;
    uint32_t last_ins_pkt = 0;
    uint32_t last_gps_pkt = 0;
    uint32_t last_filter_pkt = 0;
    uint32_t state_timeout = 0;

    // Receive buffer
    static constexpr size_t BUFFER_SIZE = 1024;
    uint8_t rx_buffer[BUFFER_SIZE];
    size_t rx_buffer_pos = 0;

    // Current sensor data
    SensorData current_sensor_data;
    
    // Buffered GNSS PVT data
    GnssPvtData buffered_gnss_pvt;
    bool has_buffered_gnss_pvt = false;
    uint32_t last_gnss_pvt_update = 0;
    static constexpr uint32_t GNSS_PVT_TIMEOUT_MS = 5000; // Consider stale after 5 seconds

    // GPS status buffering
    AP_GPS::GPS_Status last_valid_fix_type = AP_GPS::NO_FIX;
    uint8_t last_satellites_in_view = 0;
    float last_horizontal_pos_accuracy = 99.9f;
    float last_vertical_pos_accuracy = 99.9f;
    float last_horizontal_vel_accuracy = 99.9f;
    float last_hdop = 99.9f;
    float last_vdop = 99.9f;
    bool gps_status_initialized = false;

    // Communication methods
    void update_thread();
    bool write_message(const uint8_t *message, size_t length);
    size_t read_data(uint8_t *buffer, size_t max_length);
    void process_received_data();
    void handle_message(const uint8_t *message);
    void handle_mtdata2_message(const uint8_t *message);
    void handle_error_message(const uint8_t *message);

    // State machine
    void update_state_machine();
    void set_device_state(DeviceState new_state);
    const char* get_state_string(DeviceState device_state_param) const;

    // Device configuration
    bool goto_config_mode();
    bool configure_output_settings();
    bool set_ned_frame();
    bool set_sensor_frame(bool is_up);
    bool goto_measurement_mode();

    // Xbus protocol helpers
    bool check_preamble(const uint8_t* xbus_message) const;
    uint8_t get_message_id(const uint8_t* xbus_message) const;
    uint8_t get_payload_length(const uint8_t* xbus_message) const;
    void create_message(uint8_t* xbus_message, uint8_t bid, uint8_t mid, uint16_t len);
    uint8_t* get_pointer_to_payload(uint8_t* xbus_message);
    const uint8_t* get_const_pointer_to_payload(const uint8_t* xbus_message) const;
    void insert_checksum(uint8_t* xbus_message);
    bool verify_checksum(const uint8_t* xbus_message) const;
    int get_raw_length(const uint8_t* xbus_message) const;
    void set_payload_length(uint8_t* xbus_message, uint16_t payload_length);

    // Data parsing helpers
    uint8_t read_uint8(const uint8_t* data, int& index) const;
    uint16_t read_uint16(const uint8_t* data, int& index) const;
    uint32_t read_uint32(const uint8_t* data, int& index) const;
    int32_t read_int32(const uint8_t* data, int& index) const;
    float read_float(const uint8_t* data, int& index) const;
    double read_fp1632(const uint8_t* data, int& index) const;
    bool parse_mtdata2(const uint8_t* xbus_data, SensorData& sensor_data);
    bool parse_gnss_pvt_data(const uint8_t* payload, int& index, uint8_t size, GnssPvtData& gnss_pvt);

    // Data publishing
    void publish_sensor_data(const SensorData &data);
    void publish_gnss_pvt_data(const GnssPvtData &gnss_pvt);
    
    // Utility functions
    uint64_t convert_utc_time_to_unix_microseconds(const UtcTime &utc_time) const;
    AP_GPS::GPS_Status convert_fix_type(uint8_t fix_type, uint8_t flags) const;
    void calculate_gps_time_from_utc(uint16_t year, uint8_t month, uint8_t day, 
                                   uint8_t hour, uint8_t minute, uint8_t second, 
                                   int32_t nano, uint16_t &gps_week, uint32_t &ms_tow) const;
    bool is_gnss_status_valid() const;

};

#endif  // AP_EXTERNAL_AHRS_XSENS_ENABLED