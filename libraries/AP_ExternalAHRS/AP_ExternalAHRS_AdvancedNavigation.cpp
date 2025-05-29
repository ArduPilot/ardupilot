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

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ADNAV_ENABLED

#include "AP_ExternalAHRS_AdvancedNavigation.h"
#include <AP_Baro/AP_Baro.h>                     // for AP.baro()
#include <AP_Compass/AP_Compass.h>               // for AP.compass()
#include <AP_GPS/AP_GPS.h>                       // for AP.gps()
#include <AP_InertialSensor/AP_InertialSensor.h> // for AP.ins()
#include <AP_SerialManager/AP_SerialManager.h>   // for AP().serialmanager()
#include <AP_Logger/AP_Logger.h>                 // for AP().logger(...)
#include <GCS_MAVLink/GCS.h>                     // for GCS_SEND_TEXT
#include <AP_Math/AP_Math.h>                     // for degrees()

#define AN_SERIAL_INSTANCE 0                      // first instance of ExternalAHRS serial device
#define AN_TIMEOUT 5000                           // ms
#define AN_GCS_TIMEOUT 5000                       // ms
#define AN_GNSS_PACKET_RATE 10                    // Hz
#define AN_UPDATE_THREAD_PERIOD 1000              // µs
#define AN_DEVICE_INFORMATION_UPDATE_PERIOD 20000 // ms
#define AN_DEVICE_PACKET_RATE_UPDATE_PERIOD 5000  // ms
#define AN_STATE_PACKET_MAX_DELAY 500             // ms
#define AN_PACKET_TIMER_PERIODS_RATE 1000         // µs
#define AN_START_STATE_PACKETS 20
#define AN_START_CONFIGURATION_PACKETS 180
#define AN_MIN_CONNECTION_ATTEMPTS 2

extern const AP_HAL::HAL &hal;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Scope Declerations
//////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t get_epoch_timestamp(uint32_t unix_timestamp);
static uint16_t get_gps_week(uint32_t unix_timestamp);
static uint16_t get_gps_tow(uint32_t unix_timestamp, uint32_t microseconds);

//////////////////////////////////////////////////////////////////////////////////////////////////
// ANPP Types
//////////////////////////////////////////////////////////////////////////////////////////////////

/*
  Advanced Navigation Packet Protocol Packet Identifiers

  https://docs.advancednavigation.com/certus/ANPP/Advanced%20Navigation%20Packet.htm
*/
enum AN_PacketId : uint8_t
{
    AN_ACKNOWLEDGE_ID,
    AN_REQUEST_ID,
    AN_BOOT_MODE_ID,
    AN_DEVICE_INFORMATION_ID,
    AN_RESTORE_FACTORY_SETTINGS_ID,
    AN_RESET_ID,
    AN_RESERVED_6_ID,
    AN_FILE_TRANSFER_REQUEST_ID,
    AN_FILE_TRANSFER_ACKNOWLEDGE_ID,
    AN_FILE_TRANSFER_ID,
    AN_SERIAL_PORT_PASSTHROUGH_ID,
    AN_IP_CONFIGURATION_ID,
    AN_RESERVED_12_ID,
    AN_EXTENDED_DEVICE_INFORMATION_ID,
    AN_SUBCOMPONENT_INFORMATION_ID,
    AN_END_SYSTEM_PACKETS,

    AN_SYSTEM_STATE_ID = AN_START_STATE_PACKETS,
    AN_UNIX_TIME_ID,
    AN_FORMATTED_TIME_ID,
    AN_STATUS_ID,
    AN_POSITION_STANDARD_DEVIATION_ID,
    AN_VELOCITY_STANDARD_DEVIATION_ID,
    AN_EULER_ORIENTATION_STANDARD_DEVIATION_ID,
    AN_QUATERNION_ORIENTATION_STANDARD_DEVIATION_ID,
    AN_RAW_SENSORS_ID,
    AN_RAW_GNSS_ID,
    AN_SATELLITES_ID,
    AN_SATELLITES_DETAILED_ID,
    AN_GEODETIC_POSITION_ID,
    AN_ECEF_POSITION_ID,
    AN_UTM_POSITION_ID,
    AN_NED_VELOCITY_ID,
    AN_BODY_VELOCITY_ID,
    AN_ACCELERATION_ID,
    AN_BODY_ACCELERATION_ID,
    AN_EULER_ORIENTATION_ID,
    AN_QUATERNION_ORIENTATION_ID,
    AN_DCM_ORIENTATION_ID,
    AN_ANGULAR_VELOCITY_ID,
    AN_ANGULAR_ACCELERATION_ID,
    AN_EXTERNAL_POSITION_VELOCITY_ID,
    AN_EXTERNAL_POSITION_ID,
    AN_EXTERNAL_VELOCITY_ID,
    AN_EXTERNAL_BODY_VELOCITY_ID,
    AN_EXTERNAL_HEADING_ID,
    AN_RUNNING_TIME_ID,
    AN_LOCAL_MAGNETICS_ID,
    AN_ODOMETER_STATE_ID,
    AN_EXTERNAL_TIME_ID,
    AN_EXTERNAL_DEPTH_ID,
    AN_GEOID_HEIGHT_ID,
    AN_RTCM_CORRECTIONS_ID,
    AN_RESERVED_56_ID,
    AN_WIND_ID,
    AN_HEAVE_ID,
    AN_RESERVED_59_ID,
    AN_RAW_SATELLITE_DATA_ID,
    AN_RAW_SATELLITE_EPHEMERIS_ID,
    AN_RESERVED_62_ID,
    AN_RESERVED_63_ID,
    AN_RESERVED_64_ID,
    AN_RESERVED_65_ID,
    AN_GNSS_SUMMARY_ID,
    AN_EXTERNAL_ODOMETER_ID,
    AN_EXTERNAL_AIR_DATA_ID,
    AN_GNSS_RECEIVER_INFORMATION_ID,
    AN_RAW_DVL_DATA_ID,
    AN_NORTH_SEEKING_STATUS_ID,
    AN_GIMBAL_STATE_ID,
    AN_AUTOMOTIVE_ID,
    AN_RESERVED_74_ID,
    AN_EXTERNAL_MAGNETOMETERS_ID,
    AN_RESERVED_76_ID,
    AN_RESERVED_77_ID,
    AN_RESERVED_78_ID,
    AN_RESERVED_79_ID,
    AN_BASESTATION_ID,
    AN_RESERVED_81_ID,
    AN_RESERVED_82_ID,
    AN_ZERO_ANGULAR_VELOCITY_ID,
    AN_EXTENDED_SATELLITES_ID,
    AN_SENSOR_TEMPERATURES_ID,
    AN_SYSTEM_TEMPERATURE_ID,
    AN_RESERVED_87_ID,
    AN_END_STATE_PACKETS,

    AN_PACKET_TIMER_PERIOD_ID = AN_START_CONFIGURATION_PACKETS,
    AN_PACKET_PERIODS_ID,
    AN_BAUD_RATES_ID,
    AN_RESERVED_183_ID,
    AN_SENSOR_RANGES_ID,
    AN_INSTALLATION_ALIGNMENT_ID,
    AN_FILTER_OPTIONS_ID,
    AN_RESERVED_187_ID,
    AN_GPIO_CONFIGURATION_ID,
    AN_MAGNETIC_CALIBRATION_VALUES_ID,
    AN_MAGNETIC_CALIBRATION_CONFIGURATION_ID,
    AN_MAGNETIC_CALIBRATION_STATUS_ID,
    AN_ODOMETER_CONFIGURATION_ID,
    AN_ZERO_ALIGNMENT_ID,
    AN_REFERENCE_OFFSETS_ID,
    AN_GPIO_OUTPUT_CONFIGURATION_ID,
    AN_DUAL_ANTENNA_CONFIGURATION_ID,
    AN_GNSS_CONFIGURATION_ID,
    AN_USER_DATA_ID,
    AN_GPIO_INPUT_CONFIGURATION_ID,
    AN_RESERVED_200_ID,
    AN_RESERVED_201_ID,
    AN_IP_DATAPORTS_CONFIGURATION_ID,
    AN_CAN_CONFIGURATION_ID,
    AN_DEVICE_NAME_ID,
    AN_END_CONFIGURATION_PACKETS
};

/*
    Vehicle Type provided to Advanced Navigation EKF

    https://docs.advancednavigation.com/certus/ANPP/FilterOptionsPacket.htm?Highlight=Vehicle%20Type#Vehicle_Types
*/
enum AN_VehicleType : uint8_t
{
    VEHICLE_TYPE_UNLIMITED,
    VEHICLE_TYPE_BICYCLE,
    VEHICLE_TYPE_CAR,
    VEHICLE_TYPE_HOVERCRAFT,
    VEHICLE_TYPE_SUBMARINE,
    VEHICLE_TYPE_3D_UNDERWATER,
    VEHICLE_TYPE_FIXED_WING_PLANE,
    VEHICLE_TYPE_3D_AIRCRAFT,
    VEHICLE_TYPE_HUMAN,
    VEHICLE_TYPE_SMALL_BOAT,
    VEHICLE_TYPE_SHIP,
    VEHICLE_TYPE_STATIONARY,
    VEHICLE_TYPE_STUNT_PLANE,
    VEHICLE_TYPE_RACE_CAR
};

//////////////////////////////////////////////////////////////////////////////////////////////////
// ANPP Packets
//////////////////////////////////////////////////////////////////////////////////////////////////

/*
    ANPP Acknowledge Packet (0)

    https://docs.advancednavigation.com/certus/ANPP/SystemPackets.htm#Acknowledge_Packet
*/
struct PACKED AN_Acknowledge
{
    enum AN_AcknowledgeResult : uint8_t
    {
        AN_ACKNOWLEDGE_SUCCESS = 0,
        AN_ACKNOWLEDGE_CRC_ERROR = 1,
        AN_ACKNOWLEDGE_PACKET_SIZE_ERROR = 2,
        AN_ACKNOWLEDGE_RANGE_ERROR = 3,
        AN_ACKNOWLEDGE_SYSTEM_FLASH_ERROR = 4,
        AN_ACKNOWLEDGE_SYSTEM_NOT_READY = 5,
        AN_ACKNOWLEDGE_UNKNOWN_PACKET = 6,
    };

    AN_PacketId id_acknowledged; // packet id being acknowledge
    uint16_t crc_acknowledged;   // crc of packet being acknowledge
    AN_AcknowledgeResult result; // acknowledgement result

    const char *get_result_pretty() const
    {
        switch (result)
        {
        case AN_ACKNOWLEDGE_SUCCESS:
            return "Success [0]";
        case AN_ACKNOWLEDGE_CRC_ERROR:
            return "CRC error [1]";
        case AN_ACKNOWLEDGE_PACKET_SIZE_ERROR:
            return "Packet size incorrect [2]";
        case AN_ACKNOWLEDGE_RANGE_ERROR:
            return "Values outside of valid range [3]";
        case AN_ACKNOWLEDGE_SYSTEM_FLASH_ERROR:
            return "System flash memory failure [4]";
        case AN_ACKNOWLEDGE_SYSTEM_NOT_READY:
            return "System not ready [5]";
        case AN_ACKNOWLEDGE_UNKNOWN_PACKET:
            return "Unknown packet [6]";
        default:
            return "Unknown error";
        }
    }
};

/*
    ANPP Request Packet (1)

    The request packet can contain multiple requested packets (up to the size of
    the ANPP paylod (i.e. 255))

    https://docs.advancednavigation.com/certus/ANPP/SystemPackets.htm#Request_Packet
*/
struct PACKED AN_RequestPacket
{
    AN_PacketId packet_id[AN_PacketPayload::AN_MAXIMUM_PACKET_SIZE];

    uint8_t generate(uint8_t count, uint8_t id, ...)
    {
        va_list args;
        va_start(args, id);

        auto current_id = id;
        for (unsigned i = 0; i < count; i++)
        {
            this->packet_id[count - 1] = static_cast<AN_PacketId>(current_id);

            current_id = static_cast<AN_PacketId>(va_arg(args, int));
        }

        return count;
    }

    uint8_t generate(uint8_t count, uint8_t id, va_list args)
    {
        auto current_id = id;
        for (unsigned i = 0; i < count; i++)
        {
            this->packet_id[count - 1] = static_cast<AN_PacketId>(current_id);

            current_id = static_cast<AN_PacketId>(va_arg(args, int));
        }

        return count;
    }
};

/*
    ANPP Device Information Packet (3)

    https://docs.advancednavigation.com/certus/ANPP/SystemPackets.htm#Device_Information_Packet
*/
struct PACKED AN_DeviceInfo
{
    enum AN_DeviceId : uint8_t
    {
        UNINITIALISED = 0,
        SPATIAL = 1,
        ORIENTUS = 3,
        SPATIAL_FOG,
        SPATIAL_DUAL,
        OBDII_ODOMETER = 10,
        ORIENTUS_V3,
        ILU,
        AIR_DATA_UNIT,
        SPATIAL_FOG_DUAL = 16,
        MOTUS,
        GNSS_COMPASS = 19,
        CERTUS = 26,
        ARIES,
        BOREAS_D90,
        BOREAS_D90_FPGA = 35,
        BOREAS_COIL,
        CERTUS_MINI_A = 49,
        CERTUS_MINI_N,
        CERTUS_MINI_D,
    };

    uint32_t software_version;
    AN_DeviceId device_id;
    uint32_t hardware_revision;
    uint32_t serial_1;
    uint32_t serial_2;
    uint32_t serial_3;
};

/*
    ANPP System State Packet (20)

    Provides the EKF output of the vehicle state

    https://docs.advancednavigation.com/certus/ANPP/SystemStatePacket.htm
*/
struct PACKED AN_SystemState
{
    uint16_t system_status;
    uint16_t filter_status;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double llh[3];                   // rad,rad,m
    float velocity_ned[3];           // m/s
    float body_acceleration[3];      // m/s/s
    float g_force;                   // g's
    float rph[3];                    // rad
    float angular_velocity[3];       // rad/s
    float llh_standard_deviation[3]; // m

    enum AN_SystemStatusFlags
    {
        AN_SYSTEM_FAILURE = 1 << 0,
        AN_ACCELEROMETER_SENSOR_FAILURE = 1 << 1,
        AN_GYROSCOPE_SENSOR_FAILURE = 1 << 2,
        AN_MAGNETOMETER_SENSOR_FAILURE = 1 << 3,
        AN_PRESSURE_SENSOR_FAILURE = 1 << 4,
        AN_GNSS_FAILURE = 1 << 5,
        AN_ACCELEROMETER_OVER_RANGE = 1 << 6,
        AN_GYROSCOPE_OVER_RANGE = 1 << 7,
        AN_MAGNETOMETER_OVER_RANGE = 1 << 8,
        AN_PRESSURE_OVER_RANGE = 1 << 9,
        AN_MINIMUM_TEMPERATURE_ALARM = 1 << 10,
        AN_MAXIMUM_TEMPERATURE_ALARM = 1 << 11,
        AN_INTERNAL_DATA_LOGGING_ERROR = 1 << 12,
        AN_HIGH_VOLTAGE_ALARM = 1 << 13,
        AN_GNSS_ANTENNA_FAULT = 1 << 14,
        AN_SERIAL_PORT_OVERFLOW_ALARM = 1 << 15
    };

    enum AN_FilterStatusFlags
    {
        AN_ORIENTATION_FILTER_INITIALISED = 1 << 0,
        AN_INS_FILTER_INITIALISED = 1 << 1,
        AN_HEADING_INITIALISED = 1 << 2,
        AN_UTC_TIME_INITIALISED = 1 << 3,

        // Bits 4,5,6 = gnss_fix_type (3-bit field)
        AN_GNSS_FIX_TYPE_MASK = 0x0070, // bits 4–6
        AN_GNSS_FIX_TYPE_SHIFT = 4,     // to isolate/shift value

        AN_EVENT1_FLAG = 1 << 7,
        AN_EVENT2_FLAG = 1 << 8,
        AN_INTERNAL_GNSS_ENABLED = 1 << 9,
        AN_DUAL_ANTENNA_HEADING_ACTIVE = 1 << 10,
        AN_VELOCITY_HEADING_ENABLED = 1 << 11,
        AN_ATMOSPHERIC_ALTITUDE_ENABLED = 1 << 12,
        AN_EXTERNAL_POSITION_ACTIVE = 1 << 13,
        AN_EXTERNAL_VELOCITY_ACTIVE = 1 << 14,
        AN_EXTERNAL_HEADING_ACTIVE = 1 << 15
    };

    void get_location(Location &loc) const
    {
        loc.lat = (int32_t)(this->llh[0] * RAD_TO_DEG_DOUBLE * 1.0e7);
        loc.lng = (int32_t)(this->llh[1] * RAD_TO_DEG_DOUBLE * 1.0e7);
        loc.alt = (int32_t)(this->llh[2] * 1.0e2);
        loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    void to_log() const
    {
        Location loc;
        get_location(loc);

        // @LoggerMessage: ADS
        // @Description: Advanced Navigation State Packet
        // @Field: TimeUS: Time since system startup
        // @Field: VN: Velocity N
        // @Field: VE: Velocity E
        // @Field: VD: Velocity D
        // @Field: Roll: Roll (radians)
        // @Field: Pitch: Pitch (radians)
        // @Field: Yaw: Heading (radians)
        // @Field: Lat: Latitude (degrees)
        // @Field: Lng: Longitude (degrees)
        // @Field: Alt: Altitude (m above sea level)
        AP::logger().WriteStreaming("ADS", "TimeUS,VN,VE,VD,Roll,Pitch,Yaw,Lat,Lng,Alt",
                                    "snnndddDUm", "F000000GG0",
                                    "QffffffLLf",
                                    AP_HAL::micros64(),
                                    this->velocity_ned[0], this->velocity_ned[1], this->velocity_ned[2],
                                    degrees(this->rph[0]), degrees(this->rph[1]), degrees(this->rph[2]),
                                    loc.lat, loc.lng, loc.alt * 0.01);
    }
};

/*
    ANPP Velocity Standard Deviation Packet (25)

    https://docs.advancednavigation.com/certus/ANPP/VelocityStdDevPacket.htm
*/
struct PACKED AN_VelocityStandardDeviation
{
    float sd[3]; // VED error (m/s)

    float get_horizontal_velocity_accuracy() const
    {
        return (float)norm(
            this->sd[0],
            this->sd[1],
            this->sd[2]);
    }
};

/*
    ANPP Raw Sensors Packet (28)

    https://docs.advancednavigation.com/certus/ANPP/RawSensorsPacket.htm
*/
struct PACKED AN_RawSensors
{
public:
    float accelerometers[3];    // m/s/s
    float gyroscopes[3];        // rad/s
    float magnetometers[3];     // mG
    float imu_temperature;      // deg C
    float pressure;             // Pascals
    float pressure_temperature; // deg C

    void to_log() const
    {
        // @LoggerMessage: ADI
        // @Description: Advanced Navigation IMU Data
        // @Field: TimeUS: Time since system startup
        // @Field: Temp: Temprature
        // @Field: Pres: Pressure
        // @Field: MX: Magnetic feild X-axis
        // @Field: MY: Magnetic feild Y-axis
        // @Field: MZ: Magnetic feild Z-axis
        // @Field: AX: Acceleration X-axis
        // @Field: AY: Acceleration Y-axis
        // @Field: AZ: Acceleration Z-axis
        // @Field: GX: Rotation rate X-axis
        // @Field: GY: Rotation rate Y-axis
        // @Field: GZ: Rotation rate Z-axis
        AP::logger().WriteStreaming("ADI", "TimeUS,Temp,Pres,MX,MY,MZ,AX,AY,AZ,GX,GY,GZ",
                                    "sdPGGGoooEEE", "F00000000000",
                                    "Qfffffffffff",
                                    AP_HAL::micros64(),
                                    this->imu_temperature, this->pressure,
                                    this->magnetometers[0], this->magnetometers[1], this->magnetometers[2],
                                    this->accelerometers[0], this->accelerometers[1], this->accelerometers[2],
                                    this->gyroscopes[0], this->gyroscopes[1], this->gyroscopes[2]);
    }
};

/*
    ANPP Satellites Packet (30)

    https://docs.advancednavigation.com/certus/ANPP/SatellitesPacket.htm
 */
struct PACKED AN_Satellites
{
    float hdop;
    float vdop;
    uint8_t gps_satellites;
    uint8_t glonass_satellites;
    uint8_t beidou_satellites;
    uint8_t galileo_satellites;
    uint8_t sbas_satellites;

    uint8_t get_satellite_count() const
    {
        return this->beidou_satellites + this->galileo_satellites + this->glonass_satellites + this->gps_satellites + this->sbas_satellites;
    }

    float get_hdop() const
    {
        return this->hdop * 100; // in cm
    }

    float get_vdop() const
    {
        return this->vdop * 100; // in cm/s
    }
};

/*
    ANPP Raw GNSS Packet

    The raw data provide from the GNSS reciever.
        - Position not corrected for antenna position offset
        - Velocity not corrected for antenna lever arm offset

    https://docs.advancednavigation.com/certus/ANPP/RawGNSSPacket.htm
*/
struct PACKED AN_RawGnss
{
    uint32_t unix_time;
    uint32_t unix_microseconds;
    double llh[3];                   // rad,rad,m
    float velocity_ned[3];           // m/s
    float llh_standard_deviation[3]; // m
    float tilt;                      // rad
    float yaw;
    float tilt_sd;
    float yaw_sd;
    uint16_t status;

    enum AN_GnssStatusFlags
    {
        GNSS_FIX_TYPE_MASK = 0x0007,  // 3 bits (fix_type)
        GNSS_VELOCITY_VALID = 1 << 3, // 1 bit (velocity_valid)
        GNSS_TIME_VALID = 1 << 4,     // 1 bit (time_valid)
        GNSS_EXTERNAL_GNSS = 1 << 5,  // 1 bit (external_gnss)
        GNSS_TILT_VALID = 1 << 6,     // 1 bit (tilt_valid)
        GNSS_HEADING_VALID = 1 << 7   // 1 bit (heading_valid)
    };

    enum AN_GnssFixType
    {
        NONE,
        FIX_2D,
        FIX_3D,
        SBAS,
        DGPS,
        OMNISTAR,
        RTK_FLOAT,
        RTK_FIXED,
    };

    AP_GPS_FixType get_ap_fix_type() const
    {
        auto fix = this->status & AN_RawGnss::GNSS_FIX_TYPE_MASK;

        switch (fix)
        {
        case AN_RawGnss::AN_GnssFixType::NONE:
            return AP_GPS_FixType::NONE;
        case AN_RawGnss::AN_GnssFixType::FIX_2D:
            return AP_GPS_FixType::FIX_2D;
        case AN_RawGnss::AN_GnssFixType::FIX_3D:
            return AP_GPS_FixType::FIX_3D;
        case AN_RawGnss::AN_GnssFixType::SBAS:
        case AN_RawGnss::AN_GnssFixType::DGPS:
        case AN_RawGnss::AN_GnssFixType::OMNISTAR:
            return AP_GPS_FixType::DGPS;
        case AN_RawGnss::AN_GnssFixType::RTK_FLOAT:
            return AP_GPS_FixType::RTK_FLOAT;
        case AN_RawGnss::AN_GnssFixType::RTK_FIXED:
            return AP_GPS_FixType::RTK_FIXED;
        default:
            return AP_GPS_FixType::NONE;
        }
    }

    void get_location(Location &loc) const
    {
        loc.lat = (int32_t)(this->llh[0] * RAD_TO_DEG_DOUBLE * 1.0e7);
        loc.lng = (int32_t)(this->llh[1] * RAD_TO_DEG_DOUBLE * 1.0e7);
        loc.alt = (int32_t)(this->llh[2] * 1.0e2);
        loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    void get_gps_message(AP_ExternalAHRS::gps_data_message_t &packet, const AN_Satellites *satellites, const AN_VelocityStandardDeviation *standard_deviation) const
    {
        packet.gps_week = get_gps_week(this->unix_time);
        packet.ms_tow = get_gps_tow(this->unix_time, this->unix_microseconds);
        packet.fix_type = get_ap_fix_type();
        packet.satellites_in_view = satellites->get_satellite_count();
        packet.horizontal_pos_accuracy = this->get_horizontal_position_accuracy();
        packet.vertical_pos_accuracy = this->get_vertical_position_accuracy();
        packet.horizontal_vel_accuracy = standard_deviation->get_horizontal_velocity_accuracy();
        packet.hdop = satellites->get_hdop();
        packet.vdop = satellites->get_vdop();

        packet.ned_vel_north = this->velocity_ned[0];
        packet.ned_vel_east = this->velocity_ned[1];
        packet.ned_vel_down = this->velocity_ned[2];

        Location loc;
        get_location(loc);

        packet.latitude = loc.lat;
        packet.longitude = loc.lng;
        packet.msl_altitude = loc.alt;

        if (this->has_yaw())
        {
            packet.has_yaw = true;
            packet.yaw = degrees(this->yaw);
            packet.yaw = degrees(this->yaw_sd);
        }
    }

    bool has_yaw() const
    {
        return this->status & GNSS_HEADING_VALID;
    }

    float get_horizontal_position_accuracy() const
    {
        return (float)norm(this->llh_standard_deviation[0], this->llh_standard_deviation[1]);
    }

    float get_vertical_position_accuracy() const
    {
        return this->llh_standard_deviation[2];
    }

    void to_log(const AN_Satellites *satellites) const
    {
        Location loc;
        get_location(loc);

        // @LoggerMessage: ADG
        // @Description: Advanced Navigation GNSS packet
        // @Field: TimeUS: Time since system startup
        // @Field: VN: Velocity N
        // @Field: VE: Velocity E
        // @Field: VD: Velocity D
        // @Field: Yaw: Yaw (radians)
        // @Field: Lat: Latitude (degrees)
        // @Field: Lng: Longitude (degrees)
        // @Field: Alt: Altitude (m above sea level)
        // @Field: HDOP: Horizontal dilution of precision
        // @Field: VDOP: Vertical dilution of precision
        // @Field: HACC: Horizontal positional accuracy (m)
        // @Field: VACC: Vertical positional accuracy (m)
        AP::logger().WriteStreaming("ADG", "TimeUS,VN,VE,VD,Yaw,Lat,Lng,Alt,HDOP,VDOP,HACC,VACC",
                                    "snnndDUmnnnn", "F0000GG00000",
                                    "QffffLLfffff",
                                    AP_HAL::micros64(),
                                    this->velocity_ned[0], this->velocity_ned[1], this->velocity_ned[2],
                                    this->has_yaw() ? degrees(this->yaw) : NAN,
                                    loc.lat, loc.alt, (float)(loc.alt) * 0.01f,
                                    satellites->get_hdop(), satellites->get_vdop(),
                                    get_horizontal_position_accuracy(), get_vertical_position_accuracy());
    }
};

/**
    ANPP Packet Timer Period Packet

    https://docs.advancednavigation.com/certus/ANPP/PacketTimerPeriodPackets.htm
 */
struct AN_PacketTimerPeriod
{
    uint8_t permanent;
    uint8_t utc_synchronisation;
    uint16_t timer_period; // µs
};

/*
    ANPP Packets Period Packet

    Configure the packets to be send via the current communication channel.

    https://docs.advancednavigation.com/certus/ANPP/PacketsPeriodPacket.htm#Packets_Period_Packet
*/
struct PACKED AN_PacketPeriods
{
    struct PACKED AN_Period
    {
        AN_PacketId id;
        uint32_t packet_period; // ms
    };

    uint8_t permanent;
    uint8_t clear_existing_packet_periods;
    AN_Period periods[(AN_PacketPayload::AN_MAXIMUM_PACKET_SIZE - 2) / sizeof(AN_Period)];

    void generate(uint8_t count, ...)
    {
        va_list args;
        va_start(args, count);

        for (int i = 0; i < count; ++i)
        {
            auto packet_id = static_cast<AN_PacketId>(va_arg(args, int)); // promoted to int
            uint32_t rate = va_arg(args, uint32_t);                       // uint32_t is safe

            this->periods[i] = AN_Period{
                id : packet_id,
                packet_period : rate
            };
        }

        va_end(args);
    }

    void generate(uint8_t count, va_list args)
    {
        for (int i = 0; i < count; ++i)
        {
            auto packet_id = static_cast<AN_PacketId>(va_arg(args, int)); // promoted to int
            uint32_t rate = va_arg(args, uint32_t);                       // uint32_t is safe

            this->periods[i] = AN_Period{
                id : packet_id,
                packet_period : rate
            };
        }
    }
};

/*
    ANPP Filter Options Packet

    https://docs.advancednavigation.com/certus/ANPP/FilterOptionsPacket.htm#Filter_Options_Packet
*/
struct PACKED AN_FilterOptions
{
    uint8_t permanent;
    uint8_t vehicle_type;
    uint8_t internal_gnss_enabled;
    uint8_t magnetometers_enabled;
    uint8_t atmospheric_altitude_enabled;
    uint8_t velocity_heading_enabled;
    uint8_t reversing_detection_enabled;
    uint8_t motion_analysis_enabled;
    uint8_t automatic_magnetic_calibration_enabled;
    uint8_t dual_antenna_disabled;
    uint8_t reserved[7];

    AN_FilterOptions() {}

    AN_FilterOptions(bool gnss_en, uint8_t vehicle, bool persist)
    {
        generate(gnss_en, vehicle, persist);
    }

    void generate(bool gnss_en, uint8_t vehicle, bool persist)
    {
        this->permanent = persist;
        this->vehicle_type = vehicle;
        this->internal_gnss_enabled = gnss_en;
        this->magnetometers_enabled = false;
        this->atmospheric_altitude_enabled = true;
        this->velocity_heading_enabled = false;
        this->reversing_detection_enabled = false;
        this->motion_analysis_enabled = false;
        this->automatic_magnetic_calibration_enabled = true;
        this->dual_antenna_disabled = false;

        // set reserved packets to 0
        memset(this->reserved, 0, sizeof(this->reserved));
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////
// Static methods
//////////////////////////////////////////////////////////////////////////////////////////////////

// convert the device id to a product name
static const char *get_device_name(AN_DeviceInfo::AN_DeviceId id)
{
    switch (id)
    {
    case AN_DeviceInfo::AN_DeviceId::UNINITIALISED:
        return "Uninitialized Device ID";
    case AN_DeviceInfo::AN_DeviceId::SPATIAL:
        return "Advanced Navigation Spatial";
    case AN_DeviceInfo::AN_DeviceId::ORIENTUS:
    case AN_DeviceInfo::AN_DeviceId::ORIENTUS_V3:
        return "Advanced Navigation Orientus";
    case AN_DeviceInfo::AN_DeviceId::SPATIAL_FOG:
        return "Advanced Navigation Spatial FOG";
    case AN_DeviceInfo::AN_DeviceId::SPATIAL_DUAL:
        return "Advanced Navigation Spatial Dual";
    case AN_DeviceInfo::AN_DeviceId::ILU:
        return "Advanced Navigation Interface Logging Unit";
    case AN_DeviceInfo::AN_DeviceId::AIR_DATA_UNIT:
        return "Advanced Navigation Air Data Unit";
    case AN_DeviceInfo::AN_DeviceId::SPATIAL_FOG_DUAL:
        return "Advanced Navigation Spatial FOG Dual";
    case AN_DeviceInfo::AN_DeviceId::MOTUS:
        return "Advanced Navigation Motus";
    case AN_DeviceInfo::AN_DeviceId::GNSS_COMPASS:
        return "Advanced Navigation GNSS Compass";
    case AN_DeviceInfo::AN_DeviceId::CERTUS:
        return "Advanced Navigation Certus";
    case AN_DeviceInfo::AN_DeviceId::ARIES:
        return "Advanced Navigation Aries";
    case AN_DeviceInfo::AN_DeviceId::BOREAS_D90:
    case AN_DeviceInfo::AN_DeviceId::BOREAS_D90_FPGA:
    case AN_DeviceInfo::AN_DeviceId::BOREAS_COIL:
        return "Advanced Navigation Boreas";
    case AN_DeviceInfo::AN_DeviceId::CERTUS_MINI_A:
        return "Advanced Navigation Certus Mini A";
    case AN_DeviceInfo::AN_DeviceId::CERTUS_MINI_N:
        return "Advanced Navigation Certus Mini N";
    case AN_DeviceInfo::AN_DeviceId::CERTUS_MINI_D:
        return "Advanced Navigation Certus Mini D";
    case AN_DeviceInfo::AN_DeviceId::OBDII_ODOMETER:
        return "Advanced Navigation OBDII Odometer";
    default:
        return "Unknown Advanced Navigation Device ID";
    }
}

static uint32_t get_epoch_timestamp(uint32_t unix_timestamp)
{
    const uint32_t leapseconds = 18U;
    const uint32_t epoch = 86400 * (10 * 365 + (1980 - 1969) / 4 + 1 + 6 - 2) - leapseconds;
    return unix_timestamp - epoch;
}

static uint16_t get_gps_week(uint32_t unix_timestamp)
{
    return get_epoch_timestamp(unix_timestamp) / AP_SEC_PER_WEEK;
}

// return time of week to the nearest 200ms
static uint16_t get_gps_tow(uint32_t unix_timestamp, uint32_t microseconds)
{
    return (get_epoch_timestamp(unix_timestamp) % AP_SEC_PER_WEEK) * AP_SEC_PER_WEEK + ((microseconds / 200) * 200);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// AP_ExternalAHRS_AdvancedNavigation_Decoder public methods
//////////////////////////////////////////////////////////////////////////////////////////////////

// load the decoder buffer with availble messages from the provided uart
// returns the number of bytes loaded
ssize_t AP_ExternalAHRS_AdvancedNavigation_Decoder::read(AP_HAL::UARTDriver *uart)
{
    if (uart == nullptr)
    {
        return -1;
    }

    if (uart->available() <= 0)
    {
        return 0;
    }

    auto bytes = uart->read(&_buffer[_buffer_length], sizeof(_buffer) - _buffer_length);

    _buffer_length += bytes;
    _complete = false;

    return bytes;
}

// decode all messages in the decoder buffer
// return false if unsuccesfull
bool AP_ExternalAHRS_AdvancedNavigation_Decoder::decode(AN_Packet *packet)
{
    uint16_t offset = 0;

    AN_PacketHeader header;

    // Iterate through buffer until no more headers could be in buffer
    while (offset + sizeof(AN_PacketHeader) <= _buffer_length)
    {
        memcpy(&header, _buffer + offset, sizeof(AN_PacketHeader));

        if (header.check_lrc())
        {
            if (offset + sizeof(AN_PacketHeader) + header.length > _buffer_length)
            {
                // prevent buffer overflow
                break;
            }

            offset += sizeof(AN_PacketHeader);

            // valid header --> check for valid packet
            if (header.check_crc(_buffer, sizeof(_buffer), offset))
            {

                if ((header.length + sizeof(AN_PacketHeader)) > sizeof(AN_Packet))
                {
                    // prevent buffer overflow
                    return false;
                }

                packet->load(&_buffer[offset - sizeof(AN_PacketHeader)], sizeof(AN_PacketHeader) + header.length * sizeof(uint8_t));

                offset += header.length;
                break;
            }
            else
            {
                // invalid packet for the given header
                offset -= (sizeof(AN_PacketHeader) - 1);
            }
        }
        else
        {
            offset++;
        }
    }

    if (offset < _buffer_length)
    {
        if (offset > 0)
        {
            // move remaining buffer to the beginning
            memmove(&_buffer[0], &_buffer[offset], (_buffer_length - offset) * sizeof(uint8_t));

            _buffer_length -= offset;
            _complete = false;
            return true;
        }
    }
    else
    {
        _buffer_length = 0;
    }

    _complete = true;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// AP_ExternalAHRS_AdvancedNavigation public methods
//////////////////////////////////////////////////////////////////////////////////////////////////

AP_ExternalAHRS_AdvancedNavigation::AP_ExternalAHRS_AdvancedNavigation(AP_ExternalAHRS *_frontend,
                                                                       AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state), dal(AP::dal())
{
    auto &sm = AP::serialmanager();
    _uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, AN_SERIAL_INSTANCE);
    if (_uart == nullptr)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS no UART");
        return;
    }

    _baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, AN_SERIAL_INSTANCE);
    _port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, AN_SERIAL_INSTANCE);

    // don't offer IMU by default, at 50Hz it is too slow
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AdvancedNavigation::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0))
    {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
}

int8_t AP_ExternalAHRS_AdvancedNavigation::get_port(void) const
{
    return _uart == nullptr ? -1 : _port_num;
};

const char *AP_ExternalAHRS_AdvancedNavigation::get_name() const
{
    return "AdNav";
}

bool AP_ExternalAHRS_AdvancedNavigation::healthy(void) const
{
    return ((AP_HAL::millis() - _last_state_pkt_ms) < AN_STATE_PACKET_MAX_DELAY);
}

bool AP_ExternalAHRS_AdvancedNavigation::initialised(void) const
{
    return _last_state_pkt_ms != 0 && _last_device_info_pkt_ms != 0 && (has_gnss() ? _last_raw_gnss_pkt_ms != 0 : true);
}

bool AP_ExternalAHRS_AdvancedNavigation::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (AP_HAL::millis() - _last_pkt_ms > AN_TIMEOUT)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "AdNav: No Connection (last packet %8ums ago)", (unsigned int)(AP_HAL::millis() - _last_pkt_ms));
        return false;
    }
    if (!healthy())
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Device unhealthy");
        return false;
    }
    if (_system_status & AN_SystemState::AN_SystemStatusFlags::AN_GNSS_FAILURE)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "GNSS Failure");
        return false;
    }
    if (_system_status & AN_SystemState::AN_SystemStatusFlags::AN_SYSTEM_FAILURE)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "System Failure");
        return false;
    }
    if (_system_status & AN_SystemState::AN_SystemStatusFlags::AN_ACCELEROMETER_SENSOR_FAILURE)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Accelerometer Failure");
        return false;
    }
    if (_system_status & AN_SystemState::AN_SystemStatusFlags::AN_GYROSCOPE_SENSOR_FAILURE)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Gyroscope Failure");
        return false;
    }
    if (_system_status & AN_SystemState::AN_SystemStatusFlags::AN_MAGNETOMETER_SENSOR_FAILURE)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Magnetometer Failure");
        return false;
    }
    if (_system_status & AN_SystemState::AN_SystemStatusFlags::AN_PRESSURE_SENSOR_FAILURE)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Barometer Failure");
        return false;
    }
    if ((((_filter_status & AN_SystemState::AN_FilterStatusFlags::AN_GNSS_FIX_TYPE_MASK) >> AN_SystemState::AN_FilterStatusFlags::AN_GNSS_FIX_TYPE_SHIFT) < 1) && has_gnss())
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "No GPS lock");
        return false;
    }
    if (!(_filter_status & AN_SystemState::AN_FilterStatusFlags::AN_ORIENTATION_FILTER_INITIALISED))
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Orientation Filter Not Initialised");
        return false;
    }
    if (!(_filter_status & AN_SystemState::AN_FilterStatusFlags::AN_INS_FILTER_INITIALISED))
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "INS Filter Not Initialised");
        return false;
    }
    if (!(_filter_status & AN_SystemState::AN_FilterStatusFlags::AN_HEADING_INITIALISED))
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "Heading Filter Not Initialised");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_AdvancedNavigation::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (_last_state_pkt_ms == 0)
    {
        return;
    }

    status.flags.initalized = true;

    if (!healthy())
    {
        return;
    }

    status.flags.vert_pos = true;
    status.flags.attitude = true;
    status.flags.vert_vel = true;

    uint8_t gnss_fix_type = (_filter_status & AN_SystemState::AN_FilterStatusFlags::AN_GNSS_FIX_TYPE_MASK) >> AN_SystemState::AN_FilterStatusFlags::AN_GNSS_FIX_TYPE_SHIFT;

    if (gnss_fix_type > AN_RawGnss::AN_GnssFixType::NONE)
    {
        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }

    if (gnss_fix_type > AN_RawGnss::AN_GnssFixType::FIX_2D)
    {
        status.flags.gps_quality_good = true;
    }
}

void AP_ExternalAHRS_AdvancedNavigation::update()
{
    if (!get_packets())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Error Receiving Packets");
    }
}

// calculate the EKF variances
bool AP_ExternalAHRS_AdvancedNavigation::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    const auto *velocity_standard_deviation = reinterpret_cast<const AN_VelocityStandardDeviation *>(&last_velocity_standard_deviation_packet.payload);
    const auto *raw_gnss = reinterpret_cast<const AN_RawGnss *>(&last_raw_gnss_packet.payload);

    velVar = velocity_standard_deviation->get_horizontal_velocity_accuracy() * vel_gate_scale;
    posVar = raw_gnss->get_horizontal_position_accuracy() * pos_gate_scale;
    hgtVar = raw_gnss->get_vertical_position_accuracy() * hgt_gate_scale;
    tasVar = 0;
    magVar = Vector3f{0, 0, 0};

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// AP_ExternalAHRS_AdvancedNavigation private methods
//////////////////////////////////////////////////////////////////////////////////////////////////

void AP_ExternalAHRS_AdvancedNavigation::update_thread(void)
{
    auto tconnnection = AP_HAL::millis();
    auto tpackets = AP_HAL::millis();
    unsigned attempts = 0;

    // open the port in the thread
    _uart->begin(_baudrate);

    while (true)
    {
        auto now = AP_HAL::millis();
        auto init = initialised();

        if (now - tconnnection >= AN_TIMEOUT)
        {
            // If unable to initialise then send errors to GCS periodically
            if (!init && attempts > AN_MIN_CONNECTION_ATTEMPTS)
            {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Advanced Navigation Device Unresponsive");
            }

            // Request device information periodically
            if (!configure_device())
            {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Request Data Error");
            }

            attempts++;
            tconnnection = now;
        }

        hal.scheduler->delay_microseconds(AN_UPDATE_THREAD_PERIOD);

        // Always check for packets, even if not initalised
        // This ensures that packets are still processed even if TX fails
        if (!get_packets() && now - tpackets > AN_GCS_TIMEOUT)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Error Receiving Packets");
        }
    }
}

// read all messages from the uart and decode them into ANPP packets
bool AP_ExternalAHRS_AdvancedNavigation::get_packets(void)
{
    // Must be single threaded
    WITH_SEMAPHORE(_sem);

    auto bytes = _decoder.read(_uart);

    if (bytes > 0)
    {
        while (!_decoder.is_complete())
        {
            if (!_decoder.decode(&packet))
            {
                return false;
            }

            handle_packet();
        }
    }

    return bytes >= 0;
}

// request the packets with the provided identifiers
// return true if successfull
bool AP_ExternalAHRS_AdvancedNavigation::send_packet_request(uint8_t count, uint8_t id, ...)
{
    AN_RequestPacket request_packet;

    va_list args;
    va_start(args, id);

    request_packet.generate(count, id, args);

    va_end(args);

    AN_Packet message(AN_REQUEST_ID, (uint8_t *)&request_packet, count * sizeof(uint8_t));

    return send_packet(message);
}

// configure the packet periods over the current communications channel
// input arguments should be in the form (uint8_t count, AN_PacketId id1, uint32_t period1, ...)
// returns true if successfull
bool AP_ExternalAHRS_AdvancedNavigation::send_packet_period_request(uint8_t count, ...)
{
    AN_PacketPeriods packet_periods{
        permanent : true,
        clear_existing_packet_periods : true,
    };

    va_list args;
    va_start(args, count);

    packet_periods.generate(count, args);

    va_end(args);

    AN_Packet message(AN_PACKET_PERIODS_ID, (uint8_t *)&packet_periods, 2 + (count * sizeof(AN_PacketPeriods::AN_Period)));

    return send_packet(message);
}

// set the packet timer period
// returns true if successfull
bool AP_ExternalAHRS_AdvancedNavigation::send_packet_timer_period(uint16_t timer_period)
{
    AN_PacketTimerPeriod packet_timer_period{
        permanent : true,
        utc_synchronisation : true,
        timer_period : timer_period,
    };

    AN_Packet message(AN_PACKET_TIMER_PERIOD_ID, (uint8_t *)&packet_timer_period, sizeof(AN_PacketTimerPeriod));

    return send_packet(message);
}

// configure the device to provide the necessary packets and rates required for the
// ExternalAHRS driver
// returns true if successfull
bool AP_ExternalAHRS_AdvancedNavigation::configure_device(void)
{
    auto now = AP_HAL::millis();

    if (!_last_device_info_pkt_ms || (now - _last_device_info_pkt_ms > AN_DEVICE_INFORMATION_UPDATE_PERIOD))
    {
        if (!send_packet_request(1, AN_DEVICE_INFORMATION_ID))
        {
            return false;
        }
    }

    // if not set already, send packet period request message
    if ((!_rates_set || _current_rate != get_rate()) && (!_last_pkt_rate_message_sent_ms || now - _last_pkt_rate_message_sent_ms > AN_DEVICE_PACKET_RATE_UPDATE_PERIOD))
    {
        if (!send_packet_timer_period(AN_PACKET_TIMER_PERIODS_RATE))
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Failure to send timer request");
        }

        // Update the current rate
        _current_rate = get_rate();

        if (!send_packet_period_request(5, AN_SYSTEM_STATE_ID, (uint32_t)1.0e3 / _current_rate, AN_VELOCITY_STANDARD_DEVIATION_ID, (uint32_t)1.0e3 / _current_rate, AN_RAW_SENSORS_ID, (uint32_t)1.0e3 / _current_rate, AN_RAW_GNSS_ID, (uint32_t)1.0e3 / AN_GNSS_PACKET_RATE, AN_SATELLITES_ID, (uint32_t)1.0e3 / AN_GNSS_PACKET_RATE))
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Failure to send packet request");
        }

        _last_pkt_rate_message_sent_ms = AP_HAL::millis();
    }

    // update filter options if GPS_DISABLE has been changed
    if (_gnss_disable != gnss_is_disabled())
    {
        _gnss_disable = gnss_is_disabled();

        AN_VehicleType vehicle_type = AN_VehicleType::VEHICLE_TYPE_3D_AIRCRAFT;

        set_filter_options(!_gnss_disable, vehicle_type);
    }

    return true;
}

// return whether Advanced Navigation device has gps capability
bool AP_ExternalAHRS_AdvancedNavigation::has_gnss(void) const
{
    switch (_device_id)
    {
    case AN_DeviceInfo::AN_DeviceId::ORIENTUS:
    case AN_DeviceInfo::AN_DeviceId::ORIENTUS_V3:
    case AN_DeviceInfo::AN_DeviceId::AIR_DATA_UNIT:
    case AN_DeviceInfo::AN_DeviceId::MOTUS:
    case AN_DeviceInfo::AN_DeviceId::CERTUS_MINI_A:
        return false;
    default:
        return true;
    }
}

// return whether Advanced Navigation device has barometric capability
bool AP_ExternalAHRS_AdvancedNavigation::has_baro(void) const
{
    switch (_device_id)
    {
    case AN_DeviceInfo::AN_DeviceId::AIR_DATA_UNIT:
    case AN_DeviceInfo::AN_DeviceId::ORIENTUS:
    case AN_DeviceInfo::AN_DeviceId::ORIENTUS_V3:
    case AN_DeviceInfo::AN_DeviceId::GNSS_COMPASS:
    case AN_DeviceInfo::AN_DeviceId::CERTUS_MINI_A:
        return false;
    case AN_DeviceInfo::AN_DeviceId::MOTUS:
        // Motus versions prior to 2.3 didn't have a barometer enabled.
        if (_hardware_rev < 2300)
        {
            return false;
        }
        break;
    default:
        break;
    }
    return true;
}

// return whether Advanced Navigation device has compass capability
bool AP_ExternalAHRS_AdvancedNavigation::has_compass(void) const
{
    switch (_device_id)
    {
    case AN_DeviceInfo::AN_DeviceId::AIR_DATA_UNIT:
    case AN_DeviceInfo::AN_DeviceId::BOREAS_D90:
    case AN_DeviceInfo::AN_DeviceId::BOREAS_D90_FPGA:
    case AN_DeviceInfo::AN_DeviceId::BOREAS_COIL:
        return false;
    default:
        break;
    }
    return true;
}

// send ANPP packet out
// return true if successfull
bool AP_ExternalAHRS_AdvancedNavigation::send_packet(AN_Packet &an_packet)
{
    if (_uart->txspace() < an_packet.size())
    {
        return false;
    }

    _uart->write(an_packet.base(), an_packet.size());

    return true;
}

// set the EKF options
// return true if successfull
bool AP_ExternalAHRS_AdvancedNavigation::set_filter_options(bool gnss_en, uint8_t vehicle_type, bool permanent)
{
    AN_FilterOptions options(gnss_en, vehicle_type, permanent);

    AN_Packet message(AN_FILTER_OPTIONS_ID, (uint8_t *)&options, sizeof(AN_FilterOptions));

    return send_packet(message);
}

// handler for the recieved ANPP packets
void AP_ExternalAHRS_AdvancedNavigation::handle_packet()
{
    _last_pkt_ms = AP_HAL::millis();

    switch (packet.id())
    {
    case AN_ACKNOWLEDGE_ID:
    {
        handle_acknowledge_packet();
        break;
    }
    case AN_DEVICE_INFORMATION_ID:
    {
        handle_device_info_packet();
        break;
    }
    case AN_SYSTEM_STATE_ID:
    {
        handle_system_state_packet();
        break;
    }
    case AN_VELOCITY_STANDARD_DEVIATION_ID:
    {
        handle_velocity_standard_deviation_packet();
        break;
    }
    case AN_RAW_SENSORS_ID:
    {
        handle_raw_sensors_packet();
        break;
    }
    case AN_RAW_GNSS_ID:
    {
        handle_raw_gnss_packet();
        break;
    }
    case AN_SATELLITES_ID:
    {
        handle_satellites_packet();
        break;
    }
    default:
    {
        break;
    }
    }
}

void AP_ExternalAHRS_AdvancedNavigation::handle_system_state_packet()
{
    auto now = AP_HAL::millis();
    auto system_state = reinterpret_cast<const AN_SystemState *>(&packet.payload);

    // Save the status
    _system_status = system_state->system_status;
    _filter_status = system_state->filter_status;

    _last_state_pkt_ms = now;

    WITH_SEMAPHORE(state.sem);

    state.have_velocity = true;
    state.velocity = Vector3f{
        system_state->velocity_ned[0],
        system_state->velocity_ned[1],
        system_state->velocity_ned[2]};

    state.have_quaternion = true;
    state.quat.from_euler(
        system_state->rph[0],
        system_state->rph[1],
        system_state->rph[2]);

    if (has_gnss())
    {
        Location loc;
        system_state->get_location(loc);
        state.have_location = true;
        state.location = loc;
        state.last_location_update_us = AP_HAL::micros();

        if (!state.have_origin)
        {
            state.origin = loc;
            state.have_origin = true;
        }
    }

#if HAL_LOGGING_ENABLED
    if (log_rate() > 0 && now - _last_logged_state_pkt_ms >= uint32_t(1000U / log_rate()))
    {
        system_state->to_log();
        _last_logged_state_pkt_ms = now;
    }
#endif // HAL_LOGGING_ENABLED
}

void AP_ExternalAHRS_AdvancedNavigation::handle_acknowledge_packet()
{
    auto *acknowledge = reinterpret_cast<const AN_Acknowledge *>(&packet.payload);

    if (acknowledge->id_acknowledged == AN_PACKET_PERIODS_ID)
    {
        _rates_set = true;
    }
}

void AP_ExternalAHRS_AdvancedNavigation::handle_device_info_packet()
{
    _last_device_info_pkt_ms = AP_HAL::millis();
    auto *device_info = reinterpret_cast<const AN_DeviceInfo *>(&packet.payload);

    if (_device_id == AN_DeviceInfo::AN_DeviceId::UNINITIALISED && device_info->device_id != AN_DeviceInfo::AN_DeviceId::UNINITIALISED)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS: %s found", get_device_name(device_info->device_id));
    }

    _device_id = device_info->device_id;
    _hardware_rev = device_info->hardware_revision;
}

void AP_ExternalAHRS_AdvancedNavigation::handle_velocity_standard_deviation_packet()
{
    last_velocity_standard_deviation_packet.copy(&packet);
}

void AP_ExternalAHRS_AdvancedNavigation::handle_raw_sensors_packet()
{
    auto now = AP_HAL::millis();

    _last_raw_sensors_pkt_ms = now;

    auto raw_sensors = reinterpret_cast<const AN_RawSensors *>(&packet.payload);
    {
        WITH_SEMAPHORE(state.sem);

        state.accel = Vector3f{
            raw_sensors->accelerometers[0],
            raw_sensors->accelerometers[1],
            raw_sensors->accelerometers[2]};

        state.gyro = Vector3f{
            raw_sensors->gyroscopes[0],
            raw_sensors->gyroscopes[1],
            raw_sensors->gyroscopes[2]};
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    if (has_baro())
    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = raw_sensors->pressure;
        baro.temperature = raw_sensors->pressure_temperature;

        AP::baro().handle_external(baro);
    };
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (has_compass())
    {
        AP_ExternalAHRS::mag_data_message_t mag;

        mag.field = Vector3f{
            raw_sensors->magnetometers[0],
            raw_sensors->magnetometers[1],
            raw_sensors->magnetometers[2]};

        AP::compass().handle_external(mag);
    }
#endif

    AP_ExternalAHRS::ins_data_message_t ins;

    ins.accel = state.accel;
    ins.gyro = state.gyro;
    ins.temperature = raw_sensors->imu_temperature;

    AP::ins().handle_external(ins);

#if HAL_LOGGING_ENABLED
    if (log_rate() > 0 && now - _last_logged_raw_sensors_pkt_ms >= uint32_t(1000U / log_rate()))
    {
        raw_sensors->to_log();
        _last_logged_raw_sensors_pkt_ms = now;
    }
#endif // HAL_LOGGING_ENABLED
}

void AP_ExternalAHRS_AdvancedNavigation::handle_raw_gnss_packet()
{
    auto now = AP_HAL::millis();
    _last_raw_gnss_pkt_ms = now;

    last_raw_gnss_packet.copy(&packet);

    auto raw_gnss = reinterpret_cast<const AN_RawGnss *>(&packet.payload);

    auto *satellites = reinterpret_cast<const AN_Satellites *>(&last_satellites_packet.payload);

    auto *velocity_standard_deviation = reinterpret_cast<const AN_VelocityStandardDeviation *>(&last_velocity_standard_deviation_packet.payload);

    AP_ExternalAHRS::gps_data_message_t gps_message;

    raw_gnss->get_gps_message(gps_message, satellites, velocity_standard_deviation);

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance))
    {
        AP::gps().handle_external(gps_message, instance);
    }

#if HAL_LOGGING_ENABLED
    if (log_rate() > 0 && now - _last_logged_raw_gnss_pkt_ms >= uint32_t(1000U / log_rate()))
    {
        raw_gnss->to_log(satellites);
        _last_logged_raw_gnss_pkt_ms = now;
    }
#endif // HAL_LOGGING_ENABLED

    _last_raw_gnss_pkt_ms = now;
}

void AP_ExternalAHRS_AdvancedNavigation::handle_satellites_packet()
{
    last_satellites_packet.copy(&packet);
}

#endif // AP_EXTERNAL_AHRS_ADNAV_ENABLED