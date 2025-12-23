#include <AP_gtest.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_SensAItion_Parser.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace
{
// Units definitions Kebni
constexpr float UG_PER_MSS = 1e6f / 9.80665f;
constexpr float UDEGS_PER_RADS = 1e6f * 180.0f / 3.1415926f;
constexpr float MHPA_PER_PA = 1e3f / 100.0f;

using Parser = AP_ExternalAHRS_SensAItion_Parser;
}

// --- HELPER FUNCTIONS ---
static bool is_equal(const float f1, const float f2, const float eps)
{
    return (fabs(f1 - f2) < eps);
}

static void fill_be32(uint8_t* data, size_t& loc, int32_t val)
{
    data[loc++] = (val >> 24) & 0xFF;
    data[loc++] = (val >> 16) & 0xFF;
    data[loc++] = (val >> 8) & 0xFF;
    data[loc++] = val & 0xFF;
}

static void fill_be16(uint8_t* data, size_t& loc, int16_t val)
{
    data[loc++] = (val >> 8) & 0xFF;
    data[loc++] = val & 0xFF;
}

static void fill_u8(uint8_t* data, size_t& loc, uint8_t val)
{
    data[loc++] = val;
}

// --- UPDATED STRUCT (Matches Parser + Date/Week Support) ---
struct Measurement {
    Parser::MeasurementType type;

    // Packet 0 (IMU) Data
    Vector3f acceleration_mss;
    Vector3f angular_velocity_rads;
    Vector3f magnetic_field_mgauss;
    float temperature_degc;
    float air_pressure_p;

    // Packet 1 (Orientation) Data
    Quaternion orientation;

    // Packet 2 (INS) Data
    Location location;          // Lat/Lon/Alt
    Vector3f velocity_ned;      // North/East/Down m/s

    // Accuracy Metrics (Vectors)
    Vector3f pos_accuracy;      // X=Lat, Y=Lon, Z=Alt (meters)
    Vector3f vel_accuracy;      // X=VelN, Y=VelE, Z=VelD (m/s)

    // Status & Health Flags
    uint8_t alignment_status;   // 1 = Align OK
    uint8_t gnss1_fix;
    uint8_t gnss2_fix;

    uint8_t num_sats_gnss1;
    uint8_t num_sats_gnss2;

    // Time & Date (Input for Generator)
    uint32_t time_itow_ms;
    uint16_t year;
    uint8_t month;
    uint8_t day;

    // The Calculated Result (Output from Parser)
    uint16_t gps_week;

    uint32_t error_flags;       // Bitmask from sensor
    uint8_t sensor_valid;       // Byte 49 (New in v5)
};

// --- UPDATED GENERATOR (Matches 69-Byte Parser Layout) ---
static void fill_simulated_packet(uint8_t* data, size_t& data_length,
                                  const Measurement& m,
                                  Parser::ConfigMode mode)
{
    size_t idx = 0;

    // 1. Header & ID
    data[idx++] = 0xFA;
    if (mode == Parser::ConfigMode::INTERLEAVED_INS) {
        switch (m.type) {
        case Parser::MeasurementType::IMU:
            data[idx++] = 0x00;
            break;
        case Parser::MeasurementType::AHRS:
            data[idx++] = 0x01;
            break;
        case Parser::MeasurementType::INS:
            data[idx++] = 0x02;
            break;
        default:
            break;
        }
    }

    // 2. Payload
    if (m.type == Parser::MeasurementType::IMU) {
        fill_be32(data, idx, m.acceleration_mss.x * UG_PER_MSS);
        fill_be32(data, idx, m.acceleration_mss.y * UG_PER_MSS);
        fill_be32(data, idx, m.acceleration_mss.z * UG_PER_MSS);
        fill_be32(data, idx, m.angular_velocity_rads.x * UDEGS_PER_RADS);
        fill_be32(data, idx, m.angular_velocity_rads.y * UDEGS_PER_RADS);
        fill_be32(data, idx, m.angular_velocity_rads.z * UDEGS_PER_RADS);
        fill_be16(data, idx, (int16_t)((m.temperature_degc - 20.0f) / 0.008f));
        fill_be16(data, idx, m.magnetic_field_mgauss.x);
        fill_be16(data, idx, m.magnetic_field_mgauss.y);
        fill_be16(data, idx, m.magnetic_field_mgauss.z);
        fill_be32(data, idx, m.air_pressure_p * MHPA_PER_PA);

    } else if (m.type == Parser::MeasurementType::AHRS) {
        fill_be32(data, idx, m.orientation.q1 * 1e6);
        fill_be32(data, idx, m.orientation.q2 * 1e6);
        fill_be32(data, idx, m.orientation.q3 * 1e6);
        fill_be32(data, idx, m.orientation.q4 * 1e6);

    } else if (m.type == Parser::MeasurementType::INS) {
        // --- 69-BYTE LAYOUT ---

        // 0-3: Sats (Big Endian of 2 shorts)
        // GNSS2 is first Short, GNSS1 is second Short
        // We put values in the LSB of each Short: [00][Count]
        fill_u8(data, idx, 0);                  // GNSS2 Hi
        fill_u8(data, idx, m.num_sats_gnss2);   // GNSS2 Lo
        fill_u8(data, idx, 0);                  // GNSS1 Hi
        fill_u8(data, idx, m.num_sats_gnss1);   // GNSS1 Lo

        // 4-7: Flags
        fill_be32(data, idx, m.error_flags);

        // 8: Valid
        fill_u8(data, idx, m.sensor_valid);

        // 9-32: Nav Data
        fill_be32(data, idx, m.location.lat);
        fill_be32(data, idx, m.location.lng);

        // Velocity N, E, D
        fill_be32(data, idx, m.velocity_ned.x * 1000);
        fill_be32(data, idx, m.velocity_ned.y * 1000);
        fill_be32(data, idx, m.velocity_ned.z * 1000);

        // Altitude
        fill_be32(data, idx, m.location.alt * 10); // cm -> mm

        // 33: Alignment
        fill_u8(data, idx, m.alignment_status);

        // 34-37: iTOW
        fill_be32(data, idx, m.time_itow_ms);

        // 38-39: GNSS Fix (Mask 5 -> 2 Bytes)
        // Wire: [GNSS2][GNSS1]
        fill_u8(data, idx, m.gnss2_fix);
        fill_u8(data, idx, m.gnss1_fix);

        // 40-43: UTC Date (Mask F -> 4 Bytes)
        // Wire: [Y_MSB][Y_LSB][Pad][Month]
        fill_u8(data, idx, (m.year >> 8) & 0xFF);
        fill_u8(data, idx, m.year & 0xFF);
        fill_u8(data, idx, 0); // Pad
        fill_u8(data, idx, m.month);

        // 44: UTC Time (Mask 8 -> 1 Byte)
        // Wire: [Day]
        fill_u8(data, idx, m.day);

        // 45-68: Accuracy (6 x 4 Bytes)
        // Order: Lat, Lon, VelN, VelE, VelD, PosD
        fill_be32(data, idx, m.pos_accuracy.x * 1000); // Lat Acc
        fill_be32(data, idx, m.pos_accuracy.y * 1000); // Lon Acc
        fill_be32(data, idx, m.vel_accuracy.x * 1000); // Vel N
        fill_be32(data, idx, m.vel_accuracy.y * 1000); // Vel E
        fill_be32(data, idx, m.vel_accuracy.z * 1000); // Vel D
        fill_be32(data, idx, m.pos_accuracy.z * 1000); // Pos D
    }

    // 4. CRC
    uint8_t checksum = 0;
    for (size_t i = 1; i < idx; ++i) {
        checksum ^= data[i];
    }
    data[idx++] = checksum;

    data_length = idx;
}

// --- DEFAULT FACTORY ---
static Measurement default_measurement(Parser::MeasurementType type)
{
    Measurement m = {};
    m.type = type;
    if (type == Parser::MeasurementType::IMU) {
        m.acceleration_mss.x = 4.0f;
        m.acceleration_mss.y = 5.0f;
        m.acceleration_mss.z = 6.0f;
        m.angular_velocity_rads.x = 0.1f;
        m.angular_velocity_rads.y = 0.2f;
        m.angular_velocity_rads.z = 0.3f;
        m.temperature_degc = 56.0f;
        m.magnetic_field_mgauss.x = 31.0f;
        m.magnetic_field_mgauss.y = 41.0f;
        m.magnetic_field_mgauss.z = 51.0f;
        m.air_pressure_p = 1235.0f;
    }
    if (type == Parser::MeasurementType::INS) {
        m.location.lat = 590000000;
        m.location.lng = 180000000;
        m.location.alt = 5000;
        m.velocity_ned = Vector3f(1, 0, 0);

        m.pos_accuracy = Vector3f(0.5f, 0.5f, 0.8f);
        m.vel_accuracy = Vector3f(0.1f, 0.1f, 0.1f);

        m.alignment_status = 1;
        m.gnss1_fix = 3;
        m.gnss2_fix = 0;
        m.num_sats_gnss1 = 12;
        m.time_itow_ms = 1000;
        m.year = 2025;
        m.month = 12;
        m.day = 10;
        m.sensor_valid = true;
    }
    return m;
}
static bool cmp_packages(Measurement& in, Parser::Measurement& out)
{
    if (in.type != out.type) {
        return false;
    }
    //

    //
    switch (in.type) {
    case Parser::MeasurementType::IMU:
        if (!is_equal(in.acceleration_mss[0], out.acceleration_mss[0], 0.00001f) ||
            !is_equal(in.acceleration_mss[1], out.acceleration_mss[1], 0.00001f) ||
            !is_equal(in.acceleration_mss[2], out.acceleration_mss[2], 0.00001f) ||
            !is_equal(in.angular_velocity_rads[0], out.angular_velocity_rads[0], 0.000001f) ||
            !is_equal(in.angular_velocity_rads[1], out.angular_velocity_rads[1], 0.000001f) ||
            !is_equal(in.angular_velocity_rads[2], out.angular_velocity_rads[2], 0.000001f) ||
            !is_equal(in.magnetic_field_mgauss[0], out.magnetic_field_mgauss[0]) ||
            !is_equal(in.magnetic_field_mgauss[1], out.magnetic_field_mgauss[1]) ||
            !is_equal(in.magnetic_field_mgauss[2], out.magnetic_field_mgauss[2]) ||
            !is_equal(in.temperature_degc, out.temperature_degc) ||
            !is_equal(in.air_pressure_p, out.air_pressure_p)) {
            return false;
        }
        break;
    case Parser::MeasurementType::AHRS:
        if (!is_equal(in.orientation.q1, out.orientation.q1) ||
            !is_equal(in.orientation.q2, out.orientation.q2) ||
            !is_equal(in.orientation.q3, out.orientation.q3) ||
            !is_equal(in.orientation.q4, out.orientation.q4)) {
            return false;
        }
        break;
    case Parser::MeasurementType::INS:
        if (in.location.lat != out.location.lat ||
            in.location.lng != out.location.lng ||
            in.location.alt != out.location.alt ||
            !is_equal(in.velocity_ned[0], out.velocity_ned[0]) ||
            !is_equal(in.velocity_ned[1], out.velocity_ned[1]) ||
            !is_equal(in.velocity_ned[2], out.velocity_ned[2]) ||
            !is_equal(in.pos_accuracy[0], out.pos_accuracy[0]) ||
            !is_equal(in.pos_accuracy[1], out.pos_accuracy[1]) ||
            !is_equal(in.pos_accuracy[2], out.pos_accuracy[2]) ||
            !is_equal(in.vel_accuracy[0], out.vel_accuracy[0]) ||
            !is_equal(in.vel_accuracy[1], out.vel_accuracy[1]) ||
            !is_equal(in.vel_accuracy[2], out.vel_accuracy[2]) ||
            in.alignment_status != out.alignment_status ||
            in.gnss1_fix != out.gnss1_fix ||
            in.gnss2_fix != out.gnss2_fix ||
            in.num_sats_gnss1 != out.num_sats_gnss1 ||
            in.num_sats_gnss2 != out.num_sats_gnss2 ||
            in.time_itow_ms != out.time_itow_ms ||
            out.gps_week != 2396 ||
            in.error_flags != out.error_flags ||
            in.sensor_valid != out.sensor_valid) {
            return false;
        }
        break;
    default:
        return false;
    }
    return true;
}

class Measurement_Buffer
{
public:
    // Handler that receives new measurements
    void operator()(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas)
    {
        assert (no_of_messages < MAX_NO_OF_MESSAGES);
        measurements[no_of_messages++] = meas;
    }

    const static int MAX_NO_OF_MESSAGES = 10;
    AP_ExternalAHRS_SensAItion_Parser::Measurement measurements[MAX_NO_OF_MESSAGES];
    int no_of_messages = 0;
};

// ---------------------------------------------------------------------------
// LEGACY MODE TESTS
// ---------------------------------------------------------------------------

TEST(SensAItionParser, Legacy_IMU_HappyPath)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);
    EXPECT_EQ(len, 38u);

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len, mbuf);

    EXPECT_EQ(mbuf.no_of_messages, 1);
    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::IMU);
    EXPECT_TRUE(cmp_packages(in, mbuf.measurements[0]));
}

TEST(SensAItionParser, Legacy_RejectsInvalidChecksum)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);
    buffer[len - 1] += 1;
    uint32_t err_start = parser.get_parse_errors();

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len, mbuf);

    EXPECT_GT(parser.get_parse_errors(), err_start);
}

TEST(SensAItionParser, Legacy_RejectsTooSmallBuffer)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);
    uint32_t valid_start = parser.get_valid_packets();

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len - 5, mbuf);

    EXPECT_EQ(parser.get_valid_packets(), valid_start);
}

TEST(SensAItionParser, Legacy_ValidPacketsCount)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);
    uint32_t valid_start = parser.get_valid_packets();

    Measurement_Buffer mbuf;
    for (int i = 0; i < 5; i++) {
        parser.parse_stream(buffer, len, mbuf);
    }

    EXPECT_EQ(parser.get_valid_packets(), valid_start + 5);
}

TEST(SensAItionParser, Legacy_FalseHeaderInPayload)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t packet[38];
    size_t len = 38;
    fill_simulated_packet(packet, len, in, Parser::ConfigMode::IMU);
    packet[5] = 0xFA;
    uint8_t checksum = 0;
    for (size_t i = 1; i < len - 1; ++i) {
        checksum ^= packet[i];
    }
    packet[len - 1] = checksum;
    uint32_t start_valid = parser.get_valid_packets();

    Measurement_Buffer mbuf;
    for (size_t i = 0; i < len; i++) {
        parser.parse_stream(&packet[i], 1, mbuf);
    }

    EXPECT_EQ(parser.get_valid_packets(), start_valid + 1);
}

TEST(SensAItionParser, Legacy_FragmentedHeaderRecovery)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t valid[38];
    size_t len = 38;
    fill_simulated_packet(valid, len, in, Parser::ConfigMode::IMU);
    uint8_t stream[120];
    size_t slen = 0;
    stream[slen++] = 0xFA; stream[slen++] = 0x00;
    memcpy(&stream[slen], valid, 38);
    slen += 38;
    memcpy(&stream[slen], valid, 38);
    slen += 38;
    uint32_t start_valid = parser.get_valid_packets();

    Measurement_Buffer mbuf;
    for (size_t i = 0; i < slen; i++) {
        parser.parse_stream(&stream[i], 1, mbuf);
    }

    EXPECT_GE(parser.get_valid_packets() - start_valid, 1u);
}

// ---------------------------------------------------------------------------
// INTERLEAVED MODE TESTS
// ---------------------------------------------------------------------------

TEST(SensAItionParser, Interleaved_IMU_HappyPath)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    in.acceleration_mss.x = 2.5f;
    uint8_t buffer[64];
    size_t len = 64;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);
    EXPECT_EQ(len, 39u);

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len, mbuf);

    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::IMU);
    EXPECT_NEAR(mbuf.measurements[0].acceleration_mss.x, 2.5f, 0.01f);
    EXPECT_TRUE(cmp_packages(in, mbuf.measurements[0]));
}

TEST(SensAItionParser, Interleaved_INS_HappyPath)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);
    EXPECT_EQ(len, 72u); // Verified

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len, mbuf);

    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::INS);
    EXPECT_TRUE(cmp_packages(in, mbuf.measurements[0]));
}

TEST(SensAItionParser, Interleaved_InvalidID)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    uint8_t bad[] = { 0xFA, 0x99, 0x00, 0x00 };
    uint32_t start_err = parser.get_parse_errors();

    Measurement_Buffer mbuf;
    parser.parse_stream(bad, 4, mbuf);

    EXPECT_GT(parser.get_parse_errors(), start_err);
}

TEST(SensAItionParser, Handle_Interleaved_Packets)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);

    // Create a buffer with one INS and IMU packet back-to-back
    uint8_t stream[200];
    size_t len = 0;

    // Create INS Packet
    auto m_ins = default_measurement(Parser::MeasurementType::INS);
    m_ins.location.lat = 123456789; // Unique marker
    size_t ins_len = 0;
    fill_simulated_packet(&stream[0], ins_len, m_ins, Parser::ConfigMode::INTERLEAVED_INS);
    len += ins_len;

    // Create IMU Packet immediately after
    auto m_imu = default_measurement(Parser::MeasurementType::IMU);
    m_imu.acceleration_mss.z = -15.0f; // Unique marker
    size_t imu_len = 0;
    fill_simulated_packet(&stream[len], imu_len, m_imu, Parser::ConfigMode::INTERLEAVED_INS);
    len += imu_len;

    // Feed the combined buffer to parse_bytes
    Measurement_Buffer mbuf;
    parser.parse_stream(stream, len, mbuf);

    // VERIFY: Did we call the handler once for each packet?
    EXPECT_EQ(mbuf.no_of_messages, 2);
    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::INS);
    EXPECT_TRUE(cmp_packages(m_ins, mbuf.measurements[0]));
    EXPECT_EQ(mbuf.measurements[1].type, Parser::MeasurementType::IMU);
    EXPECT_TRUE(cmp_packages(m_imu, mbuf.measurements[1]));
}

TEST(SensAItionParser, Interleaved_MixedStream_Transitions)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto m_imu = default_measurement(Parser::MeasurementType::IMU);
    auto m_ahrs = default_measurement(Parser::MeasurementType::AHRS);
    auto m_ins = default_measurement(Parser::MeasurementType::INS);
    uint8_t stream[200];
    size_t len = 0;
    size_t part_len;

    part_len = 200 - len;
    fill_simulated_packet(&stream[len], part_len, m_imu, Parser::ConfigMode::INTERLEAVED_INS);
    len += part_len;
    part_len = 200 - len;
    fill_simulated_packet(&stream[len], part_len, m_ahrs, Parser::ConfigMode::INTERLEAVED_INS);
    len += part_len;
    part_len = 200 - len;
    fill_simulated_packet(&stream[len], part_len, m_ins, Parser::ConfigMode::INTERLEAVED_INS);
    len += part_len;

    Measurement_Buffer mbuf;
    parser.parse_stream(stream, len, mbuf);

    EXPECT_EQ(mbuf.no_of_messages, 3);
    EXPECT_TRUE(cmp_packages(m_imu, mbuf.measurements[0]));
    EXPECT_TRUE(cmp_packages(m_ahrs, mbuf.measurements[1]));
    EXPECT_TRUE(cmp_packages(m_ins, mbuf.measurements[2]));
}

TEST(SensAItionParser, Interleaved_INS_Fragmentation)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    uint8_t packet[100];
    size_t len = 100;
    fill_simulated_packet(packet, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    Measurement_Buffer mbuf;
    // Parse all but one byte
    parser.parse_stream(packet, len - 1, mbuf);
    EXPECT_EQ(mbuf.no_of_messages, 0);

    // Then parse the finishing byte of the message
    parser.parse_stream(&packet[len - 1], 1, mbuf);
    EXPECT_EQ(mbuf.no_of_messages, 1);
    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::INS);
}

TEST(SensAItionParser, Interleaved_Data_StressTest)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    in.location.lat = -593293230;
    in.velocity_ned.x = -15.5f;
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len, mbuf);
    EXPECT_EQ(mbuf.measurements[0].location.lat, -593293230);
    EXPECT_NEAR(mbuf.measurements[0].velocity_ned.x, -15.5f, 0.01f);
}

TEST(SensAItionParser, Interleaved_NoiseRecovery)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    uint8_t valid[100];
    size_t len = 100;
    fill_simulated_packet(valid, len, in, Parser::ConfigMode::INTERLEAVED_INS);
    uint8_t stream[200];
    memset(stream, 0xEE, 20);
    memcpy(&stream[20], valid, len);

    Measurement_Buffer mbuf;
    parser.parse_stream(stream, len + 20, mbuf);

    EXPECT_EQ(mbuf.no_of_messages, 1);
    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::INS);
}

TEST(SensAItionParser, Legacy_IMU_PartialStream)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t packet[100];
    memset(packet, 0x00, 100); // Ensure known content after the packet
    size_t chunk_size = 5;
    size_t len = 20 * chunk_size;
    fill_simulated_packet(packet, len, in, Parser::ConfigMode::IMU);

    Measurement_Buffer mbuf;
    for (size_t i = 0; i < len; i += chunk_size) {
        parser.parse_stream(&packet[i], chunk_size, mbuf);
    }

    EXPECT_EQ(mbuf.no_of_messages, 1);
    EXPECT_EQ(mbuf.measurements[0].type, Parser::MeasurementType::IMU);
}

TEST(SensAItionParser, Interleaved_INS_FullFieldVerification)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);

    // Setup Distinct Values
    in.num_sats_gnss1 = 22;
    in.num_sats_gnss2 = 18;
    in.error_flags = 0xCAFEBABE;
    in.sensor_valid = true;
    in.location.lat = -593293230;
    in.location.lng = 180685810;
    in.location.alt = 1500; // cm
    in.velocity_ned = Vector3f(-5.5f, 2.2f, 0.5f);
    in.alignment_status = 1;
    in.time_itow_ms = 987654321;
    in.gnss1_fix = 3;
    in.gnss2_fix = 2;

    // Time -> Week Calculation Check
    // 2025-12-10 is GPS Week 2396
    in.year = 2025;
    in.month = 12;
    in.day = 10;
    uint16_t expected_week = 2396;

    // Vector Accuracy
    in.pos_accuracy = Vector3f(0.1f, 0.2f, 0.3f); // Lat, Lon, Alt
    in.vel_accuracy = Vector3f(0.4f, 0.5f, 0.6f); // N, E, D

    // Generate & Parse
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    Measurement_Buffer mbuf;
    parser.parse_stream(buffer, len, mbuf);

    // VERIFICATION
    auto& out = mbuf.measurements[0];
    EXPECT_EQ(out.type, Parser::MeasurementType::INS);
    EXPECT_EQ(out.num_sats_gnss1, in.num_sats_gnss1);
    EXPECT_EQ(out.num_sats_gnss2, in.num_sats_gnss2);
    EXPECT_EQ(out.error_flags, in.error_flags);
    EXPECT_EQ(out.sensor_valid, in.sensor_valid);

    EXPECT_EQ(out.location.lat, in.location.lat);
    EXPECT_EQ(out.location.lng, in.location.lng);
    EXPECT_EQ(out.location.alt, in.location.alt);
    EXPECT_NEAR(out.velocity_ned.x, in.velocity_ned.x, 0.001f);

    EXPECT_EQ(out.alignment_status, in.alignment_status);
    EXPECT_EQ(out.time_itow_ms, in.time_itow_ms);
    EXPECT_EQ(out.gnss1_fix, in.gnss1_fix);
    EXPECT_EQ(out.gnss2_fix, in.gnss2_fix);

    // Week Verify
    EXPECT_EQ(out.gps_week, expected_week) << "GPS Week Calc Failed";

    // Vector Verify
    EXPECT_NEAR(out.pos_accuracy.x, in.pos_accuracy.x, 0.001f);
    EXPECT_NEAR(out.pos_accuracy.y, in.pos_accuracy.y, 0.001f);
    EXPECT_NEAR(out.pos_accuracy.z, in.pos_accuracy.z, 0.001f);

    EXPECT_NEAR(out.vel_accuracy.x, in.vel_accuracy.x, 0.001f);
    EXPECT_NEAR(out.vel_accuracy.y, in.vel_accuracy.y, 0.001f);
    EXPECT_NEAR(out.vel_accuracy.z, in.vel_accuracy.z, 0.001f);
}

TEST(SensAItionParser, GPS_Week_Calculation_EdgeCases)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    uint8_t buffer[100];
    size_t len = 100;
    Measurement_Buffer mbuf;

    // CASE 1: Leap Year (Feb 29 2024)
    // 2024-02-29 -> Week 2303
    in.year = 2024; in.month = 2; in.day = 29;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);
    parser.parse_stream(buffer, len, mbuf);
    EXPECT_EQ(mbuf.measurements[0].gps_week, 2303) << "Failed Leap Year Calc";

    // CASE 2: No Fix -> Week Should be 0
    in.gnss1_fix = 0; // Lost fix
    in.year = 2025; in.month = 1; in.day = 1;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);
    parser.parse_stream(buffer, len, mbuf);
    EXPECT_EQ(mbuf.measurements[1].gps_week, 0) << "Week should be 0 when fix is lost";

    // CASE 3: Pre-Epoch Date (e.g. 1970 - Error case)
    in.gnss1_fix = 3;
    in.year = 1970; in.month = 1; in.day = 1;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);
    parser.parse_stream(buffer, len, mbuf);
    EXPECT_EQ(mbuf.measurements[2].gps_week, 0) << "Week should be 0 for pre-1980 dates";
}

AP_GTEST_MAIN()
