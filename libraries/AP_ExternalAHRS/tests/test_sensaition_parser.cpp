
#include <AP_gtest.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_SensAItion_Parser.h>

// This line is copied from "test_vector2.cpp" and prevents
// the linker error "undefined reference to `hal'".
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {
constexpr float UG_PER_MSS = 1e6f / 9.80665f; // ug per m/s^2
constexpr float UDEGS_PER_RADS = 1e6f * 180.0f / 3.1415926f; // udeg/s per rad/s
constexpr float MHPA_PER_PA = 1e3f / 100.0f; // mhPa per Pa
const float MEASUREMENT_TOLERANCE = 1e-3f;

using Parser = AP_ExternalAHRS_SensAItion_Parser;
}

static void fill_with_int32_in_big_endian_order(uint8_t* data, size_t& location, const int32_t value)
{
    data[location++] = (value & 0xFF000000) >> 24;
    data[location++] = (value & 0x00FF0000) >> 16;
    data[location++] = (value & 0x0000FF00) >> 8;
    data[location++] = (value & 0x000000FF);
}

static void fill_with_int16_in_big_endian_order(uint8_t* data, size_t& location, const int16_t value)
{
    data[location++] = (value & 0xFF00) >> 8;
    data[location++] = (value & 0x00FF);
}

/*
Fill a simulated SensAItion binary packet with data corresponding to the given measurement struct,
including header byte and checksum.

data: Pointer to the buffer to fill
data_length: Input the maximum length of the data, will be reset to the actual packet length
measurement: Measurement values to pack into the packet
*/
static void fill_simulated_packet(uint8_t* data, size_t& data_length,
    const Parser::Measurement measurement)
{
    assert(measurement.type != Parser::MeasurementType::UNINITIALIZED);

    // IMU: 1 (header) + 7*4 + 4*2 + 1 (checksum) = 38 bytes
    // AHRS: IMU + 4*4 = 54 bytes
    const size_t packet_length = (measurement.type == Parser::MeasurementType::IMU ? 38 : 54);
    assert(data_length >= packet_length);
    
    size_t byte_count = 0;
    data[byte_count++] = 0xFA; // Header

    // Accelerometer (raw data is in ug)
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.acceleration_mss.x * UG_PER_MSS);
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.acceleration_mss.y * UG_PER_MSS);
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.acceleration_mss.z * UG_PER_MSS);

    // Gyro (raw data is in udeg/s)
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.angular_velocity_rads.x * UDEGS_PER_RADS);
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.angular_velocity_rads.y * UDEGS_PER_RADS);
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.angular_velocity_rads.z * UDEGS_PER_RADS);

    // Temperature (conversion formula from SensAItion user manual)
    const uint16_t temp_raw = (measurement.temperature_degc - 20.0f) / 80.0f * 1e4f;
    fill_with_int16_in_big_endian_order(data, byte_count, temp_raw);

    // Magnetometer (raw data is mgauss, limited to 16 bits)
    fill_with_int16_in_big_endian_order(data, byte_count, measurement.magnetic_field_mgauss.x);
    fill_with_int16_in_big_endian_order(data, byte_count, measurement.magnetic_field_mgauss.y);
    fill_with_int16_in_big_endian_order(data, byte_count, measurement.magnetic_field_mgauss.z);
    
    // Barometer (raw data is mhPa)
    fill_with_int32_in_big_endian_order(data, byte_count, measurement.air_pressure_p * MHPA_PER_PA);
    
    if (measurement.type == Parser::MeasurementType::AHRS) {
        // Quaternion (raw data is scaled up by 1e6)
        fill_with_int32_in_big_endian_order(data, byte_count, measurement.orientation.q1 * 1e6); // scalar
        fill_with_int32_in_big_endian_order(data, byte_count, measurement.orientation.q2 * 1e6); // q_i
        fill_with_int32_in_big_endian_order(data, byte_count, measurement.orientation.q3 * 1e6); // q_j
        fill_with_int32_in_big_endian_order(data, byte_count, measurement.orientation.q4 * 1e6); // q_k
    }

    // Compute checksum, not including the first header byte
    uint8_t checksum = 0;
    for (size_t i = 1; i < byte_count; ++i) {
        checksum = checksum ^ data[i];
    }
    data[byte_count++] = checksum;

    assert(byte_count == packet_length);
    data_length = packet_length;
}

static Parser::Measurement default_measurement(Parser::MeasurementType type)
{
    Parser::Measurement in;
    in.acceleration_mss = Vector3f(0.01f, 0.02f, 1.0f);
    in.angular_velocity_rads = Vector3f(0.01f, -0.02f, 0.03f);
    in.temperature_degc = 25.0f;
    in.magnetic_field_mgauss = Vector3f(-10.0f, 20.0f, 500.0f);
    in.air_pressure_p = 101000;
    in.orientation.from_euler(0.1f, 0.2f, -0.3f); // Will be ignored in an IMU measurement
    in.type = type;
    return in;
}

// Fill with junk to get repeatable unit test behavior
static void fill_with_junk(uint8_t* buffer, size_t buffer_length)
{
    for (size_t i = 0; i < buffer_length; ++i) {
        buffer[i] = i + 17;
    }
}

TEST(SensAItionParser, CanParseValidIMUPacket)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t packet[38];
    size_t packet_length = 38;
    fill_simulated_packet(packet, packet_length, in);

    Parser::Measurement out;
    parser.parse_bytes(packet, packet_length, out);

    EXPECT_EQ(out.type, Parser::MeasurementType::IMU);
    EXPECT_LT((out.acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);
}

TEST(SensAItionParser, CanParseValidAHRSPacket)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_AHRS);
    auto in = default_measurement(Parser::MeasurementType::AHRS);
    uint8_t packet[54];
    size_t packet_length = 54;
    fill_simulated_packet(packet, packet_length, in);

    Parser::Measurement out;
    parser.parse_bytes(packet, packet_length, out);

    EXPECT_EQ(out.type, Parser::MeasurementType::AHRS);
    EXPECT_LT((out.acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LT(abs(out.orientation[i] - in.orientation[i]), MEASUREMENT_TOLERANCE);
    }
}

TEST(SensAItionParser, CanParseValidIMUPacketInParts)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    const size_t BUFFER_SIZE = 100;
    uint8_t packet[BUFFER_SIZE];
    size_t packet_length = 38;

    fill_with_junk(packet, BUFFER_SIZE);
    fill_simulated_packet(packet, packet_length, in);

    Parser::Measurement out;

    size_t partial_length = 3;
    size_t parsed_bytes = 0;
    while (true) {
        parser.parse_bytes(packet + parsed_bytes, partial_length, out);
        parsed_bytes += partial_length;

        if (parsed_bytes < packet_length) {
            EXPECT_EQ(out.type, Parser::MeasurementType::UNINITIALIZED);
        }
        else {
            EXPECT_EQ(out.type, Parser::MeasurementType::IMU);
            EXPECT_LT((out.acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
            EXPECT_LT((out.angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
            EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
            EXPECT_LT((out.magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
            EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);
            break;
        }
    }

    // Do a first parsing step of the next (incomplete) packet
    parser.parse_bytes(packet + parsed_bytes, partial_length, out);
    EXPECT_EQ(out.type, Parser::MeasurementType::UNINITIALIZED);
}

TEST(SensAItionParser, RejectsInvalidChecksum)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_AHRS);
    auto in = default_measurement(Parser::MeasurementType::AHRS);
    uint8_t packet[54];
    size_t packet_length = 54;
    fill_simulated_packet(packet, packet_length, in);

    // Change the checksum to be invalid
    packet[packet_length-1] += 1;

    Parser::Measurement out;
    parser.parse_bytes(packet, packet_length, out);

    EXPECT_EQ(out.type, Parser::MeasurementType::UNINITIALIZED);
}

TEST(SensAItionParser, RejectsTooSmallInputBuffer)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_AHRS);
    auto in = default_measurement(Parser::MeasurementType::AHRS);
    uint8_t packet[54];
    size_t packet_length = 54;
    fill_simulated_packet(packet, packet_length, in);

    Parser::Measurement out;
    parser.parse_bytes(packet, packet_length - 5, out); // Reduce input buffer size

    EXPECT_EQ(out.type, Parser::MeasurementType::UNINITIALIZED);
}

TEST(SensAItionParser, CanHandleLargeInputBuffer)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_AHRS);
    auto in = default_measurement(Parser::MeasurementType::AHRS);
    const size_t LARGE_BUFFER_SIZE = 100; // Larger than needed, but less than two packets long
    uint8_t packet[LARGE_BUFFER_SIZE];
    size_t packet_length = 54;

    fill_with_junk(packet, LARGE_BUFFER_SIZE);
    fill_simulated_packet(packet, packet_length, in);

    Parser::Measurement out;
    parser.parse_bytes(packet, LARGE_BUFFER_SIZE, out);

    EXPECT_EQ(out.type, Parser::MeasurementType::AHRS);
}

TEST(SensAItionParser, CountsValidPacketsAndParseErrors)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t packet[38];
    size_t packet_length = 38;
    fill_simulated_packet(packet, packet_length, in);

    Parser::Measurement out;
    uint32_t no_of_valid_packets = 0;
    uint32_t no_of_invalid_packets = 0;

    EXPECT_EQ(parser.get_valid_packets(), no_of_valid_packets);
    EXPECT_EQ(parser.get_parse_errors(), no_of_invalid_packets);

    while (no_of_valid_packets < 17) {
        parser.parse_bytes(packet, packet_length, out);
        no_of_valid_packets++;

        EXPECT_EQ(out.type, Parser::MeasurementType::IMU);
        EXPECT_EQ(parser.get_valid_packets(), no_of_valid_packets);
        EXPECT_EQ(parser.get_parse_errors(), no_of_invalid_packets);
    }

    // Invalidate checksum by changing a byte in the contents
    packet[5] += 1;

    while (no_of_invalid_packets < 17) {
        parser.parse_bytes(packet, packet_length, out);
        no_of_invalid_packets++;

        EXPECT_EQ(out.type, Parser::MeasurementType::UNINITIALIZED);
        EXPECT_EQ(parser.get_valid_packets(), no_of_valid_packets);
        EXPECT_EQ(parser.get_parse_errors(), no_of_invalid_packets);
    }
}

TEST(SensAItionParser, CanParsePacketPrecededByNoise)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_AHRS);
    auto in = default_measurement(Parser::MeasurementType::AHRS);
    const size_t INPUT_BUFFER_LENGTH = 100;
    uint8_t input_buffer[INPUT_BUFFER_LENGTH];
    size_t packet_length = 54;

    fill_with_junk(input_buffer, INPUT_BUFFER_LENGTH);

    // Add the packet after some initial noise
    fill_simulated_packet(input_buffer + 13, packet_length, in);

    Parser::Measurement out;
    parser.parse_bytes(input_buffer, INPUT_BUFFER_LENGTH, out);

    EXPECT_EQ(out.type, Parser::MeasurementType::AHRS);
}

TEST(SensAItionParser, CanParseVaryingValuesInAHRSPacket)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_AHRS);
    auto in = default_measurement(Parser::MeasurementType::AHRS);
    uint8_t packet[54];
    size_t packet_length = 54;
    Parser::Measurement out;

    // Extreme temperatures and air pressures
    in.temperature_degc = -40.0f;
    in.air_pressure_p = 110e3f;
    fill_simulated_packet(packet, packet_length, in);
    parser.parse_bytes(packet, packet_length, out);
    EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);

    in.temperature_degc = 120.0f;
    in.air_pressure_p = 80e3f;
    fill_simulated_packet(packet, packet_length, in);
    parser.parse_bytes(packet, packet_length, out);
    EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);

    // Extreme acceleration AND angular velocity (outside SensAItion range)
    in.acceleration_mss.x = 24.0f*9.82f; // 24 g
    in.acceleration_mss.y = -24.0f*9.82f; // -24 g
    in.angular_velocity_rads.x = 500.0f*3.14f/180.0f; // 500 deg/s
    in.angular_velocity_rads.y = -500.0f*3.14f/180.0f; // -500 deg/s

    // Extreme magnetic fields
    in.magnetic_field_mgauss.y = -7000.0f;
    in.magnetic_field_mgauss.z = 5000.0f;

    // Try a different quaternion
    in.orientation.from_euler(1.2f, -0.9f, 57.0f);

    fill_simulated_packet(packet, packet_length, in);
    parser.parse_bytes(packet, packet_length, out);
    EXPECT_LT((out.acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LT(abs(out.orientation[i] - in.orientation[i]), MEASUREMENT_TOLERANCE);
    }
}

TEST(SensAItionParser, CanParseValidIMUTwoPackets)
{
    Parser parser(Parser::ConfigMode::CONFIG_MODE_IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    const size_t BUFFER_SIZE = 100;
    uint8_t packet[BUFFER_SIZE];
    size_t packet_length = 38;

    fill_with_junk(packet, BUFFER_SIZE);
    fill_simulated_packet(&packet[0], packet_length, in);
    fill_simulated_packet(&packet[38], packet_length, in);
    fill_simulated_packet(&packet[58], packet_length, in);

    Parser::Measurement out[2];
    for(int i = 0; i < 2; i++) {
        if(i == 0) {
            parser.parse_bytes(&packet[i], 50, out[i]);
        }
        else {
            parser.parse_bytes(&packet[50], 100-50, out[i]);
        }
        EXPECT_EQ(out[i].type, Parser::MeasurementType::IMU);
        EXPECT_LT((out[i].acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
        EXPECT_LT((out[i].angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
        EXPECT_LT(abs(out[i].temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
        EXPECT_LT((out[i].magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
        EXPECT_LT(abs(out[i].air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);
    }
}

AP_GTEST_MAIN()
