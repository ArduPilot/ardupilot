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
    Serial InertialSensor backend for the Anello X3 FOG IMU.

    The X3 is an IMU-only device (FOG + MEMS gyro, MEMS accel, mag) on a serial
    link, so it is a native AP_InertialSensor backend rather than an ExternalAHRS:
    it feeds gyro/accel straight into the EKF with the sensor's own monotonic
    clock for a jitter-free dt. Selected by SERIALx_PROTOCOL = AnelloX3.

    The gyro fed to the EKF is the FOG (the reason for the device); the on-board
    MEMS gyro and the magnetometer are parsed and logged (AX31/AX32) but not fused
    here (a magnetometer would need a separate AP_Compass backend).
*/

#pragma once

#include "AP_InertialSensor_config.h"

#if AP_INERTIALSENSOR_ANELLOX3_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/JitterCorrection.h>

class AP_InertialSensor_AnelloX3 : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_AnelloX3(AP_InertialSensor &imu, AP_HAL::UARTDriver *uart, uint8_t serial_port);

    // probe a SERIALx_PROTOCOL = AnelloX3 port; nullptr if none configured
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu);

    // start sampling: register gyro/accel and spawn the reader thread
    void start() override;

    // publish accumulated samples to the frontend
    bool update() override;

    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    // message type identifiers
    enum class DescriptorSet : uint8_t {
        IMUData = 253
    };

    // the X3's fixed output rate (Hz). The dt the EKF integrates on comes from
    // the sensor's mcu_time, so this only sets the registered/expected rate.
    static constexpr uint16_t RATE_HZ = 200;

    // reader thread: pull bytes off the UART and parse them into packets
    void update_thread();
    void build_packet();
    bool handle_byte(uint8_t b);

    // parsing state machine
    enum class ParseState : uint8_t {
        WaitingFor_SyncOne,     // 0xC5
        WaitingFor_SyncTwo,     // 0x50
        WaitingFor_Descriptor,  // 253
        WaitingFor_PayloadLength,
        WaitingFor_Data,
        WaitingFor_Checksum
    };

    // wire payload length for the descriptor-253 IMU packet (bytes on the wire).
    // NOTE: this is NOT sizeof(AnelloX3_BinaryPayload): the struct can't be
    // PACKED (Vector3<> is non-POD, so the attribute is ignored) and carries a
    // trailing pad byte (sizeof == 56). The field *offsets* still match the wire,
    // which is what the memcpy relies on and what the static_asserts below lock.
    static constexpr uint8_t IMU_PAYLOAD_LEN = 55;

    struct AnelloX3_BinaryPayload {
        uint64_t mcu_time;  // ns -- time since power on
        uint64_t sync_time; // ns -- time of external sync pulse
        Vector3<int16_t> a;     // g  = value * range * 3.05e-5  -- scaled MEMS accel
        Vector3<int16_t> w;     // dps = value * range * 3.5e-5  -- scaled MEMS rate (gyro LSB differs from accel)
        Vector3<int32_t> og_w;  // dps = value * 500 / 2^31  -- scaled FOG rate (range assumed 500dps; fog_range below is parsed but unused)
        Vector3<int16_t> mag;   // g * 4096 -- scaled magnetometer data
        int16_t temp;           // degC * 100 -- scaled temperature value
        uint16_t mems_ranges;   // top 5 bits accel range, low 11 bits gyro range (see convert_imu_data)
        uint16_t fog_range;     // fog range in dps (currently unused; conversion hardcodes 500dps)
        // bitfield flag values
        // BIT 0 Gyro discrepancy
        // BIT 1 Temperature uncontrolled
        // BIT 2 Over current error
        // BIT 3 SiPhOG supply voltage bad
        Vector3<uint8_t> fusion_status;
    };
    // Lock the field offsets to the wire layout: the memcpy parse assumes these,
    // and a compiler padding/reorder change would otherwise silently corrupt
    // every field. (offsetof is well-defined here: the struct is standard-layout.)
    static_assert(sizeof(AnelloX3_BinaryPayload) >= IMU_PAYLOAD_LEN, "AnelloX3 payload smaller than wire format");
    static_assert(offsetof(AnelloX3_BinaryPayload, mcu_time) == 0,  "AnelloX3 layout: mcu_time");
    static_assert(offsetof(AnelloX3_BinaryPayload, sync_time) == 8, "AnelloX3 layout: sync_time");
    static_assert(offsetof(AnelloX3_BinaryPayload, a) == 16,        "AnelloX3 layout: a");
    static_assert(offsetof(AnelloX3_BinaryPayload, w) == 22,        "AnelloX3 layout: w");
    static_assert(offsetof(AnelloX3_BinaryPayload, og_w) == 28,     "AnelloX3 layout: og_w");
    static_assert(offsetof(AnelloX3_BinaryPayload, mag) == 40,      "AnelloX3 layout: mag");
    static_assert(offsetof(AnelloX3_BinaryPayload, temp) == 46,     "AnelloX3 layout: temp");
    static_assert(offsetof(AnelloX3_BinaryPayload, mems_ranges) == 48, "AnelloX3 layout: mems_ranges");
    static_assert(offsetof(AnelloX3_BinaryPayload, fog_range) == 50, "AnelloX3 layout: fog_range");
    static_assert(offsetof(AnelloX3_BinaryPayload, fusion_status) == 52, "AnelloX3 layout: fusion_status");

    // full incoming packet structure
    struct PACKED AnelloX3_Packet {
        static const uint8_t SYNC_ONE = 0xC5;
        static const uint8_t SYNC_TWO = 0x50;
        uint8_t descriptor;
        uint8_t length;
        uint8_t payload[255];
        uint8_t checksum[2]; // calculated, not incl preamble nor checksum bytes
    };

    // returns true if the checksum for the packet is valid, else false.
    static bool valid_packet(const AnelloX3_Packet &packet);

    // convert raw binary data to engineering values, do timing + drop detection
    void convert_imu_data(const AnelloX3_BinaryPayload &bin_payload);

    // push the latest sample to the frontend
    void publish_imu();

    // processed data structure
    struct {
        uint64_t mcu_time_us;   // sensor power-on time of this sample (us)
        uint64_t sample_us;     // jitter-corrected local timestamp fed to the EKF (us)
        Vector3f mems_accel;
        Vector3f mems_gyro;
        Vector3f fog_gyro;
        Vector3f mag;
        float temp;
        float mems_acc_range;
        float mems_gyro_range;
        float fog_gyro_range;
        Vector3<uint8_t> fusion_status;
    } imu_data;

    // parser working state
    struct {
        AnelloX3_Packet packet;
        ParseState state;
        uint8_t index;
    } message_in;

    AP_HAL::UARTDriver *uart;
    uint8_t serial_port;

    uint8_t gyro_instance;
    uint8_t accel_instance;
    bool started;
    bool has_announced_detected;

    // maps the sensor's monotonic power-on clock to a jitter-free local
    // micros64() timestamp so the EKF integrates on the sensor's stable dt
    // rather than UART arrival-time jitter.
    JitterCorrection jitter_correction;

    // --- data-loss detection (uses the sensor's own monotonic mcu_time) ---
    uint64_t last_mcu_time_us;  // mcu_time of the previous valid IMU packet (us)
    uint32_t dropped_samples;   // estimated IMU samples missed (mcu_time gaps)
    uint32_t crc_error_count;   // packets rejected by checksum
    uint32_t last_drop_report_ms;
};

#endif  // AP_INERTIALSENSOR_ANELLOX3_ENABLED
