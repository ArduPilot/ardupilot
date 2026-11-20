#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_XSENS_ENABLED

#include "AP_ExternalAHRS_Xsens.h"

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_Xsens::AP_ExternalAHRS_Xsens(AP_ExternalAHRS *frontend_ptr,
                                             AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(frontend_ptr, _state)
{
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));
    auto &serial = AP::serialmanager();
    uart = serial.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = serial.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = uart ? serial.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0) : -1;
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens ExternalAHRS no UART");
    }
}

void AP_ExternalAHRS_Xsens::send_command(uint8_t mid, const uint8_t *payload, uint8_t length)
{
    uint8_t packet[32] {0xFA, 0xFF, mid, length};
    uint8_t checksum = uint8_t(0xFF + mid + length);
    for (uint8_t i = 0; i < length; i++) {
        packet[4 + i] = payload[i];
        checksum += payload[i];
    }
    packet[4 + length] = uint8_t(-checksum);
    uart->write(packet, 5 + length);
    last_command_ms = AP_HAL::millis();
}

void AP_ExternalAHRS_Xsens::publish(const AP_ExternalAHRS_Xsens_Parser::Sample &sample)
{
    if (have_counter && sample.counter == last_counter) {
        return;
    }
    have_counter = true;
    last_counter = sample.counter;
    const Vector3f accel {sample.accel_mps2[0], sample.accel_mps2[1], sample.accel_mps2[2]};
    const Vector3f gyro {sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2]};
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro = gyro;
        state.quat = Quaternion{sample.quaternion_wxyz[0], sample.quaternion_wxyz[1],
                                sample.quaternion_wxyz[2], sample.quaternion_wxyz[3]};
        state.have_quaternion = true;
    }
    const AP_ExternalAHRS::ins_data_message_t ins {accel, gyro, -300};
    AP::ins().handle_external(ins);
    last_sample_ms = AP_HAL::millis();
}

void AP_ExternalAHRS_Xsens::update()
{
    if (!uart) {
        return;
    }
    if (!port_open) {
        uart->begin(baudrate);
        port_open = true;
    }
    const uint32_t now = AP_HAL::millis();
    static const uint8_t output_config[] {
        0x20, 0x14, 0x00, 0x64, // quaternion in NED at 100 Hz
        0x80, 0x20, 0x00, 0x64, // gyro in sensor frame at 100 Hz
        0x40, 0x20, 0x00, 0x64, // acceleration in sensor frame at 100 Hz
        0x10, 0x20, 0x00, 0x64, // packet counter
    };
    if (setup_step == 0) {
        send_command(0x30, nullptr, 0); // GoToConfig
        setup_step = 1;
    } else if (setup_step == 1 && now - last_command_ms >= 100) {
        send_command(0xC0, output_config, sizeof(output_config));
        setup_step = 2;
    } else if (setup_step == 2 && now - last_command_ms >= 100) {
        send_command(0x10, nullptr, 0); // GoToMeasurement
        setup_step = 3;
    } else if (setup_step == 3 && !healthy() && now - last_command_ms > 1500) {
        setup_step = 0;
        have_counter = false;
    }
    uint32_t available = MIN(uart->available(), 1024u);
    while (available-- > 0) {
        uint8_t byte;
        if (!uart->read(byte)) {
            break;
        }
        AP_ExternalAHRS_Xsens_Parser::Sample sample;
        if (setup_step == 3 && parser.feed(byte, sample)) {
            publish(sample);
        }
    }
}

bool AP_ExternalAHRS_Xsens::healthy() const
{
    return last_sample_ms != 0 && AP_HAL::millis() - last_sample_ms < 100;
}

bool AP_ExternalAHRS_Xsens::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens MTi-300 IMU unhealthy");
        return false;
    }
    return true;
}

#endif // AP_EXTERNAL_AHRS_XSENS_ENABLED
