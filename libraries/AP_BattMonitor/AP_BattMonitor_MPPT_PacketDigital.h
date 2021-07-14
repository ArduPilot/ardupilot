#pragma once


#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#ifndef HAL_MPPT_PACKETDIGITAL_CAN_ENABLE
    #define HAL_MPPT_PACKETDIGITAL_CAN_ENABLE (!defined(HAL_BUILD_AP_PERIPH) && HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024) || (defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_BATTERY_MPPT_PACKETDIGITAL))
#endif

#if HAL_MPPT_PACKETDIGITAL_CAN_ENABLE

#ifndef MPPT_PACKETIGITAL_DEVICE_COUNT_MAX
    #define MPPT_PACKETIGITAL_DEVICE_COUNT_MAX 5
#endif

class AP_BattMonitor_MPPT_PacketDigital : public CANSensor, public AP_BattMonitor_Backend {
public:

    // construct the CAN Sensor
    AP_BattMonitor_MPPT_PacketDigital(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params),
        CANSensor("MPPT", AP_CANManager::Driver_Type_MPPT_PacketDigital)
    { };

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

protected:
    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    // Message groups form part of the CAN ID of each frame
    enum class PacketType : uint8_t {
        ACK             = 0x21,
        NACK            = 0x22,
        PING            = 0x23,
        FAULT           = 0x42,
        ALGORITHM_SET   = 0x44,
        ALGORITHM_GET   = 0x45,
        STREAM_FAULT    = 0x47,
        STREAM_INPUT    = 0x48,
        STREAM_OUTPUT   = 0x49,
        VOLTAGE_SET     = 0x4B,
        VOLTAGE_GET     = 0x4C,
        CVT_SET         = 0x4D,
        CVT_GET         = 0x4E,
    };

    enum class FaultFlags : uint8_t {
        OVER_VOLTAGE        = (1<<0),
        UNDER_VOLTAGE       = (1<<1),
        OVER_CURRENT        = (1<<2),
        OVER_TEMPERATURE    = (1<<3),
    };

    enum class ReportMode : uint8_t {
        DISABLED            = 0,
        REPORT_INPUTS       = 1,
        REPORT_OUTPUTS      = 2,
        REPORT_VOUT_CVT_ALG = 3,
    };

    // find first MPPT_devices[] index that contains this serialnumber. If not found, returns UINT8_MAX
    uint8_t get_device_index(const uint16_t serial_number) const;

    // send command to MPPT device
    void send_command(const PacketType type, const uint16_t serialnumber, const float data = 0.0f);

    bool is_healthy_by_index(const uint8_t index) const {
        if (index >= ARRAY_SIZE(MPPT_devices) || index >= device_count) {
            return false;
        }
        return MPPT_devices[index].is_healthy();
    }

    const char* get_fault_code_string(const FaultFlags fault) const;

    //Frames received from the MPPT will use CAN Extended ID: 0x0042XXXX,
    // the least significant 16 bits contain the serial number of the MPPT.
    static constexpr uint32_t EXPECTED_FRAME_ID = (0x00420000 | AP_HAL::CANFrame::FlagEFF);

    bool get_voltage_and_current_and_temp(const int32_t serialnumber, float &voltage, float &current, float &temperature) const;

    void perform_logging() const;

    HAL_Semaphore _sem_static;
    uint32_t logger_last_ms;

    uint8_t device_count;
    PacketType packet_sent_prev;

    struct MPPT_device {
        uint16_t serialnumber;
        uint32_t timestamp_ms;
        uint8_t sequence;

        FaultFlags faults;
        int8_t temperature;
        uint8_t algorithm;
        float output_voltage_fixed;
        float cvt;

        struct {
            float voltage;
            float current;
            float power;
        } input, output;

        bool is_healthy() const {
            return ((timestamp_ms > 0) && ((AP_HAL::millis() - timestamp_ms) < 2000));
        }
    } MPPT_devices[MPPT_PACKETIGITAL_DEVICE_COUNT_MAX];
};

#endif // HAL_MPPT_PACKETDIGITAL_CAN_ENABLE

