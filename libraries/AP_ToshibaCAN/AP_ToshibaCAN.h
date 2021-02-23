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

#pragma once

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>

#define TOSHIBACAN_MAX_NUM_ESCS 12

class CANTester;

class AP_ToshibaCAN : public AP_CANDriver, public AP_ESC_Telem_Backend {
    friend class CANTester;
public:
    AP_ToshibaCAN();
    ~AP_ToshibaCAN();

    /* Do not allow copies */
    AP_ToshibaCAN(const AP_ToshibaCAN &other) = delete;
    AP_ToshibaCAN &operator=(const AP_ToshibaCAN&) = delete;

    // Return ToshibaCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_ToshibaCAN *get_tcan(uint8_t driver_index);

    // initialise ToshibaCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from SRV_Channels
    void update();

    // return a bitmask of escs that are "present" which means they are responding to requests.  Bitmask matches RC outputs
    uint16_t get_present_mask() const { return _esc_present_bitmask; }

private:

    // data format for messages from flight controller
    static constexpr uint8_t COMMAND_STOP = 0x0;
    static constexpr uint8_t COMMAND_LOCK = 0x10;
    static constexpr uint8_t COMMAND_REQUEST_DATA = 0x20;
    static constexpr uint8_t COMMAND_MOTOR3 = 0x3B;
    static constexpr uint8_t COMMAND_MOTOR2 = 0x3D;
    static constexpr uint8_t COMMAND_MOTOR1 = 0x3F;

    // data format for messages from ESC
    static constexpr uint8_t MOTOR_DATA1 = 0x40;
    static constexpr uint8_t MOTOR_DATA2 = 0x50;
    static constexpr uint8_t MOTOR_DATA3 = 0x60;
    static constexpr uint8_t MOTOR_DATA5 = 0x80;

    // processing definitions
    static constexpr uint16_t TOSHIBACAN_OUTPUT_MIN = 6300;
    static constexpr uint16_t TOSHIBACAN_OUTPUT_MAX = 32000;
    static const uint16_t TOSHIBACAN_SEND_TIMEOUT_US;
    static constexpr uint8_t CAN_IFACE_INDEX = 0;

    // telemetry definitions
    static constexpr uint32_t TOSHIBA_CAN_ESC_UPDATE_MS = 100;

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    // update esc_present_bitmask
    void update_esc_present_bitmask();

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;
    // PWM output
    HAL_Semaphore _rc_out_sem;
    uint16_t _scaled_output[TOSHIBACAN_MAX_NUM_ESCS];
    uint16_t update_count;          // counter increments each time main thread updates outputs
    uint16_t update_count_buffered; // counter when outputs copied to buffer before before sending to ESCs
    uint16_t update_count_sent;     // counter of outputs successfully sent
    uint8_t send_stage;             // stage of sending algorithm (each stage sends one frame to ESCs)

    struct telemetry_info_t {
        uint32_t last_update_ms;    // system time telemetry was last update (used to calc total current)
        float current_tot_mah;      // total current in mAh
    } _telemetry[TOSHIBACAN_MAX_NUM_ESCS];
    uint32_t _telemetry_req_ms;     // system time (in milliseconds) to request data from escs (updated at 10hz)
    uint8_t _telemetry_temp_req_counter;    // counter used to trigger temp data requests from ESCs (10x slower than other telem data)
    uint8_t _telemetry_usage_req_counter;   // counter used to trigger usage data requests from ESCs (100x slower than other telem data)
    const float amp_ms_to_mah = 1.0f / 3600.0f;  // for converting amp milliseconds to mAh

    // variables for updating bitmask of responsive escs
    uint16_t _esc_present_bitmask;      // bitmask of which escs seem to be present
    uint16_t _esc_present_bitmask_recent;   // bitmask of escs that have responded in the last second
    uint32_t _esc_present_update_ms;    // system time _esc_present_bitmask was last updated

    // structure for sending motor lock command to ESC
    union motor_lock_cmd_t {
        struct PACKED {
            uint8_t motor2:4;
            uint8_t motor1:4;
            uint8_t motor4:4;
            uint8_t motor3:4;
            uint8_t motor6:4;
            uint8_t motor5:4;
            uint8_t motor8:4;
            uint8_t motor7:4;
            uint8_t motor10:4;
            uint8_t motor9:4;
            uint8_t motor12:4;
            uint8_t motor11:4;
        };
        uint8_t data[6];
    };

    // structure for sending turn rate command to ESC
    union motor_rotation_cmd_t {
        struct PACKED {
            uint16_t motor4;
            uint16_t motor3;
            uint16_t motor2;
            uint16_t motor1;
        };
        uint8_t data[8];
    };

    // structure for requesting data from ESC
    union motor_request_data_cmd_t {
        struct PACKED {
            uint8_t motor2:4;
            uint8_t motor1:4;
            uint8_t motor4:4;
            uint8_t motor3:4;
            uint8_t motor6:4;
            uint8_t motor5:4;
            uint8_t motor8:4;
            uint8_t motor7:4;
            uint8_t motor10:4;
            uint8_t motor9:4;
            uint8_t motor12:4;
            uint8_t motor11:4;
        };
        uint8_t data[6];
    };

    // helper function to create motor_request_data_cmd_t
    motor_request_data_cmd_t get_motor_request_data_cmd(uint8_t request_id) const;

    // structure for replies from ESC of data1 (rpm and voltage)
    union motor_reply_data1_t {
        struct PACKED {
            uint8_t rxng:1;         // RX No Good. "1" if ESC encountered error receiving a message since MOTOR_DATA1 was last sent
            uint8_t stepout:1;      // "1" if a "step out" has occured since MOTOR_DATA1 was last sent
            uint8_t state:6;
            int16_t rpm;
            uint16_t current_ma;    // current in milliamps
            uint16_t voltage_mv;    // voltage in millivolts
            uint8_t position_est_error;
        };
        uint8_t data[8];
    };

    // frames to be sent
    AP_HAL::CANFrame unlock_frame;
    AP_HAL::CANFrame mot_rot_frame1;
    AP_HAL::CANFrame mot_rot_frame2;
    AP_HAL::CANFrame mot_rot_frame3;
};
