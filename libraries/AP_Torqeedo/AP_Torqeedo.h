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
   This driver supports communicating with Torqeedo motors that implement the "TQ Bus" protocol
   which includes the Ultralight, Cruise 2.0 R, Cruise 4.0 R, Travel 503, Travel 1003 and Cruise 10kW

   The autopilot should be connected either to the battery's tiller connector or directly to the motor
   as described on the ArduPilot wiki. https://ardupilot.org/rover/docs/common-torqeedo.html
   TQ Bus is a serial protocol over RS-485 meaning that a serial to RS-485 converter is required.

       Tiller connection: Autopilot <-> Battery (master) <-> Motor
       Motor connection:  Autopilot (master) <-> Motor

    Communication between the components is always initiated by the master with replies sent within 25ms

    Example "Remote (0x01)" reply message to allow tiller to control motor speed
    Byte        Field Definition    Example Value   Comments
    ---------------------------------------------------------------------------------
    byte 0      Header              0xAC
    byte 1      TargetAddress       0x00            see MsgAddress enum
    byte 2      Message ID          0x00            only master populates this. replies have this set to zero
    byte 3      Flags               0x05            bit0=pin present, bit2=motor speed valid
    byte 4      Status              0x00            0x20 if byte3=4, 0x0 is byte3=5
    byte 5      Motor Speed MSB     ----            Motor Speed MSB (-1000 to +1000)
    byte 6      Motor Speed LSB     ----            Motor Speed LSB (-1000 to +1000)
    byte 7      CRC-Maxim           ----            CRC-Maxim value
    byte 8      Footer              0xAD

   More details of the TQ Bus protocol are available from Torqeedo after signing an NDA.
 */

#pragma once

#include "AP_Torqeedo_config.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_Param/AP_Param.h>

#define TORQEEDO_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

class AP_Torqeedo : public AP_ESC_Telem_Backend {
public:
    AP_Torqeedo();

    CLASS_NO_COPY(AP_Torqeedo);

    static AP_Torqeedo* get_singleton();

    // initialise driver
    void init();

    // consume incoming messages from motor, reply with latest motor speed
    // runs in background thread
    void thread_main();

    // returns true if communicating with the motor
    bool healthy();

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    // report changes in error codes to user
    void report_error_codes();

    // clear motor errors
    void clear_motor_error() { _motor_clear_error = true; }

    // get latest battery status info.  returns true on success and populates arguments
    bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED;
    bool get_batt_capacity_Ah(uint16_t &amp_hours) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // message addresses
    enum class MsgAddress : uint8_t {
        BUS_MASTER = 0x00,
        REMOTE1 = 0x14,
        DISPLAY = 0x20,
        MOTOR = 0x30,
        BATTERY = 0x80
    };

    // Remote specific message ids
    enum class RemoteMsgId : uint8_t {
        INFO = 0x00,
        REMOTE = 0x01,
        SETUP = 0x02
    };

    // Display specific message ids
    enum class DisplayMsgId : uint8_t {
        INFO = 0x00,
        SYSTEM_STATE = 0x41,
        SYSTEM_SETUP = 0x42
    };

    // Motor specific message ids
    enum class MotorMsgId : uint8_t {
        INFO = 0x00,
        STATUS = 0x01,
        PARAM = 0x03,
        CONFIG = 0x04,
        DRIVE = 0x82
    };

    enum class ParseState {
        WAITING_FOR_HEADER = 0,
        WAITING_FOR_FOOTER,
    };

    // TYPE parameter values
    enum class ConnectionType : uint8_t {
        TYPE_DISABLED = 0,
        TYPE_TILLER = 1,
        TYPE_MOTOR = 2
    };

    // OPTIONS parameter values
    enum options {
        LOG             = 1<<0,
        DEBUG_TO_GCS    = 1<<1,
    };

    // initialise serial port and gpio pins (run from background thread)
    // returns true on success
    bool init_internals();

    // returns true if the driver is enabled
    bool enabled() const;

    // process a single byte received on serial port
    // return true if a complete message has been received (the message will be held in _received_buff)
    bool parse_byte(uint8_t b);

    // process message held in _received_buff
    void parse_message();

    // returns true if it is safe to send a message
    bool safe_to_send() const { return ((_send_delay_us == 0) && (_reply_wait_start_ms == 0)); }

    // set pin to enable sending a message
    void send_start();

    // check for timeout after sending a message and unset pin if required
    void check_for_send_end();

    // calculate delay required to allow message to be completely sent
    uint32_t calc_send_delay_us(uint8_t num_bytes);

    // record msgid of message to wait for and set timer for reply timeout handling
    void set_expected_reply_msgid(uint8_t msg_id);

    // check for timeout waiting for reply
    void check_for_reply_timeout();

    // mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
    void set_reply_received();

    // send a message to the motor with the specified message contents
    // msg_contents should not include the header, footer or CRC
    // returns true on success
    bool send_message(const uint8_t msg_contents[], uint8_t num_bytes);

    // add a byte to a message buffer including adding the escape character (0xAE) if necessary
    // this should only be used when adding the contents to the buffer, not the header and footer
    // num_bytes is updated to the next free byte
    bool add_byte_to_message(uint8_t byte_to_add, uint8_t msg_buff[], uint8_t msg_buff_size, uint8_t &num_bytes) const;

    // send a motor speed command as a value from -1000 to +1000
    // value is taken directly from SRV_Channel
    void send_motor_speed_cmd();

    // send request to motor to reply with a particular message
    // msg_id can be INFO, STATUS or PARAM
    void send_motor_msg_request(MotorMsgId msg_id);

    // calculate the limited motor speed that is sent to the motors
    // desired_motor_speed argument and returned value are in the range -1000 to 1000
    int16_t calc_motor_speed_limited(int16_t desired_motor_speed);
    int16_t get_motor_speed_limited() const { return (int16_t)_motor_speed_limited; }

    // log TRQD message which holds high level status and latest desired motors peed
    // force_logging should be true to immediately write log bypassing timing check to avoid spamming
    void log_TRQD(bool force_logging);

    // send ESC telemetry
    void update_esc_telem(float rpm, float voltage, float current_amps, float esc_tempC, float motor_tempC);

    // parameters
    AP_Enum<ConnectionType> _type;      // connector type used (0:disabled, 1:tiller connector, 2: motor connector)
    AP_Int8 _pin_onoff;     // Pin number connected to Torqeedo's on/off pin. -1 to disable turning motor on/off from autopilot
    AP_Int8 _pin_de;        // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to motor
    AP_Int16 _options;      // options bitmask
    AP_Int8 _motor_power;   // motor power (0 ~ 100).  only applied when using motor connection
    AP_Float _slew_time;    // slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  A value of zero disables the limit
    AP_Float _dir_delay;    // direction change delay.  output will remain at zero for this many seconds when transitioning between forward and backwards rotation

    // members
    AP_HAL::UARTDriver *_uart;      // serial port to communicate with motor
    bool _initialised;              // true once driver has been initialised
    bool _send_motor_speed;         // true if motor speed should be sent at next opportunity
    int16_t _motor_speed_desired;   // desired motor speed (set from within update method)
    uint32_t _last_send_motor_ms;   // system time (in millis) last motor speed command was sent (used for health reporting)
    bool _motor_clear_error;        // true if the motor error should be cleared (sent in "Drive" message)
    uint32_t _send_start_us;        // system time (in micros) when last message started being sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;        // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying

    // motor speed limit variables
    float _motor_speed_limited;     // limited desired motor speed. this value is actually sent to the motor
    uint32_t _motor_speed_limited_ms; // system time that _motor_speed_limited was last updated
    int8_t _dir_limit;              // acceptable directions for output to motor (+1 = positive OK, -1 = negative OK, 0 = either positive or negative OK)
    uint32_t _motor_speed_zero_ms;  // system time that _motor_speed_limited reached zero.  0 if currently not zero

    // health reporting
    HAL_Semaphore _last_healthy_sem;// semaphore protecting reading and updating of _last_send_motor_ms and _last_received_ms
    uint32_t _last_log_TRQD_ms;     // system time (in millis) that TRQD was last logged

    // message parsing members
    ParseState _parse_state;        // current state of parsing
    bool _parse_escape_received;    // true if the escape character has been received so we must XOR the next byte
    uint32_t _parse_error_count;    // total number of parsing errors (for reporting)
    uint32_t _parse_success_count;  // number of messages successfully parsed (for reporting)
    uint8_t _received_buff[TORQEEDO_MESSAGE_LEN_MAX];   // characters received
    uint8_t _received_buff_len;     // number of characters received
    uint32_t _last_received_ms;     // system time (in millis) that a message was successfully parsed (for health reporting)

    // reply message handling
    uint8_t _reply_msgid;           // replies expected msgid (reply often does not specify the msgid so we must record it)
    uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message

    // Display system state flags
    typedef union PACKED {
        struct {
            uint8_t set_throttle_stop   : 1;    // 0, warning that user must set throttle to stop before motor can run
            uint8_t setup_allowed       : 1;    // 1, remote is allowed to enter setup mode
            uint8_t in_charge           : 1;    // 2, master is in charging state
            uint8_t in_setup            : 1;    // 3, master is in setup state
            uint8_t bank_available      : 1;    // 4
            uint8_t no_menu             : 1;    // 5
            uint8_t menu_off            : 1;    // 6
            uint8_t reserved7           : 1;    // 7, unused
            uint8_t temp_warning        : 1;    // 8, motor or battery temp warning
            uint8_t batt_charge_valid   : 1;    // 9, battery charge valid
            uint8_t batt_nearly_empty   : 1;    // 10, battery nearly empty
            uint8_t batt_charging       : 1;    // 11, battery charging
            uint8_t gps_searching       : 1;    // 12, gps searching for satellites
            uint8_t gps_speed_valid     : 1;    // 13, gps speed is valid
            uint8_t range_miles_valid   : 1;    // 14, range (in miles) is valid
            uint8_t range_minutes_valid : 1;    // 15, range (in minutes) is valid
        };
        uint16_t value;
    } DisplaySystemStateFlags;

    // Display system state
    struct DisplaySystemState {
        DisplaySystemStateFlags flags;  // flags, see above for individual bit definitions
        uint8_t master_state;           // deprecated
        uint8_t master_error_code;      // error code (0=no error)
        float motor_voltage;            // motor voltage in volts
        float motor_current;            // motor current in amps
        uint16_t motor_power;           // motor power in watts
        int16_t motor_rpm;              // motor speed in rpm
        uint8_t motor_pcb_temp;         // motor pcb temp in C
        uint8_t motor_stator_temp;      // motor stator temp in C
        uint8_t batt_charge_pct;        // battery state of charge (0 to 100%)
        float batt_voltage;             // battery voltage in volts
        float batt_current;             // battery current in amps
        uint16_t gps_speed;             // gps speed in knots * 100
        uint16_t range_miles;           // range in nautical miles * 10
        uint16_t range_minutes;         // range in minutes (at current speed and current draw)
        uint8_t temp_sw;                // master PCB temp in C (close to motor power switches)
        uint8_t temp_rp;                // master PCB temp in C (close to reverse voltage protection)
        uint32_t last_update_ms;        // system time that system state was last updated
    } _display_system_state;

    // Display system setup
    struct DisplaySystemSetup {
        uint8_t flags;              // 0 : battery config valid, all other bits unused
        uint8_t motor_type;         // motor type (0 or 3:Unknown, 1:Ultralight, 2:Cruise2, 4:Cruise4, 5:Travel503, 6:Travel1003, 7:Cruise10kW)
        uint16_t motor_sw_version;  // motor software version
        uint16_t batt_capacity;     // battery capacity in amp hours
        uint8_t batt_charge_pct;    // battery state of charge (0 to 100%)
        uint8_t batt_type;          // battery type (0:lead acid, 1:Lithium)
        uint16_t master_sw_version; // master software version
    } _display_system_setup;

    // Motor status
    struct MotorStatus {
        union PACKED {
            struct {
                uint8_t temp_limit_motor    : 1;    // 0, motor speed limited due to motor temp
                uint8_t temp_limit_pcb      : 1;    // 1, motor speed limited tue to PCB temp
                uint8_t emergency_stop      : 1;    // 2, motor in emergency stop (must be cleared by master)
                uint8_t running             : 1;    // 3, motor running
                uint8_t power_limit         : 1;    // 4, motor power limited
                uint8_t low_voltage_limit   : 1;    // 5, motor speed limited because of low voltage
                uint8_t tilt                : 1;    // 6, motor is tilted
                uint8_t reserved7           : 1;    // 7, unused (always zero)
            } status_flags;
            uint8_t status_flags_value;
        };
        union PACKED {
            struct {
                uint8_t overcurrent         : 1;    // 0, motor stopped because of overcurrent
                uint8_t blocked             : 1;    // 1, motor stopped because it is blocked
                uint8_t overvoltage_static  : 1;    // 2, motor stopped because voltage too high
                uint8_t undervoltage_static : 1;    // 3, motor stopped because voltage too low
                uint8_t overvoltage_current : 1;    // 4, motor stopped because voltage spiked high
                uint8_t undervoltage_current: 1;    // 5, motor stopped because voltage spiked low
                uint8_t overtemp_motor      : 1;    // 6, motor stopped because stator temp too high
                uint8_t overtemp_pcb        : 1;    // 7, motor stopped because pcb temp too high
                uint8_t timeout_rs485       : 1;    // 8, motor stopped because Drive message not received for too long
                uint8_t temp_sensor_error   : 1;    // 9, motor temp sensor is defective (motor will not stop)
                uint8_t tilt                : 1;    // 10, motor stopped because it was tilted
                uint8_t unused11to15        : 5;    // 11 ~ 15 (always zero)
            } error_flags;
            uint16_t error_flags_value;
        };
    } _motor_status;
    uint32_t _last_send_motor_status_request_ms;    // system time (in milliseconds) that last motor status request was sent

    // Motor params
    struct MotorParam {
        int16_t rpm;            // motor rpm
        uint16_t power;         // motor power consumption in Watts
        float voltage;          // motor voltage in volts
        float current;          // motor current in amps
        float pcb_temp;         // pcb temp in C
        float stator_temp;      // stator temp in C
        uint32_t last_update_ms;// system time that above values were updated
    } _motor_param;
    uint32_t _last_send_motor_param_request_ms;     // system time (in milliseconds) that last motor param request was sent

    // error reporting
    DisplaySystemStateFlags _display_system_state_flags_prev;   // backup of display system state flags
    uint8_t _display_system_state_master_error_code_prev;       // backup of display system state master_error_code
    uint32_t _last_error_report_ms;                             // system time that flag changes were last reported (used to prevent spamming user)
    MotorStatus _motor_status_prev;                             // backup of motor status
    static AP_Torqeedo *_singleton;

    // returns a human-readable string corresponding the passed-in
    // master error code (see page 93 of https://media.torqeedo.com/downloads/manuals/torqeedo-Travel-manual-DE-EN.pdf)
    // If no conversion is available then nullptr is returned
    const char *map_master_error_code_to_string(uint8_t code) const;
};

namespace AP {
    AP_Torqeedo *torqeedo();
};

#endif // HAL_TORQEEDO_ENABLED
