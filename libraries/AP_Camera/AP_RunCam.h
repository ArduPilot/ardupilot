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
  implementation of RunCam camera protocols 

  With thanks to betaflight for a great reference
  implementation. Several of the functions below are based on
  betaflight equivalent functions
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_RUNCAM_ENABLED
#define HAL_RUNCAM_ENABLED 1
#endif

#if HAL_RUNCAM_ENABLED

#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_OSD/AP_OSD.h>

#define RUNCAM_MAX_PACKET_SIZE               64
#define RUNCAM_DEFAULT_BUTTON_PRESS_DELAY   300
// 5-key OSD auto-repeats if pressed for too long
#define RUNCAM_5KEY_BUTTON_PRESS_DELAY      100


/// @class	AP_RunCam
/// @brief	Object managing a RunCam device
class AP_RunCam
{
public:
    AP_RunCam();

    // do not allow copies
    CLASS_NO_COPY(AP_RunCam);

    // get singleton instance
    static AP_RunCam *get_singleton() {
        return _singleton;
    }

    enum class DeviceType {
        Disabled = 0,
        SplitMicro = 1, // video support only
        Split = 2, // camera and video support
        Split4k = 3, // video support only + 5key OSD
        Hybrid = 4, // video support + QR mode switch
    };

    // operation of camera button simulation
    enum class ControlOperation {
        RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN = 0x00, // WiFi/Mode button
        RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN = 0x01,
        RCDEVICE_PROTOCOL_CHANGE_MODE = 0x02,
        RCDEVICE_PROTOCOL_CHANGE_START_RECORDING = 0x03,
        RCDEVICE_PROTOCOL_CHANGE_STOP_RECORDING = 0x04,
        UNKNOWN_CAMERA_OPERATION = 0xFF
    };

    // control for OSD menu entry
    enum class ControlOption {
        STICK_YAW_RIGHT = (1 << 0),
        STICK_ROLL_RIGHT = (1 << 1),
        THREE_POS_SWITCH = (1 << 2),
        TWO_POS_SWITCH = (1 << 3),
        VIDEO_RECORDING_AT_BOOT = (1 << 4)
    };

    // initialize the RunCam driver
    void init();
    // camera button simulation
    bool simulate_camera_button(const ControlOperation operation, const uint32_t transition_timeout = RUNCAM_DEFAULT_BUTTON_PRESS_DELAY);
    // start the video
    void start_recording();
    // stop the video
    void stop_recording();
    // enter the OSD menu
    void enter_osd();
    // exit the OSD menu
    void exit_osd();
    // OSD control determined by camera options
    void osd_option();
    // update loop
    void update();
    // Check whether arming is allowed
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    // definitions prefixed with RCDEVICE taken from https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
    // possible supported features
    // RunCam Split 3S micro reports 0x77 (POWER, WIFI, MODE, SETTING, DPORT, START)
    // RunCam Split 2S reports 0x57 (POWER, WIFI, MODE, SETTING, START)
    // RunCam Racer 3 reports 0x08 (OSD)
    enum class Feature {
        RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON = (1 << 0),
        RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON = (1 << 1), // WiFi/Mode button
        RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE = (1 << 2),
        RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE = (1 << 3),
        RCDEVICE_PROTOCOL_FEATURE_DEVICE_SETTINGS_ACCESS = (1 << 4),
        RCDEVICE_PROTOCOL_FEATURE_DISPLAY_PORT = (1 << 5),
        RCDEVICE_PROTOCOL_FEATURE_START_RECORDING = (1 << 6),
        RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING = (1 << 7),
        FEATURES_OVERRIDE = (1 << 14)
    };

    const uint16_t RCDEVICE_PROTOCOL_FEATURE_2_KEY_OSD =
        uint16_t(Feature::RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE)
        | uint16_t(Feature::RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON)
        | uint16_t(Feature::RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON);

    // camera control commands
    enum class Command {
        RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO = 0x00,
        RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL = 0x01,
        RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS = 0x02,
        RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE = 0x03,
        RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION = 0x04,
        COMMAND_NONE
    };

    // operation of RC5KEY_CONNECTION
    enum class ConnectionOperation {
        RCDEVICE_PROTOCOL_5KEY_FUNCTION_OPEN = 0x01,
        RCDEVICE_PROTOCOL_5KEY_FUNCTION_CLOSE = 0x02
    };

    // operation of 5 Key OSD cable simulation
    enum class SimulationOperation {
        SIMULATION_NONE = 0x00,
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET = 0x01,
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT = 0x02,
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT = 0x03,
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP = 0x04,
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN = 0x05
    };

    // protocol versions, only version 1.0 is supported
    enum class ProtocolVersion {
        RCSPLIT_VERSION = 0x00, // unsupported firmware version <= 1.1.0
        VERSION_1_0 = 0x01,
        UNKNOWN
    };

    // status of command
    enum class RequestStatus {
        NONE,
        PENDING,
        SUCCESS,
        INCORRECT_CRC,
        TIMEOUT
    };

    enum class State {
        INITIALIZING, // uart open
        INITIALIZED,  // features received
        READY,
        VIDEO_RECORDING,
        ENTERING_MENU,
        IN_MENU,
        EXITING_MENU
    };

    enum class Event {
        NONE,
        ENTER_MENU,
        EXIT_MENU,
        IN_MENU_ENTER,
        IN_MENU_RIGHT, // only used by the 5-key process
        IN_MENU_UP,
        IN_MENU_DOWN,
        IN_MENU_EXIT,
        BUTTON_RELEASE,
        STOP_RECORDING,
        START_RECORDING
    };

    enum class OSDOption {
        NONE,
        ENTER,
        EXIT,
        OPTION,
        NO_OPTION
    };

    enum class VideoOption {
        NOT_RECORDING = 0,
        RECORDING = 1
    };

    enum class ButtonState {
        NONE,
        PRESSED,
        RELEASED
    };

    static const uint8_t  RUNCAM_NUM_SUB_MENUS =          5;
    static const uint8_t  RUNCAM_NUM_EXPECTED_RESPONSES = 4;
    static const uint8_t  RUNCAM_MAX_MENUS =              1;
    static const uint8_t  RUNCAM_MAX_MENU_LENGTH =        6;
    static const uint8_t  RUNCAM_MAX_DEVICE_TYPES =       4;

    // supported features, usually probed from the device
    AP_Int16 _features;
    // delay time to make sure the camera is fully booted
    AP_Int32 _boot_delay_ms;
    // delay time to make sure a button press has been activated
    AP_Int32 _button_delay_ms;
    // delay time to make sure a mode change has been activated
    AP_Int32 _mode_delay_ms;
    // runcam type/firmware revision
    AP_Int8 _cam_type;
    // runcam control options
    AP_Int8 _cam_control_option;

    // video on/off
    VideoOption _video_recording = VideoOption::NOT_RECORDING;
    // detected protocol version
    ProtocolVersion _protocol_version = ProtocolVersion::UNKNOWN;
    // uart for the device
    AP_HAL::UARTDriver *uart;
    // camera state
    State _state = State::INITIALIZING;
    // time since last OSD cycle
    uint32_t _last_osd_update_ms;
    // start time of the current button press or boot sequence
    uint32_t _transition_start_ms;
    // timeout of the current button press or boot sequence
    uint32_t _transition_timeout_ms;
    // record last state transition to avoid spurious transitions
    Event _last_rc_event;
    State _last_state = State::INITIALIZING;
    OSDOption _last_osd_option = OSDOption::NONE;
    int8_t _last_in_menu;
    VideoOption _last_video_recording = VideoOption::NOT_RECORDING;
    // OSD state machine: button has been pressed
    ButtonState _button_pressed = ButtonState::NONE;
    // OSD state machine: waiting for a response
    bool _waiting_device_response;
    // OSD option from RC switches
    OSDOption _osd_option;
    // OSD state mechine: in the menu, value indicates depth
    int8_t _in_menu;
    // the starting value of _in_menu
    int8_t _menu_enter_level;
    // OSD state machine: current selection in the top menu
    int8_t _top_menu_pos;
    // OSD state machine: current selection in the sub menu
    uint8_t _sub_menu_pos;
    // lengths of the sub-menus
    static uint8_t _sub_menu_lengths[RUNCAM_NUM_SUB_MENUS];
    // shared inbound scratch space
    uint8_t _recv_buf[RUNCAM_MAX_PACKET_SIZE]; // all the response contexts use same recv buffer

    class Request;

    FUNCTOR_TYPEDEF(parse_func_t, void, const Request&);

    // class to represent a request
    class Request
    {
        friend class AP_RunCam;

    public:
        Request(AP_RunCam *device, Command commandID, uint8_t param,
                       uint32_t timeout, uint16_t maxRetryTimes, parse_func_t parserFunc);
        Request() { _command = Command::COMMAND_NONE; }

        uint8_t *_recv_buf; // response data buffer
        AP_RunCam *_device; // parent device
        Command _command; // command for which a response is expected
        uint8_t _param; // parameter data, the protocol can take more but we never use it

    private:
        uint8_t _recv_response_length;     // length of the data received
        uint8_t _expected_response_length; // total length of response data wanted
        uint32_t _timeout_ms; // how long to wait before giving up
        uint32_t _request_timestamp_ms; // when the request was sent, if it's zero keep waiting for the response
        uint16_t _max_retry_times; // number of times to resend the request
        parse_func_t _parser_func; // function to parse the response
        RequestStatus _result; // whether we were successful or not

        // get the length of the expected response
        uint8_t get_expected_response_length(const Command command) const;
        // calculate a crc
        uint8_t get_crc() const;
        // parse the response
        void parse_response() {
            if (_parser_func != nullptr) {
                _parser_func(*this);
            }
        }

        struct Length {
            Command command;
            uint8_t reponse_length;
        };

        static Length _expected_responses_length[RUNCAM_NUM_EXPECTED_RESPONSES];
    } _pending_request;

    // menu structure of the runcam device
    struct Menu {
        uint8_t _top_menu_length;
        uint8_t _sub_menu_lengths[RUNCAM_MAX_MENU_LENGTH];
    };

    static Menu _menus[RUNCAM_MAX_DEVICE_TYPES];

    // return the length of the top menu
    uint8_t get_top_menu_length() const {
        return _menus[_cam_type - 1]._top_menu_length;
    }

    // return the length of a particular sub-menu
    uint8_t get_sub_menu_length(uint8_t submenu) const {
        return _menus[_cam_type - 1]._sub_menu_lengths[submenu];
    }

    // disable the OSD display
    void disable_osd() {
#if OSD_ENABLED
        AP_OSD* osd = AP::osd();
        if (osd != nullptr) {
            osd->disable();
        }
#endif
    }
    // enable the OSD display
    void enable_osd() {
#if OSD_ENABLED
        AP_OSD* osd = AP::osd();
        if (osd != nullptr) {
            osd->enable();
        }
#endif
    }

    // OSD update loop
    void update_osd();
    // update the state machine when armed or flying
    void update_state_machine_armed();
    // update the state machine when disarmed
    void update_state_machine_disarmed();
    // handle the initialized state
    void handle_initialized(Event ev);
    // handle the ready state
    void handle_ready(Event ev);
    // handle the recording state
    void handle_recording(Event ev);
    // run the 2-key OSD simulation process
    void handle_in_menu(Event ev);
    // map rc input to an event
    AP_RunCam::Event map_rc_input_to_event() const;

    // run the 2-key OSD simulation process
    void handle_2_key_simulation_process(Event ev);
    // eexit the 2 key OSD menu
    void exit_2_key_osd_menu();

    // run the 5-key OSD simulation process
    void handle_5_key_simulation_process(Event ev);
    // handle a response
    void handle_5_key_simulation_response(const Request& request);
    // commands to start and stop recording
    ControlOperation start_recording_command() const;
    ControlOperation stop_recording_command() const;
    // process a response from the serial port
    void receive();
    // empty the receive side of the serial port
    void drain();
    // start the uart with appropriate settings
    void start_uart();

    // get the RunCam device information
    void get_device_info();
    // 5 key osd cable simulation
    SimulationOperation map_key_to_protocol_operation(const Event ev) const;
    // send an event
    void send_5_key_OSD_cable_simulation_event(const Event key, const uint32_t transition_timeout = RUNCAM_5KEY_BUTTON_PRESS_DELAY);
    // enter the menu
    void open_5_key_OSD_cable_connection(parse_func_t parseFunc);
    // exit the menu
    void close_5_key_OSD_cable_connection(parse_func_t parseFunc);
    // press a button
    void simulate_5_key_OSD_cable_button_press(const SimulationOperation operation, parse_func_t parseFunc);
    // release a button
    void simulate_5_key_OSD_cable_button_release(parse_func_t parseFunc);
    // send a RunCam request and register a response to be processed
    void send_request_and_waiting_response(Command commandID, uint8_t param, uint32_t timeout,
        uint16_t maxRetryTimes, parse_func_t parseFunc);
    // send a packet to the serial port
    void send_packet(Command command, uint8_t param);
    // handle a device info response
    void parse_device_info(const Request& request);
    // wait for the RunCam device to be fully ready
    bool camera_ready() const;
    // whether or not the requested feature is supported
    bool has_feature(const Feature feature) const { return _features.get() & uint16_t(feature); }
    // input mode
    bool has_2_key_OSD() const {
        return (_features.get() & RCDEVICE_PROTOCOL_FEATURE_2_KEY_OSD) == RCDEVICE_PROTOCOL_FEATURE_2_KEY_OSD;
    }
    bool has_5_key_OSD() const {
        // RunCam Hybrid lies about supporting both 5-key and 2-key
        return !has_2_key_OSD() && has_feature(Feature::RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE);
    }

    // whether or not we can arm
    bool is_arming_prevented() const { return _in_menu > _menu_enter_level; }
    // error handler for OSD simulation
    void simulation_OSD_cable_failed(const Request& request);
    // process pending request, retrying as necessary
    bool request_pending(uint32_t now);

    static AP_RunCam *_singleton;
};

namespace AP
{
AP_RunCam *runcam();
};

#endif  // HAL_RUNCAM_ENABLED
