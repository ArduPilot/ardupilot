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

  RunCam protocol specification can be found at https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 */
#include "AP_RunCam.h"

#if HAL_RUNCAM_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>

const AP_Param::GroupInfo AP_RunCam::var_info[] = {
    // @Param: TYPE
    // @DisplayName: RunCam device type
    // @Description: RunCam deviee type used to determine OSD menu structure and shutter options.
    // @Values: 0:Disabled, 1:RunCam Split Micro/RunCam with UART, 2:RunCam Split, 3:RunCam Split4 4k, 4:RunCam Hybrid/RunCam Thumb Pro, 5:Runcam 2 4k
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_RunCam, _cam_type, int(DeviceType::Disabled), AP_PARAM_FLAG_ENABLE),

    // @Param: FEATURES
    // @DisplayName: RunCam features available
    // @Description: The available features of the attached RunCam device. If 0 then the RunCam device will be queried for the features it supports, otherwise this setting is used.
    // @User: Advanced
    // @Bitmask: 0:Power Button,1:WiFi Button,2:Change Mode,3:5-Key OSD,4:Settings Access,5:DisplayPort,6:Start Recording,7:Stop Recording
    AP_GROUPINFO("FEATURES", 2, AP_RunCam, _features, 0),

    // @Param: BT_DELAY
    // @DisplayName: RunCam boot delay before allowing updates
    // @Description: Time it takes for the RunCam to become fully ready in ms. If this is too short then commands can get out of sync.
    // @User: Advanced
    AP_GROUPINFO("BT_DELAY", 3, AP_RunCam, _boot_delay_ms, 7000),

    // @Param: BTN_DELAY
    // @DisplayName: RunCam button delay before allowing further button presses
    // @Description: Time it takes for the a RunCam button press to be actived in ms. If this is too short then commands can get out of sync.
    // @User: Advanced
    AP_GROUPINFO("BTN_DELAY", 4, AP_RunCam, _button_delay_ms, RUNCAM_DEFAULT_BUTTON_PRESS_DELAY),

    // @Param: MDE_DELAY
    // @DisplayName: RunCam mode delay before allowing further button presses
    // @Description: Time it takes for the a RunCam mode button press to be actived in ms. If a mode change first requires a video recording change then double this value is used. If this is too short then commands can get out of sync.
    // @User: Advanced
    AP_GROUPINFO("MDE_DELAY", 5, AP_RunCam, _mode_delay_ms, 800),

    // @Param: CONTROL
    // @DisplayName: RunCam control option
    // @Description: Specifies the allowed actions required to enter the OSD menu and other option like autorecording
    // @Bitmask: 0:Stick yaw right,1:Stick roll right,2:3-position switch,3:2-position switch,4:Autorecording enabled
    // @User: Advanced
    AP_GROUPINFO("CONTROL", 6, AP_RunCam, _cam_control_option, uint8_t(ControlOption::STICK_ROLL_RIGHT) | uint8_t(ControlOption::TWO_POS_SWITCH)),

    AP_GROUPEND
};

#define RUNCAM_DEBUG 0

#if RUNCAM_DEBUG
static const char* event_names[11] = {
        "NONE", "ENTER_MENU", "EXIT_MENU",
        "IN_MENU_ENTER", "IN_MENU_RIGHT", "IN_MENU_UP", "IN_MENU_DOWN", "IN_MENU_EXIT",
        "BUTTON_RELEASE", "STOP_RECORDING", "START_RECORDING"
};
static const char* state_names[7] = {
        "INITIALIZING", "INITIALIZED", "READY", "VIDEO_RECORDING", "ENTERING_MENU", "IN_MENU", "EXITING_MENU"
};
#define debug(fmt, args ...) do { hal.console->printf("RunCam[%s]: " fmt, state_names[int(_state)], ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

// singleton instance
AP_RunCam *AP_RunCam::_singleton;

AP_RunCam::Request::Length AP_RunCam::Request::_expected_responses_length[RUNCAM_NUM_EXPECTED_RESPONSES] = {
    { Command::RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO, 5 },
    { Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS, 2 },
    { Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE, 2 },
    { Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION, 3 },
};

// the protocol for Runcam Device definition
static const uint8_t RUNCAM_HEADER = 0xCC;
static const uint8_t RUNCAM_OSD_MENU_DEPTH = 2;
static const uint32_t RUNCAM_INIT_INTERVAL_MS = 1000;
static const uint32_t RUNCAM_OSD_UPDATE_INTERVAL_MS = 100; // 10Hz

// menu structures of runcam devices
AP_RunCam::Menu AP_RunCam::_menus[RUNCAM_MAX_DEVICE_TYPES] = {
    // these are correct for the runcam split micro v2.4.4, others may vary
    // Video, Image, TV-OUT, Micro SD Card, General
    { 6, { 5, 8, 3, 3, 7 }}, // SplitMicro
    { 0, { 0 }}, // Split
    { 6, { 4, 10, 3, 3, 7 }}, // Split4 4K
    { 1, { 0 }}, // Hybrid, simple mode switch
	{ 6, { 3, 10, 2, 2, 8 }}, // Runcam 2 4K
};

AP_RunCam::AP_RunCam()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_RunCam must be singleton");
    }
    _singleton = this;
    _cam_type.set(constrain_int16(_cam_type, 0, RUNCAM_MAX_DEVICE_TYPES));
    _video_recording = VideoOption(_cam_control_option & uint8_t(ControlOption::VIDEO_RECORDING_AT_BOOT));
}

// init the runcam device by finding a serial device configured for the RunCam protocol
void AP_RunCam::init()
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager) {
        uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_RunCam, 0);
    }
    if (uart != nullptr) {
        /*
          if the user has setup a serial port as a runcam then default
          type to the split micro (Andy's development platform!). This makes setup a bit easier for most
          users while still enabling parameters to be hidden for users
          without a runcam
         */
        _cam_type.set_default(int8_t(DeviceType::SplitMicro));
        AP_Param::invalidate_count();
    }
    if (_cam_type.get() == int8_t(DeviceType::Disabled)) {
        uart = nullptr;
        return;
    }

    if (uart == nullptr) {
        return;
    }

    // Split and Runcam 2 4k requires two mode presses to get into the menu
    if (_cam_type.get() == int8_t(DeviceType::Split) || _cam_type.get() == int8_t(DeviceType::Run24k)) {
        _menu_enter_level = -1;
        _in_menu = -1;
    }

    start_uart();

    // first transition is from initialized to ready
    _transition_start_ms = AP_HAL::millis();
    _transition_timeout_ms = _boot_delay_ms;

    get_device_info();
}

// simulate pressing the camera button
bool AP_RunCam::simulate_camera_button(const ControlOperation operation, const uint32_t transition_timeout)
{
    if (!uart || _protocol_version != ProtocolVersion::VERSION_1_0) {
        return false;
    }

    _transition_timeout_ms = transition_timeout;
    debug("press button %d, timeout=%dms\n", int(operation), int(transition_timeout));
    send_packet(Command::RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL, uint8_t(operation));

    return true;
}

// start the video
void AP_RunCam::start_recording() {
    debug("start recording(%d)\n", int(_state));
    _video_recording = VideoOption::RECORDING;
    _osd_option = OSDOption::NO_OPTION;
}

// stop the video
void AP_RunCam::stop_recording() {
    debug("stop recording(%d)\n", int(_state));
    _video_recording = VideoOption::NOT_RECORDING;
    _osd_option = OSDOption::NO_OPTION;
}

// enter the OSD menu
void AP_RunCam::enter_osd()
{
    debug("enter osd(%d)\n", int(_state));
    _osd_option = OSDOption::ENTER;
}

// exit the OSD menu
void AP_RunCam::exit_osd()
{
    debug("exit osd(%d)\n", int(_state));
    _osd_option = OSDOption::EXIT;
}

// OSD control determined by camera options
void AP_RunCam::osd_option() {
    debug("osd option\n");
    _osd_option = OSDOption::OPTION;
}

// input update loop
void AP_RunCam::update()
{
    if (uart == nullptr || _cam_type.get() == int8_t(DeviceType::Disabled)) {
        return;
    }

    // process any pending packets
    receive();

    uint32_t now = AP_HAL::millis();
    if ((now - _last_osd_update_ms) > RUNCAM_OSD_UPDATE_INTERVAL_MS) {
        update_osd();
        _last_osd_update_ms = now;
    }
}

// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AP_RunCam::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
    // if not enabled return true
    if (!uart) {
        return true;
    }

    // currently in the OSD menu, do not allow arming
    if (is_arming_prevented()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "In OSD menu");
        return false;
    }

    if (!camera_ready()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Camera not ready");
        return false;
    }

    // if we got this far everything must be ok
    return true;

}

// OSD update loop
void AP_RunCam::update_osd()
{
    bool use_armed_state_machine = hal.util->get_soft_armed();
#if OSD_ENABLED
    // prevent runcam stick gestures interferring with osd stick gestures
    if (!use_armed_state_machine) {
        const AP_OSD* osd = AP::osd();
        if (osd != nullptr) {
            use_armed_state_machine = !osd->is_readonly_screen();
        }
    }
#endif
    // run a reduced state simulation process when armed
    if (use_armed_state_machine) {
        update_state_machine_armed();
        return;
    }

    update_state_machine_disarmed();
}

// update the state machine when armed or flying
void AP_RunCam::update_state_machine_armed()
{
    const uint32_t now = AP_HAL::millis();
    if ((now - _transition_start_ms) < _transition_timeout_ms) {
        return;
    }

    _transition_start_ms = now;
    _transition_timeout_ms = 0;

    switch (_state) {
    case State::READY:
        handle_ready(_video_recording == VideoOption::RECORDING && has_feature(Feature::RCDEVICE_PROTOCOL_FEATURE_START_RECORDING) ? Event::START_RECORDING : Event::NONE);
        break;
    case State::VIDEO_RECORDING:
        handle_recording(_video_recording == VideoOption::NOT_RECORDING && has_feature(Feature::RCDEVICE_PROTOCOL_FEATURE_START_RECORDING) ? Event::STOP_RECORDING : Event::NONE);
        break;
    case State::INITIALIZING:
    case State::INITIALIZED:
    case State::ENTERING_MENU:
    case State::IN_MENU:
    case State::EXITING_MENU:
        break;
    }
}

// update the state machine when disarmed
void AP_RunCam::update_state_machine_disarmed()
{
    const uint32_t now = AP_HAL::millis();
    if (_waiting_device_response || (now - _transition_start_ms) < _transition_timeout_ms) {
        _last_rc_event = Event::NONE;
        return;
    }

    _transition_start_ms = now;
    _transition_timeout_ms = 0;

    const Event ev = map_rc_input_to_event();
    // only take action on transitions
    if (ev == _last_rc_event && _state == _last_state && _osd_option == _last_osd_option
        && _last_in_menu == _in_menu && _last_video_recording == _video_recording) {
        return;
    }

    debug("update_state_machine_disarmed(%s)\n", event_names[int(ev)]);

    _last_rc_event = ev;
    _last_state = _state;
    _last_osd_option = _osd_option;
    _last_in_menu = _in_menu;
    _last_video_recording = _video_recording;

    switch (_state) {
    case State::INITIALIZING:
        break;
    case State::INITIALIZED:
        handle_initialized(ev);
        break;
    case State::READY:
        handle_ready(ev);
        break;
    case State::VIDEO_RECORDING:
        handle_recording(ev);
        break;
    case State::ENTERING_MENU:
        handle_in_menu(Event::ENTER_MENU);
        break;
    case State::IN_MENU:
        handle_in_menu(ev);
        break;
    case State::EXITING_MENU:
        handle_in_menu(Event::EXIT_MENU);
        break;
    }
}

// handle the initialized state
void AP_RunCam::handle_initialized(Event ev)
{
    // the camera should be configured to start with recording mode off by default
    // a recording change needs significantly extra time to process
    if (_video_recording == VideoOption::RECORDING && has_feature(Feature::RCDEVICE_PROTOCOL_FEATURE_START_RECORDING)) {
        if (!(_cam_control_option & uint8_t(ControlOption::VIDEO_RECORDING_AT_BOOT))) {
            simulate_camera_button(start_recording_command(), _mode_delay_ms * 2);
        }
        _state = State::VIDEO_RECORDING;
    } else if (_video_recording == VideoOption::NOT_RECORDING && has_feature(Feature::RCDEVICE_PROTOCOL_FEATURE_START_RECORDING)) {
        if (_cam_control_option & uint8_t(ControlOption::VIDEO_RECORDING_AT_BOOT)) {
            simulate_camera_button(stop_recording_command(), _mode_delay_ms * 2);
        }
        _state = State::READY;
    } else {
        _state = State::READY;
    }
    debug("device fully booted after %ums\n", unsigned(AP_HAL::millis()));
}

// handle the ready state
void AP_RunCam::handle_ready(Event ev)
{
    switch (ev) {
    case Event::ENTER_MENU:
    case Event::IN_MENU_ENTER:
    case Event::IN_MENU_RIGHT:
        if (ev == Event::ENTER_MENU || _cam_control_option & uint8_t(ControlOption::STICK_ROLL_RIGHT)) {
            _top_menu_pos = -1;
            _sub_menu_pos = 0;
            _state = State::ENTERING_MENU;
        }
        break;
    case Event::START_RECORDING:
        simulate_camera_button(start_recording_command(), _mode_delay_ms);
        _state = State::VIDEO_RECORDING;
        break;
    case Event::NONE:
    case Event::EXIT_MENU:
    case Event::IN_MENU_UP:
    case Event::IN_MENU_DOWN:
    case Event::IN_MENU_EXIT:
    case Event::BUTTON_RELEASE:
    case Event::STOP_RECORDING:
        break;
    }
}

// handle the recording state
void AP_RunCam::handle_recording(Event ev)
{
    switch (ev) {
    case Event::ENTER_MENU:
    case Event::IN_MENU_ENTER:
    case Event::IN_MENU_RIGHT:
        if (ev == Event::ENTER_MENU || _cam_control_option & uint8_t(ControlOption::STICK_ROLL_RIGHT)) {
            simulate_camera_button(stop_recording_command(), _mode_delay_ms);
            _top_menu_pos = -1;
            _sub_menu_pos = 0;
            _state = State::ENTERING_MENU;
        }
        break;
    case Event::STOP_RECORDING:
        simulate_camera_button(stop_recording_command(), _mode_delay_ms);
        _state = State::READY;
        break;
    case Event::NONE:
    case Event::EXIT_MENU:
    case Event::IN_MENU_UP:
    case Event::IN_MENU_DOWN:
    case Event::IN_MENU_EXIT:
    case Event::BUTTON_RELEASE:
    case Event::START_RECORDING:
        break;
    }
}

// handle the in_menu state
void AP_RunCam::handle_in_menu(Event ev)
{
    if (has_5_key_OSD()) {
        handle_5_key_simulation_process(ev);
    } else if (has_2_key_OSD()) {
        // otherwise the simpler 2 key OSD simulation, requires firmware 2.4.4 on the split micro
        handle_2_key_simulation_process(ev);
    }
}

// map rc input to an event
AP_RunCam::Event AP_RunCam::map_rc_input_to_event() const
{
    const RC_Channel::AuxSwitchPos throttle = rc().get_channel_pos(AP::rcmap()->throttle());
    const RC_Channel::AuxSwitchPos yaw = rc().get_channel_pos(AP::rcmap()->yaw());
    const RC_Channel::AuxSwitchPos roll = rc().get_channel_pos(AP::rcmap()->roll());
    const RC_Channel::AuxSwitchPos pitch = rc().get_channel_pos(AP::rcmap()->pitch());

    Event result = Event::NONE;

    if (_button_pressed != ButtonState::NONE) {
        if (_button_pressed == ButtonState::PRESSED && yaw == RC_Channel::AuxSwitchPos::MIDDLE && pitch == RC_Channel::AuxSwitchPos::MIDDLE && roll == RC_Channel::AuxSwitchPos::MIDDLE) {
            result = Event::BUTTON_RELEASE;
        } else {
            result = Event::NONE; // still waiting to be released
        }
    } else if (throttle == RC_Channel::AuxSwitchPos::MIDDLE && yaw == RC_Channel::AuxSwitchPos::LOW
        && pitch == RC_Channel::AuxSwitchPos::MIDDLE && roll == RC_Channel::AuxSwitchPos::MIDDLE
        // don't allow an action close to arming unless the user had configured it or arming is not possible
        // but don't prevent the 5-Key control actually working
        && (_cam_control_option & uint8_t(ControlOption::STICK_YAW_RIGHT) || is_arming_prevented())) {
        result = Event::EXIT_MENU;
    } else if (throttle == RC_Channel::AuxSwitchPos::MIDDLE && yaw == RC_Channel::AuxSwitchPos::HIGH
        && pitch == RC_Channel::AuxSwitchPos::MIDDLE && roll == RC_Channel::AuxSwitchPos::MIDDLE
        && (_cam_control_option & uint8_t(ControlOption::STICK_YAW_RIGHT) || is_arming_prevented())) {
        result = Event::ENTER_MENU;
    } else if (roll == RC_Channel::AuxSwitchPos::LOW) {
        result = Event::IN_MENU_EXIT;
    } else if (yaw == RC_Channel::AuxSwitchPos::MIDDLE && pitch == RC_Channel::AuxSwitchPos::MIDDLE && roll == RC_Channel::AuxSwitchPos::HIGH) {
        if (has_5_key_OSD()) {
            result = Event::IN_MENU_RIGHT;
        } else {
            result = Event::IN_MENU_ENTER;
        }
    } else if (pitch == RC_Channel::AuxSwitchPos::LOW) {
        result = Event::IN_MENU_UP;
    } else if (pitch == RC_Channel::AuxSwitchPos::HIGH) {
        result = Event::IN_MENU_DOWN;
    } else if (_video_recording != _last_video_recording) {
        switch (_video_recording) {
            case VideoOption::NOT_RECORDING:
                result = Event::STOP_RECORDING;
                break;
            case VideoOption::RECORDING:
                result = Event::START_RECORDING;
                break;
        }
    } else if (_osd_option == _last_osd_option) {
        // OSD option has not changed so assume stick re-centering
        result = Event::NONE;
    } else if (_osd_option == OSDOption::ENTER
        && _cam_control_option & uint8_t(ControlOption::TWO_POS_SWITCH)) {
        result = Event::ENTER_MENU;
    } else if ((_osd_option == OSDOption::OPTION || _osd_option == OSDOption::ENTER)
        && _cam_control_option & uint8_t(ControlOption::THREE_POS_SWITCH)) {
        result = Event::ENTER_MENU;
    } else if (_osd_option == OSDOption::EXIT
        && _cam_control_option & uint8_t(ControlOption::TWO_POS_SWITCH)) {
        result = Event::EXIT_MENU;
    } else if ((_osd_option == OSDOption::NO_OPTION || _osd_option == OSDOption::EXIT)
        && _cam_control_option & uint8_t(ControlOption::THREE_POS_SWITCH)) {
        result = Event::EXIT_MENU;
    } else {
        debug("map_rc_input_to_event(): nothing selected\n");
    }
    return result;
}

// run the 2-key OSD simulation process, this involves using the power and mode (wifi) buttons
// to cycle through options. unfortunately these are one-way requests so we need to use delays
// to make sure that the camera obeys
void AP_RunCam::handle_2_key_simulation_process(Event ev)
{
    debug("%s,M:%d,V:%d,O:%d\n", event_names[int(ev)], _in_menu, int(_video_recording), int(_osd_option));

    switch (ev) {
    case Event::ENTER_MENU:
        if (_in_menu <= 0) {
            _in_menu++;
            simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_CHANGE_MODE, _mode_delay_ms);
            if (_in_menu > 0) {
                // turn off built-in OSD so that the runcam OSD is visible
                disable_osd();
                _state = State::IN_MENU;
            } else {
                _state = State::ENTERING_MENU;
            }
        }
        break;

    case Event::EXIT_MENU:
        // keep changing mode until we are fully out of the menu
        if (_in_menu > 0) {
            _in_menu--;
            simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_CHANGE_MODE, _mode_delay_ms);
            _state = State::EXITING_MENU;
        } else {
            exit_2_key_osd_menu();
        }
        break;

    case Event::IN_MENU_ENTER:
        // in a sub-menu and save-and-exit was selected
        if (_in_menu > 1 && get_top_menu_length() > 0 && _sub_menu_pos == (get_sub_menu_length(_top_menu_pos) - 1) && DeviceType(_cam_type.get()) != DeviceType::Run24k) {
            simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN, _button_delay_ms);
            _sub_menu_pos = 0;
            _in_menu--;
        // in the top-menu and save-and-exit was selected
        } else if (_in_menu == 1 && get_top_menu_length() > 0 && _top_menu_pos == (get_top_menu_length() - 1) && DeviceType(_cam_type.get()) != DeviceType::Run24k) {
            simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN, _mode_delay_ms);
            _in_menu--;
            _state = State::EXITING_MENU;
        } else if (_top_menu_pos >= 0 && get_sub_menu_length(_top_menu_pos) > 0) {
            simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_SIMULATE_WIFI_BTN, _button_delay_ms);
            _in_menu = MIN(_in_menu + 1, RUNCAM_OSD_MENU_DEPTH);
        }
        break;

    case Event::IN_MENU_UP:
    case Event::IN_MENU_DOWN:
        simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN, _button_delay_ms);    // move to setting
        if (_in_menu > 1) {
            // in a sub-menu, keep track of the selected position
            _sub_menu_pos = (_sub_menu_pos + 1) % get_sub_menu_length(_top_menu_pos);
        } else {
            // in the top-menu, keep track of the selected position
            _top_menu_pos = (_top_menu_pos + 1) % get_top_menu_length();
        }
        break;

    case Event::IN_MENU_EXIT:
        // if we are in a sub-menu this will move us out, if we are in the root menu this will
        // exit causing the state machine to get out of sync. the OSD menu hierachy is consistently
        // 2 deep so we can count and be reasonably confident of where we are.
        // the only exception is if someone hits save and exit on the root menu - then we are lost.
        if (_in_menu > 0) {
            _in_menu--;
            _sub_menu_pos = 0;
            simulate_camera_button(ControlOperation::RCDEVICE_PROTOCOL_CHANGE_MODE, _mode_delay_ms);     // move up/out a menu
        }
        // no longer in the menu so trigger the OSD re-enablement
        if (_in_menu == 0) {
            _in_menu = _menu_enter_level;
            _state = State::EXITING_MENU;
        }
        break;

    case Event::NONE:
    case Event::IN_MENU_RIGHT:
    case Event::BUTTON_RELEASE:
    case Event::START_RECORDING:
    case Event::STOP_RECORDING:
        break;
    }
}

// exit the 2 key OSD menu
void AP_RunCam::exit_2_key_osd_menu()
{
    _in_menu = _menu_enter_level;

    // turn built-in OSD back on
    enable_osd();

    if (_video_recording == VideoOption::RECORDING && has_feature(Feature::RCDEVICE_PROTOCOL_FEATURE_START_RECORDING)) {
        simulate_camera_button(start_recording_command(), _mode_delay_ms);
        _state = State::VIDEO_RECORDING;
    } else {
        _state = State::READY;
    }
}

// run the 5-key OSD simulation process
void AP_RunCam::handle_5_key_simulation_process(Event ev) 
{
    debug("%s,M:%d,B:%d,O:%d\n", event_names[int(ev)], _in_menu, int(_button_pressed), int(_osd_option));

    switch (ev) {
    case Event::BUTTON_RELEASE:
        send_5_key_OSD_cable_simulation_event(ev);
        break;

    case Event::ENTER_MENU:
        if (_in_menu == 0) {
            // turn off built-in OSD so that the runcam OSD is visible
            disable_osd();
            send_5_key_OSD_cable_simulation_event(ev);
            _in_menu = 1;
        } else {
            send_5_key_OSD_cable_simulation_event(Event::IN_MENU_ENTER);
        }
        break;

    case Event::EXIT_MENU:
        if (_in_menu > 0) {
            // turn built-in OSD back on
            enable_osd();
            send_5_key_OSD_cable_simulation_event(Event::EXIT_MENU);
            _in_menu = 0;
        }
        break;

    case Event::NONE:
        break;

    case Event::IN_MENU_EXIT:
    case Event::IN_MENU_RIGHT:
    case Event::IN_MENU_ENTER:
    case Event::IN_MENU_UP:
    case Event::IN_MENU_DOWN:
    case Event::START_RECORDING:
    case Event::STOP_RECORDING:
        send_5_key_OSD_cable_simulation_event(ev);
        break;
    }
}

// handle a response
void AP_RunCam::handle_5_key_simulation_response(const Request& request)
{
    debug("response for command %d result: %d\n", int(request._command), int(request._result));
    if (request._result != RequestStatus::SUCCESS) {
        simulation_OSD_cable_failed(request);
        _button_pressed = ButtonState::NONE;
        _waiting_device_response = false;
        return;
    }

    switch (request._command) {
    case Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE:
        _button_pressed = ButtonState::NONE;
        break;
    case Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION:
    {
        // the high 4 bits is the operationID that we sent
        // the low 4 bits is the result code
        const ConnectionOperation operationID = ConnectionOperation(request._param);
        const uint8_t errorCode = (request._recv_buf[1] & 0x0F);
        switch (operationID) {
        case ConnectionOperation::RCDEVICE_PROTOCOL_5KEY_FUNCTION_OPEN:
            if (errorCode > 0) {
                _state = State::IN_MENU;
            }
            break;
        case ConnectionOperation::RCDEVICE_PROTOCOL_5KEY_FUNCTION_CLOSE:
            if (errorCode > 0) {
                _state = State::READY;
            }
            break;
        }
        break;
    }
    case Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS:
    case Command::RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO:
    case Command::RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL:
    case Command::COMMAND_NONE:
        break;
    }

    _waiting_device_response = false;
}

// command to start recording
AP_RunCam::ControlOperation AP_RunCam::start_recording_command() const {
    if (DeviceType(_cam_type.get()) == DeviceType::Split4k || DeviceType(_cam_type.get()) == DeviceType::Hybrid || DeviceType(_cam_type.get()) == DeviceType::Run24k) {
        return ControlOperation::RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN;
    } else {
        return ControlOperation::RCDEVICE_PROTOCOL_CHANGE_START_RECORDING;
    }
}

// command to stop recording
AP_RunCam::ControlOperation AP_RunCam::stop_recording_command() const {
    if (DeviceType(_cam_type.get()) == DeviceType::Split4k || DeviceType(_cam_type.get()) == DeviceType::Hybrid || DeviceType(_cam_type.get()) == DeviceType::Run24k) {
        return ControlOperation::RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN;
    } else {
        return ControlOperation::RCDEVICE_PROTOCOL_CHANGE_STOP_RECORDING;
    }
}

// process a response from the serial port
void AP_RunCam::receive()
{
    if (!uart) {
        return;
    }
    // process any pending request at least once-per cycle, regardless of available bytes
    if (!request_pending(AP_HAL::millis())) {
        return;
    }

    uint32_t avail = MIN(uart->available(), (uint32_t)RUNCAM_MAX_PACKET_SIZE);

    for (uint32_t i = 0; i < avail; i++) {

        if (!request_pending(AP_HAL::millis())) {
             return;
        }

        const uint8_t c = uart->read();
        if (_pending_request._recv_response_length == 0) {
            // Only start receiving packet when we found a header
            if (c != RUNCAM_HEADER) {
                continue;
            }
        }

        _pending_request._recv_buf[_pending_request._recv_response_length] = c;
        _pending_request._recv_response_length += 1;

        // if data received done, trigger callback to parse response data, and update RUNCAM state
        if (_pending_request._recv_response_length == _pending_request._expected_response_length) {
            uint8_t crc = _pending_request.get_crc();

            _pending_request._result = (crc == 0) ? RequestStatus::SUCCESS : RequestStatus::INCORRECT_CRC;

            debug("received response for command %d\n", int(_pending_request._command));
            _pending_request.parse_response();
            // we no longer have a pending request
            _pending_request._result = RequestStatus::NONE;
        }
    }
}

// every time we send a packet to device and want to get a response
// it's better to clear the rx buffer before the sending the packet
// otherwise useless data in rx buffer will cause the response decoding
// to fail
void AP_RunCam::drain()
{
    if (!uart) {
        return;
    }

    uart->discard_input();
}

// start the uart if we have one
void AP_RunCam::start_uart()
{
    // 8N1 communication
    uart->configure_parity(0);
    uart->set_stop_bits(1);
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->set_options(uart->get_options() | AP_HAL::UARTDriver::OPTION_NODMA_TX | AP_HAL::UARTDriver::OPTION_NODMA_RX);
    uart->begin(115200, 10, 10);
    uart->discard_input();
}

// get the device info (firmware version, protocol version and features)
void AP_RunCam::get_device_info()
{
    send_request_and_waiting_response(Command::RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO, 0, RUNCAM_INIT_INTERVAL_MS * 4,
        UINT16_MAX, FUNCTOR_BIND_MEMBER(&AP_RunCam::parse_device_info, void, const Request&));
}

// map a Event to a SimulationOperation
AP_RunCam::SimulationOperation AP_RunCam::map_key_to_protocol_operation(const Event key) const
{
    SimulationOperation operation = SimulationOperation::SIMULATION_NONE;
    switch (key) {
    case Event::IN_MENU_EXIT:
        operation = SimulationOperation::RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT;
        break;
    case Event::IN_MENU_UP:
        operation = SimulationOperation::RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP;
        break;
    case Event::IN_MENU_RIGHT:
        operation = SimulationOperation::RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT;
        break;
    case Event::IN_MENU_DOWN:
        operation = SimulationOperation::RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN;
        break;
    case Event::IN_MENU_ENTER:
        operation = SimulationOperation::RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET;
        break;
    case Event::BUTTON_RELEASE:
    case Event::NONE:
    case Event::ENTER_MENU:
    case Event::EXIT_MENU:
    case Event::STOP_RECORDING:
    case Event::START_RECORDING:
        break;
    }
    return operation;
}

// send an event
void AP_RunCam::send_5_key_OSD_cable_simulation_event(const Event key, const uint32_t transition_timeout)
{
    debug("OSD cable simulation event %s\n", event_names[int(key)]);
    _waiting_device_response = true;
    // although we can control press/release, this causes the state machine to behave in the same way
    // as the 2-key process
    _transition_timeout_ms = transition_timeout;

    switch (key) {
    case Event::ENTER_MENU:
        open_5_key_OSD_cable_connection(FUNCTOR_BIND_MEMBER(&AP_RunCam::handle_5_key_simulation_response, void, const Request&));
        break;
    case Event::EXIT_MENU:
        close_5_key_OSD_cable_connection(FUNCTOR_BIND_MEMBER(&AP_RunCam::handle_5_key_simulation_response, void, const Request&));
        break;
    case Event::IN_MENU_UP:
    case Event::IN_MENU_RIGHT:
    case Event::IN_MENU_DOWN:
    case Event::IN_MENU_ENTER:
    case Event::IN_MENU_EXIT:
        simulate_5_key_OSD_cable_button_press(map_key_to_protocol_operation(key), FUNCTOR_BIND_MEMBER(&AP_RunCam::handle_5_key_simulation_response, void, const Request&));
        break;
    case Event::BUTTON_RELEASE:
        simulate_5_key_OSD_cable_button_release(FUNCTOR_BIND_MEMBER(&AP_RunCam::handle_5_key_simulation_response, void, const Request&));
        break;
    case Event::STOP_RECORDING:
    case Event::START_RECORDING:
    case Event::NONE:
        break;
    }
}

// every time we run the OSD menu simulation it's necessary to open the connection
void AP_RunCam::open_5_key_OSD_cable_connection(parse_func_t parseFunc)
{
    send_request_and_waiting_response(Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION,
        uint8_t(ConnectionOperation::RCDEVICE_PROTOCOL_5KEY_FUNCTION_OPEN), 400, 2, parseFunc);
}

// every time we exit the OSD menu simulation it's necessary to close the connection
void AP_RunCam::close_5_key_OSD_cable_connection(parse_func_t parseFunc)
{
    send_request_and_waiting_response(Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION,
        uint8_t(ConnectionOperation::RCDEVICE_PROTOCOL_5KEY_FUNCTION_CLOSE), 400, 2, parseFunc);
}

// simulate button press event of 5 key OSD cable with special button
void AP_RunCam::simulate_5_key_OSD_cable_button_press(const SimulationOperation operation, parse_func_t parseFunc)
{
    if (operation == SimulationOperation::SIMULATION_NONE) {
        return;
    }

    _button_pressed = ButtonState::PRESSED;

    send_request_and_waiting_response(Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS, uint8_t(operation), 400, 2, parseFunc);
}

// simulate button release event of 5 key OSD cable
void AP_RunCam::simulate_5_key_OSD_cable_button_release(parse_func_t parseFunc)
{
    _button_pressed = ButtonState::RELEASED;

    send_request_and_waiting_response(Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE,
        uint8_t(SimulationOperation::SIMULATION_NONE), 400, 2, parseFunc);
}

// send a RunCam request and register a response to be processed
void AP_RunCam::send_request_and_waiting_response(Command commandID, uint8_t param,
    uint32_t timeout, uint16_t maxRetryTimes, parse_func_t parserFunc)
{
    drain();

    _pending_request = Request(this, commandID, param, timeout, maxRetryTimes, parserFunc);
    debug("sending command: %d, op: %d\n", int(commandID), int(param));
    // send packet
    send_packet(commandID, param);
}

// send a packet to the serial port
void AP_RunCam::send_packet(Command command, uint8_t param)
{
    // is this device open?
    if (!uart) {
        return;
    }

    uint8_t buffer[4];

    bool have_param = param > 0 || command == Command::RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL;
    uint8_t buffer_len = have_param ? 4 : 3;

    buffer[0] = RUNCAM_HEADER;
    buffer[1] = uint8_t(command);
    if (have_param) {
        buffer[2] = param;
    }

    uint8_t crc = 0;
    for (uint8_t i = 0; i < buffer_len - 1; i++) {
        crc = crc8_dvb_s2(crc, buffer[i]);
    }

    buffer[buffer_len - 1] = crc;

    // send data if possible
    uart->write(buffer, buffer_len);
    uart->flush();
}

// handle a device info response
void AP_RunCam::parse_device_info(const Request& request)
{
    _protocol_version = ProtocolVersion(request._recv_buf[1]);

    uint8_t featureLowBits = request._recv_buf[2];
    uint8_t featureHighBits = request._recv_buf[3];
    if (!has_feature(Feature::FEATURES_OVERRIDE)) {
        _features.set((featureHighBits << 8) | featureLowBits);
    }
    if (_features > 0) {
        _state = State::INITIALIZED;
        gcs().send_text(MAV_SEVERITY_INFO, "RunCam initialized, features 0x%04X, %d-key OSD\n", _features.get(),
            has_5_key_OSD() ? 5 : has_2_key_OSD() ? 2 : 0);
    } else {
        // nothing as as nothing does
        gcs().send_text(MAV_SEVERITY_WARNING, "RunCam device not found\n");
    }
    debug("RunCam: initialized state: video: %d, osd: %d, cam: %d\n", int(_video_recording), int(_osd_option), int(_cam_control_option));
}

// wait for the RunCam device to be fully ready
bool AP_RunCam::camera_ready() const
{
    if (_state != State::INITIALIZING && _state != State::INITIALIZED) {
        return true;
    }
    return false;
}

// error handler for OSD simulation
void AP_RunCam::simulation_OSD_cable_failed(const Request& request)
{
    _waiting_device_response = false;
    if (request._command == Command::RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION) {
        uint8_t operationID = request._param;
        if (operationID == uint8_t(ConnectionOperation::RCDEVICE_PROTOCOL_5KEY_FUNCTION_CLOSE)) {
            return;
        }
    }
}

// process all of the pending responses, retrying as necessary
bool AP_RunCam::request_pending(uint32_t now)
{
    if (_pending_request._result == RequestStatus::NONE) {
        return false;
    }

    if (_pending_request._request_timestamp_ms != 0 && (now - _pending_request._request_timestamp_ms) < _pending_request._timeout_ms) {
        // request still in play
        return true;
    }

    if (_pending_request._max_retry_times > 0) {
        // request timed out, so resend
        debug("retrying[%d] command 0x%X, op 0x%X\n", int(_pending_request._max_retry_times), int(_pending_request._command), int(_pending_request._param));
        start_uart();
        _pending_request._device->send_packet(_pending_request._command, _pending_request._param);
        _pending_request._recv_response_length = 0;
        _pending_request._request_timestamp_ms = now;
        _pending_request._max_retry_times -= 1;

        return false;
    }
    debug("timeout command 0x%X, op 0x%X\n", int(_pending_request._command), int(_pending_request._param));
    // too many retries, fail the request
    _pending_request._result = RequestStatus::TIMEOUT;
    _pending_request.parse_response();
    _pending_request._result = RequestStatus::NONE;

    return false;
}

// constructor for a response structure
AP_RunCam::Request::Request(AP_RunCam* device, Command commandID, uint8_t param,
    uint32_t timeout, uint16_t maxRetryTimes, parse_func_t parserFunc)
    : _recv_buf(device->_recv_buf),
    _command(commandID),
    _max_retry_times(maxRetryTimes),
    _timeout_ms(timeout),
    _device(device),
    _param(param),
    _parser_func(parserFunc),
    _recv_response_length(0),
    _result(RequestStatus::PENDING)
{
    _request_timestamp_ms = AP_HAL::millis();
    _expected_response_length = get_expected_response_length(commandID);
}

uint8_t AP_RunCam::Request::get_crc() const
{
    uint8_t crc = 0;
    for (int i = 0; i < _recv_response_length; i++) {
        crc = crc8_dvb_s2(crc, _recv_buf[i]);
    }
    return crc;
}

// get the length of a response
uint8_t AP_RunCam::Request::get_expected_response_length(const Command command) const
{
    for (uint16_t i = 0; i < RUNCAM_NUM_EXPECTED_RESPONSES; i++) {
        if (_expected_responses_length[i].command == command) {
            return _expected_responses_length[i].reponse_length;
        }
    }

    return 0;
}

AP_RunCam *AP::runcam() {
    return AP_RunCam::get_singleton();
}

#endif  // HAL_RUNCAM_ENABLED
