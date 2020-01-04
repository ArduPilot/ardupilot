/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN
#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <uavcan/transport/can_acceptance_filter_configurator.hpp>

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan/equipment/indication/SingleLightCommand.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp>
#include <uavcan/equipment/indication/RGB565.hpp>
#include <ardupilot/indication/SafetyState.hpp>
#include <ardupilot/indication/Button.hpp>
#include <ardupilot/equipment/trafficmonitor/TrafficReport.hpp>
#include <uavcan/equipment/gnss/RTCMStream.hpp>

#include <AP_Baro/AP_Baro_UAVCAN.h>
#include <AP_RangeFinder/AP_RangeFinder_UAVCAN.h>
#include <AP_GPS/AP_GPS_UAVCAN.h>
#include <AP_BattMonitor/AP_BattMonitor_UAVCAN.h>
#include <AP_Compass/AP_Compass_UAVCAN.h>
#include <AP_Airspeed/AP_Airspeed_UAVCAN.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_OpticalFlow/AP_OpticalFlow_HereFlow.h>
#include <AP_ADSB/AP_ADSB.h>
#include "AP_UAVCAN_Server.h"
#include <AP_Logger/AP_Logger.h>

#define LED_DELAY_US 50000

extern const AP_HAL::HAL& hal;

#define debug_uavcan(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_UAVCAN, _uavcan_node, 10),

    // @Param: SRV_BM
    // @DisplayName: RC Out channels to be transmitted as servo over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over UAVCAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_UAVCAN, _servo_bm, 0),

    // @Param: ESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_UAVCAN, _esc_bm, 0),

    // @Param: SRV_RT
    // @DisplayName: Servo output rate
    // @Description: Maximum transmit rate for servo outputs
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SRV_RT", 4, AP_UAVCAN, _servo_rate_hz, 50),

    AP_GROUPEND
};

// this is the timeout in milliseconds for periodic message types. We
// set this to 1 to minimise resend of stale msgs
#define CAN_PERIODIC_TX_TIMEOUT_MS 2

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>* act_out_array[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::esc::RawCommand>* esc_raw[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::indication::LightsCommand>* rgb_led[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::indication::BeepCommand>* buzzer[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<ardupilot::indication::SafetyState>* safety_state[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>* rtcm_stream[MAX_NUMBER_OF_CAN_DRIVERS];

// subscribers

// handler SafteyButton
UC_REGISTRY_BINDER(ButtonCb, ardupilot::indication::Button);
static uavcan::Subscriber<ardupilot::indication::Button, ButtonCb> *safety_button_listener[MAX_NUMBER_OF_CAN_DRIVERS];

// handler TrafficReport
UC_REGISTRY_BINDER(TrafficReportCb, ardupilot::equipment::trafficmonitor::TrafficReport);
static uavcan::Subscriber<ardupilot::equipment::trafficmonitor::TrafficReport, TrafficReportCb> *traffic_report_listener[MAX_NUMBER_OF_CAN_DRIVERS];

// handler actuator status
UC_REGISTRY_BINDER(ActuatorStatusCb, uavcan::equipment::actuator::Status);
static uavcan::Subscriber<uavcan::equipment::actuator::Status, ActuatorStatusCb> *actuator_status_listener[MAX_NUMBER_OF_CAN_DRIVERS];

// handler ESC status
UC_REGISTRY_BINDER(ESCStatusCb, uavcan::equipment::esc::Status);
static uavcan::Subscriber<uavcan::equipment::esc::Status, ESCStatusCb> *esc_status_listener[MAX_NUMBER_OF_CAN_DRIVERS];


AP_UAVCAN::AP_UAVCAN() :
    _node_allocator()
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
        _SRV_conf[i].servo_pending = false;
    }

    debug_uavcan(2, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

AP_UAVCAN *AP_UAVCAN::get_uavcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_UAVCAN) {
        return nullptr;
    }
    return static_cast<AP_UAVCAN*>(AP::can().get_driver(driver_index));
}

void AP_UAVCAN::init(uint8_t driver_index, bool enable_filters)
{
    if (_initialized) {
        debug_uavcan(2, "UAVCAN: init called more than once\n\r");
        return;
    }

    _driver_index = driver_index;

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];
    if (can_mgr == nullptr) {
        debug_uavcan(2, "UAVCAN: init called for inexisting CAN driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_uavcan(1, "UAVCAN: CAN driver not initialized\n\r");
        return;
    }

    uavcan::ICanDriver* driver = can_mgr->get_driver();
    if (driver == nullptr) {
        debug_uavcan(2, "UAVCAN: can't get UAVCAN interface driver\n\r");
        return;
    }

    _node = new uavcan::Node<0>(*driver, SystemClock::instance(), _node_allocator);

    if (_node == nullptr) {
        debug_uavcan(1, "UAVCAN: couldn't allocate node\n\r");
        return;
    }

    if (_node->isStarted()) {
        debug_uavcan(2, "UAVCAN: node was already started?\n\r");
        return;
    }

    uavcan::NodeID self_node_id(_uavcan_node);
    _node->setNodeID(self_node_id);

    char ndname[20];
    snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", driver_index);

    uavcan::NodeStatusProvider::NodeName name(ndname);
    _node->setName(name);

    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
    _node->setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;

    const uint8_t uid_buf_len = hw_version.unique_id.capacity();
    uint8_t uid_len = uid_buf_len;
    uint8_t unique_id[uid_buf_len];


    if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
        //This is because we are maintaining a common Server Record for all UAVCAN Instances.
        //In case the node IDs are different, and unique id same, it will create
        //conflict in the Server Record.
        unique_id[uid_len - 1] += _uavcan_node;
        uavcan::copy(unique_id, unique_id + uid_len, hw_version.unique_id.begin());
    }
    _node->setHardwareVersion(hw_version);

    int start_res = _node->start();
    if (start_res < 0) {
        debug_uavcan(1, "UAVCAN: node start problem, error %d\n\r", start_res);
        return;
    }

    //Start Servers
    if (!AP::uavcan_server().init(this)) {
        debug_uavcan(1, "UAVCAN: Failed to start DNA Server\n\r");
        return;
    }

    // Roundup all subscribers from supported drivers
    AP_UAVCAN_Server::subscribe_msgs(this);
    AP_GPS_UAVCAN::subscribe_msgs(this);
    AP_Compass_UAVCAN::subscribe_msgs(this);
    AP_Baro_UAVCAN::subscribe_msgs(this);
    AP_BattMonitor_UAVCAN::subscribe_msgs(this);
    AP_Airspeed_UAVCAN::subscribe_msgs(this);
    AP_OpticalFlow_HereFlow::subscribe_msgs(this);
    AP_RangeFinder_UAVCAN::subscribe_msgs(this);

    act_out_array[driver_index] = new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*_node);
    act_out_array[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(2));
    act_out_array[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    esc_raw[driver_index] = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*_node);
    esc_raw[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(2));
    esc_raw[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    rgb_led[driver_index] = new uavcan::Publisher<uavcan::equipment::indication::LightsCommand>(*_node);
    rgb_led[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    rgb_led[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    buzzer[driver_index] = new uavcan::Publisher<uavcan::equipment::indication::BeepCommand>(*_node);
    buzzer[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    buzzer[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    safety_state[driver_index] = new uavcan::Publisher<ardupilot::indication::SafetyState>(*_node);
    safety_state[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    safety_state[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    rtcm_stream[driver_index] = new uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>(*_node);
    rtcm_stream[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    rtcm_stream[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);
    
    safety_button_listener[driver_index] = new uavcan::Subscriber<ardupilot::indication::Button, ButtonCb>(*_node);
    if (safety_button_listener[driver_index]) {
        safety_button_listener[driver_index]->start(ButtonCb(this, &handle_button));
    }

    traffic_report_listener[driver_index] = new uavcan::Subscriber<ardupilot::equipment::trafficmonitor::TrafficReport, TrafficReportCb>(*_node);
    if (traffic_report_listener[driver_index]) {
        traffic_report_listener[driver_index]->start(TrafficReportCb(this, &handle_traffic_report));
    }

    actuator_status_listener[driver_index] = new uavcan::Subscriber<uavcan::equipment::actuator::Status, ActuatorStatusCb>(*_node);
    if (actuator_status_listener[driver_index]) {
        actuator_status_listener[driver_index]->start(ActuatorStatusCb(this, &handle_actuator_status));
    }

    esc_status_listener[driver_index] = new uavcan::Subscriber<uavcan::equipment::esc::Status, ESCStatusCb>(*_node);
    if (esc_status_listener[driver_index]) {
        esc_status_listener[driver_index]->start(ESCStatusCb(this, &handle_ESC_status));
    }
    
    _led_conf.devices_count = 0;
    if (enable_filters) {
        configureCanAcceptanceFilters(*_node);
    }

    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    _node->setModeOperational();

    // Spin node for device discovery
    _node->spin(uavcan::MonotonicDuration::fromMSec(5000));

    snprintf(_thread_name, sizeof(_thread_name), "uavcan_%u", driver_index);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_UAVCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        _node->setModeOfflineAndPublish();
        debug_uavcan(1, "UAVCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;
    debug_uavcan(2, "UAVCAN: init done\n\r");
}

void AP_UAVCAN::loop(void)
{
    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        const int error = _node->spin(uavcan::MonotonicDuration::fromMSec(1));

        if (error < 0) {
            hal.scheduler->delay_microseconds(100);
            continue;
        }

        if (_SRV_armed) {
            bool sent_servos = false;

            if (_servo_bm > 0) {
                // if we have any Servos in bitmask
                uint32_t now = AP_HAL::micros();
                const uint32_t servo_period_us = 1000000UL / unsigned(_servo_rate_hz.get());
                if (now - _SRV_last_send_us >= servo_period_us) {
                    _SRV_last_send_us = now;
                    SRV_send_actuator();
                    sent_servos = true;
                    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                        _SRV_conf[i].servo_pending = false;
                    }
                }
            }

            // if we have any ESC's in bitmask
            if (_esc_bm > 0 && !sent_servos) {
                SRV_send_esc();
            }

            for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                _SRV_conf[i].esc_pending = false;
            }
        }

        led_out_send();
        buzzer_send();
        rtcm_stream_send();
        safety_state_send();
        AP::uavcan_server().verify_nodes(this);
    }
}


///// SRV output /////

void AP_UAVCAN::SRV_send_actuator(void)
{
    uint8_t starting_servo = 0;
    bool repeat_send;

    WITH_SEMAPHORE(SRV_sem);

    do {
        repeat_send = false;
        uavcan::equipment::actuator::ArrayCommand msg;

        uint8_t i;
        // UAVCAN can hold maximum of 15 commands in one frame
        for (i = 0; starting_servo < UAVCAN_SRV_NUMBER && i < 15; starting_servo++) {
            uavcan::equipment::actuator::Command cmd;

            /*
             * Servo output uses a range of 1000-2000 PWM for scaling.
             * This converts output PWM from [1000:2000] range to [-1:1] range that
             * is passed to servo as unitless type via UAVCAN.
             * This approach allows for MIN/TRIM/MAX values to be used fully on
             * autopilot side and for servo it should have the setup to provide maximum
             * physically possible throws at [-1:1] limits.
             */

            if (_SRV_conf[starting_servo].servo_pending && ((((uint32_t) 1) << starting_servo) & _servo_bm)) {
                cmd.actuator_id = starting_servo + 1;

                // TODO: other types
                cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;

                // TODO: failsafe, safety
                cmd.command_value = constrain_float(((float) _SRV_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);

                msg.commands.push_back(cmd);

                i++;
            }
        }

        if (i > 0) {
            act_out_array[_driver_index]->broadcast(msg);

            if (i == 15) {
                repeat_send = true;
            }
        }
    } while (repeat_send);
}

void AP_UAVCAN::SRV_send_esc(void)
{
    static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
    uavcan::equipment::esc::RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    WITH_SEMAPHORE(SRV_sem);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
            max_esc_num = i + 1;
            if (_SRV_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
        k = 0;

        for (uint8_t i = 0; i < max_esc_num && k < 20; i++) {
            if ((((uint32_t) 1) << i) & _esc_bm) {
                // TODO: ESC negative scaling for reverse thrust and reverse rotation
                float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[i].pulse) + 1.0) / 2.0;

                scaled = constrain_float(scaled, 0, cmd_max);

                esc_msg.cmd.push_back(static_cast<int>(scaled));
            } else {
                esc_msg.cmd.push_back(static_cast<unsigned>(0));
            }

            k++;
        }

        esc_raw[_driver_index]->broadcast(esc_msg);
    }
}

void AP_UAVCAN::SRV_push_servos()
{
    WITH_SEMAPHORE(SRV_sem);

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i)) {
            _SRV_conf[i].pulse = SRV_Channels::srv_channel(i)->get_output_pwm();
            _SRV_conf[i].esc_pending = true;
            _SRV_conf[i].servo_pending = true;
        }
    }

    _SRV_armed = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}


///// LED /////

void AP_UAVCAN::led_out_send()
{
    uint64_t now = AP_HAL::micros64();

    if ((now - _led_conf.last_update) < LED_DELAY_US) {
        return;
    }

    uavcan::equipment::indication::LightsCommand msg;
    {
        WITH_SEMAPHORE(_led_out_sem);

        if (_led_conf.devices_count == 0) {
            return;
        }

        uavcan::equipment::indication::SingleLightCommand cmd;

        for (uint8_t i = 0; i < _led_conf.devices_count; i++) {
            cmd.light_id =_led_conf.devices[i].led_index;
            cmd.color.red = _led_conf.devices[i].red >> 3;
            cmd.color.green = _led_conf.devices[i].green >> 2;
            cmd.color.blue = _led_conf.devices[i].blue >> 3;

            msg.commands.push_back(cmd);
        }
    }

    rgb_led[_driver_index]->broadcast(msg);
    _led_conf.last_update = now;
}

bool AP_UAVCAN::led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (_led_conf.devices_count >= AP_UAVCAN_MAX_LED_DEVICES) {
        return false;
    }

    WITH_SEMAPHORE(_led_out_sem);

    // check if a device instance exists. if so, break so the instance index is remembered
    uint8_t instance = 0;
    for (; instance < _led_conf.devices_count; instance++) {
        if (_led_conf.devices[instance].led_index == led_index) {
            break;
        }
    }

    // load into the correct instance.
    // if an existing instance was found in above for loop search,
    // then instance value is < _led_conf.devices_count.
    // otherwise a new one was just found so we increment the count.
    // Either way, the correct instance is the current value of instance
    _led_conf.devices[instance].led_index = led_index;
    _led_conf.devices[instance].red = red;
    _led_conf.devices[instance].green = green;
    _led_conf.devices[instance].blue = blue;

    if (instance == _led_conf.devices_count) {
        _led_conf.devices_count++;
    }

    return true;
}

// buzzer send
void AP_UAVCAN::buzzer_send()
{
    uavcan::equipment::indication::BeepCommand msg;
    WITH_SEMAPHORE(_buzzer.sem);
    uint8_t mask = (1U << _driver_index);
    if ((_buzzer.pending_mask & mask) == 0) {
        return;
    }
    _buzzer.pending_mask &= ~mask;
    msg.frequency = _buzzer.frequency;
    msg.duration = _buzzer.duration;
    buzzer[_driver_index]->broadcast(msg);
}

// buzzer support
void AP_UAVCAN::set_buzzer_tone(float frequency, float duration_s)
{
    WITH_SEMAPHORE(_buzzer.sem);
    _buzzer.frequency = frequency;
    _buzzer.duration = duration_s;
    _buzzer.pending_mask = 0xFF;
}

void AP_UAVCAN::rtcm_stream_send()
{
    WITH_SEMAPHORE(_rtcm_stream.sem);
    if (_rtcm_stream.buf == nullptr ||
        _rtcm_stream.buf->available() == 0) {
        // nothing to send
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _rtcm_stream.last_send_ms < 20) {
        // don't send more than 50 per second
        return;
    }
    _rtcm_stream.last_send_ms = now;
    uavcan::equipment::gnss::RTCMStream msg;
    uint32_t len = _rtcm_stream.buf->available();
    if (len > 128) {
        len = 128;
    }
    msg.protocol_id = uavcan::equipment::gnss::RTCMStream::PROTOCOL_ID_RTCM3;
    for (uint8_t i=0; i<len; i++) {
        uint8_t b;
        if (!_rtcm_stream.buf->read_byte(&b)) {
            return;
        }
        msg.data.push_back(b);
    }
    rtcm_stream[_driver_index]->broadcast(msg);
}

// SafetyState send
void AP_UAVCAN::safety_state_send()
{
    ardupilot::indication::SafetyState msg;
    uint32_t now = AP_HAL::millis();
    if (now - _last_safety_state_ms < 500) {
        // update at 2Hz
        return;
    }
    _last_safety_state_ms = now;
    switch (hal.util->safety_switch_state()) {
    case AP_HAL::Util::SAFETY_ARMED:
        msg.status = ardupilot::indication::SafetyState::STATUS_SAFETY_OFF;
        break;
    case AP_HAL::Util::SAFETY_DISARMED:
        msg.status = ardupilot::indication::SafetyState::STATUS_SAFETY_ON;
        break;
    default:
        // nothing to send
        return;
    }
    safety_state[_driver_index]->broadcast(msg);
}

/*
 send RTCMStream packet on all active UAVCAN drivers
*/
void AP_UAVCAN::send_RTCMStream(const uint8_t *data, uint32_t len)
{
    WITH_SEMAPHORE(_rtcm_stream.sem);
    if (_rtcm_stream.buf == nullptr) {
        // give enough space for a full round from a NTRIP server with all
        // constellations
        _rtcm_stream.buf = new ByteBuffer(2400);
    }
    if (_rtcm_stream.buf == nullptr) {
        return;
    }
    _rtcm_stream.buf->write(data, len);
}

/*
  handle Button message
 */
void AP_UAVCAN::handle_button(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ButtonCb &cb)
{
    switch (cb.msg->button) {
    case ardupilot::indication::Button::BUTTON_SAFETY: {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (brdconfig && brdconfig->safety_button_handle_pressed(cb.msg->press_time)) {
            AP_HAL::Util::safety_state state = hal.util->safety_switch_state();
            if (state == AP_HAL::Util::SAFETY_ARMED) {
                hal.rcout->force_safety_on();
            } else {
                hal.rcout->force_safety_off();
            }
        }
        break;
    }
    }
}

/*
  handle traffic report
 */
void AP_UAVCAN::handle_traffic_report(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TrafficReportCb &cb)
{
    AP_ADSB *adsb = AP::ADSB();
    if (!adsb || !adsb->enabled()) {
        // ADSB not enabled
        return;
    }

    const ardupilot::equipment::trafficmonitor::TrafficReport &msg = cb.msg[0];
    AP_ADSB::adsb_vehicle_t vehicle;
    mavlink_adsb_vehicle_t &pkt = vehicle.info;

    pkt.ICAO_address = msg.icao_address;
    pkt.tslc = msg.tslc;
    pkt.lat = msg.latitude_deg_1e7;
    pkt.lon = msg.longitude_deg_1e7;
    pkt.altitude = msg.alt_m * 1000;
    pkt.heading = degrees(msg.heading) * 100;
    pkt.hor_velocity = norm(msg.velocity[0], msg.velocity[1]) * 100;
    pkt.ver_velocity = -msg.velocity[2] * 100;
    pkt.squawk = msg.squawk;
    for (uint8_t i=0; i<9; i++) {
        pkt.callsign[i] = msg.callsign[i];
    }
    pkt.emitter_type = msg.traffic_type;

    if (msg.alt_type == ardupilot::equipment::trafficmonitor::TrafficReport::ALT_TYPE_PRESSURE_AMSL) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
    } else if (msg.alt_type == ardupilot::equipment::trafficmonitor::TrafficReport::ALT_TYPE_WGS84) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
    }

    if (msg.lat_lon_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (msg.heading_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (msg.velocity_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (msg.callsign_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    }
    if (msg.ident_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_SQUAWK;
    }
    if (msg.simulated_report) {
        pkt.flags |= ADSB_FLAGS_SIMULATED;
    }
    // flags not in common.xml yet
    if (msg.vertical_velocity_valid) {
        pkt.flags |= 0x80;
    }
    if (msg.baro_valid) {
        pkt.flags |= 0x100;
    }

    vehicle.last_update_ms = AP_HAL::millis() - (vehicle.info.tslc * 1000);
    adsb->handle_adsb_vehicle(vehicle);
}

/*
  handle actuator status message
 */
void AP_UAVCAN::handle_actuator_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorStatusCb &cb)
{
    // log as CSRV message
    AP::logger().Write_ServoStatus(AP_HAL::micros64(),
                                   cb.msg->actuator_id,
                                   cb.msg->position,
                                   cb.msg->force,
                                   cb.msg->speed,
                                   cb.msg->power_rating_pct);
}

/*
  handle ESC status message
 */
void AP_UAVCAN::handle_ESC_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ESCStatusCb &cb)
{
    // log as CESC message
    AP::logger().Write_ESCStatus(AP_HAL::micros64(),
                                 cb.msg->esc_index,
                                 cb.msg->error_count,
                                 cb.msg->voltage,
                                 cb.msg->current,
                                 cb.msg->temperature - C_TO_KELVIN,
                                 cb.msg->rpm,
                                 cb.msg->power_rating_pct);

}

#endif // HAL_WITH_UAVCAN
