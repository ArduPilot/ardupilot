/*
 * AP_UAVCAN.cpp
 *
 *      Author: Eugene Shamaev
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

// Zubax GPS and other GPS, baro, magnetic sensors
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define debug_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// TODO: temperature can come not only from baro. There should be separation on node ID
// to check where it belongs to. If it is not baro that is the source, separate layer
// of listeners/nodes should be added.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for Ardupilot
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_UAVCAN, _uavcan_node, 10),

    AP_GROUPEND
};

static uavcan::Subscriber<uavcan::equipment::gnss::Fix> *gnss_fix;
static void gnss_fix_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_GPS::GPS_State *state = ap_uavcan->find_gps_node(msg.getSrcNodeID().get());

            if (state != nullptr) {
                bool process = false;

                if (msg.status == uavcan::equipment::gnss::Fix::STATUS_NO_FIX) {
                    state->status = AP_GPS::GPS_Status::NO_FIX;
                } else {
                    if (msg.status == uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY) {
                        state->status = AP_GPS::GPS_Status::NO_FIX;
                    } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_2D_FIX) {
                        state->status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
                        process = true;
                    } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_3D_FIX) {
                        state->status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
                        process = true;
                    }

                    if (msg.gnss_time_standard == uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC) {
                        uint64_t epoch_ms = uavcan::UtcTime(msg.gnss_timestamp).toUSec();
                        epoch_ms /= 1000;
                        uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
                        state->time_week = (uint16_t)(gps_ms / MSEC_PER_WEEK);
                        state->time_week_ms = (uint32_t)(gps_ms - (state->time_week) * MSEC_PER_WEEK);
                    }
                }

                if (process) {
                    Location loc = { };
                    loc.lat = msg.latitude_deg_1e8 / 10;
                    loc.lng = msg.longitude_deg_1e8 / 10;
                    loc.alt = msg.height_msl_mm / 10;
                    state->location = loc;
                    state->location.options = 0;

                    if (!uavcan::isNaN(msg.ned_velocity[0])) {
                        Vector3f vel(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);
                        state->velocity = vel;
                        state->ground_speed = norm(vel.x, vel.y);
                        state->ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
                        state->have_vertical_velocity = true;
                    } else {
                        state->have_vertical_velocity = false;
                    }

                    float pos_cov[9];
                    msg.position_covariance.unpackSquareMatrix(pos_cov);
                    if (!uavcan::isNaN(pos_cov[8])) {
                        if (pos_cov[8] > 0) {
                            state->vertical_accuracy = sqrtf(pos_cov[8]);
                            state->have_vertical_accuracy = true;
                        } else {
                            state->have_vertical_accuracy = false;
                        }
                    } else {
                        state->have_vertical_accuracy = false;
                    }

                    const float horizontal_pos_variance = MAX(pos_cov[0], pos_cov[4]);
                    if (!uavcan::isNaN(horizontal_pos_variance)) {
                        if (horizontal_pos_variance > 0) {
                            state->horizontal_accuracy = sqrtf(horizontal_pos_variance);
                            state->have_horizontal_accuracy = true;
                        } else {
                            state->have_horizontal_accuracy = false;
                        }
                    } else {
                        state->have_horizontal_accuracy = false;
                    }

                    float vel_cov[9];
                    msg.velocity_covariance.unpackSquareMatrix(vel_cov);
                    if (!uavcan::isNaN(vel_cov[0])) {
                        state->speed_accuracy = sqrtf((vel_cov[0] + vel_cov[4] + vel_cov[8]) / 3.0);
                        state->have_speed_accuracy = true;
                    } else {
                        state->have_speed_accuracy = false;
                    }

                    state->num_sats = msg.sats_used;
                } else {
                    state->have_vertical_velocity = false;
                    state->have_vertical_accuracy = false;
                    state->have_horizontal_accuracy = false;
                    state->have_speed_accuracy = false;
                    state->num_sats = 0;
                }

                state->last_gps_time_ms = AP_HAL::millis();

                // after all is filled, update all listeners with new data
                ap_uavcan->update_gps_state(msg.getSrcNodeID().get());
            }
        }
    }
}

static uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary> *gnss_aux;
static void gnss_aux_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_GPS::GPS_State *state = ap_uavcan->find_gps_node(msg.getSrcNodeID().get());

            if (state != nullptr) {
                if (!uavcan::isNaN(msg.hdop)) {
                    state->hdop = msg.hdop * 100.0;
                }

                if (!uavcan::isNaN(msg.vdop)) {
                    state->vdop = msg.vdop * 100.0;
                }
            }
        }
    }
}

static uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength> *magnetic;
static void magnetic_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_UAVCAN::Mag_Info *state = ap_uavcan->find_mag_node(msg.getSrcNodeID().get());
            if (state != nullptr) {
                state->mag_vector[0] = msg.magnetic_field_ga[0];
                state->mag_vector[1] = msg.magnetic_field_ga[1];
                state->mag_vector[2] = msg.magnetic_field_ga[2];

                // after all is filled, update all listeners with new data
                ap_uavcan->update_mag_state(msg.getSrcNodeID().get());
            }
        }
    }
}

static uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure> *air_data_sp;
static void air_data_sp_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(msg.getSrcNodeID().get());
            if (state != nullptr) {
                state->pressure = msg.static_pressure;
                state->pressure_variance = msg.static_pressure_variance;

                // after all is filled, update all listeners with new data
                ap_uavcan->update_baro_state(msg.getSrcNodeID().get());
            }
        }
    }
}

// Temperature is not main parameter so do not update listeners when it is received
static uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature> *air_data_st;
static void air_data_st_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(msg.getSrcNodeID().get());
            if (state != nullptr) {
                state->temperature = msg.static_temperature;
                state->temperature_variance = msg.static_temperature_variance;
            }
        }
    }
}

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> *act_out_array;
static uavcan::Publisher<uavcan::equipment::esc::RawCommand> *esc_raw;

AP_UAVCAN::AP_UAVCAN() :
    _initialized(false), _rco_armed(false), _rco_safety(false), _rc_out_sem(nullptr), _node_allocator(
        UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_SIZE)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        _rco_conf[i].active = false;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        _gps_nodes[i] = 255;
        _gps_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        _baro_nodes[i] = 255;
        _baro_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        _mag_nodes[i] = 255;
        _mag_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        _gps_listener_to_node[i] = 255;
        _gps_listeners[i] = nullptr;

        _baro_listener_to_node[i] = 255;
        _baro_listeners[i] = nullptr;

        _mag_listener_to_node[i] = 255;
        _mag_listeners[i] = nullptr;
    }

    _rc_out_sem = hal.util->new_semaphore();

    debug_uavcan(2, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

bool AP_UAVCAN::try_init(void)
{
    if (hal.can_mgr != nullptr) {
        if (hal.can_mgr->is_initialized() && !_initialized) {
            auto *node = get_node();

            if (node != nullptr) {
                if (!node->isStarted()) {
                    uavcan::NodeID self_node_id(_uavcan_node);
                    node->setNodeID(self_node_id);

                    uavcan::NodeStatusProvider::NodeName name("org.ardupilot");
                    node->setName(name);

                    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
                    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
                    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
                    node->setSoftwareVersion(sw_version);

                    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

                    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
                    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;
                    node->setHardwareVersion(hw_version);

                    const int node_start_res = node->start();
                    if (node_start_res < 0) {
                        debug_uavcan(1, "UAVCAN: node start problem\n\r");
                    }

                    gnss_fix = new uavcan::Subscriber<uavcan::equipment::gnss::Fix>(*node);
                    const int gnss_fix_start_res = gnss_fix->start(gnss_fix_cb);
                    if (gnss_fix_start_res < 0) {
                        debug_uavcan(1, "UAVCAN GNSS subscriber start problem\n\r");
                        return false;
                    }

                    gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary>(*node);
                    const int gnss_aux_start_res = gnss_aux->start(gnss_aux_cb);
                    if (gnss_aux_start_res < 0) {
                        debug_uavcan(1, "UAVCAN GNSS Aux subscriber start problem\n\r");
                        return false;
                    }

                    magnetic = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength>(*node);
                    const int magnetic_start_res = magnetic->start(magnetic_cb);
                    if (magnetic_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Compass subscriber start problem\n\r");
                        return false;
                    }

                    air_data_sp = new uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure>(*node);
                    const int air_data_sp_start_res = air_data_sp->start(air_data_sp_cb);
                    if (air_data_sp_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Baro subscriber start problem\n\r");
                        return false;
                    }

                    air_data_st = new uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature>(*node);
                    const int air_data_st_start_res = air_data_st->start(air_data_st_cb);
                    if (air_data_st_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Temperature subscriber start problem\n\r");
                        return false;
                    }

                    act_out_array = new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*node);
                    act_out_array->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                    act_out_array->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

                    esc_raw = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*node);
                    esc_raw->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                    esc_raw->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

                    /*
                     * Informing other nodes that we're ready to work.
                     * Default mode is INITIALIZING.
                     */
                    node->setModeOperational();

                    _initialized = true;

                    debug_uavcan(1, "UAVCAN: init done\n\r");

                    return true;
                }
            }
        }

        if (_initialized) {
            return true;
        }
    }

    return false;
}

bool AP_UAVCAN::rc_out_sem_take()
{
    bool sem_ret = _rc_out_sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN RCOut semaphore fail\n\r");
    }
    return sem_ret;
}

void AP_UAVCAN::rc_out_sem_give()
{
    _rc_out_sem->give();
}

void AP_UAVCAN::do_cyclic(void)
{

    uint32_t _servo_bm = SRV_Channels::get_can_servo_bm();
    uint32_t _esc_bm = SRV_Channels::get_can_esc_bm();

    if (_initialized) {
        auto *node = get_node();
        const int error = node->spin(uavcan::MonotonicDuration::fromMSec(1));
        if (error < 0) {
            hal.scheduler->delay_microseconds(1000);
        } else {
            if (rc_out_sem_take()) {
                if (_rco_armed) {
                    // TODO: several transmittions if did not fit in 15 servos
                    uavcan::equipment::actuator::ArrayCommand msg;

                    uint8_t k = 0;
                    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER && k < 15; i++) {
                        uavcan::equipment::actuator::Command cmd;

                        /*
                         * Servo output uses a range of 1000-2000 PWM for scaling.
                         * This converts output PWM from [1000:2000] range to [-1:1] range that
                         * is passed to servo as unitless type via UAVCAN.
                         * This approach allows for MIN/TRIM/MAX values to be used fully on
                         * autopilot side and for servo it should have the setup to provide maximum
                         * physically possible throws at [-1:1] limits.
                         */

                        if (_rco_conf[i].active && ((((uint32_t) 1) << i) & _servo_bm)) {
                            cmd.actuator_id = i + 1;

                            // TODO: other types
                            cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;

                            // TODO: failsafe, safety
                            cmd.command_value = constrain_value(((float) _rco_conf[i].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);

                            msg.commands.push_back(cmd);

                            // UAVCAN can hold maximum of 15 commands in one frame
                            k++;
                        }
                    }

                    if (msg.commands.size() > 0) {
                        act_out_array->broadcast(msg);
                    }

                    // if we have any ESC's in bitmask
                    if (_esc_bm > 0) {
                        static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
                        uavcan::equipment::esc::RawCommand esc_msg;

                        uint8_t active_esc_num = 0, max_esc_num = 0;

                        // find out how many esc we have enabled and if they are active at all
                        for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
                            if ((((uint32_t) 1) << i) & _esc_bm) {
                                max_esc_num = i;
                                if (_rco_conf[i].active) {
                                    active_esc_num++;
                                }
                            }
                        }

                        // if at least one is active (update) we need to send to all
                        if (active_esc_num > 0) {
                            k = 0;

                            for (uint8_t i = 0; i < max_esc_num && k < 20; i++) {
                                uavcan::equipment::actuator::Command cmd;

                                if ((((uint32_t) 1) << i) & _esc_bm) {
                                    // TODO: ESC negative scaling for reverse thrust and reverse rotation
                                    float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_rco_conf[i].pulse) + 1.0) / 2.0;

                                    scaled = constrain_float(scaled, 0, cmd_max);

                                    esc_msg.cmd.push_back(static_cast<int>(scaled));
                                } else {
                                    esc_msg.cmd.push_back(static_cast<unsigned>(0));
                                }

                                k++;
                            }

                            esc_raw->broadcast(esc_msg);
                        }
                    }
                }

                for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
                    // mark as transmitted
                    _rco_conf[i].active = false;
                }

                rc_out_sem_give();
            }
        }
    }
}

uavcan::ISystemClock & AP_UAVCAN::get_system_clock()
{
    return SystemClock::instance();
}

uavcan::ICanDriver * AP_UAVCAN::get_can_driver()
{
    if (hal.can_mgr != nullptr) {
        if (hal.can_mgr->is_initialized() == false) {
            return nullptr;
        } else {
            return hal.can_mgr;
        }
    }
    return nullptr;
}

uavcan::Node<0> *AP_UAVCAN::get_node()
{
    if (_node == nullptr && get_can_driver() != nullptr) {
        _node = new uavcan::Node<0>(*get_can_driver(), get_system_clock(), _node_allocator);
    }

    return _node;
}

void AP_UAVCAN::rco_set_safety_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _rco_conf[i].safety_pulse = pulse_len;
        }
    }
}

void AP_UAVCAN::rco_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _rco_conf[i].failsafe_pulse = pulse_len;
        }
    }
}

void AP_UAVCAN::rco_force_safety_on(void)
{
    _rco_safety = true;
}

void AP_UAVCAN::rco_force_safety_off(void)
{
    _rco_safety = false;
}

void AP_UAVCAN::rco_arm_actuators(bool arm)
{
    _rco_armed = arm;
}

void AP_UAVCAN::rco_write(uint16_t pulse_len, uint8_t ch)
{
    _rco_conf[ch].pulse = pulse_len;
    _rco_conf[ch].active = true;
}

uint8_t AP_UAVCAN::register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = 255, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != 255) {
        if (preferred_channel != 0) {
            if (preferred_channel <= AP_UAVCAN_MAX_GPS_NODES) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = preferred_channel - 1;
                _gps_node_taken[_gps_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
                if (_gps_node_taken[i] == 0) {
                    _gps_listeners[sel_place] = new_listener;
                    _gps_listener_to_node[sel_place] = i;
                    _gps_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

void AP_UAVCAN::remove_gps_listener(AP_GPS_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == rem_listener) {
            _gps_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_gps_node_taken[_gps_listener_to_node[i]] > 0) {
                _gps_node_taken[_gps_listener_to_node[i]]--;
            }
            _gps_listener_to_node[i] = 255;
        }
    }
}

AP_GPS::GPS_State *AP_UAVCAN::find_gps_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            return &_gps_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == 255) {
            _gps_nodes[i] = node;
            return &_gps_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_gps_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_gps_listener_to_node[j] == i) {
                    _gps_listeners[j]->handle_gnss_msg(_gps_node_state[i]);
                }
            }
        }
    }
}

uint8_t AP_UAVCAN::register_baro_listener(AP_Baro_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = 255, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != 255) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_UAVCAN_MAX_BARO_NODES) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = preferred_channel - 1;
                _baro_node_taken[_baro_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_Baro place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
                if (_baro_node_taken[i] == 0) {
                    _baro_listeners[sel_place] = new_listener;
                    _baro_listener_to_node[sel_place] = i;
                    _baro_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

void AP_UAVCAN::remove_baro_listener(AP_Baro_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == rem_listener) {
            _baro_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_baro_node_taken[_baro_listener_to_node[i]] > 0) {
                _baro_node_taken[_baro_listener_to_node[i]]--;
            }
            _baro_listener_to_node[i] = 255;
        }
    }
}

AP_UAVCAN::Baro_Info *AP_UAVCAN::find_baro_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            return &_baro_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == 255) {
            _baro_nodes[i] = node;
            return &_baro_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_baro_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_baro_listener_to_node[j] == i) {
                    _baro_listeners[j]->handle_baro_msg(_baro_node_state[i].pressure, _baro_node_state[i].temperature);
                }
            }
        }
    }
}

uint8_t AP_UAVCAN::register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = 255, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != 255) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_UAVCAN_MAX_MAG_NODES) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = preferred_channel - 1;
                _mag_node_taken[_mag_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_Compass place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
                if (_mag_node_taken[i] == 0) {
                    _mag_listeners[sel_place] = new_listener;
                    _mag_listener_to_node[sel_place] = i;
                    _mag_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

void AP_UAVCAN::remove_mag_listener(AP_Compass_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == rem_listener) {
            _mag_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_mag_node_taken[_mag_listener_to_node[i]] > 0) {
                _mag_node_taken[_mag_listener_to_node[i]]--;
            }
            _mag_listener_to_node[i] = 255;
        }
    }
}

AP_UAVCAN::Mag_Info *AP_UAVCAN::find_mag_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == node) {
            return &_mag_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == 255) {
            _mag_nodes[i] = node;
            return &_mag_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_mag_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_mag_listener_to_node[j] == i) {
                    _mag_listeners[j]->handle_mag_msg(_mag_node_state[i].mag_vector);
                }
            }
        }
    }
}

#endif // HAL_WITH_UAVCAN
