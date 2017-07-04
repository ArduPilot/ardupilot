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

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

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
#include <uavcan/equipment/ecu/Status.hpp>

extern const AP_HAL::HAL& hal;

#define debug_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

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
    AP_GROUPINFO("SRV_BM", 2, AP_UAVCAN, _servo_bm, 255),

    // @Param: ESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_UAVCAN, _esc_bm, 255),

    AP_GROUPEND
};

static void ecu_status_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ecu::Status>& msg)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            EFI_State *state = ap_uavcan->find_efi_node(msg.getSrcNodeID().get());

            // Conversion as described in 1120.Status.uavcan
            uint8_t ecu_index = msg.ecu_index;
            bool end_of_start = msg.end_of_start;
            bool crank_sensor_error = msg.crank_sensor_error;
            uint8_t engine_load_percent = msg.engine_load_percents;
            float spark_dwell_time_ms = msg.spark_dwell_time / 10.0;
            float engine_speed_rpm = msg.engine_speed / 8.0;
            float barometric_pressure_kpa = msg.barometric_pressure / 2.0;
            float throttle_position_percent = msg.throttle_position / 2.5;
            float coolant_temperature_degc = msg.coolant_temperature - 40.0;
            float battery_voltage_volts = msg.battery_voltage / 20.0;
            float intake_manifold_pressure_kpa = msg.intake_manifold_pressure * 2.0;
            float intake_manifold_temperature_degc = msg.intake_manifold_temperature - 40.0;
            float fuel_level_percent = msg.fuel_level / 2.5;
            float fuel_flow_grams_per_min = msg.fuel_flow / 86.251509;

            for (int i = 0; i < 4; i++) {
                state->ignition_timing_crank_angle[i] = (msg.ignition_timing[i] / 128.0) - 200.0;
            }

            for (int i = 0; i < 4; i++) {
                state->injection_time_ms[i] = msg.inject_time[i] / 1000.0;
            }

            // Copy remaining data to state struct
            state->ecu_index = ecu_index;
            state->end_of_start = end_of_start;
            state->crank_sensor_error = crank_sensor_error;
            state->engine_load_percent = engine_load_percent;
            state->spark_dwell_time_ms = spark_dwell_time_ms;
            state->rpm = engine_speed_rpm;
            state->barometric_pressure = barometric_pressure_kpa;
            state->throttle_position_percent = throttle_position_percent;
            state->coolant_temperature = coolant_temperature_degc;
            state->battery_voltage = battery_voltage_volts;
            state->intake_manifold_pressure = intake_manifold_pressure_kpa;
            state->intake_manifold_temperature = intake_manifold_temperature_degc;
            state->fuel_level_percent = fuel_level_percent;
            state->fuel_flow_rate = fuel_flow_grams_per_min;
            
            // Update listeners
            ap_uavcan->update_efi_state(msg.getSrcNodeID().get());
        } 
        
    }
}

static void gnss_fix_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
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
                        state->time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
                        state->time_week_ms = (uint32_t)(gps_ms - (state->time_week) * AP_MSEC_PER_WEEK);
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

static void gnss_fix_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{   gnss_fix_cb(msg, 0); }
static void gnss_fix_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{   gnss_fix_cb(msg, 1); }
static void (*gnss_fix_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
        = { gnss_fix_cb0, gnss_fix_cb1 };

static void gnss_aux_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
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

static void gnss_aux_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{   gnss_aux_cb(msg, 0); }
static void gnss_aux_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{   gnss_aux_cb(msg, 1); }
static void (*gnss_aux_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
        = { gnss_aux_cb0, gnss_aux_cb1 };

static void magnetic_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
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

static void magnetic_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{   magnetic_cb(msg, 0); }
static void magnetic_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{   magnetic_cb(msg, 1); }
static void (*magnetic_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
        = { magnetic_cb0, magnetic_cb1 };

static void air_data_sp_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
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

static void air_data_sp_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{   air_data_sp_cb(msg, 0); }
static void air_data_sp_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{   air_data_sp_cb(msg, 1); }
static void (*air_data_sp_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
        = { air_data_sp_cb0, air_data_sp_cb1 };

// Temperature is not main parameter so do not update listeners when it is received
static void air_data_st_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(msg.getSrcNodeID().get());

            if (state != nullptr) {
                state->temperature = msg.static_temperature;
                state->temperature_variance = msg.static_temperature_variance;
            }
        }
    }
}

static void air_data_st_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{   air_data_st_cb(msg, 0); }
static void air_data_st_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{   air_data_st_cb(msg, 1); }
static void (*air_data_st_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
        = { air_data_st_cb0, air_data_st_cb1 };

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>* act_out_array[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::esc::RawCommand>* esc_raw[MAX_NUMBER_OF_CAN_DRIVERS];

AP_UAVCAN::AP_UAVCAN() :
    _node_allocator(
        UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_SIZE)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        _rco_conf[i].active = false;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        _gps_nodes[i] = UINT8_MAX;
        _gps_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        _baro_nodes[i] = UINT8_MAX;
        _baro_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        _mag_nodes[i] = UINT8_MAX;
        _mag_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_EFI_NODES; i++) {
        _efi_nodes[i] = 255;
        _efi_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        _gps_listener_to_node[i] = UINT8_MAX;
        _gps_listeners[i] = nullptr;

        _baro_listener_to_node[i] = UINT8_MAX;
        _baro_listeners[i] = nullptr;

        _mag_listener_to_node[i] = UINT8_MAX;
        _mag_listeners[i] = nullptr;

        _efi_listener_to_node[i] = 255;
        _efi_listeners[i] = nullptr;
    }

    _rc_out_sem = hal.util->new_semaphore();

    debug_uavcan(2, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

bool AP_UAVCAN::try_init(void)
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() && !_initialized) {

            _uavcan_i = UINT8_MAX;
            for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
                if (_parent_can_mgr == hal.can_mgr[i]) {
                    _uavcan_i = i;
                    break;
                }
            }

            if(_uavcan_i == UINT8_MAX) {
                return false;
            }

            auto *node = get_node();

            if (node != nullptr) {
                if (!node->isStarted()) {
                    uavcan::NodeID self_node_id(_uavcan_node);
                    node->setNodeID(self_node_id);

                    char ndname[20];
                    snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", _uavcan_i);

                    uavcan::NodeStatusProvider::NodeName name(ndname);
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

                    uavcan::Subscriber<uavcan::equipment::ecu::Status> *ecu_status;
                    ecu_status = new uavcan::Subscriber<uavcan::equipment::ecu::Status>(*node);
                    const int ecu_status_start_res = ecu_status->start(ecu_status_cb);
                    if (ecu_status_start_res < 0) {
                        debug_uavcan(1, "UAVCAN ECU Subscriber start failure!\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::gnss::Fix> *gnss_fix;
                    gnss_fix = new uavcan::Subscriber<uavcan::equipment::gnss::Fix>(*node);
                    const int gnss_fix_start_res = gnss_fix->start(gnss_fix_cb_arr[_uavcan_i]);
                    if (gnss_fix_start_res < 0) {
                        debug_uavcan(1, "UAVCAN GNSS subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary> *gnss_aux;
                    gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary>(*node);
                    const int gnss_aux_start_res = gnss_aux->start(gnss_aux_cb_arr[_uavcan_i]);
                    if (gnss_aux_start_res < 0) {
                        debug_uavcan(1, "UAVCAN GNSS Aux subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength> *magnetic;
                    magnetic = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength>(*node);
                    const int magnetic_start_res = magnetic->start(magnetic_cb_arr[_uavcan_i]);
                    if (magnetic_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Compass subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure> *air_data_sp;
                    air_data_sp = new uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure>(*node);
                    const int air_data_sp_start_res = air_data_sp->start(air_data_sp_cb_arr[_uavcan_i]);
                    if (air_data_sp_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Baro subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature> *air_data_st;
                    air_data_st = new uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature>(*node);
                    const int air_data_st_start_res = air_data_st->start(air_data_st_cb_arr[_uavcan_i]);
                    if (air_data_st_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Temperature subscriber start problem\n\r");
                        return false;
                    }

                    act_out_array[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*node);
                    act_out_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                    act_out_array[_uavcan_i]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

                    esc_raw[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*node);
                    esc_raw[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                    esc_raw[_uavcan_i]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

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
    if (_initialized) {
        auto *node = get_node();

        const int error = node->spin(uavcan::MonotonicDuration::fromMSec(1));

        if (error < 0) {
            hal.scheduler->delay_microseconds(1000);
        } else {
            if (rc_out_sem_take()) {
                if (_rco_armed) {
                    bool repeat_send;

                    // if we have any Servos in bitmask
                    if (_servo_bm > 0) {
                        uint8_t starting_servo = 0;

                        do {
                            repeat_send = false;
                            uavcan::equipment::actuator::ArrayCommand msg;

                            uint8_t i;
                            // UAVCAN can hold maximum of 15 commands in one frame
                            for (i = 0; starting_servo < UAVCAN_RCO_NUMBER && i < 15; starting_servo++) {
                                uavcan::equipment::actuator::Command cmd;

                                /*
                                 * Servo output uses a range of 1000-2000 PWM for scaling.
                                 * This converts output PWM from [1000:2000] range to [-1:1] range that
                                 * is passed to servo as unitless type via UAVCAN.
                                 * This approach allows for MIN/TRIM/MAX values to be used fully on
                                 * autopilot side and for servo it should have the setup to provide maximum
                                 * physically possible throws at [-1:1] limits.
                                 */

                                if (_rco_conf[starting_servo].active && ((((uint32_t) 1) << starting_servo) & _servo_bm)) {
                                    cmd.actuator_id = starting_servo + 1;

                                    // TODO: other types
                                    cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;

                                    // TODO: failsafe, safety
                                    cmd.command_value = constrain_float(((float) _rco_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);

                                    msg.commands.push_back(cmd);

                                    i++;
                                }
                            }

                            if (i > 0) {
                                act_out_array[_uavcan_i]->broadcast(msg);

                                if (i == 15) {
                                    repeat_send = true;
                                }
                            }
                        } while (repeat_send);
                    }

                    // if we have any ESC's in bitmask
                    if (_esc_bm > 0) {
                        static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
                        uavcan::equipment::esc::RawCommand esc_msg;

                        uint8_t active_esc_num = 0, max_esc_num = 0;
                        uint8_t k = 0;

                        // find out how many esc we have enabled and if they are active at all
                        for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
                            if ((((uint32_t) 1) << i) & _esc_bm) {
                                max_esc_num = i + 1;
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

                            esc_raw[_uavcan_i]->broadcast(esc_msg);
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
    } else {
        hal.scheduler->delay_microseconds(1000);
    }
}

uavcan::ISystemClock & AP_UAVCAN::get_system_clock()
{
    return SystemClock::instance();
}

uavcan::ICanDriver * AP_UAVCAN::get_can_driver()
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() == false) {
            return nullptr;
        } else {
            return _parent_can_mgr;
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

uint8_t AP_UAVCAN::find_gps_without_listener(void)
{
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr && _gps_nodes[i] != UINT8_MAX) {
            return _gps_nodes[i];
        }
    }

    return UINT8_MAX;
}

uint8_t AP_UAVCAN::register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
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

uint8_t AP_UAVCAN::register_gps_listener_to_node(AP_GPS_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
            if (_gps_nodes[i] == node) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = i;
                _gps_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                break;
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
            _gps_listener_to_node[i] = UINT8_MAX;
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
        if (_gps_nodes[i] == UINT8_MAX) {
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
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
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

uint8_t AP_UAVCAN::register_baro_listener_to_node(AP_Baro_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
            if (_baro_nodes[i] == node) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = i;
                _baro_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                break;
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
            _baro_listener_to_node[i] = UINT8_MAX;
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
        if (_baro_nodes[i] == UINT8_MAX) {

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

/*
 * Find discovered not taken baro node with smallest node ID
 */
uint8_t AP_UAVCAN::find_smallest_free_baro_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_node_taken[i] == 0) {
            ret = MIN(ret, _baro_nodes[i]);
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
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

uint8_t AP_UAVCAN::register_mag_listener_to_node(AP_Compass_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
            if (_mag_nodes[i] == node) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = i;
                _mag_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                break;
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
            _mag_listener_to_node[i] = UINT8_MAX;
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
        if (_mag_nodes[i] == UINT8_MAX) {
            _mag_nodes[i] = node;
            return &_mag_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

/*
 * Find discovered not taken mag node with smallest node ID
 */
uint8_t AP_UAVCAN::find_smallest_free_mag_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_node_taken[i] == 0) {
            ret = MIN(ret, _mag_nodes[i]);
        }
    }

    return ret;
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

//EFI
uint8_t AP_UAVCAN::register_efi_listener(AP_EcotronsEFI_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = 255, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_efi_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != 255) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_UAVCAN_MAX_EFI_NODES) {
                _efi_listeners[sel_place] = new_listener;
                _efi_listener_to_node[sel_place] = preferred_channel - 1;
                _efi_node_taken[_efi_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_EcotronsEFI place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_EFI_NODES; i++) {
                if (_efi_node_taken[i] == 0) {
                    _efi_listeners[sel_place] = new_listener;
                    _efi_listener_to_node[sel_place] = i;
                    _efi_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_EFI place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

void AP_UAVCAN::remove_efi_listener(AP_EcotronsEFI_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_efi_listeners[i] == rem_listener) {
            _efi_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_efi_node_taken[_efi_listener_to_node[i]] > 0) {
                _efi_node_taken[_efi_listener_to_node[i]]--;
            }
            _efi_listener_to_node[i] = 255;
        }
    }
}

EFI_State *AP_UAVCAN::find_efi_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_EFI_NODES; i++) {
        if (_efi_nodes[i] == node) {
            return &_efi_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_EFI_NODES; i++) {
        if (_efi_nodes[i] == 255) {
            _efi_nodes[i] = node;
            return &_efi_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_efi_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_EFI_NODES; i++) {
        if (_efi_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_efi_listener_to_node[j] == i) {
                    _efi_listeners[j]->handle_efi_msg(_efi_node_state[i]);
                }
            }
        }
    }
}



#endif // HAL_WITH_UAVCAN
