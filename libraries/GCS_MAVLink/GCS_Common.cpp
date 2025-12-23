/*
  Common GCS MAVLink functions for all vehicle types

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

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RCTelemetry/AP_Spektrum_Telem.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_EFI/AP_EFI.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Scripting/AP_Scripting.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Winch/AP_Winch.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AIS/AP_AIS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_LandingGear/AP_LandingGear.h>
#include <AP_Landing/AP_Landing_config.h>
#include <AP_Generator/AP_Generator_Loweheiser.h>

#include "MissionItemProtocol_Waypoints.h"
#include "MissionItemProtocol_Rally.h"
#include "MissionItemProtocol_Fence.h"

#include <AP_CANManager/AP_MAVLinkCAN.h>

#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle_config.h>

#include <stdio.h>

#if AP_RADIO_ENABLED
#include <AP_Radio/AP_Radio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
  #include <AP_CANManager/AP_CANManager.h>
  #include <AP_Common/AP_Common.h>

  #include <AP_PiccoloCAN/AP_PiccoloCAN.h>
  #include <AP_DroneCAN/AP_DroneCAN.h>
#endif

#include <AP_BattMonitor/AP_BattMonitor_config.h>
#if AP_BATTERY_ENABLED
#include <AP_BattMonitor/AP_BattMonitor.h>
#endif
#include <AP_GPS/AP_GPS.h>

#include <AP_RCProtocol/AP_RCProtocol_config.h>
#if AP_RCPROTOCOL_ENABLED
#include <AP_RCProtocol/AP_RCProtocol.h>
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#include "GCS_FTP.h"

#include <ctype.h>

extern const AP_HAL::HAL& hal;

struct GCS_MAVLINK::LastRadioStatus GCS_MAVLINK::last_radio_status;
mavlink_channel_mask_t GCS_MAVLINK::mavlink_active = 0;
mavlink_channel_mask_t GCS_MAVLINK::chan_is_streaming = 0;
uint32_t GCS_MAVLINK::reserve_param_space_start_ms;

// private channels are ones used for point-to-point protocols, and
// don't get broadcasts or fwded packets
mavlink_channel_mask_t GCS_MAVLINK::mavlink_private = 0;

GCS *GCS::_singleton = nullptr;

GCS_MAVLINK_InProgress GCS_MAVLINK_InProgress::in_progress_tasks[1];
uint32_t GCS_MAVLINK_InProgress::last_check_ms;

GCS_MAVLINK::GCS_MAVLINK(AP_HAL::UARTDriver &uart)
{
    AP_Param::setup_object_defaults(this, var_info);

    _port = &uart;
}

bool GCS_MAVLINK::init(uint8_t instance)
{
    // get associated mavlink channel
    chan = (mavlink_channel_t)(MAVLINK_COMM_0 + instance);
    if (!valid_channel(chan)) {
        return false;
    }

    // find instance of MAVLink protocol; the protocol_match method in
    // AP_SerialManager means this will match MAVLink2 and MAVLinkHL,
    // too:
    uartstate = AP::serialmanager().find_protocol_instance(AP_SerialManager::SerialProtocol_MAVLink, instance);
    if (uartstate == nullptr) {
        return false;
    }

    // PARAMETER_CONVERSION - Added: May-2025 for ArduPilot-4.7
    // convert parameters; we used to use bits in the UARTDriver to
    // remember whether the mavlink connection on that interface was
    // "private" or not, and whether to ignore streamrate sets via
    // REQUEST_DATA_STREAM.  We moved that into the MAVn_OPTIONS, this
    // is the conversion:
    if (!options_were_converted) {
        auto &sm = AP::serialmanager();
        options_were_converted.set_and_save(1);
        if (_port->option_is_set(AP_HAL::UARTDriver::Option::OPTION_MAVLINK_NO_FORWARD_old)) {
            enable_option(Option::NO_FORWARD);
            // turn the bit off in the UART options.  We can only get
            // away with this because we are (in a very nearby commit)
            // changing the width of the SERIALn_OPTIONS.  This means
            // older firmwares will still see the SERIALn_OPTIONS bits
            // as set, newer firmwares will see zeroes.  We have a
            // prearm check that users have not set the old bit.
            // Sorry.
            sm.disable_option(uartstate->idx, AP_HAL::UARTDriver::Option::OPTION_MAVLINK_NO_FORWARD_old);
        }
        if (_port->option_is_set(AP_HAL::UARTDriver::Option::OPTION_NOSTREAMOVERRIDE_old)) {
            enable_option(Option::NOSTREAMOVERRIDE);
            // turn the bit off in the UART options.  We can only get
            // away with this because we are (in a very nearby commit)
            // changing the width of the SERIALn_OPTIONS.  This means
            // older firmwares will still see the SERIALn_OPTIONS bits
            // as set, newer firmwares will see zeroes.  We have a
            // prearm check that users have not set the old bit.
            // Sorry.
            sm.disable_option(uartstate->idx, AP_HAL::UARTDriver::Option::OPTION_NOSTREAMOVERRIDE_old);
        }
    }

    // and init the gcs instance

    // whether this port is considered "private":
    if (option_enabled(Option::NO_FORWARD)) {
        set_channel_private(chan);
    }

    /*
      Now try to cope with SiK radios that may be stuck in bootloader
      mode because CTS was held while powering on. This tells the
      bootloader to wait for a firmware. It affects any SiK radio with
      CTS connected that is externally powered. To cope we send 0x30
      0x20 at 115200 on startup, which tells the bootloader to reset
      and boot normally
     */
    _port->begin(115200);
    AP_HAL::UARTDriver::flow_control old_flow_control = _port->get_flow_control();
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    for (uint8_t i=0; i<3; i++) {
        hal.scheduler->delay(1);
        _port->write(0x30);
        _port->write(0x20);
    }
    // since tcdrain() and TCSADRAIN may not be implemented...
    hal.scheduler->delay(1);
    
    _port->set_flow_control(old_flow_control);

    // now change back to desired baudrate
    _port->begin(uartstate->baudrate());

    mavlink_comm_port[chan] = _port;

    const auto mavlink_protocol = uartstate->get_protocol();

    if (mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLink2 ||
        mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLinkHL) {
#if AP_MAVLINK_SIGNING_ENABLED
        // load signing key
        load_signing_key();
#endif  // AP_MAVLINK_SIGNING_ENABLED
    } else {
        // user has asked to only send MAVLink1
        _channel_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }

#if HAL_HIGH_LATENCY2_ENABLED
    if (mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLinkHL) {
        is_high_latency_link = true;
    }
#endif
    return true;
}

void GCS_MAVLINK::send_meminfo(void)
{
    unsigned __brkval = 0;
    uint32_t memory = hal.util->available_memory();
    mavlink_msg_meminfo_send(chan, __brkval, MIN(memory, 0xFFFFU), memory);
}

// report power supply status
void GCS_MAVLINK::send_power_status(void)
{
    if (!gcs().vehicle_initialised()) {
        // avoid unnecessary errors being reported to user
        return;
    }
    mavlink_msg_power_status_send(chan,
                                  hal.analogin->board_voltage() * 1000,
                                  hal.analogin->servorail_voltage() * 1000,
                                  hal.analogin->power_status_flags());
}

#if AP_SCHEDULER_ENABLED
// cap the MAVLink message rate. It can't be greater than 0.8 * SCHED_LOOP_RATE
uint16_t GCS_MAVLINK::cap_message_interval(uint16_t interval_ms) const
{
    if (interval_ms == 0) {
        return 0;
    }
    if (interval_ms*800 < AP::scheduler().get_loop_period_us()) {
        return AP::scheduler().get_loop_period_us()/800.0f;
    }
    return interval_ms;
}
#endif

#if HAL_WITH_MCU_MONITORING
// report MCU voltage/temperature status
void GCS_MAVLINK::send_mcu_status(void)
{
    if (!gcs().vehicle_initialised()) {
        // avoid unnecessary errors being reported to user
        return;
    }
    mavlink_msg_mcu_status_send(chan,
                                0, // only one MCU
                                hal.analogin->mcu_temperature() * 100,
                                hal.analogin->mcu_voltage() * 1000,
                                hal.analogin->mcu_voltage_min() * 1000,
                                hal.analogin->mcu_voltage_max() * 1000);
}
#endif

#if AP_BATTERY_ENABLED
// returns the battery remaining percentage if valid, -1 otherwise
int8_t GCS_MAVLINK::battery_remaining_pct(const uint8_t instance) const {
    uint8_t percentage;
    return AP::battery().capacity_remaining_pct(percentage, instance) ? MIN(percentage, INT8_MAX) : -1;
}

void GCS_MAVLINK::send_battery_status(const uint8_t instance) const
{
    // catch the battery backend not supporting the required number of cells
    static_assert(sizeof(AP_BattMonitor::cells) >= (sizeof(uint16_t) * MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN),
                  "Not enough battery cells for the MAVLink message");

    const AP_BattMonitor &battery = AP::battery();
    float temp;
    bool got_temperature = battery.get_temperature(temp, instance);

    // prepare arrays of individual cell voltages
    uint16_t cell_mvolts[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN];
    uint16_t cell_mvolts_ext[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN];
    const uint16_t max_cell_mV = 0xFFFEU;
    const uint16_t invalid_cell_mV = 0xFFFFU;

    if (battery.has_cell_voltages(instance)) {
        const AP_BattMonitor::cells& batt_cells = battery.get_cell_voltages(instance);
        static_assert(sizeof(cell_mvolts) <= sizeof(batt_cells.cells), "cell array length not large enough");

        // copy the first 10 cells
        memcpy(cell_mvolts, batt_cells.cells, sizeof(cell_mvolts));
        // 11 ... 14 use a second cell_volts_ext array
        for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN; i++) {
            if (MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN+i < uint8_t(ARRAY_SIZE(batt_cells.cells))) {
                cell_mvolts_ext[i] = batt_cells.cells[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN+i];
            } else {
                cell_mvolts_ext[i] = 0;
            }
        }
        /*
          now adjust voltages to cope with two things:
             1) we may be reporting sag corrected voltage
             2) the battery may have more cells than can be reported by the backend, so the actual voltage may be higher than the sum
        */
        const float voltage_mV = battery.gcs_voltage(instance) * 1e3f;
        float voltage_mV_sum = 0;
        uint8_t non_zero_cell_count = 0;
        for (uint8_t i=0; i<MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; i++) {
            if (cell_mvolts[i] > 0 && cell_mvolts[i] != invalid_cell_mV) {
                non_zero_cell_count++;
                voltage_mV_sum += cell_mvolts[i];
            }
        }
        for (uint8_t i=0; i<MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN; i++) {
            if (cell_mvolts_ext[i] > 0 && cell_mvolts_ext[i] != invalid_cell_mV) {
                non_zero_cell_count++;
                voltage_mV_sum += cell_mvolts_ext[i];
            }
        }
        if (voltage_mV > voltage_mV_sum && non_zero_cell_count > 0) {
            // distribute the extra voltage over the non-zero cells
            uint32_t extra_mV = (voltage_mV - voltage_mV_sum) / non_zero_cell_count;
            for (uint8_t i=0; i<MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; i++) {
                if (cell_mvolts[i] > 0 && cell_mvolts[i] != invalid_cell_mV) {
                    cell_mvolts[i] = MIN(cell_mvolts[i] + extra_mV, max_cell_mV);
                }
            }
            for (uint8_t i=0; i<MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN; i++) {
                if (cell_mvolts_ext[i] > 0 && cell_mvolts_ext[i] != invalid_cell_mV) {
                    cell_mvolts_ext[i] = MIN(cell_mvolts_ext[i] + extra_mV, max_cell_mV);
                }
            }
        }
    } else {
        // for battery monitors that cannot provide voltages for individual cells the battery's total voltage is put into the first cell
        // if the total voltage cannot fit into a single field, the remainder into subsequent fields.
        // the GCS can then recover the pack voltage by summing all non ignored cell values an we can report a pack up to 655.34 V
        float voltage_mV = battery.gcs_voltage(instance) * 1e3f;
        for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; i++) {
          if (voltage_mV < 0.001f) {
              // too small to send to the GCS, set it to the no cell value
              cell_mvolts[i] = UINT16_MAX;
          } else {
              cell_mvolts[i] = MIN(voltage_mV, max_cell_mV); // Can't send more then UINT16_MAX - 1 in a cell
              voltage_mV -= max_cell_mV;
          }
        }
        for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN; i++) {
            cell_mvolts_ext[i] = 0;
        }
    }

    float current, consumed_mah, consumed_wh;
    const int8_t percentage = battery_remaining_pct(instance);
    
    if (battery.current_amps(current, instance)) {
         current = constrain_float(current * 100,-INT16_MAX,INT16_MAX);
    } else {
        current = -1;
    }
    if (!battery.consumed_mah(consumed_mah, instance)) {
        consumed_mah = -1;
    }
    if (battery.consumed_wh(consumed_wh, instance)) {
        consumed_wh *= 36;
    } else {
        consumed_wh = -1;
    }
    uint32_t time_remaining;
    if (!battery.time_remaining(time_remaining, instance)) {
        time_remaining = 0;
    }

    mavlink_msg_battery_status_send(chan,
                                    instance, // id
                                    MAV_BATTERY_FUNCTION_UNKNOWN, // function
                                    MAV_BATTERY_TYPE_UNKNOWN, // type
                                    got_temperature ? ((int16_t) (temp * 100)) : INT16_MAX, // temperature. INT16_MAX if unknown
                                    cell_mvolts, // cell voltages
                                    current,      // current in centiampere
                                    consumed_mah, // total consumed current in milliampere.hour
                                    consumed_wh,  // consumed energy in hJ (hecto-Joules)
                                    constrain_int16(percentage, -1, 100),
                                    time_remaining, // time remaining, seconds
                                    battery.get_mavlink_charge_state(instance), // battery charge state
                                    cell_mvolts_ext, // Cell 11..14 voltages
                                    0, // battery mode
                                    battery.get_mavlink_fault_bitmask(instance));   // fault_bitmask
}

// returns true if all battery instances were reported
bool GCS_MAVLINK::send_battery_status()
{
    const AP_BattMonitor &battery = AP::battery();

    for(uint8_t i = 0; i < AP_BATT_MONITOR_MAX_INSTANCES; i++) {
        const uint8_t battery_id = (last_battery_status_idx + 1) % AP_BATT_MONITOR_MAX_INSTANCES;
        const auto configured_type = battery.configured_type(battery_id);
        if (configured_type != AP_BattMonitor::Type::NONE &&
            configured_type == battery.allocated_type(battery_id) &&
            !battery.option_is_set(battery_id, AP_BattMonitor_Params::Options::InternalUseOnly)) {
            CHECK_PAYLOAD_SIZE(BATTERY_STATUS);
            send_battery_status(battery_id);
            last_battery_status_idx = battery_id;
            return true;
        } else {
            last_battery_status_idx = battery_id;
        }
    }
    return true;
}
#endif  // AP_BATTERY_ENABLED

#if AP_RANGEFINDER_ENABLED
void GCS_MAVLINK::send_distance_sensor(const AP_RangeFinder_Backend *sensor, const uint8_t instance) const
{
    if (!sensor->has_data()) {
        return;
    }

    int8_t quality_pct = sensor->signal_quality_pct();
    // ardupilot defines this field as -1 is unknown, 0 is poor, 100 is excellent
    // mavlink defines this field as 0 is unknown, 1 is invalid, 100 is perfect
    uint8_t quality;
    if (quality_pct == RangeFinder::SIGNAL_QUALITY_UNKNOWN) {
        quality = 0;
    } else if (quality_pct > 1 && quality_pct <= RangeFinder::SIGNAL_QUALITY_MAX) {
        quality = quality_pct;
    } else {
        quality = 1;
    }

    mavlink_msg_distance_sensor_send(
        chan,
        AP_HAL::millis(),                        // time since system boot TODO: take time of measurement
        MIN(sensor->min_distance() * 100, 65535),// minimum distance the sensor can measure in centimeters
        MIN(sensor->max_distance() * 100, 65535),// maximum distance the sensor can measure in centimeters
        MIN(sensor->distance() * 100, 65535),    // current distance reading
        sensor->get_mav_distance_sensor_type(),  // type from MAV_DISTANCE_SENSOR enum
        instance,                                // onboard ID of the sensor == instance
        sensor->orientation(),                   // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
        0,                                       // Measurement covariance in centimeters, 0 for unknown / invalid readings
        0,                                       // horizontal FOV
        0,                                       // vertical FOV
        (const float *)nullptr,                  // quaternion of sensor orientation for MAV_SENSOR_ROTATION_CUSTOM
        quality);                                // Signal quality of the sensor. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.
}
#endif  // AP_RANGEFINDER_ENABLED

// send any and all distance_sensor messages.  This starts by sending
// any distance sensors not used by a Proximity sensor, then sends the
// proximity sensor ones.
void GCS_MAVLINK::send_distance_sensor()
{
#if AP_RANGEFINDER_ENABLED
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        return;
    }

    // if we have a proximity backend that utilizes rangefinders cull
    // sending them here, and allow the later proximity code to manage
    // them
    bool filter_possible_proximity_sensors = false;

#if HAL_PROXIMITY_ENABLED
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity != nullptr) {
        for (uint8_t i = 0; i < proximity->num_sensors(); i++) {
#if AP_PROXIMITY_RANGEFINDER_ENABLED
            if (proximity->get_type(i) == AP_Proximity::Type::RangeFinder) {
                filter_possible_proximity_sensors = true;
            }
#endif
        }
    }
#endif

    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
            return;
        }
        AP_RangeFinder_Backend *sensor = rangefinder->get_backend(i);
        if (sensor == nullptr) {
            continue;
        }
        enum Rotation orient = sensor->orientation();
        if (!filter_possible_proximity_sensors ||
            (orient > ROTATION_YAW_315 && orient != ROTATION_PITCH_90)) {
            send_distance_sensor(sensor, i);
        }
    }
#endif  // AP_RANGEFINDER_ENABLED

#if HAL_PROXIMITY_ENABLED
    send_proximity();
#endif
}

#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
void GCS_MAVLINK::send_rangefinder() const
{
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        return;
    }
    AP_RangeFinder_Backend *s = rangefinder->find_instance(ROTATION_PITCH_270);
    if (s == nullptr) {
        return;
    }
    mavlink_msg_rangefinder_send(
            chan,
            s->distance(),
            s->voltage_mv() * 0.001f);
}
#endif  // AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED

#if HAL_PROXIMITY_ENABLED
void GCS_MAVLINK::send_proximity()
{
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity == nullptr) {
        return; // this is wrong, but pretend we sent data and don't requeue
    }

    // get min/max distances
    const uint16_t dist_min_cm = (uint16_t)(proximity->distance_min_m() * 100.0f);  // minimum distance the sensor can measure in centimeters
    const uint16_t dist_max_cm = (uint16_t)(proximity->distance_max_m() * 100.0f);  // maximum distance the sensor can measure in centimeters

    // send horizontal distances
    if (proximity->get_status() == AP_Proximity::Status::Good) {
        Proximity_Distance_Array dist_array;
        if (proximity->get_horizontal_distances(dist_array)) {
            for (uint8_t i = 0; i < PROXIMITY_MAX_DIRECTION; i++) {
                if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
                    return;
                }
                if (dist_array.valid(i)) {
                    proximity_ever_valid_bitmask |= (1U << i);
                } else if (!(proximity_ever_valid_bitmask & (1U << i))) {
                    // we've never sent this distance out, so we don't
                    // need to send an invalid one.
                    continue;
                }
                mavlink_msg_distance_sensor_send(
                        chan,
                        AP_HAL::millis(),                               // time since system boot
                        dist_min_cm,                                    // minimum distance the sensor can measure in centimeters
                        dist_max_cm,                                    // maximum distance the sensor can measure in centimeters
                        (uint16_t)(dist_array.distance[i] * 100.0f),    // current distance reading
                        MAV_DISTANCE_SENSOR_LASER,                      // type from MAV_DISTANCE_SENSOR enum
                        PROXIMITY_SENSOR_ID_START + i,                  // onboard ID of the sensor
                        dist_array.orientation[i],                      // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
                        0,                                              // Measurement covariance in centimeters, 0 for unknown / invalid readings
                        0, 0, nullptr, 0);
            }
        }
    }

    // send upward distance
    float dist_up_m;
    if (proximity->get_upward_distance(dist_up_m)) {
        if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
            return;
        }
        mavlink_msg_distance_sensor_send(
                chan,
                AP_HAL::millis(),                                           // time since system boot
                dist_min_cm,                                                // minimum distance the sensor can measure in centimeters
                dist_max_cm,                                                // maximum distance the sensor can measure in centimeters
                (uint16_t)(dist_up_m * 100.0f),                             // current distance reading
                MAV_DISTANCE_SENSOR_LASER,                                  // type from MAV_DISTANCE_SENSOR enum
                PROXIMITY_SENSOR_ID_START + PROXIMITY_MAX_DIRECTION + 1,    // onboard ID of the sensor
                MAV_SENSOR_ROTATION_PITCH_90,                               // direction upwards
                0,                                                          // Measurement covariance in centimeters, 0 for unknown / invalid readings
                0, 0, nullptr, 0);
    }
}
#endif // HAL_PROXIMITY_ENABLED

#if AP_AHRS_ENABLED
// report AHRS2 state
void GCS_MAVLINK::send_ahrs2()
{
    const AP_AHRS &ahrs = AP::ahrs();
    Vector3f euler;
    Location loc {};
    // we want one or both of these, use | to avoid short-circuiting:
    if (uint8_t(ahrs.get_secondary_attitude(euler)) |
        uint8_t(ahrs.get_secondary_position(loc))) {
        mavlink_msg_ahrs2_send(chan,
                               euler.x,
                               euler.y,
                               euler.z,
                               loc.alt*1.0e-2f,
                               loc.lat,
                               loc.lng);
    }
}
#endif  // AP_AHRS_ENABLED

MissionItemProtocol *GCS::get_prot_for_mission_type(const MAV_MISSION_TYPE mission_type) const
{
    if (mission_type >= ARRAY_SIZE(missionitemprotocols)) {
        return nullptr;
    }
    return missionitemprotocols[mission_type];
}

// handle a request for the number of items we have stored for a mission type:
void GCS_MAVLINK::handle_mission_request_list(const mavlink_message_t &msg)
{
    // decode
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(&msg, &packet);

    MissionItemProtocol *prot = gcs().get_prot_for_mission_type((MAV_MISSION_TYPE)packet.mission_type);
    if (prot == nullptr) {
        mavlink_msg_mission_ack_send(chan,
                                     msg.sysid,
                                     msg.compid,
                                     MAV_MISSION_UNSUPPORTED,
                                     packet.mission_type);
        return;
    }

    prot->handle_mission_request_list(*this, packet, msg);
}

/*
  handle a MISSION_REQUEST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request_int(const mavlink_message_t &msg)
{
        // decode
        mavlink_mission_request_int_t packet;
        mavlink_msg_mission_request_int_decode(&msg, &packet);

        MissionItemProtocol *prot = gcs().get_prot_for_mission_type((MAV_MISSION_TYPE)packet.mission_type);
        if (prot == nullptr) {
            return;
        }
        prot->handle_mission_request_int(*this, packet, msg);
}

#if AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
void GCS_MAVLINK::handle_mission_request(const mavlink_message_t &msg)
{
        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(&msg, &packet);

        MissionItemProtocol *prot = gcs().get_prot_for_mission_type((MAV_MISSION_TYPE)packet.mission_type);
        if (prot == nullptr) {
            return;
        }
        prot->handle_mission_request(*this, packet, msg);
}
#endif

// returns a MISSION_STATE numeration value best describing out
// current mission state.
MISSION_STATE GCS_MAVLINK::mission_state(const AP_Mission &mission) const
{
    if (!mission.present()) {
        return MISSION_STATE_NO_MISSION;
    }
    switch (mission.state()) {
    case AP_Mission::mission_state::MISSION_STOPPED:
        return MISSION_STATE_NOT_STARTED;
    case AP_Mission::mission_state::MISSION_RUNNING:
        return MISSION_STATE_ACTIVE;
    case AP_Mission::mission_state::MISSION_COMPLETE:
        return MISSION_STATE_COMPLETE;
    }

    // compiler ensures we can't get here as no default case in above enumeration

    return MISSION_STATE_UNKNOWN;
}

void GCS_MAVLINK::send_mission_current(const class AP_Mission &mission, uint16_t seq)
{
    auto num_commands = mission.num_commands();
    if (num_commands > 0) {
        // exclude home location from the count; see message definition.
        num_commands -= 1;
    }

#if AP_VEHICLE_ENABLED
    const uint8_t mission_mode = AP::vehicle()->current_mode_requires_mission() ? 1 : 0;
#else
    const uint8_t mission_mode = 0;
#endif

    mavlink_msg_mission_current_send(
        chan,
        seq,
        num_commands, // total
        mission_state(mission), // mission_state
        mission_mode);  // mission_mode
}

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
/*
  handle a MISSION_SET_CURRENT mavlink packet

  Note that there exists a relatively new mavlink DO command,
  MAV_CMD_DO_SET_MISSION_CURRENT which provides an acknowledgement
  that the command has been received, rather than the GCS having to
  rely on getting back an identical sequence number as some currently
  do.
 */
void GCS_MAVLINK::handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg)
{
    // send_received_message_deprecation_warning("MISSION_SET_CURRENT");

    // decode
    mavlink_mission_set_current_t packet;
    mavlink_msg_mission_set_current_decode(&msg, &packet);

    // set current command
    if (mission.set_current_cmd(packet.seq)) {
        // because MISSION_SET_CURRENT is a message not a command,
        // there is not ACK associated with us successfully changing
        // our waypoint.  Some GCSs use the fact we return exactly the
        // same mission sequence number in this packet as an ACK - so
        // if they send a MISSION_SET_CURRENT with seq number of 4
        // then they expect to receive a MISSION_CURRENT message with
        // exactly that sequence number in it, even if ArduPilot never
        // actually holds that as a sequence number (e.g. packet.seq==0).
        if (HAVE_PAYLOAD_SPACE(chan, MISSION_CURRENT)) {
            send_mission_current(mission, packet.seq);
        } else {
            // schedule it for later:
            send_message(MSG_CURRENT_WAYPOINT);
        }
    }
}
#endif  // AP_MAVLINK_MISSION_SET_CURRENT_ENABLED

/*
  handle a MISSION_COUNT mavlink packet
 */
void GCS_MAVLINK::handle_mission_count(const mavlink_message_t &msg)
{
    // decode
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(&msg, &packet);

    MissionItemProtocol *prot = gcs().get_prot_for_mission_type((MAV_MISSION_TYPE)packet.mission_type);
    if (prot == nullptr) {
        mavlink_msg_mission_ack_send(chan,
                                     msg.sysid,
                                     msg.compid,
                                     MAV_MISSION_UNSUPPORTED,
                                     packet.mission_type);
        return;
    }

    prot->handle_mission_count(*this, packet, msg);
}

/*
  handle a MISSION_CLEAR_ALL mavlink packet
 */
void GCS_MAVLINK::handle_mission_clear_all(const mavlink_message_t &msg)
{
    // decode
    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(&msg, &packet);

    const MAV_MISSION_TYPE mission_type = (MAV_MISSION_TYPE)packet.mission_type;
    MissionItemProtocol *prot = gcs().get_prot_for_mission_type(mission_type);
    if (prot == nullptr) {
        send_mission_ack(msg, mission_type, MAV_MISSION_UNSUPPORTED);
        return;
    }

    prot->handle_mission_clear_all(*this, msg);
}

bool GCS_MAVLINK::requesting_mission_items() const
{
    for (const auto *prot : gcs().missionitemprotocols) {
        if (prot && prot->receiving && prot->active_link_is(this)) {
            return true;
        }
    }
    return false;
}

void GCS_MAVLINK::handle_mission_write_partial_list(const mavlink_message_t &msg)
{
    // decode
    mavlink_mission_write_partial_list_t packet;
    mavlink_msg_mission_write_partial_list_decode(&msg, &packet);

    MissionItemProtocol *use_prot = gcs().get_prot_for_mission_type((MAV_MISSION_TYPE)packet.mission_type);
    if (use_prot == nullptr) {
        send_mission_ack(msg, (MAV_MISSION_TYPE)packet.mission_type, MAV_MISSION_UNSUPPORTED);
        return;
    }
    use_prot->handle_mission_write_partial_list(*this, msg, packet);
}

#if HAL_MOUNT_ENABLED
/*
  pass mavlink messages to the AP_Mount singleton
 */
void GCS_MAVLINK::handle_mount_message(const mavlink_message_t &msg)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->handle_message(chan, msg);
}

#endif

/*
  pass parameter value messages through to mount library
 */
void GCS_MAVLINK::handle_param_value(const mavlink_message_t &msg)
{
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->handle_param_value(msg);
#endif
}

void GCS_MAVLINK::send_text(MAV_SEVERITY severity, const char *fmt, ...) const
{
    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(severity, fmt, arg_list, (1<<chan));
    va_end(arg_list);
}

float GCS_MAVLINK::telemetry_radio_rssi()
{
    if (AP_HAL::millis() - last_radio_status.received_ms > 5000) {
        // telemetry radio has disappeared?!
        return 0;
    }
    if (last_radio_status.rssi == 255) {
        // see RADIO_STATUS packet definition
        return 0;
    }
    return last_radio_status.rssi/254.0f;
}

bool GCS_MAVLINK::last_txbuf_is_greater(uint8_t txbuf_limit)
{
    if (AP_HAL::millis() - last_radio_status.received_ms > 5000) {
        // stale report
        return true;
    }
    return last_radio_status.txbuf > txbuf_limit;
}

void GCS_MAVLINK::handle_radio_status(const mavlink_message_t &msg)
{
    mavlink_radio_t packet;
    mavlink_msg_radio_decode(&msg, &packet);

    const uint32_t now = AP_HAL::millis();

    last_radio_status.received_ms = now;
    last_radio_status.rssi = packet.rssi;

    // record if the GCS has been receiving radio messages from
    // the aircraft
    if (packet.remrssi != 0) {
        last_radio_status.remrssi_ms = now;
    }

    last_radio_status.txbuf = packet.txbuf;

    // use the state of the transmit buffer in the radio to
    // control the stream rate, giving us adaptive software
    // flow control
    if (packet.txbuf < 20 && stream_slowdown_ms < 2000) {
        // we are very low on space - slow down a lot
        stream_slowdown_ms += 60;
    } else if (packet.txbuf < 50 && stream_slowdown_ms < 2000) {
        // we are a bit low on space, slow down slightly
        stream_slowdown_ms += 20;
    } else if (packet.txbuf > 95 && stream_slowdown_ms > 200) {
        // the buffer has plenty of space, speed up a lot
        stream_slowdown_ms -= 40;
    } else if (packet.txbuf > 90 && stream_slowdown_ms != 0) {
        // the buffer has enough space, speed up a bit
        if (stream_slowdown_ms > 20) {
            stream_slowdown_ms -= 20;
        } else {
            stream_slowdown_ms = 0;
        }
    }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    if (stream_slowdown_ms > max_slowdown_ms) {
        max_slowdown_ms = stream_slowdown_ms;
    }
#endif

#if HAL_LOGGING_ENABLED
    //log rssi, noise, etc if logging Performance monitoring data
    if (AP::logger().should_log(log_radio_bit())) {
        AP::logger().Write_Radio(packet);
    }
#endif
}

void GCS_MAVLINK::handle_mission_item(const mavlink_message_t &msg)
{
    mavlink_mission_item_int_t mission_item_int;
    bool send_mission_item_warning = false;
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
        mavlink_mission_item_t mission_item;
        mavlink_msg_mission_item_decode(&msg, &mission_item);
#if AP_MISSION_ENABLED
        MAV_MISSION_RESULT ret = AP_Mission::convert_MISSION_ITEM_to_MISSION_ITEM_INT(mission_item, mission_item_int);
        if (ret != MAV_MISSION_ACCEPTED) {
            const MAV_MISSION_TYPE type = (MAV_MISSION_TYPE)mission_item_int.mission_type;
            send_mission_ack(msg, type, ret);
            return;
        }
#else
        // No mission library, so we can't convert from MISSION_ITEM
        // to MISSION_ITEM_INT.  Since we shouldn't be receiving fence
        // or rally items via MISSION_ITEM, we don't need to work hard
        // here, just fail:
        const MAV_MISSION_TYPE type = (MAV_MISSION_TYPE)mission_item.mission_type;
        send_mission_ack(msg, type, MAV_MISSION_UNSUPPORTED);
        return;
#endif
        send_mission_item_warning = true;
    } else {
        mavlink_msg_mission_item_int_decode(&msg, &mission_item_int);
    }
    const MAV_MISSION_TYPE type = (MAV_MISSION_TYPE)mission_item_int.mission_type;

#if AP_MISSION_ENABLED
    const uint8_t current = mission_item_int.current;

    if (type == MAV_MISSION_TYPE_MISSION && (current == 2 || current == 3)) {
        struct AP_Mission::Mission_Command cmd = {};
        MAV_MISSION_RESULT result = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
        if (result != MAV_MISSION_ACCEPTED) {
            //decode failed
            send_mission_ack(msg, MAV_MISSION_TYPE_MISSION, result);
            return;
        }
        // guided or change-alt
        if (current == 2) {
            // current = 2 is a flag to tell us this is a "guided mode"
            // waypoint and not for the mission
            result = (handle_guided_request(cmd) ? MAV_MISSION_ACCEPTED
                      : MAV_MISSION_ERROR) ;
        } else if (current == 3) {
            //current = 3 is a flag to tell us this is a alt change only
            // add home alt if needed
            handle_change_alt_request(cmd.content.location);

            // verify we received the command
            result = MAV_MISSION_ACCEPTED;
        }
        send_mission_ack(msg, MAV_MISSION_TYPE_MISSION, result);
        return;
    }
#endif

    // not a guided-mode reqest
    MissionItemProtocol *prot = gcs().get_prot_for_mission_type(type);
    if (prot == nullptr) {
        send_mission_ack(msg, type, MAV_MISSION_UNSUPPORTED);
        return;
    }

    if (send_mission_item_warning) {
        prot->send_mission_item_warning();
    }

    if (!prot->receiving) {
        send_mission_ack(msg, type, MAV_MISSION_ERROR);
        return;
    }

    prot->handle_mission_item(msg, mission_item_int);
}

ap_message GCS_MAVLINK::mavlink_id_to_ap_message_id(const uint32_t mavlink_id) const
{
    // MSG_NEXT_MISSION_REQUEST doesn't correspond to a mavlink message directly.
    // It is used to request the next waypoint after receiving one.

    // MSG_NEXT_PARAM doesn't correspond to a mavlink message directly.
    // It is used to send the next parameter in a stream after sending one

    // MSG_NAMED_FLOAT messages can't really be "streamed"...

    static const struct {
        uint32_t mavlink_id;
        ap_message msg_id;
    } map[] {
        { MAVLINK_MSG_ID_HEARTBEAT,             MSG_HEARTBEAT},
        { MAVLINK_MSG_ID_HOME_POSITION,         MSG_HOME},
        { MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,     MSG_ORIGIN},
        { MAVLINK_MSG_ID_SYS_STATUS,            MSG_SYS_STATUS},
        { MAVLINK_MSG_ID_POWER_STATUS,          MSG_POWER_STATUS},
#if HAL_WITH_MCU_MONITORING
        { MAVLINK_MSG_ID_MCU_STATUS,            MSG_MCU_STATUS},
#endif
        { MAVLINK_MSG_ID_MEMINFO,               MSG_MEMINFO},
        { MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_NAV_CONTROLLER_OUTPUT},
        { MAVLINK_MSG_ID_MISSION_CURRENT,       MSG_CURRENT_WAYPOINT},
        { MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,      MSG_SERVO_OUTPUT_RAW},
        { MAVLINK_MSG_ID_RC_CHANNELS,           MSG_RC_CHANNELS},
#if AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
        { MAVLINK_MSG_ID_RC_CHANNELS_RAW,       MSG_RC_CHANNELS_RAW},
#endif
        { MAVLINK_MSG_ID_RAW_IMU,               MSG_RAW_IMU},
        { MAVLINK_MSG_ID_SCALED_IMU,            MSG_SCALED_IMU},
        { MAVLINK_MSG_ID_SCALED_IMU2,           MSG_SCALED_IMU2},
        { MAVLINK_MSG_ID_SCALED_IMU3,           MSG_SCALED_IMU3},
#if AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
        { MAVLINK_MSG_ID_HIGHRES_IMU,           MSG_HIGHRES_IMU},
#endif
        { MAVLINK_MSG_ID_SCALED_PRESSURE,       MSG_SCALED_PRESSURE},
        { MAVLINK_MSG_ID_SCALED_PRESSURE2,      MSG_SCALED_PRESSURE2},
        { MAVLINK_MSG_ID_SCALED_PRESSURE3,      MSG_SCALED_PRESSURE3},
#if AP_GPS_GPS_RAW_INT_SENDING_ENABLED
        { MAVLINK_MSG_ID_GPS_RAW_INT,           MSG_GPS_RAW},
#endif
#if AP_GPS_GPS_RTK_SENDING_ENABLED
        { MAVLINK_MSG_ID_GPS_RTK,               MSG_GPS_RTK},
#endif
#if AP_GPS_GPS2_RAW_SENDING_ENABLED
        { MAVLINK_MSG_ID_GPS2_RAW,              MSG_GPS2_RAW},
#endif
#if AP_GPS_GPS2_RTK_SENDING_ENABLED
        { MAVLINK_MSG_ID_GPS2_RTK,              MSG_GPS2_RTK},
#endif
        { MAVLINK_MSG_ID_SYSTEM_TIME,           MSG_SYSTEM_TIME},
#if APM_BUILD_TYPE(APM_BUILD_Rover)
        { MAVLINK_MSG_ID_RC_CHANNELS_SCALED,    MSG_SERVO_OUT},
#endif  // APM_BUILD_TYPE(APM_BUILD_Rover)
        { MAVLINK_MSG_ID_PARAM_VALUE,           MSG_NEXT_PARAM},
#if AP_FENCE_ENABLED
        { MAVLINK_MSG_ID_FENCE_STATUS,          MSG_FENCE_STATUS},
#endif
#if AP_SIM_ENABLED
        { MAVLINK_MSG_ID_SIMSTATE,              MSG_SIMSTATE},
        { MAVLINK_MSG_ID_SIM_STATE,             MSG_SIM_STATE},
#endif
#if AP_AHRS_ENABLED
        { MAVLINK_MSG_ID_AHRS2,                 MSG_AHRS2},
        { MAVLINK_MSG_ID_AHRS,                  MSG_AHRS},
        { MAVLINK_MSG_ID_ATTITUDE,              MSG_ATTITUDE},
        { MAVLINK_MSG_ID_ATTITUDE_QUATERNION,   MSG_ATTITUDE_QUATERNION},
        { MAVLINK_MSG_ID_GLOBAL_POSITION_INT,   MSG_LOCATION},
#if AP_AHRS_ENABLED
        { MAVLINK_MSG_ID_LOCAL_POSITION_NED,    MSG_LOCAL_POSITION},
#endif  // AP_AHRS_ENABLED
        { MAVLINK_MSG_ID_VFR_HUD,               MSG_VFR_HUD},
#endif
#if AP_MAVLINK_MSG_HWSTATUS_ENABLED
        { MAVLINK_MSG_ID_HWSTATUS,              MSG_HWSTATUS},
#endif  // AP_MAVLINK_MSG_HWSTATUS_ENABLED
#if AP_MAVLINK_MSG_WIND_ENABLED
        { MAVLINK_MSG_ID_WIND,                  MSG_WIND},
#endif  // AP_MAVLINK_MSG_WIND_ENABLED
#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
        { MAVLINK_MSG_ID_RANGEFINDER,           MSG_RANGEFINDER},
#endif  // AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
        { MAVLINK_MSG_ID_DISTANCE_SENSOR,       MSG_DISTANCE_SENSOR},
#if AP_TERRAIN_AVAILABLE
        { MAVLINK_MSG_ID_TERRAIN_REQUEST,       MSG_TERRAIN_REQUEST},
        { MAVLINK_MSG_ID_TERRAIN_REPORT,        MSG_TERRAIN_REPORT},
#endif
#if AP_CAMERA_ENABLED
        { MAVLINK_MSG_ID_CAMERA_FEEDBACK,       MSG_CAMERA_FEEDBACK},
        { MAVLINK_MSG_ID_CAMERA_INFORMATION,    MSG_CAMERA_INFORMATION},
        { MAVLINK_MSG_ID_CAMERA_SETTINGS,       MSG_CAMERA_SETTINGS},
#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
        { MAVLINK_MSG_ID_CAMERA_FOV_STATUS,     MSG_CAMERA_FOV_STATUS},
#endif
        { MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, MSG_CAMERA_CAPTURE_STATUS},
#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
        { MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE,  MSG_CAMERA_THERMAL_RANGE},
#endif // AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
        { MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, MSG_VIDEO_STREAM_INFORMATION},
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
#endif // AP_CAMERA_ENABLED
#if HAL_MOUNT_ENABLED
        { MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, MSG_GIMBAL_DEVICE_ATTITUDE_STATUS},
        { MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE},
        { MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION, MSG_GIMBAL_MANAGER_INFORMATION},
        { MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, MSG_GIMBAL_MANAGER_STATUS},
#endif
#if AP_OPTICALFLOW_ENABLED
        { MAVLINK_MSG_ID_OPTICAL_FLOW,          MSG_OPTICAL_FLOW},
#endif
#if COMPASS_CAL_ENABLED
        { MAVLINK_MSG_ID_MAG_CAL_PROGRESS,      MSG_MAG_CAL_PROGRESS},
        { MAVLINK_MSG_ID_MAG_CAL_REPORT,        MSG_MAG_CAL_REPORT},
#endif
#if AP_AHRS_ENABLED
        { MAVLINK_MSG_ID_EKF_STATUS_REPORT,     MSG_EKF_STATUS_REPORT},
#endif  // AP_AHRS_ENABLED
        { MAVLINK_MSG_ID_PID_TUNING,            MSG_PID_TUNING},
        { MAVLINK_MSG_ID_VIBRATION,             MSG_VIBRATION},
#if AP_RPM_ENABLED
        { MAVLINK_MSG_ID_RPM,                   MSG_RPM},
#endif
        { MAVLINK_MSG_ID_MISSION_ITEM_REACHED,  MSG_MISSION_ITEM_REACHED},
        { MAVLINK_MSG_ID_ATTITUDE_TARGET,       MSG_ATTITUDE_TARGET},
        { MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,  MSG_POSITION_TARGET_GLOBAL_INT},
        { MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,  MSG_POSITION_TARGET_LOCAL_NED},
#if HAL_ADSB_ENABLED
        { MAVLINK_MSG_ID_ADSB_VEHICLE,          MSG_ADSB_VEHICLE},
#endif
#if AP_BATTERY_ENABLED
        { MAVLINK_MSG_ID_BATTERY_STATUS,        MSG_BATTERY_STATUS},
#endif
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        { MAVLINK_MSG_ID_AOA_SSA,               MSG_AOA_SSA},
#endif  // APM_BUILD_ArduPlane
#if HAL_LANDING_DEEPSTALL_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        { MAVLINK_MSG_ID_DEEPSTALL,             MSG_LANDING},
#endif
        { MAVLINK_MSG_ID_EXTENDED_SYS_STATE,    MSG_EXTENDED_SYS_STATE},
        { MAVLINK_MSG_ID_AUTOPILOT_VERSION,     MSG_AUTOPILOT_VERSION},
#if HAL_EFI_ENABLED
        { MAVLINK_MSG_ID_EFI_STATUS,            MSG_EFI_STATUS},
#endif
#if HAL_GENERATOR_ENABLED
        { MAVLINK_MSG_ID_GENERATOR_STATUS,      MSG_GENERATOR_STATUS},
#endif
#if AP_WINCH_ENABLED
        { MAVLINK_MSG_ID_WINCH_STATUS,          MSG_WINCH_STATUS},
#endif
#if HAL_WITH_ESC_TELEM
        { MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4,  MSG_ESC_TELEMETRY},
#endif
#if AP_RANGEFINDER_ENABLED && APM_BUILD_TYPE(APM_BUILD_Rover)
        { MAVLINK_MSG_ID_WATER_DEPTH,           MSG_WATER_DEPTH},
#endif
#if HAL_HIGH_LATENCY2_ENABLED
        { MAVLINK_MSG_ID_HIGH_LATENCY2,         MSG_HIGH_LATENCY2},
#endif
#if AP_AIS_ENABLED
        { MAVLINK_MSG_ID_AIS_VESSEL,            MSG_AIS_VESSEL},
#endif
#if AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
        { MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, MSG_UAVIONIX_ADSB_OUT_STATUS},
#endif
#if AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
        { MAVLINK_MSG_ID_RELAY_STATUS, MSG_RELAY_STATUS},
#endif
#if AP_AIRSPEED_ENABLED
        { MAVLINK_MSG_ID_AIRSPEED, MSG_AIRSPEED},
#endif
        { MAVLINK_MSG_ID_AVAILABLE_MODES, MSG_AVAILABLE_MODES},
        { MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, MSG_AVAILABLE_MODES_MONITOR},
#if AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
        { MAVLINK_MSG_ID_FLIGHT_INFORMATION, MSG_FLIGHT_INFORMATION},
#endif
    };

    for (uint8_t i=0; i<ARRAY_SIZE(map); i++) {
        if (map[i].mavlink_id == mavlink_id) {
            return map[i].msg_id;
        }
    }
    return MSG_LAST;
}

bool GCS_MAVLINK::set_mavlink_message_id_interval(const uint32_t mavlink_id,
                                                  const uint16_t interval_ms)
{
    const ap_message id = mavlink_id_to_ap_message_id(mavlink_id);
    if (id == MSG_LAST) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "No ap_message for mavlink id (%u)", (unsigned int)mavlink_id);
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
        return false;
    }
    return set_ap_message_interval(id, interval_ms);
}

bool GCS_MAVLINK::should_send_message_in_delay_callback(const ap_message id) const
{
    // No ID we return true for may take more than a few hundred
    // microseconds to return!

    switch (id) {
    case MSG_NEXT_PARAM:
    case MSG_HEARTBEAT:
#if HAL_HIGH_LATENCY2_ENABLED
    case MSG_HIGH_LATENCY2:
#endif
    case MSG_AUTOPILOT_VERSION:
        return true;
    default:
        return false;
    }

    return false;
}

uint16_t GCS_MAVLINK::get_reschedule_interval_ms(const deferred_message_bucket_t &deferred) const
{
    uint32_t interval_ms = deferred.interval_ms;

    interval_ms += stream_slowdown_ms;

    // slow most messages down if we're transfering parameters or
    // waypoints:
    if (_queued_parameter) {
        // we are sending parameters, penalize streams:
        interval_ms *= 4;
    }
    if (requesting_mission_items()) {
        // we are sending requests for waypoints, penalize streams:
        interval_ms *= 4;
    }
#if AP_MAVLINK_FTP_ENABLED
    if (AP_HAL::millis() - GCS_FTP::get_last_send_ms(chan) < 1000) {
        // we are sending ftp replies
        interval_ms *= 4;
    }
#endif

    if (interval_ms > 60000) {
        return 60000;
    }

    return interval_ms;
}

// typical runtime on fmuv3: 5 microseconds for 3 buckets
void GCS_MAVLINK::find_next_bucket_to_send(uint16_t now16_ms)
{
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    void *data = hal.scheduler->disable_interrupts_save();
    uint32_t start_us = AP_HAL::micros();
#endif

    // all done sending this bucket... find another bucket...
    sending_bucket_id = no_bucket_to_send;
    uint16_t ms_before_send_next_bucket_to_send = UINT16_MAX;
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
        if (deferred_message_bucket[i].ap_message_ids.count() == 0) {
            // no entries
            continue;
        }
        const uint16_t interval = get_reschedule_interval_ms(deferred_message_bucket[i]);
        const uint16_t ms_since_last_sent = now16_ms - deferred_message_bucket[i].last_sent_ms;
        uint16_t ms_before_send_this_bucket;
        if (ms_since_last_sent > interval) {
            // should already have sent this bucket!
            ms_before_send_this_bucket = 0;
        } else {
            ms_before_send_this_bucket = interval - ms_since_last_sent;
        }
        if (ms_before_send_this_bucket < ms_before_send_next_bucket_to_send) {
            sending_bucket_id = i;
            ms_before_send_next_bucket_to_send = ms_before_send_this_bucket;
        }
    }
    if (sending_bucket_id != no_bucket_to_send) {
        bucket_message_ids_to_send = deferred_message_bucket[sending_bucket_id].ap_message_ids;
    } else {
        bucket_message_ids_to_send.clearall();
    }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    uint32_t delta_us = AP_HAL::micros() - start_us;
    hal.scheduler->restore_interrupts(data);
    if (delta_us > try_send_message_stats.fnbts_maxtime) {
        try_send_message_stats.fnbts_maxtime = delta_us;
    }
#endif
}

ap_message GCS_MAVLINK::next_deferred_bucket_message_to_send(uint16_t now16_ms)
{
    if (sending_bucket_id == no_bucket_to_send) {
        // could happen if all streamrates are zero?
        return no_message_to_send;
    }

    const uint16_t ms_since_last_sent = now16_ms - deferred_message_bucket[sending_bucket_id].last_sent_ms;
    if (ms_since_last_sent < get_reschedule_interval_ms(deferred_message_bucket[sending_bucket_id])) {
        // not time to send this bucket
        return no_message_to_send;
    }

    const int16_t next = bucket_message_ids_to_send.first_set();
    if (next == -1) {
        // should not happen
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("next_deferred_bucket_message_to_send called on empty bucket");
#endif
        find_next_bucket_to_send(now16_ms);
        return no_message_to_send;
    }
    return (ap_message)next;
}

// call try_send_message if appropriate.  Incorporates debug code to
// record how long it takes to send a message.  try_send_message is
// expected to be overridden, not this function.
bool GCS_MAVLINK::do_try_send_message(const ap_message id)
{
    const bool in_delay_callback = hal.scheduler->in_delay_callback();
    if (in_delay_callback && !should_send_message_in_delay_callback(id)) {
        return true;
    }
    if (telemetry_delayed()) {
        return false;
    }
    WITH_SEMAPHORE(comm_chan_lock(chan));
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    void *data = hal.scheduler->disable_interrupts_save();
    uint32_t start_send_message_us = AP_HAL::micros();
#endif
    if (!try_send_message(id)) {
        // didn't fit in buffer...
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
        try_send_message_stats.no_space_for_message++;
        hal.scheduler->restore_interrupts(data);
#endif
        return false;
    }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    const uint32_t delta_us = AP_HAL::micros() - start_send_message_us;
    hal.scheduler->restore_interrupts(data);
    if (delta_us > try_send_message_stats.longest_time_us) {
        try_send_message_stats.longest_time_us = delta_us;
        try_send_message_stats.longest_id = id;
    }
#endif
    return true;
}

int8_t GCS_MAVLINK::get_deferred_message_index(const ap_message id) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message); i++) {
        if (deferred_message[i].id == id) {
            return i;
        }
    }
    return -1;
}

int8_t GCS_MAVLINK::deferred_message_to_send_index(uint16_t now16_ms)
{

    if (next_deferred_message_to_send_cache == -1) {
        uint16_t ms_before_next_message_to_send = UINT16_MAX;
        for (uint8_t i=0; i<ARRAY_SIZE(deferred_message); i++) {
            const uint16_t interval_ms = deferred_message[i].interval_ms;
            if (interval_ms == 0) {
                continue;
            }
            const uint16_t ms_since_last_sent = now16_ms - deferred_message[i].last_sent_ms;
            uint16_t ms_before_send_this_message;
            if (ms_since_last_sent > interval_ms) {
                // should already have sent this one!
                ms_before_send_this_message = 0;
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
                try_send_message_stats.behind++;
#endif
            } else {
                ms_before_send_this_message = interval_ms - ms_since_last_sent;
            }
            if (ms_before_send_this_message < ms_before_next_message_to_send) {
                next_deferred_message_to_send_cache = i;
                ms_before_next_message_to_send = ms_before_send_this_message;
            }
        }
    }

    if (next_deferred_message_to_send_cache == -1) {
        // this really shouldn't happen; we force parameter rates, for example.
        return -1;
    }

    // an intermediate 16-bit variable is used here to strictly avoid
    // type promotion creating 32-bit (or higher) maths here.  Do not
    // be tempted to remove ms_since_last_sent here.
    const uint16_t ms_since_last_sent = now16_ms - deferred_message[next_deferred_message_to_send_cache].last_sent_ms;
    if (ms_since_last_sent < deferred_message[next_deferred_message_to_send_cache].interval_ms) {
        return -1;
    }

    return next_deferred_message_to_send_cache;
}

bool GCS_MAVLINK_InProgress::send_ack(MAV_RESULT result)
{
    if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_ACK)) {
        // can't fit the ACK, come back to this later
        return false;
    }

    mavlink_msg_command_ack_send(
        chan,
        mav_cmd,
        result,
        0,
        0,
        requesting_sysid,
        requesting_compid
        );

    return true;
}

bool GCS_MAVLINK_InProgress::send_in_progress()
{
    return send_ack(MAV_RESULT_IN_PROGRESS);
}

bool GCS_MAVLINK_InProgress::conclude(MAV_RESULT result)
{
    if (!send_ack(result)) {
        return false;
    }
    task = Type::NONE;
    return true;
}

GCS_MAVLINK_InProgress *GCS_MAVLINK_InProgress::get_task(MAV_CMD mav_cmd, GCS_MAVLINK_InProgress::Type t, uint8_t sysid, uint8_t compid, mavlink_channel_t chan)
{
    // we can't have two outstanding tasks for the same command from
    // the same mavlink node or the result is ambiguous:
    for (auto &_task : in_progress_tasks) {
        if (_task.task == Type::NONE) {
            continue;
        }
        if (_task.mav_cmd == mav_cmd &&
            _task.requesting_sysid == sysid &&
            _task.requesting_compid == compid) {
            return nullptr;
        }
    }

    for (auto &_task : in_progress_tasks) {
        if (_task.task != Type::NONE) {
            continue;
        }
        _task.chan = chan;
        _task.task = t;
        _task.mav_cmd = mav_cmd;
        _task.requesting_sysid = sysid;
        _task.requesting_compid = compid;
        return &_task;
    }
    return nullptr;
}


void GCS_MAVLINK_InProgress::check_tasks()
{
    // run these checks only intermittently (rate-limits the
    // MAV_RESULT_IN_PROGRESS messages):
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_check_ms < 1000) {
        return;
    }
    last_check_ms = now_ms;

    for (auto &task : in_progress_tasks) {
        switch (task.task) {
        case Type::NONE:
            break;
        case Type::AIRSPEED_CAL: {
#if AP_AIRSPEED_ENABLED
            const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
            switch (airspeed->get_calibration_state()) {
            case AP_Airspeed::CalibrationState::NOT_STARTED:
            case AP_Airspeed::CalibrationState::NOT_REQUIRED_ZERO_OFFSET:
                // we shouldn't get here
                task.conclude(MAV_RESULT_FAILED);
                break;
            case AP_Airspeed::CalibrationState::IN_PROGRESS:
                task.send_in_progress();
                break;
            case AP_Airspeed::CalibrationState::FAILED:
                task.conclude(MAV_RESULT_FAILED);
                break;
            case AP_Airspeed::CalibrationState::SUCCESS:
                task.conclude(MAV_RESULT_ACCEPTED);
                break;
            }
#endif
            }
            break;
        case Type::SD_FORMAT:
#if AP_FILESYSTEM_FORMAT_ENABLED
            switch (AP::FS().get_format_status()) {
            case AP_Filesystem_Backend::FormatStatus::NOT_STARTED:
                // we shouldn't get here
                task.conclude(MAV_RESULT_FAILED);
                break;
            case AP_Filesystem_Backend::FormatStatus::IN_PROGRESS:
            case AP_Filesystem_Backend::FormatStatus::PENDING:
                task.send_in_progress();
                break;
            case AP_Filesystem_Backend::FormatStatus::SUCCESS:
                task.conclude(MAV_RESULT_ACCEPTED);
                break;
            case AP_Filesystem_Backend::FormatStatus::FAILURE:
                task.conclude(MAV_RESULT_FAILED);
                break;
            }
#endif
            break;
        }
    }
}

void GCS_MAVLINK::update_send()
{
#if HAL_LOGGING_ENABLED
    if (!hal.scheduler->in_delay_callback()) {
        // AP_Logger will not send log data if we are armed.
        AP::logger().handle_log_send();
    }
#endif
    if (!deferred_messages_initialised) {
        initialise_message_intervals_from_streamrates();
#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
        initialise_message_intervals_from_config_files();
#endif
        deferred_messages_initialised = true;
    }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    uint32_t retry_deferred_body_start = AP_HAL::micros();
#endif

    // check for any in-progress tasks; check_tasks does its own rate-limiting
    GCS_MAVLINK_InProgress::check_tasks();

    const uint32_t start = AP_HAL::millis();
    const uint16_t start16 = start & 0xFFFF;
    while (AP_HAL::millis() - start < 5) { // spend a max of 5ms sending messages.  This should never trigger - out_of_time() should become true
        if (gcs().out_of_time()) {
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
            try_send_message_stats.out_of_time++;
#endif
            break;
        }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
        retry_deferred_body_start = AP_HAL::micros();
#endif

        // check if any "specially handled" messages should be sent out
        {
            const int8_t next = deferred_message_to_send_index(start16);
            if (next != -1) {
                if (!do_try_send_message(deferred_message[next].id)) {
                    break;
                }
                // we try to keep output on a regular clock to avoid
                // user support questions:
                const uint16_t interval_ms = deferred_message[next].interval_ms;
                deferred_message[next].last_sent_ms += interval_ms;
                // but we do not want to try to catch up too much:
                if (uint16_t(start16 - deferred_message[next].last_sent_ms) > interval_ms) {
                    deferred_message[next].last_sent_ms = start16;
                }

                next_deferred_message_to_send_cache = -1; // deferred_message_to_send will recalculate
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
                const uint32_t stop = AP_HAL::micros();
                const uint32_t delta = stop - retry_deferred_body_start;
                if (delta > try_send_message_stats.max_retry_deferred_body_us) {
                    try_send_message_stats.max_retry_deferred_body_us = delta;
                    try_send_message_stats.max_retry_deferred_body_type = 1;
                }
#endif
                continue;
            }
        }

        // check for any messages that the code has explicitly sent
        const int16_t fs = pushed_ap_message_ids.first_set();
        if (fs != -1) {
            ap_message next = (ap_message)fs;
            if (!do_try_send_message(next)) {
                break;
            }
            pushed_ap_message_ids.clear(next);
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
            const uint32_t stop = AP_HAL::micros();
            const uint32_t delta = stop - retry_deferred_body_start;
            if (delta > try_send_message_stats.max_retry_deferred_body_us) {
                try_send_message_stats.max_retry_deferred_body_us = delta;
                try_send_message_stats.max_retry_deferred_body_type = 2;
            }
#endif
            continue;
        }

        ap_message next = next_deferred_bucket_message_to_send(start16);
        if (next != no_message_to_send) {
            if (!do_try_send_message(next)) {
                break;
            }
            bucket_message_ids_to_send.clear(next);
            if (bucket_message_ids_to_send.count() == 0) {
                // we sent everything in the bucket.  Reschedule it.
                // we try to keep output on a regular clock to avoid
                // user support questions:
                const uint16_t interval_ms = get_reschedule_interval_ms(deferred_message_bucket[sending_bucket_id]);
                deferred_message_bucket[sending_bucket_id].last_sent_ms += interval_ms;
                // but we do not want to try to catch up too much:
                if (uint16_t(start16 - deferred_message_bucket[sending_bucket_id].last_sent_ms) > interval_ms) {
                    deferred_message_bucket[sending_bucket_id].last_sent_ms = start16;
                }
                find_next_bucket_to_send(start16);
            }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
                const uint32_t stop = AP_HAL::micros();
                const uint32_t delta = stop - retry_deferred_body_start;
                if (delta > try_send_message_stats.max_retry_deferred_body_us) {
                    try_send_message_stats.max_retry_deferred_body_us = delta;
                    try_send_message_stats.max_retry_deferred_body_type = 3;
                }
#endif
            continue;
        }
        break;
    }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    const uint32_t stop = AP_HAL::micros();
    const uint32_t delta = stop - retry_deferred_body_start;
    if (delta > try_send_message_stats.max_retry_deferred_body_us) {
        try_send_message_stats.max_retry_deferred_body_us = delta;
        try_send_message_stats.max_retry_deferred_body_type = 4;
    }
#endif

    // update the number of packets transmitted base on seqno, making
    // the assumption that we don't send more than 256 messages
    // between the last pass through here
    send_packet_count += uint8_t(_channel_status.current_tx_seq - last_tx_seq);
    last_tx_seq = _channel_status.current_tx_seq;
}

void GCS_MAVLINK::remove_message_from_bucket(int8_t bucket, ap_message id)
{
    deferred_message_bucket[bucket].ap_message_ids.clear(id);
    if (deferred_message_bucket[bucket].ap_message_ids.count() == 0) {
        // bucket empty.  Free it:
        deferred_message_bucket[bucket].interval_ms = 0;
        deferred_message_bucket[bucket].last_sent_ms = 0;
    }

    if (bucket == sending_bucket_id) {
        bucket_message_ids_to_send.clear(id);
        if (bucket_message_ids_to_send.count() == 0) {
            find_next_bucket_to_send(AP_HAL::millis16());
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            if (deferred_message_bucket[bucket].interval_ms == 0 &&
                deferred_message_bucket[bucket].last_sent_ms == 0) {
                // we just freed this bucket!  this would mean that
                // somehow our messages-still-to-send was a superset
                // of the messages in the bucket we were sending,
                // which would be bad.
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
#endif
        }
    }
}

bool GCS_MAVLINK::set_ap_message_interval(enum ap_message id, uint16_t interval_ms)
{
    if (id == MSG_NEXT_PARAM) {
        // force parameters to *always* get streamed so a vehicle is
        // recoverable from bad configuration:
        if (interval_ms == 0) {
            interval_ms = 100;
        } else if (interval_ms > 1000) {
            interval_ms = 1000;
        }
    }

#if AP_SCHEDULER_ENABLED
    interval_ms = cap_message_interval(interval_ms);
#endif

    // check if it's a specially-handled message:
    const int8_t deferred_offset = get_deferred_message_index(id);
    if (deferred_offset != -1) {
        deferred_message[deferred_offset].interval_ms = interval_ms;
        deferred_message[deferred_offset].last_sent_ms = AP_HAL::millis16();
        return true;
    }

    // see which bucket has the closest interval:
    int8_t closest_bucket = -1;
    uint16_t closest_bucket_interval_delta = UINT16_MAX;
    int8_t in_bucket = -1;
    int8_t empty_bucket_id = -1;
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
        const deferred_message_bucket_t &bucket = deferred_message_bucket[i];
        if (bucket.interval_ms == 0) {
            // unused bucket
            if (empty_bucket_id == -1) {
                empty_bucket_id = i;
            }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            if (bucket.ap_message_ids.count() != 0) {
                AP_HAL::panic("Bucket %u has zero interval but with ids set", i);
            }
#endif
            continue;
        }
        if (bucket.ap_message_ids.get(id)) {
            in_bucket = i;
        }
        const uint16_t interval_delta = abs(bucket.interval_ms - interval_ms);
        if (interval_delta < closest_bucket_interval_delta) {
            closest_bucket = i;
            closest_bucket_interval_delta = interval_delta;
        }
    }

    if (in_bucket == -1 && interval_ms == 0) {
        // not in a bucket and told to remove from scheduling
        return true;
    }

    if (in_bucket != -1) {
        if (interval_ms == 0) {
            // remove it
            remove_message_from_bucket(in_bucket, id);
            return true;
        }
        if (closest_bucket_interval_delta == 0 &&
            in_bucket == closest_bucket) {
            // don't need to move it
            return true;
        }
        // remove from existing bucket
        remove_message_from_bucket(in_bucket, id);
        if (empty_bucket_id == -1 &&
            deferred_message_bucket[in_bucket].ap_message_ids.count() == 0) {
            empty_bucket_id = in_bucket;
        }
    }

    if (closest_bucket == -1 && empty_bucket_id == -1) {
        // gah?!
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::fprintf(stderr, "no buckets?!\n");
        abort();
#endif
        return false;
    }

    if (closest_bucket_interval_delta != 0 &&
        empty_bucket_id != -1) {
        // allocate a bucket for this interval
        deferred_message_bucket[empty_bucket_id].interval_ms = interval_ms;
        deferred_message_bucket[empty_bucket_id].last_sent_ms = AP_HAL::millis16();
        closest_bucket = empty_bucket_id;
    }

    deferred_message_bucket[closest_bucket].ap_message_ids.set(id);

    if (sending_bucket_id == no_bucket_to_send) {
        sending_bucket_id = closest_bucket;
        bucket_message_ids_to_send = deferred_message_bucket[closest_bucket].ap_message_ids;
    }

    return true;
}

// queue a message to be sent (try_send_message does the *actual*
// mavlink work!)
void GCS_MAVLINK::send_message(enum ap_message id)
{
    switch (id) {
    case MSG_HEARTBEAT:
#if HAL_HIGH_LATENCY2_ENABLED
    case MSG_HIGH_LATENCY2:
#endif
#if AP_MAVLINK_SIGNING_ENABLED
        save_signing_timestamp(false);
#endif // AP_MAVLINK_SIGNING_ENABLED
        // update the mask of all streaming channels
        if (is_streaming()) {
            GCS_MAVLINK::chan_is_streaming |= (1U<<(chan-MAVLINK_COMM_0));
        } else {
            GCS_MAVLINK::chan_is_streaming &= ~(1U<<(chan-MAVLINK_COMM_0));
        }
        break;
    default:
        break;
    }

    pushed_ap_message_ids.set(id);
}

// Plane only enables follow when scripting is enabled:
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_MAVLINK_FOLLOW_HANDLING_ENABLED (AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED)
#elif APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_COPTER_OR_HELI
#define AP_MAVLINK_FOLLOW_HANDLING_ENABLED AP_FOLLOW_ENABLED
#else
#define AP_MAVLINK_FOLLOW_HANDLING_ENABLED 0
#endif

void GCS_MAVLINK::packetReceived(const mavlink_status_t &status,
                                 const mavlink_message_t &msg)
{
    // we exclude radio packets because we historically used this to
    // make it possible to use the CLI over the radio
    if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
        mavlink_active |= (1U<<(chan-MAVLINK_COMM_0));
    }
    const auto mavlink_protocol = uartstate->get_protocol();
    if (!(status.flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) &&
        (status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) &&
        (mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLink2 ||
         mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLinkHL)) {
        // if we receive any MAVLink2 packets on a connection
        // currently sending MAVLink1 then switch to sending
        // MAVLink2
        _channel_status.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
}

void GCS_MAVLINK::raw_packetReceived(uint8_t framing_status,
                                     const mavlink_status_t &status,
                                     const mavlink_message_t &msg)
{
    if (framing_status == MAVLINK_FRAMING_OK) {
        packetReceived(status, msg);
    }

    if (!routing.check_and_forward(framing_status, *this, msg)) {
        // the routing code has indicated we should not handle this packet locally
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
#if HAL_MOUNT_ENABLED
        // allow mounts to see the location of other vehicles
        handle_mount_message(msg);
#endif
    }
#if AP_SCRIPTING_ENABLED
    {
        AP_Scripting *scripting = AP_Scripting::get_singleton();
        if (scripting != nullptr) {
            scripting->handle_message(msg, chan);
        }
    }
#endif // AP_SCRIPTING_ENABLED

#if AP_MAVLINK_FOLLOW_HANDLING_ENABLED
    {
        AP_Follow *follow = AP_Follow::get_singleton();
        if (follow != nullptr) {
            follow->handle_msg(msg);
        }
    }
#endif

    if (!accept_packet(status, msg)) {
        // e.g. enforce-sysid says we shouldn't look at this packet
        return;
    }
    handle_message(msg);
}

void
GCS_MAVLINK::update_receive(uint32_t max_time_us)
{
    // do absolutely nothing if we are locked
    if (locked()) {
        return;
    }

    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t tstart_us = AP_HAL::micros();
    uint32_t now_ms = AP_HAL::millis();

    status.packet_rx_drop_count = 0;

    const uint16_t nbytes = _port->available();
    for (uint16_t i=0; i<nbytes; i++)
    {
        const uint8_t c = (uint8_t)_port->read();
        const uint32_t protocol_timeout = 4000;
        
        if (alternative.handler &&
            now_ms - alternative.last_mavlink_ms > protocol_timeout) {
            /*
              we have an alternative protocol handler installed and we
              haven't parsed a MAVLink packet for 4 seconds. Try
              parsing using alternative handler
             */
            if (alternative.handler(c, mavlink_comm_port[chan])) {
                alternative.last_alternate_ms = now_ms;
                gcs_alternative_active[chan] = true;
            }
            
            /*
              we may also try parsing as MAVLink if we haven't had a
              successful parse on the alternative protocol for 4s
             */
            if (now_ms - alternative.last_alternate_ms <= protocol_timeout) {
                continue;
            }
        }

        bool parsed_packet = false;

        // Try to get a new message
        const uint8_t framing = mavlink_frame_char_buffer(channel_buffer(), channel_status(), c, &msg, &status);
        if (framing != MAVLINK_FRAMING_INCOMPLETE) {
            hal.util->persistent_data.last_mavlink_msgid = msg.msgid;
            raw_packetReceived(framing, status, msg);
            if (framing == MAVLINK_FRAMING_OK) {
                parsed_packet = true;
                gcs_alternative_active[chan] = false;
                alternative.last_mavlink_ms = now_ms;
            }
            hal.util->persistent_data.last_mavlink_msgid = 0;

        }
#if AP_SCRIPTING_ENABLED
        if (framing == MAVLINK_FRAMING_BAD_CRC) {
            // This may be a valid message that we don't know the crc extra for, pass it to scripting which might
            AP_Scripting *scripting = AP_Scripting::get_singleton();
            if (scripting != nullptr) {
                scripting->handle_message(msg, chan);
            }
        }
#endif // AP_SCRIPTING_ENABLED

        if (parsed_packet || i % 100 == 0) {
            // make sure we don't spend too much time parsing mavlink messages
            if (AP_HAL::micros() - tstart_us > max_time_us) {
                break;
            }
        }
    }

    const uint32_t tnow = AP_HAL::millis();

    // send a timesync message every 10 seconds; this is for data
    // collection purposes
#if HAL_HIGH_LATENCY2_ENABLED
    if (tnow - _timesync_request.last_sent_ms > _timesync_request.interval_ms && !is_private() && !is_high_latency_link) {
#else
    if (tnow - _timesync_request.last_sent_ms > _timesync_request.interval_ms && !is_private()) {
#endif
        if (HAVE_PAYLOAD_SPACE(chan, TIMESYNC)) {
            send_timesync();
            _timesync_request.last_sent_ms = tnow;
        }
    }

#if HAL_LOGGING_ENABLED
    // consider logging mavlink stats:
    if (is_active() || is_streaming()) {
        if (tnow - last_mavlink_stats_logged > 1000) {
            log_mavlink_stats();
            last_mavlink_stats_logged = tnow;
        }
    }
#endif

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS

    const uint16_t now16_ms{AP_HAL::millis16()};

    if (uint16_t(now16_ms - try_send_message_stats.statustext_last_sent_ms) > 10000U) {
        if (try_send_message_stats.longest_time_us) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): ap_msg=%u took %uus to send",
                            chan,
                            try_send_message_stats.longest_id,
                            try_send_message_stats.longest_time_us);
            try_send_message_stats.longest_time_us = 0;
        }
        if (try_send_message_stats.no_space_for_message &&
            (is_active() || is_streaming())) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): out-of-space: %u",
                            chan,
                            try_send_message_stats.no_space_for_message);
            try_send_message_stats.no_space_for_message = 0;
        }
        if (try_send_message_stats.out_of_time) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): out-of-time=%u",
                            chan,
                            try_send_message_stats.out_of_time);
            try_send_message_stats.out_of_time = 0;
        }
        if (max_slowdown_ms) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): max slowdown=%u",
                            chan,
                            max_slowdown_ms);
            max_slowdown_ms = 0;
        }
        if (try_send_message_stats.behind) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): behind=%u",
                            chan,
                            try_send_message_stats.behind);
            try_send_message_stats.behind = 0;
        }
        if (try_send_message_stats.fnbts_maxtime) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): fnbts_maxtime=%uus",
                            chan,
                            try_send_message_stats.fnbts_maxtime);
            try_send_message_stats.fnbts_maxtime = 0;
        }
        if (try_send_message_stats.max_retry_deferred_body_us) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): retry_body_maxtime=%uus (%u)",
                            chan,
                            try_send_message_stats.max_retry_deferred_body_us,
                            try_send_message_stats.max_retry_deferred_body_type
                );
            try_send_message_stats.max_retry_deferred_body_us = 0;
        }

        for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                            "B. intvl. (%u): %u %u %u %u %u",
                            chan,
                            deferred_message_bucket[0].interval_ms,
                            deferred_message_bucket[1].interval_ms,
                            deferred_message_bucket[2].interval_ms,
                            deferred_message_bucket[3].interval_ms,
                            deferred_message_bucket[4].interval_ms);
        }

        try_send_message_stats.statustext_last_sent_ms = now16_ms;
    }
#endif
}

#if HAL_LOGGING_ENABLED
/*
  record stats about this link to logger
*/
void GCS_MAVLINK::log_mavlink_stats()
{
    uint8_t flags = 0;
#if AP_MAVLINK_SIGNING_ENABLED
    if (signing_enabled()) {
        flags |= (uint8_t)Flags::USING_SIGNING;
    }
#endif  // AP_MAVLINK_SIGNING_ENABLED
    if (is_streaming()) {
        flags |= (uint8_t)Flags::STREAMING;
    }
    if (is_active()) {
        flags |= (uint8_t)Flags::ACTIVE;
    }
    if (is_private()) {
        flags |= (uint8_t)Flags::PRIVATE;
    }
    if (locked()) {
        flags |= (uint8_t)Flags::LOCKED;
    }

    const struct log_MAV pkt{
    LOG_PACKET_HEADER_INIT(LOG_MAV_MSG),
    time_us                : AP_HAL::micros64(),
    chan                   : (uint8_t)chan,
    packet_tx_count        : send_packet_count,
    packet_rx_success_count: _channel_status.packet_rx_success_count,
    packet_rx_drop_count   : _channel_status.packet_rx_drop_count,
    flags                  : flags,
    stream_slowdown_ms     : stream_slowdown_ms,
    times_full             : out_of_space_to_send_count,
    GCS_SYSID_last_seen_ms : _sysid_gcs_last_seen_time_ms,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif

/*
  send the SYSTEM_TIME message
 */
void GCS_MAVLINK::send_system_time() const
{
    uint64_t time_unix = 0;
#if AP_RTC_ENABLED
    AP::rtc().get_utc_usec(time_unix); // may fail, leaving time_unix at 0
#endif

    mavlink_msg_system_time_send(
        chan,
        time_unix,
        AP_HAL::millis());
}


bool GCS_MAVLINK::sending_mavlink1() const
{
    return ((_channel_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0);
}

#if AP_RC_CHANNEL_ENABLED
/*
  send RC_CHANNELS messages
 */
void GCS_MAVLINK::send_rc_channels() const
{
    uint16_t values[18] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));

    mavlink_msg_rc_channels_send(
        chan,
        AP_HAL::millis(),
        RC_Channels::get_valid_channel_count(),
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
        values[7],
        values[8],
        values[9],
        values[10],
        values[11],
        values[12],
        values[13],
        values[14],
        values[15],
        values[16],
        values[17],
#if AP_RSSI_ENABLED
        receiver_rssi()
#else
        255  // meaning "unknown"
#endif
        );
}

#if AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
void GCS_MAVLINK::send_rc_channels_raw() const
{
    // for mavlink1 send RC_CHANNELS_RAW, for compatibility with OSD
    // implementations
    if (!sending_mavlink1()) {
        return;
    }

    uint16_t values[8] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));

    mavlink_msg_rc_channels_raw_send(
        chan,
        AP_HAL::millis(),
        0,
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
        values[7],
#if AP_RSSI_ENABLED
        receiver_rssi()
#else
        255  // meaning "unknown"
#endif
);
}
#endif  // AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
#endif  // AP_RC_CHANNEL_ENABLED

void GCS_MAVLINK::send_raw_imu()
{
#if AP_INERTIALSENSOR_ENABLED
    const AP_InertialSensor &ins = AP::ins();

    const Vector3f &accel = ins.get_accel(0);
    const Vector3f &gyro = ins.get_gyro(0);
    Vector3f mag;
#if AP_COMPASS_ENABLED
    const Compass &compass = AP::compass();
    if (compass.get_count() >= 1) {
        mag = compass.get_field(0);
    }
#endif

    mavlink_msg_raw_imu_send(
        chan,
        AP_HAL::micros64(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z,
        0,  // we use SCALED_IMU and SCALED_IMU2 for other IMUs
        int16_t(ins.get_temperature(0)*100));
#endif
}

#if AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
void GCS_MAVLINK::send_highres_imu()
{
    static const uint16_t HIGHRES_IMU_UPDATED_XACC = 0x01;
    static const uint16_t HIGHRES_IMU_UPDATED_YACC = 0x02;
    static const uint16_t HIGHRES_IMU_UPDATED_ZACC = 0x04;
    static const uint16_t HIGHRES_IMU_UPDATED_XGYRO = 0x08;
    static const uint16_t HIGHRES_IMU_UPDATED_YGYRO = 0x10;
    static const uint16_t HIGHRES_IMU_UPDATED_ZGYRO = 0x20;
#if AP_COMPASS_ENABLED
    static const uint16_t HIGHRES_IMU_UPDATED_XMAG = 0x40;
    static const uint16_t HIGHRES_IMU_UPDATED_YMAG = 0x80;
    static const uint16_t HIGHRES_IMU_UPDATED_ZMAG = 0x100;
#endif  // AP_COMPASS_ENABLED
#if AP_BARO_ENABLED
    static const uint16_t HIGHRES_IMU_UPDATED_ABS_PRESSURE = 0x200;
    static const uint16_t HIGHRES_IMU_UPDATED_DIFF_PRESSURE = 0x400;
    static const uint16_t HIGHRES_IMU_UPDATED_PRESSURE_ALT = 0x800;
    static const uint16_t HIGHRES_IMU_UPDATED_TEMPERATURE = 0x1000;
#endif  // AP_BARO_ENABLED

    const AP_InertialSensor &ins = AP::ins();
    const Vector3f& accel = ins.get_accel();
    const Vector3f& gyro = ins.get_gyro();

    mavlink_highres_imu_t reply = {
        .time_usec = AP_HAL::micros64(),
        .xacc = accel.x,
        .yacc = accel.y,
        .zacc = accel.z,
        .xgyro = gyro.x,
        .ygyro = gyro.y,
        .zgyro = gyro.z,
        .xmag = 0.0, 
        .ymag = 0.0, 
        .zmag = 0.0, 
        .abs_pressure = 0.0, 
        .diff_pressure = 0.0,
        .pressure_alt = 0.0, 
        .temperature = 0.0,  
        .fields_updated = (HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_YACC | HIGHRES_IMU_UPDATED_ZACC |
            HIGHRES_IMU_UPDATED_XGYRO | HIGHRES_IMU_UPDATED_YGYRO | HIGHRES_IMU_UPDATED_ZGYRO), 
        .id = ins.get_first_usable_accel(),
    };

#if AP_COMPASS_ENABLED
    const Compass &compass = AP::compass();
    if (compass.get_count() >= 1) {
        const Vector3f field = compass.get_field() * 1000.0f;
        reply.xmag = field.x; // convert to gauss
        reply.ymag = field.y;
        reply.zmag = field.z;
        reply.fields_updated |= (HIGHRES_IMU_UPDATED_XMAG | HIGHRES_IMU_UPDATED_YMAG | HIGHRES_IMU_UPDATED_ZMAG);
    }
#endif

#if AP_BARO_ENABLED
    const AP_Baro &barometer = AP::baro();
    reply.abs_pressure = barometer.get_pressure() * 0.01f;
    reply.temperature = barometer.get_temperature();
    reply.pressure_alt = barometer.get_altitude_AMSL();
    reply.diff_pressure = reply.abs_pressure - barometer.get_ground_pressure() * 0.01f;
    reply.fields_updated |= (HIGHRES_IMU_UPDATED_ABS_PRESSURE | HIGHRES_IMU_UPDATED_DIFF_PRESSURE |
        HIGHRES_IMU_UPDATED_PRESSURE_ALT | HIGHRES_IMU_UPDATED_TEMPERATURE);
#endif

    mavlink_msg_highres_imu_send_struct(chan, &reply);
}
#endif // AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED

void GCS_MAVLINK::send_scaled_imu(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature))
{
#if AP_INERTIALSENSOR_ENABLED
    const AP_InertialSensor &ins = AP::ins();
    int16_t _temperature = 0;

    bool have_data = false;
    Vector3f accel{};
    if (ins.get_accel_count() > instance) {
        accel = ins.get_accel(instance);
        _temperature = ins.get_temperature(instance)*100;
        have_data = true;
    }
    Vector3f gyro{};
    if (ins.get_gyro_count() > instance) {
        gyro = ins.get_gyro(instance);
        have_data = true;
    }
    Vector3f mag;
#if AP_COMPASS_ENABLED
    const Compass &compass = AP::compass();
    if (compass.get_count() > instance) {
        mag = compass.get_field(instance);
        have_data = true;
    }
#endif
    if (!have_data) {
        return;
    }
    send_fn(
        chan,
        AP_HAL::millis(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z,
        _temperature);
#endif
}


// send data for barometer and airspeed sensors instances.  In the
// case that we run out of instances of one before the other we send
// the relevant fields as 0.
void GCS_MAVLINK::send_scaled_pressure_instance(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff))
{
    const AP_Baro &barometer = AP::baro();

    bool have_data = false;

    float press_abs = 0.0f;
    int16_t temperature = 0; // Absolute pressure temperature
    int16_t temperature_press_diff = 0; // Differential pressure temperature
    if (instance < barometer.num_instances()) {
        press_abs = barometer.get_pressure(instance) * 0.01f;
        temperature = barometer.get_temperature(instance)*100;
        have_data = true;
    }

    float press_diff = 0; // pascal
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr &&
        airspeed->enabled(instance)) {
        press_diff = airspeed->get_differential_pressure(instance) * 0.01f;
        float temp;
        if (airspeed->get_temperature(instance,temp)) {
            temperature_press_diff = temp * 100;
            if (temperature_press_diff == 0) {
                // don't send zero as that is the value for 'no data'
                temperature_press_diff = 1;
            }
        }
        have_data = true;
    }
#endif

    if (!have_data) {
        return;
    }

    send_fn(
        chan,
        AP_HAL::millis(),
        press_abs, // hectopascal
        press_diff, // hectopascal
        temperature, // 0.01 degrees C
        temperature_press_diff); // 0.01 degrees C
}

void GCS_MAVLINK::send_scaled_pressure()
{
    send_scaled_pressure_instance(0, mavlink_msg_scaled_pressure_send);
}

void GCS_MAVLINK::send_scaled_pressure2()
{
    send_scaled_pressure_instance(1, mavlink_msg_scaled_pressure2_send);
}

void GCS_MAVLINK::send_scaled_pressure3()
{
    send_scaled_pressure_instance(2, mavlink_msg_scaled_pressure3_send);
}

#if AP_AIRSPEED_ENABLED
void GCS_MAVLINK::send_airspeed()
{
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed == nullptr) {
        return;
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        // Try and send the next sensor
        const uint8_t index = (last_airspeed_idx + 1 + i) % AIRSPEED_MAX_SENSORS;
        if (!airspeed->enabled(index)) {
            continue;
        }

        float temperature_float;
        int16_t temperature = INT16_MAX;
        if (airspeed->get_temperature(index, temperature_float)) {
            temperature = int16_t(temperature_float * 100);
        }

        uint8_t flags = 0;
        // Set unhealthy flag
        if (!airspeed->healthy(index)) {
            flags |= AIRSPEED_SENSOR_FLAGS::AIRSPEED_SENSOR_UNHEALTHY;
        }

#if AP_AHRS_ENABLED
        // Set using flag if the AHRS is using this sensor
        const AP_AHRS &ahrs = AP::ahrs();
        if (ahrs.using_airspeed_sensor() && (ahrs.get_active_airspeed_index() == index)) {
            flags |= AIRSPEED_SENSOR_FLAGS::AIRSPEED_SENSOR_USING;
        }
#endif

        // Assemble message and send
        const mavlink_airspeed_t msg {
            airspeed    : airspeed->get_airspeed(index),
            raw_press   : airspeed->get_differential_pressure(index),
            temperature : temperature,
            id          : index,
            flags       : flags
        };

        mavlink_msg_airspeed_send_struct(chan, &msg);

        // Only send one msg per call
        last_airspeed_idx = index;
        return;
    }

}
#endif // AP_AIRSPEED_ENABLED

#if AP_AHRS_ENABLED
void GCS_MAVLINK::send_ahrs()
{
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f &omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}
#endif  // AP_AHRS_ENABLED

/*
    send a statustext text string to specific MAVLink bitmask
*/
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, mavlink_channel_mask_t dest_bitmask)
{
    char first_piece_of_text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1]{};

    do {
        // send_text can be called from multiple threads; we must
        // protect the "text" member with _statustext_sem
        WITH_SEMAPHORE(_statustext_queue.semaphore());
        hal.util->vsnprintf(statustext_printf_buffer, sizeof(statustext_printf_buffer), fmt, arg_list);
        memcpy(first_piece_of_text, statustext_printf_buffer, ARRAY_SIZE(first_piece_of_text)-1);

        // filter destination ports to only allow active ports.
        statustext_t statustext{};
        if (update_send_has_been_called) {
            statustext.bitmask = statustext_send_channel_mask();
        } else {
            // we have not yet initialised the streaming-channel-mask,
            // which is done as part of the update() call.  So just send
            // it to all channels:
            statustext.bitmask = (1<<_num_gcs)-1;
        }
        statustext.bitmask &= dest_bitmask;
        if (!statustext.bitmask) {
            // nowhere to send
            break;
        }

        statustext.entry_created_ms = AP_HAL::millis16();
        statustext.msg.severity = severity;

        static uint16_t msgid;
        if (strlen(statustext_printf_buffer) > sizeof(statustext.msg.text)) {
            msgid++;
            if (msgid == 0) {
                msgid = 1;
            }
            statustext.msg.id = msgid;
        }

        const char *remainder = statustext_printf_buffer;
        for (uint8_t i=0; i<_status_capacity; i++) {
            statustext.msg.chunk_seq = i;
            const size_t remainder_len = strlen(remainder);
            // note that remainder_len may be zero here!
            uint16_t n = MIN(sizeof(statustext.msg.text), remainder_len);
            if (i == _status_capacity -1 && n == sizeof(statustext.msg.text)) {
                // fantastic.  This us a very long statustext and
                // we've actually managed to push everything else out
                // of the queue - this is the last chunk, so we MUST
                // null-terminate.
                n -= 1;
            }
            memset(statustext.msg.text, '\0', sizeof(statustext.msg.text));
            memcpy(statustext.msg.text, remainder, n);
            // The force push will ensure comm links do not block other comm links forever if they fail.
            // If we push to a full buffer then we overwrite the oldest entry, effectively removing the
            // block but not until the buffer fills up.
            _statustext_queue.push_force(statustext);
            remainder = &remainder[n];

            // note that remainder_len here is the remainder length for
            // the *old* remainder!
            if (remainder_len < sizeof(statustext.msg.text) || statustext.msg.id == 0) {
                break;
            }
        }
    } while (false);

#if HAL_LOGGING_ENABLED
    // given we don't really know what these methods get up to, we
    // don't hold the statustext semaphore while doing them:
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Message(first_piece_of_text);
    }
#endif

#if AP_FRSKY_TELEM_ENABLED
    frsky = AP::frsky_telem();
    if (frsky != nullptr) {
        frsky->queue_message(severity, first_piece_of_text);
    }
#endif
#if HAL_SPEKTRUM_TELEM_ENABLED
    AP_Spektrum_Telem* spektrum = AP::spektrum_telem();
    if (spektrum != nullptr) {
        spektrum->queue_message(severity, first_piece_of_text);
    }
#endif
#if HAL_CRSF_TELEM_ENABLED
    AP_CRSF_Telem* crsf = AP::crsf_telem();
    if (crsf != nullptr) {
        crsf->queue_message(severity, first_piece_of_text);
    }
#endif
    AP_Notify *notify = AP_Notify::get_singleton();
    if (notify) {
        notify->send_text(first_piece_of_text);
    }

    // push the messages out straight away until the vehicle states
    // that it is initialised.  At that point we can assume
    // update_send is being called
    if (!vehicle_initialised()) {
        service_statustext();
    }
}

void GCS::service_statustext(void)
{
    WITH_SEMAPHORE(_statustext_queue.semaphore());

    if (_statustext_queue.is_empty()) {
        // nothing to do
        return;
    }

    for (uint8_t i=first_backend_to_send; i<num_gcs(); i++) {
        chan(i)->service_statustext();
    }
    for (uint8_t i=0; i<first_backend_to_send; i++) {
        chan(i)->service_statustext();
    }

    _statustext_queue.prune();
}

void GCS::StatusTextQueue::prune(void)
{
    // consider pruning the statustext queue of ancient entries
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_prune_ms < 1000) {
        return;
    }
    last_prune_ms = now_ms;

    const uint16_t now16_ms = AP_HAL::millis16();
    for (uint8_t idx=0; idx<available(); ) {
        const GCS::statustext_t *statustext = (*this)[idx];
        if (statustext == nullptr) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        // be wary of integer promotion here
        const uint16_t age = now16_ms - statustext->entry_created_ms;
        if (age > 5000) {
            // too old.  Purge it.
            remove(idx);
            continue;
        }
        // this is a queue.  If this one wasn't too old then the next
        // one isn't either.
        break;
    }
}

/*
    send a statustext message to specific MAVLink connections in a bitmask

    must be called with semaphore held
 */
void GCS_MAVLINK::service_statustext(void)
{
    GCS::StatusTextQueue &_statustext_queue = gcs().statustext_queue();

    const mavlink_channel_mask_t chan_bit = (1U<<chan);
    // note the lack of idx++ here.  We may remove the iteration item
    // from the queue as the last thing we do, in which case we don't
    // want to move idx.
    const uint16_t payload_size = PAYLOAD_SIZE(chan, STATUSTEXT);
    for (uint8_t idx=0; idx<_statustext_queue.available(); ) {
        WITH_SEMAPHORE(comm_chan_lock(chan));

        if (txspace() < payload_size) {
            break;
        }
        GCS::statustext_t *statustext = _statustext_queue[idx];
        if (statustext == nullptr) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }

        // check to see if we need to send this queue entry:
        if (statustext->bitmask & chan_bit) {
            mavlink_msg_statustext_send(chan,
                                        statustext->msg.severity,
                                        statustext->msg.text,
                                        statustext->msg.id,
                                        statustext->msg.chunk_seq);
            // indicate we've sent the message:
            statustext->bitmask &= ~chan_bit;

            if (statustext->bitmask == 0) {
                // sent everywhere it needs to be sent, remove it from the
                // queue but leave idx as-is as we want to handle the
                // remaining items which have been bumped up to out
                // current index
                _statustext_queue.remove(idx);
                continue;
            }
        }
        // this item still has places to go.  Continue to iterate over the queue
        idx++;
    }
}

void GCS::send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        chan(i)->send_message(id);
    }
}

void GCS::update_send()
{
    // cope with changes to mavlink system ID parameter
    mavlink_system.sysid = sysid;

    update_send_has_been_called = true;

    if (!initialised_missionitemprotocol_objects) {
        initialised_missionitemprotocol_objects = true;
        // once-only initialisation of MissionItemProtocol objects:
#if AP_MISSION_ENABLED
        AP_Mission *mission = AP::mission();
        if (mission != nullptr) {
            missionitemprotocols[MAV_MISSION_TYPE_MISSION] = NEW_NOTHROW MissionItemProtocol_Waypoints(*mission);
        }
#endif
#if HAL_RALLY_ENABLED
        AP_Rally *rally = AP::rally();
        if (rally != nullptr) {
            missionitemprotocols[MAV_MISSION_TYPE_RALLY] = NEW_NOTHROW MissionItemProtocol_Rally(*rally);
        }
#endif
#if AP_FENCE_ENABLED
        AC_Fence *fence = AP::fence();
        if (fence != nullptr) {
            missionitemprotocols[MAV_MISSION_TYPE_FENCE] = NEW_NOTHROW MissionItemProtocol_Fence(*fence);
        }
#endif
    }

    for (auto *prot : missionitemprotocols) {
        if (prot == nullptr) {
            continue;
        }
        prot->update();
    }

    // round-robin the GCS_MAVLINK backend that gets to go first so
    // one backend doesn't monopolise all of the time allowed for sending
    // messages
    for (uint8_t i=first_backend_to_send; i<num_gcs(); i++) {
        chan(i)->update_send();
    }
    for (uint8_t i=0; i<first_backend_to_send; i++) {
        chan(i)->update_send();
    }

    service_statustext();

    first_backend_to_send++;
    if (first_backend_to_send >= num_gcs()) {
        first_backend_to_send = 0;
    }
}

void GCS::update_receive(void)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        chan(i)->update_receive();
    }
    // also update UART pass-thru, if enabled
    update_passthru();
}

void GCS::send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        chan(i)->mission_item_reached_index = mission_index;
        chan(i)->send_message(MSG_MISSION_ITEM_REACHED);
    }
}

void GCS::setup_console()
{
    AP_HAL::UARTDriver *uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_MAVLink, 0);
    if (uart == nullptr) {
        // this is probably not going to end well.
        return;
    }
    if (ARRAY_SIZE(_chan_var_info) == 0) {
        return;
    }
    create_gcs_mavlink_backend(*uart);
}


void GCS::create_gcs_mavlink_backend(AP_HAL::UARTDriver &uart)
{
    if (_num_gcs >= ARRAY_SIZE(_chan_var_info)) {
        return;
    }
    _chan[_num_gcs] = new_gcs_mavlink_backend(uart);
    if (_chan[_num_gcs] == nullptr) {
        return;
    }

    // prepare parameters:
    _chan_var_info[_num_gcs] = _chan[_num_gcs]->var_info;
    AP_Param::load_object_from_eeprom(_chan[_num_gcs], _chan_var_info[_num_gcs]);

    if (!_chan[_num_gcs]->init(_num_gcs)) {
        delete _chan[_num_gcs];
        _chan[_num_gcs] = nullptr;
        return;
    }

    _num_gcs++;
}

void GCS::setup_uarts()
{
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        AP_HAL::UARTDriver *uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_MAVLink, i);
        if (uart == nullptr) {
            // no more mavlink uarts
            break;
        }
        create_gcs_mavlink_backend(*uart);
    }

#if AP_FRSKY_TELEM_ENABLED
    if (frsky == nullptr) {
        frsky = NEW_NOTHROW AP_Frsky_Telem();
        if (frsky == nullptr || !frsky->init()) {
            delete frsky;
            frsky = nullptr;
        }
    }
#endif

#if AP_LTM_TELEM_ENABLED
    ltm_telemetry.init();
#endif

#if AP_DEVO_TELEM_ENABLED
    devo_telemetry.init();
#endif
}

/*
  handle a SET_MODE MAVLink message
 */
void GCS_MAVLINK::handle_set_mode(const mavlink_message_t &msg)
{
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(&msg, &packet);

    const uint8_t _base_mode = (uint8_t)packet.base_mode;
    const uint32_t _custom_mode = packet.custom_mode;

    _set_mode_common(_base_mode, _custom_mode);
}

/*
  code common to both SET_MODE mavlink message and command long set_mode msg
*/
MAV_RESULT GCS_MAVLINK::_set_mode_common(const uint8_t _base_mode, const uint32_t _custom_mode)
{
    // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
#if AP_VEHICLE_ENABLED
    if ((_base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0) {
        if (!AP::vehicle()->set_mode(_custom_mode, ModeReason::GCS_COMMAND)) {
            // often we should be returning DENIED rather than FAILED
            // here.  Perhaps a "has_mode" callback on AP_::vehicle()
            // would do?
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }
#endif

    if (_base_mode == MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
        // set the safety switch position. Must be in a command by itself
        if (_custom_mode == 0) {
            // turn safety off (pwm outputs flow to the motors)
            hal.rcout->force_safety_off();
            return MAV_RESULT_ACCEPTED;
        }
        if (_custom_mode == 1) {
            // turn safety on (no pwm outputs to the motors)
            if (hal.rcout->force_safety_on()) {
                return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_DENIED;
    }

    // Command is invalid (is supported but has invalid parameters)
    return MAV_RESULT_DENIED;
}

#if AP_OPTICALFLOW_ENABLED
/*
  send OPTICAL_FLOW message
 */
void GCS_MAVLINK::send_opticalflow()
{
    const AP_OpticalFlow *optflow = AP::opticalflow();

    // exit immediately if no optical flow sensor or not healthy
    if (optflow == nullptr ||
        !optflow->healthy()) {
        return;
    }

    // get rates from sensor
    const Vector2f &flowRate = optflow->flowRate();
    const Vector2f &bodyRate = optflow->bodyRate();

    float hagl = 0;
#if AP_AHRS_ENABLED
    if (!AP::ahrs().get_hagl(hagl)) {
        hagl = 0;
    }
#endif

    // populate and send message
    mavlink_msg_optical_flow_send(
        chan,
        AP_HAL::millis(),
        0, // sensor id is zero
        flowRate.x,
        flowRate.y,
        flowRate.x - bodyRate.x,
        flowRate.y - bodyRate.y,
        optflow->quality(),
        hagl,  // ground distance (in meters) set to zero
        flowRate.x,
        flowRate.y);
}
#endif  // AP_OPTICALFLOW_ENABLED

/*
  send AUTOPILOT_VERSION packet
 */
void GCS_MAVLINK::send_autopilot_version() const
{
    uint32_t flight_sw_version;
    uint32_t middleware_sw_version = 0;
#ifdef APJ_BOARD_ID
    uint32_t board_version { uint32_t(APJ_BOARD_ID) << 16 };
#else
    uint32_t board_version = 0;
#endif
    char flight_custom_version[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN]{};
    char middleware_custom_version[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN]{};
    char os_custom_version[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN]{};
#ifdef HAL_USB_VENDOR_ID
    const uint16_t vendor_id { HAL_USB_VENDOR_ID };
    const uint16_t product_id { HAL_USB_PRODUCT_ID };
#else
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
#endif
    uint64_t uid = 0;
    uint8_t  uid2[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_LEN] = {0};

    uint8_t uid_len = sizeof(uid2); // taken as reference and modified
                                    // by following call:
    hal.util->get_system_id_unformatted(uid2, uid_len);

    const AP_FWVersion &version = AP::fwversion();

    flight_sw_version = version.major << (8 * 3) | \
                        version.minor << (8 * 2) | \
                        version.patch << (8 * 1) | \
                        (uint32_t)(version.fw_type) << (8 * 0);

    if (version.fw_hash_str) {
        strncpy_noterm(flight_custom_version, version.fw_hash_str, ARRAY_SIZE(flight_custom_version));
    }

    if (version.middleware_hash_str) {
        strncpy_noterm(middleware_custom_version, version.middleware_hash_str, ARRAY_SIZE(middleware_custom_version));
    }

    if (version.os_hash_str) {
        strncpy_noterm(os_custom_version, version.os_hash_str, ARRAY_SIZE(os_custom_version));
    }

    mavlink_msg_autopilot_version_send(
        chan,
        capabilities(),
        flight_sw_version,
        middleware_sw_version,
        version.os_sw_version,
        board_version,
        (uint8_t *)flight_custom_version,
        (uint8_t *)middleware_custom_version,
        (uint8_t *)os_custom_version,
        vendor_id,
        product_id,
        uid,
        uid2
    );
}


#if AP_AHRS_ENABLED
/*
  send LOCAL_POSITION_NED message
 */
void GCS_MAVLINK::send_local_position() const
{
    const AP_AHRS &ahrs = AP::ahrs();

    Vector3f local_position, velocity;
    if (!ahrs.get_relative_position_NED_origin_float(local_position) ||
        !ahrs.get_velocity_NED(velocity)) {
        // we don't know the position and velocity
        return;
    }

    mavlink_msg_local_position_ned_send(
        chan,
        AP_HAL::millis(),
        local_position.x,
        local_position.y,
        local_position.z,
        velocity.x,
        velocity.y,
        velocity.z);
}
#endif

/*
  send VIBRATION message
 */
void GCS_MAVLINK::send_vibration() const
{
#if AP_INERTIALSENSOR_ENABLED
    const AP_InertialSensor &ins = AP::ins();

    Vector3f vibration = ins.get_vibration_levels();

    mavlink_msg_vibration_send(
        chan,
        AP_HAL::micros64(),
        vibration.x,
        vibration.y,
        vibration.z,
        ins.get_accel_clip_count(0),
        ins.get_accel_clip_count(1),
        ins.get_accel_clip_count(2));
#endif
}

void GCS_MAVLINK::send_named_float(const char *name, float value) const
{
    char float_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN+1] {};
    strncpy(float_name, name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
    mavlink_msg_named_value_float_send(chan, AP_HAL::millis(), float_name, value);
}

#if AP_AHRS_ENABLED
void GCS_MAVLINK::send_home_position() const
{
    if (!AP::ahrs().home_is_set()) {
        return;
    }

    const Location &home = AP::ahrs().get_home();

    // get home position from origin
    Vector3f home_pos_ned;
    if (home.get_vector_from_origin_NEU_cm(home_pos_ned)) {
        // convert NEU in cm to NED in meters
        home_pos_ned *= 0.01f;
        home_pos_ned.z *= -1;
    } else {
        home_pos_ned = Vector3f{NAN, NAN, NAN};
    }

    const float q[4] {NAN, NAN, NAN, NAN};
    mavlink_msg_home_position_send(
        chan,
        home.lat,
        home.lng,
        home.alt * 10,
        home_pos_ned.x,
        home_pos_ned.y,
        home_pos_ned.z,
        q,
        0.0f, 0.0f, 0.0f,
        AP_HAL::micros64());
}

void GCS_MAVLINK::send_gps_global_origin() const
{
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return;
    }
    mavlink_msg_gps_global_origin_send(
        chan,
        ekf_origin.lat,
        ekf_origin.lng,
        ekf_origin.alt * 10,
        AP_HAL::micros64());
}
#endif  // AP_AHRS_ENABLED

MAV_STATE GCS_MAVLINK::system_status() const
{
    MAV_STATE _system_status = vehicle_system_status();
    if (_system_status < MAV_STATE_CRITICAL) {
        // note that POWEROFF and FLIGHT_TERMINATION are both >
        // CRITICAL, so we will not overwrite POWEROFF and
        // FLIGHT_TERMINATION even if we have internal errors.  If new
        // enum entries are added then this may also not overwrite
        // those.
        if (AP::internalerror().errors()) {
            _system_status = MAV_STATE_CRITICAL;
        }
    }
    return _system_status;
}

/*
  Send MAVLink heartbeat
 */
void GCS_MAVLINK::send_heartbeat() const
{
    mavlink_msg_heartbeat_send(
        chan,
        gcs().frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode(),
        gcs().custom_mode(),
        system_status());
}

#if AP_RC_CHANNEL_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_do_aux_function(const mavlink_command_int_t &packet)
{
    if (packet.param2 > 2) {
        return MAV_RESULT_DENIED;
    }
    const RC_Channel::AUX_FUNC aux_func = (RC_Channel::AUX_FUNC)packet.param1;
    const RC_Channel::AuxSwitchPos position = (RC_Channel::AuxSwitchPos)packet.param2;
    if (!rc().run_aux_function(aux_func, position, RC_Channel::AuxFuncTrigger::Source::MAVLINK, chan)) {
        // note that this is not quite right; we could be more nuanced
        // about our return code here.
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif  // AP_RC_CHANNEL_ENABLED

MAV_RESULT GCS_MAVLINK::handle_command_set_message_interval(const mavlink_command_int_t &packet)
{
    if (!is_zero(packet.param3)) {
        return MAV_RESULT_DENIED;
    }
    return set_message_interval((uint32_t)packet.param1, (int32_t)packet.param2);
}

MAV_RESULT GCS_MAVLINK::set_message_interval(uint32_t msg_id, int32_t interval_us)
{
    const ap_message id = mavlink_id_to_ap_message_id(msg_id);
    if (id == MSG_LAST) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "No ap_message for mavlink id (%u)", (unsigned int)msg_id);
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
        return MAV_RESULT_DENIED;
    }

    uint16_t interval_ms;
    if (interval_us == 0) {
        // zero is "reset to default rate"
        if (!get_default_interval_for_ap_message(id, interval_ms)) {
            // if we don't have a default interval then we assume that
            // we do not send that message by default.  That may not
            // be strictly true if some random piece of code has set a
            // rate as part of its initialisation - in which case that
            // piece of code should probably be adding something into
            // whatever get_default_interval_for_ap_message is looking
            // at.
            interval_ms = 0;
        }
    } else if (interval_us == -1) {
        // minus-one is "stop sending"
        interval_ms = 0;
    } else if (interval_us < 0) {  
        return MAV_RESULT_DENIED; 
    } else if (interval_us < 1000) {
        // don't squash sub-ms times to zero
        interval_ms = 1;
    } else if (interval_us > 60000000) {
        interval_ms = 60000;
    } else {
        interval_ms = interval_us / 1000;
    }
#if AP_SCHEDULER_ENABLED
    if (interval_ms != 0 && cap_message_interval(interval_ms) > interval_ms) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Requested rate for message ID %u too fast. Increase SCHED_LOOP_RATE", (unsigned int)msg_id);
        return MAV_RESULT_DENIED;
    }
#endif
    if (set_ap_message_interval(id, interval_ms)) {
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}

/*
  this function is reserved for use by scripting
 */
MAV_RESULT GCS::set_message_interval(uint8_t port_num, uint32_t msg_id, int32_t interval_us)
{
    uint8_t channel = get_channel_from_port_number(port_num);

    GCS_MAVLINK *link = chan(channel);
    if (link == nullptr) {
        return MAV_RESULT_FAILED;
    }
    return link->set_message_interval(msg_id, interval_us);
}

uint8_t GCS::get_channel_from_port_number(uint8_t port_num)
{
    const AP_HAL::UARTDriver *u = AP::serialmanager().get_serial_by_id(port_num);
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i)->get_uart() == u) {
            return i;
        }
    }

    return UINT8_MAX;
}

MAV_RESULT GCS_MAVLINK::handle_command_request_message(const mavlink_command_int_t &packet)
{
    const uint32_t mavlink_id = (uint32_t)packet.param1;

    if (mavlink_id == MAVLINK_MSG_ID_MESSAGE_INTERVAL) {
        const mavlink_command_int_t msg_interval_cmd = { .param1 = packet.param2, };
        return handle_command_get_message_interval(msg_interval_cmd);
    }

    const ap_message id = mavlink_id_to_ap_message_id(mavlink_id);
    if (id == MSG_LAST) {
        return MAV_RESULT_FAILED;
    }

    switch(id) {
    case MSG_AVAILABLE_MODES:
        available_modes.should_send = true;
        available_modes.next_index = 1;
        available_modes.requested_index = (uint8_t)packet.param2;

        // After the first request sequnece is streamed in the AVAILABLE_MODES_MONITOR message
        // This allows the GCS to re-request modes if there is a change
        set_ap_message_interval(MSG_AVAILABLE_MODES_MONITOR, 5000);
        break;

    default:
        break;
    }

    send_message(id);
    return MAV_RESULT_ACCEPTED;
}

bool GCS_MAVLINK::get_ap_message_interval(ap_message id, uint16_t &interval_ms) const
{
    // check if it's a specially-handled message:
    const int8_t deferred_offset = get_deferred_message_index(id);
    if (deferred_offset != -1) {
        interval_ms = deferred_message[deferred_offset].interval_ms;
        return true;
    }

    // check the deferred message buckets:
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
        const deferred_message_bucket_t &bucket = deferred_message_bucket[i];
        if (bucket.ap_message_ids.get(id)) {
            interval_ms = bucket.interval_ms;
            return true;
        }
    }

    return false;
}

MAV_RESULT GCS_MAVLINK::handle_command_get_message_interval(const mavlink_command_int_t &packet)
{
    if (txspace() < PAYLOAD_SIZE(chan, MESSAGE_INTERVAL) + PAYLOAD_SIZE(chan, COMMAND_ACK)) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    const uint32_t mavlink_id = (uint32_t)packet.param1;
    if (mavlink_id >= 2 << 15) {
        // response packet limits range this works against!
        mavlink_msg_message_interval_send(chan, mavlink_id, 0); // not available
        return MAV_RESULT_FAILED;
    }

    const ap_message id = mavlink_id_to_ap_message_id(mavlink_id);
    if (id == MSG_LAST) {
        mavlink_msg_message_interval_send(chan, mavlink_id, 0); // not available
        return MAV_RESULT_FAILED;
    }

    uint16_t interval_ms = 0;
    if (!get_ap_message_interval(id, interval_ms) || interval_ms == 0) {
        // not streaming this message at the moment...
        mavlink_msg_message_interval_send(chan, mavlink_id, -1); // disabled
        return MAV_RESULT_ACCEPTED;
    }

    mavlink_msg_message_interval_send(chan, mavlink_id, interval_ms * 1000);
    return MAV_RESULT_ACCEPTED;
}


// are we still delaying telemetry to try to avoid Xbee bricking?
bool GCS_MAVLINK::telemetry_delayed() const
{
    uint32_t tnow = AP_HAL::millis() >> 10;
    if (tnow >= gcs().telem_delay()) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


/*
  send SERVO_OUTPUT_RAW
 */
void GCS_MAVLINK::send_servo_output_raw()
{
    const uint32_t enabled_mask = ~SRV_Channels::get_output_channel_mask(SRV_Channel::k_GPIO);
    if (enabled_mask == 0) {
        return;
    }

#if NUM_SERVO_CHANNELS >= 17
    static const uint8_t max_channels = 32;
#else
    static const uint8_t max_channels = 16;
#endif

    uint16_t values[max_channels] {};
    hal.rcout->read(values, max_channels);
    for (uint8_t i=0; i<max_channels; i++) {
        if (values[i] == 65535) {
            values[i] = 0;
        }
    }
    if ((enabled_mask & 0xFFFF) != 0) {
        mavlink_msg_servo_output_raw_send(
                chan,
                AP_HAL::micros(),
                0,     // port
                values[0],  values[1],  values[2],  values[3],
                values[4],  values[5],  values[6],  values[7],
                values[8],  values[9],  values[10], values[11],
                values[12], values[13], values[14], values[15]);
    }

#if NUM_SERVO_CHANNELS >= 17
    if ((enabled_mask & 0xFFFF0000) != 0) {
        mavlink_msg_servo_output_raw_send(
                chan,
                AP_HAL::micros(),
                1,     // port
                values[16],  values[17],  values[18],  values[19],
                values[20],  values[21],  values[22],  values[23],
                values[24],  values[25],  values[26], values[27],
                values[28], values[29], values[30], values[31]);
    }
#endif
}


void GCS_MAVLINK::send_accelcal_vehicle_position(uint32_t position)
{
    if (HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
        mavlink_msg_command_long_send(
            chan,
            0,
            0,
            MAV_CMD_ACCELCAL_VEHICLE_POS,
            0,
            (float) position,
            0, 0, 0, 0, 0, 0);
    }
}


float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();
    }
#endif

#if AP_GPS_ENABLED
    // because most vehicles don't have airspeed sensors, we return a
    // different sort of speed estimate in the relevant field for
    // comparison's sake.
    return AP::gps().ground_speed();
#endif

    return 0.0;
}

float GCS_MAVLINK::vfr_hud_climbrate() const
{
#if AP_AHRS_ENABLED
    Vector3f velned;
    if (AP::ahrs().get_velocity_NED(velned) || 
        AP::ahrs().get_vert_pos_rate_D(velned.z)) {
        return -velned.z;
    }
#endif
    return 0;
}

#if AP_AHRS_ENABLED
float GCS_MAVLINK::vfr_hud_alt() const
{
    return global_position_current_loc.alt * 0.01f; // cm -> m
}

void GCS_MAVLINK::send_vfr_hud()
{
    AP_AHRS &ahrs = AP::ahrs();

    // return values ignored; we send stale data
    UNUSED_RESULT(ahrs.get_location(global_position_current_loc));

    mavlink_msg_vfr_hud_send(
        chan,
        vfr_hud_airspeed(),
        ahrs.groundspeed(),
        ahrs.get_yaw_deg(),
        abs(vfr_hud_throttle()),
        vfr_hud_alt(),
        vfr_hud_climbrate());
}
#endif  // AP_AHRS_ENABLED

/*
  handle a MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command 

  Optionally disable PX4IO overrides. This is done for quadplanes to
  prevent the mixer running while rebooting which can start the VTOL
  motors. That can be dangerous when a preflight reboot is done with
  the pilot close to the aircraft and can also damage the aircraft
 */
MAV_RESULT GCS_MAVLINK::handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if ((packet.target_component != MAV_COMP_ID_ALL) && (packet.target_component != mavlink_system.compid)) {
        // Make sure reboot/shutdown commands sent to a component don't cause the autopilot to reboot.
        return MAV_RESULT_DENIED;
    }

    if (is_equal(packet.param1, 42.0f) &&
        is_equal(packet.param2, 24.0f) &&
        is_equal(packet.param3, 71.0f)) {
#if AP_MAVLINK_FAILURE_CREATION_ENABLED
        if (is_equal(packet.param4, 93.0f)) {
            // this is a magic sequence to force the main loop to
            // lockup. This is for testing the stm32 watchdog
            // functionality
            while (true) {
                send_text(MAV_SEVERITY_WARNING,"entering lockup");
                hal.scheduler->delay(250);
            }
        }
        if (is_equal(packet.param4, 94.0f)) {
            // the following text is unlikely to make it out...
            send_text(MAV_SEVERITY_WARNING,"dereferencing a bad thing");

#if CONFIG_HAL_BOARD != HAL_BOARD_ESP32
// esp32 can't do this bit, skip it, return an error
            void *foo = (void*)0xE000ED38;

            typedef void (*fptr)();
            fptr gptr = (fptr) (void *) foo;
            gptr();
#endif
            return MAV_RESULT_FAILED;
        }
        if (is_equal(packet.param4, 95.0f)) {
            // the following text is unlikely to make it out...
            send_text(MAV_SEVERITY_WARNING,"calling AP_HAL::panic(...)");

            AP_HAL::panic("panicing");

            // keep calm and carry on
        }
        if (is_equal(packet.param4, 96.0f)) {
            // deliberately corrupt parameter storage
            send_text(MAV_SEVERITY_WARNING,"wiping parameter storage header");
            StorageAccess param_storage{StorageManager::StorageParam};
            uint8_t zeros[40] {};
            param_storage.write_block(0, zeros, sizeof(zeros));
            return MAV_RESULT_ACCEPTED;
        }
        if (is_equal(packet.param4, 97.0f)) {
            // create a really long loop
            send_text(MAV_SEVERITY_WARNING,"Creating long loop");
            // 250ms:
            for (uint8_t i=0; i<250; i++) {
                hal.scheduler->delay_microseconds(1000);
            }
            return MAV_RESULT_ACCEPTED;
        }
        if (is_equal(packet.param4, 98.0f)) {
            send_text(MAV_SEVERITY_WARNING,"Creating internal error");
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return MAV_RESULT_ACCEPTED;
        }
        if (is_equal(packet.param4, 100.0f)) {
            send_text(MAV_SEVERITY_WARNING,"Creating mutex deadlock");
            hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&GCS_MAVLINK::deadlock_sem, void));
            while (!_deadlock_sem.taken) {
                hal.scheduler->delay(1);
            }
            WITH_SEMAPHORE(_deadlock_sem.sem);
            send_text(MAV_SEVERITY_WARNING,"deadlock passed");
            return MAV_RESULT_ACCEPTED;
        }
        if (is_equal(packet.param4, 101.0f)) {
            // the capital-U and ~ here are actually important for
            // testing a MissionPlanner bug!
            AP_BoardConfig::config_error("YOU~RE WELCOME!");
        }
        if (is_equal(packet.param4, 102.0f)) {
            // attempt to write to address 0x5 (in the bottom 1kB on H7)
            // which we either memory-protect or check for
            // non-zeroness.  We don't want to use 0x0 as that *even
            // more magic*.  So choose an offset which looks like
            // we're dereferencing nullptr:
            uint8_t *foo = (uint8_t*)0x05;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if !defined(__clang__)  // avoid -Wunknown-warning-option
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
            *foo = 0xab;
#pragma GCC diagnostic pop

            return MAV_RESULT_ACCEPTED;
        }
        if (is_equal(packet.param4, 103.0f)) {
            // attempt to read from address 0x5 (in the bottom 1kB on
            // H7) which we either memory-protect or check for
            // non-zeroness.  We don't want to use 0x0 as that *even
            // more magic*.  So choose an offset which looks like
            // we're dereferencing nullptr:
            uint8_t *foo = (uint8_t*)0x05;

            // we use send_text here to ensure we don't get elided.
            // String is kept short for space reasons.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if !defined(__clang__)  // avoid -Wunknown-warning-option
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
            send_text(MAV_SEVERITY_INFO, "x: %u", (unsigned)*foo);
#pragma GCSS diagnostic pop

            return MAV_RESULT_ACCEPTED;
        }
#endif  // AP_MAVLINK_FAILURE_CREATION_ENABLED

#if HAL_ENABLE_DFU_BOOT
        if (is_equal(packet.param4, 99.0f)) {
#if AP_SIGNED_FIRMWARE
            send_text(MAV_SEVERITY_ERROR, "Refusing DFU for secure firmware");
            return MAV_RESULT_FAILED;
#else
            send_text(MAV_SEVERITY_WARNING, "Entering DFU mode");
            hal.util->boot_to_dfu();
            return MAV_RESULT_ACCEPTED;
#endif
        }
#endif
    }

    // refuse reboot when armed:
    if (hal.util->get_soft_armed()) {
        /// but allow it if forced:
        const uint32_t magic_force_reboot_value = 20190226;
        if (packet.y != magic_force_reboot_value) {
            return MAV_RESULT_FAILED;
        }
    }

    if (!(is_equal(packet.param1, 1.0f) || is_equal(packet.param1, 3.0f))) {
        // param1 must be 1 or 3 - 1 being reboot, 3 being reboot-to-bootloader
        return MAV_RESULT_UNSUPPORTED;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    {  // autotest relies in receiving the ACK for the reboot.  Ensure
       // there is space for it:
        const uint32_t tstart_ms = AP_HAL::millis();
        while (AP_HAL::millis() - tstart_ms < 1000) {
            if (HAVE_PAYLOAD_SPACE(chan, COMMAND_ACK)) {
                break;
            }
            hal.scheduler->delay(10);
        }
    }
#endif

    // send ack before we reboot
    mavlink_msg_command_ack_send(chan, packet.command, MAV_RESULT_ACCEPTED,
                                 0, 0,
                                 msg.sysid,
                                 msg.compid);

    // when packet.param1 == 3 we reboot to hold in bootloader
    const bool hold_in_bootloader = is_equal(packet.param1, 3.0f);

#if AP_VEHICLE_ENABLED
    AP::vehicle()->reboot(hold_in_bootloader);  // not expected to return
#else
    hal.scheduler->reboot(hold_in_bootloader);
#endif

    return MAV_RESULT_FAILED;
}

#if AP_MAVLINK_FAILURE_CREATION_ENABLED
/*
  take a semaphore and do not release it, triggering a deadlock
 */
void GCS_MAVLINK::deadlock_sem(void)
{
    if (!_deadlock_sem.taken) {
        _deadlock_sem.taken = true;
        _deadlock_sem.sem.take_blocking();
    }
}
#endif

/*
  handle a flight termination request
 */
MAV_RESULT GCS_MAVLINK::handle_flight_termination(const mavlink_command_int_t &packet)
{
#if AP_ADVANCEDFAILSAFE_ENABLED
    AP_AdvancedFailsafe *failsafe = AP::advancedfailsafe();
    if (failsafe == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    bool should_terminate = packet.param1 > 0.5f;

    if (failsafe->gcs_terminate(should_terminate, "GCS request")) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
#else
    return MAV_RESULT_UNSUPPORTED;
#endif
}

#if AP_RCPROTOCOL_ENABLED
/*
  handle a R/C bind request (for spektrum)
 */
MAV_RESULT GCS_MAVLINK::handle_START_RX_PAIR(const mavlink_command_int_t &packet)
{
    // initiate bind procedure. We houls accept the DSM type from either
    // param1 or param2 due to a past mixup with what parameter is the
    // right one: const int dsmMode = (packet.param2 > 0) ? packet.param2 : packet.param1;
    AP::RC().start_bind();

    return MAV_RESULT_ACCEPTED;
}
#endif  // AP_RCPROTOCOL_ENABLED

uint64_t GCS_MAVLINK::timesync_receive_timestamp_ns() const
{
    uint64_t ret = _port->receive_time_constraint_us(PAYLOAD_SIZE(chan, TIMESYNC));
    if (ret == 0) {
        ret = AP_HAL::micros64();
    }
    return ret*1000LL;
}

uint64_t GCS_MAVLINK::timesync_timestamp_ns() const
{
    // we add in our own system id try to ensure we only consider
    // responses to our own timesync request messages
    return AP_HAL::micros64()*1000LL + mavlink_system.sysid;
}

/*
  return a timesync request
  Sends back ts1 as received, and tc1 is the local timestamp in usec
 */
void GCS_MAVLINK::handle_timesync(const mavlink_message_t &msg)
{
    // decode incoming timesync message
    mavlink_timesync_t tsync;
    mavlink_msg_timesync_decode(&msg, &tsync);

    if (tsync.tc1 != 0) {
        // this is a response to a timesync request
        if (tsync.ts1 != _timesync_request.sent_ts1) {
            // we didn't actually send the request.... or it's a
            // response to an ancient request...
            return;
        }
#if 0
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                        "timesync response sysid=%u (latency=%fms)",
                        msg.sysid,
                        round_trip_time_us*0.001f);
#endif

#if HAL_LOGGING_ENABLED
        const uint64_t round_trip_time_us = (timesync_receive_timestamp_ns() - _timesync_request.sent_ts1)*0.001f;
        AP_Logger *logger = AP_Logger::get_singleton();
        if (logger != nullptr) {
            AP::logger().Write(
                "TSYN",
                "TimeUS,SysID,RTT",
                "s-s",
                "F-F",
                "QBQ",
                AP_HAL::micros64(),
                msg.sysid,
                round_trip_time_us
                );
        }
#endif  // HAL_LOGGING_ENABLED
        return;
    }

    if (!HAVE_PAYLOAD_SPACE(chan, TIMESYNC)) {
        // drop this timesync request entirely
        return;
    }

    // create new timesync struct with tc1 field as system time in
    // nanoseconds.  The client timestamp is as close as possible to
    // the time we received the TIMESYNC message.
    mavlink_timesync_t rsync;
    rsync.tc1 = timesync_receive_timestamp_ns();
    rsync.ts1 = tsync.ts1;

    // respond with a timesync message
    mavlink_msg_timesync_send(
        chan,
        rsync.tc1,
        rsync.ts1
        );
}

/*
 * broadcast a timesync message.  We may get multiple responses to this request.
 */
void GCS_MAVLINK::send_timesync()
{
    _timesync_request.sent_ts1 = timesync_timestamp_ns();
    mavlink_msg_timesync_send(
        chan,
        0,
        _timesync_request.sent_ts1
        );
}

void GCS_MAVLINK::handle_statustext(const mavlink_message_t &msg)
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }

    mavlink_statustext_t packet;
    mavlink_msg_statustext_decode(&msg, &packet);

    const uint8_t max_prefix_len = 14;
    const uint8_t text_len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1+max_prefix_len;
    if (msg.sysid != statustext_chunking.last_src_system ||
        msg.compid != statustext_chunking.last_src_system ||
        packet.id != statustext_chunking.last_id) {
        statustext_chunking.last_src_system = msg.sysid;
        statustext_chunking.last_src_component = msg.compid;
        statustext_chunking.last_id = packet.id;
        statustext_chunking.msg_id = AP::logger().get_MSG_id();
    }

    uint8_t offset = 0;
    char text[text_len] = {};
    if (packet.chunk_seq == 0) {
        // prefix with origin information
        if (gcs().sysid_is_gcs(msg.sysid)) {
            strncpy(text, "GCS:", ARRAY_SIZE(text));
            offset = strlen(text);
        } else {
            offset = hal.util->snprintf(text,
                                        max_prefix_len,
                                        "SRC=%u/%u:",
                                        msg.sysid,
                                        msg.compid);
            offset = MIN(offset, max_prefix_len);
        }
    }

    memcpy(&text[offset], packet.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

    logger->Write_MessageChunk(statustext_chunking.msg_id, text, packet.chunk_seq);
#endif
}


/*
  handle logging of named values from mavlink.
 */
void GCS_MAVLINK::handle_named_value(const mavlink_message_t &msg) const
{
#if HAL_LOGGING_ENABLED
    auto *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    mavlink_named_value_float_t p;
    mavlink_msg_named_value_float_decode(&msg, &p);
    char s[11] {};
    strncpy(s, p.name, sizeof(s)-1);
    logger->Write("NVAL", "TimeUS,TimeBootMS,Name,Value,SSys,SCom", "ss#---", "FC----", "QINfBB",
                  AP_HAL::micros64(),
                  p.time_boot_ms,
                  s,
                  p.value,
                  msg.sysid,
                  msg.compid);
#endif
}

#if AP_RTC_ENABLED
void GCS_MAVLINK::handle_system_time_message(const mavlink_message_t &msg)
{
    mavlink_system_time_t packet;
    mavlink_msg_system_time_decode(&msg, &packet);

    AP::rtc().set_utc_usec(packet.time_unix_usec, AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME);
}
#endif

#if AP_CAMERA_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_camera(const mavlink_command_int_t &packet)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    return camera->handle_command(packet);
}
#endif


#if AP_AHRS_ENABLED
// sets ekf_origin if it has not been set.
//  should only be used when there is no GPS to provide an absolute position
MAV_RESULT GCS_MAVLINK::set_ekf_origin(const Location& loc)
{
    // check location is valid
    if (loc.is_zero()) {
        return MAV_RESULT_DENIED;
    }
    if (!loc.check_latlng()) {
        return MAV_RESULT_DENIED;
    }

    AP_AHRS &ahrs = AP::ahrs();

    // check if EKF origin has already been set
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        return MAV_RESULT_FAILED;
    }

    if (!ahrs.set_origin(loc)) {
        return MAV_RESULT_FAILED;
    }

    // send ekf origin to GCS
    if (!try_send_message(MSG_ORIGIN)) {
        // try again later
        send_message(MSG_ORIGIN);
    }
    return MAV_RESULT_ACCEPTED;
}

#if AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
void GCS_MAVLINK::handle_set_gps_global_origin(const mavlink_message_t &msg)
{
    mavlink_set_gps_global_origin_t packet;
    mavlink_msg_set_gps_global_origin_decode(&msg, &packet);

    // sanity check location
    if (!check_latlng(packet.latitude, packet.longitude)) {
        // silently drop the request
        return;
    }
    // sanity check altitude in mm, note that abs can't handle INT32_MIN correctly
    if (packet.altitude < -LOCATION_ALT_MAX_M * 1000 || packet.altitude > LOCATION_ALT_MAX_M * 1000) {
        // silently drop the request
        return;
    }

    const Location ekf_origin {
        packet.latitude,
        packet.longitude,
        int32_t(packet.altitude * 0.1f),  // mm -> cm
        Location::AltFrame::ABSOLUTE
    };
    set_ekf_origin(ekf_origin);
}
#endif  // AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED

#endif  // AP_AHRS_ENABLED

/*
  handle a DATA96 message
 */
void GCS_MAVLINK::handle_data_packet(const mavlink_message_t &msg)
{
#if AP_RADIO_ENABLED
    mavlink_data96_t m;
    mavlink_msg_data96_decode(&msg, &m);
    switch (m.type) {
    case 42:
    case 43: {
        // pass to AP_Radio (for firmware upload and playing test tunes)
        AP_Radio *radio = AP_Radio::get_singleton();
        if (radio != nullptr) {
            radio->handle_data_packet(chan, m);
        }
        break;
    }
    default:
        // unknown
        break;
    }
#endif
}

#if HAL_VISUALODOM_ENABLED
void GCS_MAVLINK::handle_vision_position_delta(const mavlink_message_t &msg)
{
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    visual_odom->handle_vision_position_delta_msg(msg);
}

void GCS_MAVLINK::handle_vision_position_estimate(const mavlink_message_t &msg)
{
    mavlink_vision_position_estimate_t m;
    mavlink_msg_vision_position_estimate_decode(&msg, &m);

    handle_common_vision_position_estimate_data(m.usec, m.x, m.y, m.z, m.roll, m.pitch, m.yaw, m.covariance, m.reset_counter,
                                                PAYLOAD_SIZE(chan, VISION_POSITION_ESTIMATE));
}

void GCS_MAVLINK::handle_global_vision_position_estimate(const mavlink_message_t &msg)
{
    mavlink_global_vision_position_estimate_t m;
    mavlink_msg_global_vision_position_estimate_decode(&msg, &m);

    handle_common_vision_position_estimate_data(m.usec, m.x, m.y, m.z, m.roll, m.pitch, m.yaw, m.covariance, m.reset_counter,
                                                PAYLOAD_SIZE(chan, GLOBAL_VISION_POSITION_ESTIMATE));
}

void GCS_MAVLINK::handle_vicon_position_estimate(const mavlink_message_t &msg)
{
    mavlink_vicon_position_estimate_t m;
    mavlink_msg_vicon_position_estimate_decode(&msg, &m);

    // vicon position estimate does not include reset counter
    handle_common_vision_position_estimate_data(m.usec, m.x, m.y, m.z, m.roll, m.pitch, m.yaw, m.covariance, 0,
                                                PAYLOAD_SIZE(chan, VICON_POSITION_ESTIMATE));
}

/*
  handle ODOMETRY message. This message combines position, velocity
  and attitude data
 */
void GCS_MAVLINK::handle_odometry(const mavlink_message_t &msg)
{
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    mavlink_odometry_t m;
    mavlink_msg_odometry_decode(&msg, &m);

    if (m.frame_id != MAV_FRAME_LOCAL_FRD ||
        m.child_frame_id != MAV_FRAME_BODY_FRD) {
        // only support local FRD frame data
        return;
    }

    Quaternion q{m.q[0],m.q[1],m.q[2],m.q[3]};

    float posErr = 0;
    float angErr = 0;
    if (!isnan(m.pose_covariance[0])) {
        posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));
        angErr = cbrtf(sq(m.pose_covariance[15])+sq(m.pose_covariance[18])+sq(m.pose_covariance[20]));
    }

    const uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, PAYLOAD_SIZE(chan, ODOMETRY));
    visual_odom->handle_pose_estimate(m.time_usec, timestamp_ms, m.x, m.y, m.z, q, posErr, angErr, m.reset_counter, m.quality);

    // convert velocity vector from FRD to NED frame
    Vector3f vel{m.vx, m.vy, m.vz};
    vel = q * vel;
    visual_odom->handle_vision_speed_estimate(m.time_usec, timestamp_ms, vel, m.reset_counter, m.quality);
}

// there are several messages which all have identical fields in them.
// This function provides common handling for the data contained in
// these packets
void GCS_MAVLINK::handle_common_vision_position_estimate_data(const uint64_t usec,
                                                              const float x,
                                                              const float y,
                                                              const float z,
                                                              const float roll,
                                                              const float pitch,
                                                              const float yaw,
                                                              const float covariance[21],
                                                              const uint8_t reset_counter,
                                                              const uint16_t payload_size)
{
    float posErr = 0;
    float angErr = 0;
    // correct offboard timestamp to be in local ms since boot
    uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(usec, payload_size);

    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    if (!isnan(covariance[0])) {
        posErr = cbrtf(sq(covariance[0])+sq(covariance[6])+sq(covariance[11]));
        angErr = cbrtf(sq(covariance[15])+sq(covariance[18])+sq(covariance[20]));
    }

    visual_odom->handle_pose_estimate(usec, timestamp_ms, x, y, z, roll, pitch, yaw, posErr, angErr, reset_counter, 0);
}

void GCS_MAVLINK::handle_att_pos_mocap(const mavlink_message_t &msg)
{
    mavlink_att_pos_mocap_t m;
    mavlink_msg_att_pos_mocap_decode(&msg, &m);

    // correct offboard timestamp to be in local ms since boot
    uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, PAYLOAD_SIZE(chan, ATT_POS_MOCAP));
   
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    // note: att_pos_mocap does not include reset counter
    visual_odom->handle_pose_estimate(m.time_usec, timestamp_ms, m.x, m.y, m.z, m.q, 0, 0, 0, 0);
}

void GCS_MAVLINK::handle_vision_speed_estimate(const mavlink_message_t &msg)
{
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    mavlink_vision_speed_estimate_t m;
    mavlink_msg_vision_speed_estimate_decode(&msg, &m);
    const Vector3f vel = {m.x, m.y, m.z};
    uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.usec, PAYLOAD_SIZE(chan, VISION_SPEED_ESTIMATE));
    visual_odom->handle_vision_speed_estimate(m.usec, timestamp_ms, vel, m.reset_counter, 0);
}
#endif  // HAL_VISUALODOM_ENABLED

void GCS_MAVLINK::handle_command_ack(const mavlink_message_t &msg)
{
#if HAL_INS_ACCELCAL_ENABLED
    mavlink_command_ack_t packet;
    mavlink_msg_command_ack_decode(&msg, &packet);

    AP_AccelCal *accelcal = AP::ins().get_acal();
    if (accelcal != nullptr) {
        accelcal->handle_command_ack(packet, msg.sysid, msg.compid);
    }
#if AP_GENERATOR_LOWEHEISER_ENABLED
    // this might be an ACK from a loweheiser generator:
    handle_generator_message(msg);
#endif

#endif
}

#if AP_RC_CHANNEL_ENABLED
// allow override of RC channel values for complete GCS
// control of switch position and RC PWM values.
void GCS_MAVLINK::handle_rc_channels_override(const mavlink_message_t &msg)
{
    if (!gcs().sysid_is_gcs(msg.sysid)) {
        return; // Only accept control from our gcs
    }

    const uint32_t tnow = AP_HAL::millis();

    mavlink_rc_channels_override_t packet;
    mavlink_msg_rc_channels_override_decode(&msg, &packet);

    const uint16_t override_data[] = {
        packet.chan1_raw,
        packet.chan2_raw,
        packet.chan3_raw,
        packet.chan4_raw,
        packet.chan5_raw,
        packet.chan6_raw,
        packet.chan7_raw,
        packet.chan8_raw,
        packet.chan9_raw,
        packet.chan10_raw,
        packet.chan11_raw,
        packet.chan12_raw,
        packet.chan13_raw,
        packet.chan14_raw,
        packet.chan15_raw,
        packet.chan16_raw
    };

    for (uint8_t i=0; i<8; i++) {
        // Per MAVLink spec a value of UINT16_MAX means to ignore this field.
        if (override_data[i] != UINT16_MAX) {
            RC_Channels::set_override(i, override_data[i], tnow);
        }
    }
    for (uint8_t i=8; i<ARRAY_SIZE(override_data); i++) {
        // Per MAVLink spec a value of zero or UINT16_MAX means to
        // ignore this field.
        if (override_data[i] != 0 && override_data[i] != UINT16_MAX) {
            // per the mavlink spec, a value of UINT16_MAX-1 means
            // return the field to RC radio values:
            const uint16_t value = override_data[i] == (UINT16_MAX-1) ? 0 : override_data[i];
            RC_Channels::set_override(i, value, tnow);
        }
    }

    sysid_mygcs_seen(tnow);

}
#endif  // AP_RC_CHANNEL_ENABLED

#if AP_OPTICALFLOW_ENABLED
void GCS_MAVLINK::handle_optical_flow(const mavlink_message_t &msg)
{
    AP_OpticalFlow *optflow = AP::opticalflow();
    if (optflow == nullptr) {
        return;
    }
    optflow->handle_msg(msg);
}
#endif


#if AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
/*
  handle MAV_CMD_FIXED_MAG_CAL_YAW
 */
MAV_RESULT GCS_MAVLINK::handle_command_fixed_mag_cal_yaw(const mavlink_command_int_t &packet)
{
    Compass &compass = AP::compass();
    if (!compass.mag_cal_fixed_yaw(packet.param1,
                                   uint8_t(packet.param2),
                                   packet.param3,
                                   packet.param4)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif  // AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED

#if COMPASS_CAL_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_mag_cal(const mavlink_command_int_t &packet)
{
    return AP::compass().handle_mag_cal_command(packet);
}
#endif  // COMPASS_CAL_ENABLED

#if AP_MAVLINKCAN_ENABLED
/*
  handle MAV_CMD_CAN_FORWARD
 */
MAV_RESULT GCS_MAVLINK::handle_can_forward(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    return AP_MAVLinkCAN::handle_can_forward(chan, packet, msg) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
}

/*
  handle CAN_FRAME messages
 */
void GCS_MAVLINK::handle_can_frame(const mavlink_message_t &msg) const
{
    AP_MAVLinkCAN::handle_can_frame(msg);
}
#endif  // AP_MAVLINKCAN_ENABLED

void GCS_MAVLINK::handle_distance_sensor(const mavlink_message_t &msg)
{
#if AP_RANGEFINDER_ENABLED
    RangeFinder *rangefinder = AP::rangefinder();
    if (rangefinder != nullptr) {
        rangefinder->handle_msg(msg);
    }
#endif

#if HAL_PROXIMITY_ENABLED
    AP_Proximity *proximity = AP::proximity();
    if (proximity != nullptr) {
        proximity->handle_msg(msg);
    }
#endif
}

#if HAL_PROXIMITY_ENABLED
void GCS_MAVLINK::handle_obstacle_distance(const mavlink_message_t &msg)
{
    AP_Proximity *proximity = AP::proximity();
    if (proximity != nullptr) {
        proximity->handle_msg(msg);
    }
}

void GCS_MAVLINK::handle_obstacle_distance_3d(const mavlink_message_t &msg)
{
    AP_Proximity *proximity = AP::proximity();
    if (proximity != nullptr) {
        proximity->handle_msg(msg);
    }
}
#endif

#if HAL_ADSB_ENABLED
void GCS_MAVLINK::handle_adsb_message(const mavlink_message_t &msg)
{
    AP_ADSB *adsb = AP::ADSB();
    if (adsb != nullptr) {
        adsb->handle_message(chan, msg);
    }
}
#endif

#if OSD_PARAM_ENABLED
void GCS_MAVLINK::handle_osd_param_config(const mavlink_message_t &msg) const
{
    AP_OSD *osd = AP::osd();
    if (osd != nullptr) {
        osd->handle_msg(msg, *this);
    }
}
#endif

void GCS_MAVLINK::handle_heartbeat(const mavlink_message_t &msg)
{
    // if the heartbeat is from our GCS then we don't failsafe for
    // now...
    if (gcs().sysid_is_gcs(msg.sysid)) {
        sysid_mygcs_seen(AP_HAL::millis());
    }
}

/*
  handle messages which don't require vehicle specific data
 */
void GCS_MAVLINK::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        handle_heartbeat(msg);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK: {
        handle_command_ack(msg);
        break;
    }

    case MAVLINK_MSG_ID_SETUP_SIGNING:
        handle_setup_signing(msg);
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_common_param_message(msg);
        break;

#if AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        handle_set_gps_global_origin(msg);
        break;
#endif

#if AP_MAVLINK_MSG_DEVICE_OP_ENABLED
    case MAVLINK_MSG_ID_DEVICE_OP_READ:
        handle_device_op_read(msg);
        break;
    case MAVLINK_MSG_ID_DEVICE_OP_WRITE:
        handle_device_op_write(msg);
        break;
#endif

    case MAVLINK_MSG_ID_TIMESYNC:
        handle_timesync(msg);
        break;
#if HAL_LOGGING_ENABLED
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        AP::logger().handle_mavlink_msg(*this, msg);
        break;
#endif

#if AP_MAVLINK_FTP_ENABLED
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        GCS_FTP::handle_file_transfer_protocol(msg, chan);
        break;
#endif

#if AP_CAMERA_ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    case MAVLINK_MSG_ID_GOPRO_HEARTBEAT: // heartbeat from a GoPro in Solo gimbal
    case MAVLINK_MSG_ID_CAMERA_INFORMATION:
        {
            AP_Camera *camera = AP::camera();
            if (camera == nullptr) {
                return;
            }
            camera->handle_message(chan, msg);
        }
        break;
#endif

    case MAVLINK_MSG_ID_SET_MODE:
        handle_set_mode(msg);
        break;

#if AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        handle_send_autopilot_version(msg);
        break;
#endif

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    case MAVLINK_MSG_ID_MISSION_COUNT:
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_ACK:
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        handle_common_mission_message(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_INT:
        handle_command_int(msg);
        break;

#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
    case MAVLINK_MSG_ID_FENCE_POINT:
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        send_received_message_deprecation_warning("FENCE_FETCH_POINT");
        handle_fence_message(msg);
        break;
#endif

#if HAL_MOUNT_ENABLED
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW:
        handle_mount_message(msg);
        break;
#endif

    case MAVLINK_MSG_ID_PARAM_VALUE:
        handle_param_value(msg);
        break;

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
        handle_radio_status(msg);
        break;

#if AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg);
        break;
#endif

#if AP_GPS_ENABLED
    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
    case MAVLINK_MSG_ID_GPS_INPUT:
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        AP::gps().handle_msg(chan, msg);
        break;
#endif

    case MAVLINK_MSG_ID_STATUSTEXT:
        handle_statustext(msg);
        break;

#if AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED
    case MAVLINK_MSG_ID_LED_CONTROL:
        // send message to Notify
        AP_Notify::handle_led_control(msg);
        break;
#endif

#if AP_RC_CHANNEL_ENABLED
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        handle_manual_control(msg);
        break;
#endif

#if AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED
    case MAVLINK_MSG_ID_PLAY_TUNE:
        // send message to Notify
        AP_Notify::handle_play_tune(msg);
        break;
#endif

#if AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED
    case MAVLINK_MSG_ID_RALLY_POINT:
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        send_received_message_deprecation_warning("RALLY_FETCH_POINT");
        handle_common_rally_message(msg);
        break;
#endif

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        if (option_enabled(Option::NOSTREAMOVERRIDE)) {
            // options indicate we are to ignore this stream rate
            // request
            break;
        }
        handle_request_data_stream(msg);
        break;

    case MAVLINK_MSG_ID_DATA96:
        handle_data_packet(msg);
        break;        

#if HAL_VISUALODOM_ENABLED
    case MAVLINK_MSG_ID_VISION_POSITION_DELTA:
        handle_vision_position_delta(msg);
        break;

    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
        handle_vision_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
        handle_global_vision_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
        handle_vicon_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_ODOMETRY:
        handle_odometry(msg);
        break;

    case MAVLINK_MSG_ID_ATT_POS_MOCAP:
        handle_att_pos_mocap(msg);
        break;

    case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
        handle_vision_speed_estimate(msg);
        break;
#endif  // HAL_VISUALODOM_ENABLED

#if AP_RTC_ENABLED
    case MAVLINK_MSG_ID_SYSTEM_TIME:
        handle_system_time_message(msg);
        break;
#endif

#if AP_RC_CHANNEL_ENABLED
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        handle_rc_channels_override(msg);
        break;
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
    case MAVLINK_MSG_ID_RADIO_RC_CHANNELS:
        handle_radio_rc_channels(msg);
        break;
#endif
#endif

#if AP_OPTICALFLOW_ENABLED
    case MAVLINK_MSG_ID_OPTICAL_FLOW:
        handle_optical_flow(msg);
        break;
#endif

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        handle_distance_sensor(msg);
        break;

#if HAL_PROXIMITY_ENABLED
    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
        handle_obstacle_distance(msg);
        break;

    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D:
        handle_obstacle_distance_3d(msg);
        break;
#endif

#if OSD_PARAM_ENABLED
    case MAVLINK_MSG_ID_OSD_PARAM_CONFIG:
    case MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG:
        handle_osd_param_config(msg);
        break;
#endif

#if HAL_ADSB_ENABLED
    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL:
        handle_adsb_message(msg);
        break;
#endif

    case MAVLINK_MSG_ID_LANDING_TARGET:
        handle_landing_target(msg);
        break;

    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        handle_named_value(msg);
        break;

#if HAL_CANMANAGER_ENABLED
    case MAVLINK_MSG_ID_CAN_FRAME:
    case MAVLINK_MSG_ID_CANFD_FRAME:
        handle_can_frame(msg);
        break;
#endif

#if AP_MAVLINKCAN_ENABLED
    case MAVLINK_MSG_ID_CAN_FILTER_MODIFY:
        AP_MAVLinkCAN::handle_can_filter_modify(msg);
        break;
#endif  // AP_MAVLINKCAN_ENABLED

#if AP_OPENDRONEID_ENABLED
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE:
        AP::opendroneid().handle_msg(chan, msg);
        break;
#endif

#if AP_SIGNED_FIRMWARE
    case MAVLINK_MSG_ID_SECURE_COMMAND:
    case MAVLINK_MSG_ID_SECURE_COMMAND_REPLY:
        AP_CheckFirmware::handle_msg(chan, msg);
        break;
#endif

#if AP_EFI_MAV_ENABLED
    case MAVLINK_MSG_ID_EFI_STATUS:
    {
        AP_EFI *efi = AP::EFI();
        if (efi) {
            efi->handle_EFI_message(msg);
        }
        break;
    }
#endif
#if AP_GENERATOR_LOWEHEISER_ENABLED
    case MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI:
        // message received from Loweheiser mavlink connection
        handle_generator_message(msg);
        break;
#endif
    }

}

void GCS_MAVLINK::handle_generator_message(const mavlink_message_t &msg)
{
#if AP_GENERATOR_LOWEHEISER_ENABLED
    AP_Generator *generator = AP::generator();
    if (generator == nullptr) {
        return;
    }

    auto *backend = generator->get_loweheiser();
    if (backend == nullptr) {
        return;
    }

    backend->handle_mavlink_msg(*this, msg);
#endif
}

void GCS_MAVLINK::handle_common_mission_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: // MAV ID: 38
    {
        handle_mission_write_partial_list(msg);
        break;
    }

    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:           // MAV ID: 39
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        handle_mission_item(msg);
        break;

    // read an individual command from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        handle_mission_request_int(msg);
        break;

#if AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        handle_mission_request(msg);
        break;
#endif

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:    // MAV ID: 41
    {
        AP_Mission *_mission = AP::mission();
        if (_mission != nullptr) {
            handle_mission_set_current(*_mission, msg);
        }
        break;
    }
#endif

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:       // MAV ID: 43
    {
        handle_mission_request_list(msg);
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:          // MAV ID: 44
    {
        handle_mission_count(msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // MAV ID: 45
    {
        handle_mission_clear_all(msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_ACK:
        /* not used */
        break;
    }
}

#if AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
void GCS_MAVLINK::handle_send_autopilot_version(const mavlink_message_t &msg)
{
    send_message(MSG_AUTOPILOT_VERSION);
}
#endif

void GCS_MAVLINK::send_banner()
{
    // mark the firmware version in the tlog
    const AP_FWVersion &fwver = AP::fwversion();

    send_text(MAV_SEVERITY_INFO, "%s", fwver.fw_string);

    if (fwver.middleware_name && fwver.os_name) {
        send_text(MAV_SEVERITY_INFO, "%s: %s %s: %s",
                  fwver.middleware_name, fwver.middleware_hash_str,
                  fwver.os_name, fwver.os_hash_str);
    } else if (fwver.os_name) {
        send_text(MAV_SEVERITY_INFO, "%s: %s",
                  fwver.os_name, fwver.os_hash_str);
    }

    // send system ID if we can
    char sysid[50];
    if (hal.util->get_system_id(sysid)) {
        send_text(MAV_SEVERITY_INFO, "%s", sysid);
    }

    // send MCUID if we can
#if HAL_WITH_IO_MCU
#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF
    if (AP_BoardConfig::io_enabled()) {
        uint32_t mcuid = iomcu.get_mcu_id();
        send_text(MAV_SEVERITY_INFO, "IOMCU: %x %x %lx", uint16_t(mcuid & DEVID_MASK), uint16_t((mcuid & REVID_MASK) >> 16U),
            iomcu.get_cpu_id());
    }
#endif

    // send RC output mode info if available
    char banner_msg[50];
    if (hal.rcout->get_output_mode_banner(banner_msg, sizeof(banner_msg))) {
        send_text(MAV_SEVERITY_INFO, "%s", banner_msg);
    }

#if AP_INERTIALSENSOR_ENABLED
    // output any fast sampling status messages
    for (uint8_t i = 0; i < INS_MAX_BACKENDS; i++) {
        if (AP::ins().get_output_banner(i, banner_msg, sizeof(banner_msg))) {
            send_text(MAV_SEVERITY_INFO, "%s", banner_msg);
        }
    }
#endif
}


#if AP_SIM_ENABLED
void GCS_MAVLINK::send_simstate() const
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    sitl->simstate_send(get_chan());
}

void GCS_MAVLINK::send_sim_state() const
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    sitl->sim_state_send(get_chan());
}
#endif

#if AP_AHRS_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_do_set_global_origin(const mavlink_command_int_t &packet)
{
    Location global_origin;
    if (!location_from_command_t(packet, global_origin)) {
        return MAV_RESULT_DENIED;
    }

    return set_ekf_origin(global_origin);
}
#endif  // AP_AHRS_ENABLED

#if AP_BOOTLOADER_FLASHING_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_flash_bootloader(const mavlink_command_int_t &packet)
{
    if (packet.x != 290876) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Magic not set");
        return MAV_RESULT_FAILED;
    }

    switch (hal.util->flash_bootloader()) {
    case AP_HAL::Util::FlashBootloader::OK:
    case AP_HAL::Util::FlashBootloader::NO_CHANGE:
        // consider NO_CHANGE as success (so as not to display error to user)
        return MAV_RESULT_ACCEPTED;
#if AP_SIGNED_FIRMWARE
    case AP_HAL::Util::FlashBootloader::NOT_SIGNED:
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Bootloader not signed");
        break;
#endif
    default:
        break;
    }

    return MAV_RESULT_FAILED;
}
#endif  // AP_BOOTLOADER_FLASHING_ENABLED

MAV_RESULT GCS_MAVLINK::_handle_command_preflight_calibration_baro(const mavlink_message_t &msg)
{
    // fast barometer calibration
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Updating barometer calibration");
    AP::baro().update_calibration();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Barometer calibration complete");

#if AP_AIRSPEED_ENABLED

    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->enabled()) {
        GCS_MAVLINK_InProgress *task = GCS_MAVLINK_InProgress::get_task(MAV_CMD_PREFLIGHT_CALIBRATION, GCS_MAVLINK_InProgress::Type::AIRSPEED_CAL, msg.sysid, msg.compid, chan);
        if (task == nullptr) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        airspeed->calibrate(false);
        return MAV_RESULT_IN_PROGRESS;
    }
#endif

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    MAV_RESULT ret = MAV_RESULT_UNSUPPORTED;

    EXPECT_DELAY_MS(30000);
    if (is_equal(packet.param1,1.0f)) {
#if AP_INERTIALSENSOR_ENABLED
        if (!AP::ins().calibrate_gyros()) {
            return MAV_RESULT_FAILED;
        }
#else
        return MAV_RESULT_UNSUPPORTED;
#endif
        return MAV_RESULT_ACCEPTED;
    }

    if (is_equal(packet.param3,1.0f)) {
        return _handle_command_preflight_calibration_baro(msg);
    }

#if AP_RC_CHANNEL_ENABLED
    rc().calibrating(is_positive(packet.param4));
#endif

#if HAL_INS_ACCELCAL_ENABLED
    if (packet.x == 1) {
        // start with gyro calibration
        if (!AP::ins().calibrate_gyros()) {
            return MAV_RESULT_FAILED;
        }
        // start accel cal
        AP::ins().acal_init();
        AP::ins().get_acal()->start(this, msg.sysid, msg.compid);
        return MAV_RESULT_ACCEPTED;
    }
#endif

#if AP_INERTIALSENSOR_ENABLED
#if AP_AHRS_ENABLED
    if (packet.x == 2) {
        return AP::ins().calibrate_trim();
    }
#endif

    if (packet.x == 4) {
        // simple accel calibration
        return AP::ins().simple_accel_cal();
    }

    /*
      allow GCS to force an existing calibration of accel and/or
      compass to be written as valid. This is useful when reloading
      parameters after a full parameter erase
     */
    if (packet.x == 76) {
        // force existing accel calibration to be accepted as valid
        AP::ins().force_save_calibration();
        ret = MAV_RESULT_ACCEPTED;
    }
#endif  // AP_INERTIALSENSOR_ENABLED

#if AP_COMPASS_ENABLED
    if (is_equal(packet.param2,76.0f)) {
        // force existing compass calibration to be accepted as valid
        AP::compass().force_save_calibration();
        ret = MAV_RESULT_ACCEPTED;
    }
#endif

    return ret;
}

MAV_RESULT GCS_MAVLINK::handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (hal.util->get_soft_armed()) {
        // *preflight*, remember?
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Disarm to allow calibration");
        return MAV_RESULT_FAILED;
    }
    // now call subclass methods:
    return _handle_command_preflight_calibration(packet, msg);
}

#if AP_ARMING_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_run_prearm_checks(const mavlink_command_int_t &packet)
{
    if (hal.util->get_soft_armed()) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }
    (void)AP::arming().pre_arm_checks(true);
    return MAV_RESULT_ACCEPTED;
}
#endif  // AP_ARMING_ENABLED

#if AP_MISSION_ENABLED
// changes the current waypoint; at time of writing GCS
// implementations use the mavlink message MISSION_SET_CURRENT to set
// the current waypoint, rather than this DO command.  It is hoped we
// can move to this command in the future to avoid acknowledgement
// issues with MISSION_SET_CURRENT
MAV_RESULT GCS_MAVLINK::handle_command_do_set_mission_current(const mavlink_command_int_t &packet)
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    const uint32_t seq = (uint32_t)packet.param1;
    if (!mission->is_valid_index(seq)) {
        return MAV_RESULT_DENIED;
    }

    // From https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MISSION_CURRENT:
    //   Param 2: Reset Mission
    //     - Resets mission. 1: true, 0: false. Resets jump counters to initial values
    //       and changes mission state "completed" to be "active" or "paused".
    const bool reset_and_restart = is_equal(packet.param2, 1.0f);
    if (reset_and_restart) {
        mission->reset();
    }
    if (!mission->set_current_cmd(seq)) {
        return MAV_RESULT_FAILED;
    }
    if (reset_and_restart) {
        mission->resume();
    }

    // volunteer the new current waypoint for all listeners
    send_message(MSG_CURRENT_WAYPOINT);

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_do_jump_tag(const mavlink_command_int_t &packet)
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    const uint32_t tag = (uint32_t)packet.param1;
    if (tag > UINT16_MAX) {
        return MAV_RESULT_DENIED;
    }
    if (!mission->jump_to_tag(tag)) {
        return MAV_RESULT_FAILED;
    }

    // volunteer the new current waypoint for all listeners
    send_message(MSG_CURRENT_WAYPOINT);

    return MAV_RESULT_ACCEPTED;
}
#endif

#if AP_BATTERY_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_battery_reset(const mavlink_command_int_t &packet)
{
    const uint16_t battery_mask = packet.param1;
    const float percentage = packet.param2;
    if (AP::battery().reset_remaining_mask(battery_mask, percentage)) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}
#endif

#if AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_request_autopilot_capabilities(const mavlink_command_int_t &packet)
{
    if (!is_equal(packet.param1,1.0f)) {
        return MAV_RESULT_FAILED;
    }

    send_message(MSG_AUTOPILOT_VERSION);

    return MAV_RESULT_ACCEPTED;
}
#endif

MAV_RESULT GCS_MAVLINK::handle_command_do_set_mode(const mavlink_command_int_t &packet)
{
    const uint8_t _base_mode = (uint8_t)packet.param1;
    const uint32_t _custom_mode = (uint32_t)packet.param2;

    return _set_mode_common(_base_mode, _custom_mode);
}

#if AP_AHRS_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_get_home_position(const mavlink_command_int_t &packet)
{
    if (!AP::ahrs().home_is_set()) {
        return MAV_RESULT_FAILED;
    }
    if (!try_send_message(MSG_HOME)) {
        // try again later
        send_message(MSG_HOME);
    }
    if (!try_send_message(MSG_ORIGIN)) {
        // try again later
        send_message(MSG_ORIGIN);
    }

    return MAV_RESULT_ACCEPTED;
}
#endif  // AP_AHRS_ENABLED

MAV_RESULT GCS_MAVLINK::handle_command_debug_trap(const mavlink_command_int_t &packet)
{
    // magic number must be supplied to trap; you must *really* mean it.
    if (uint32_t(packet.param1) != 32451) {
        return MAV_RESULT_DENIED;
    }
    if (hal.util->trap()) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_UNSUPPORTED;
}

#if AP_AHRS_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_set_ekf_source_set(const mavlink_command_int_t &packet)
{
    // source set must be between 1 and 3
    uint32_t source_set = uint32_t(packet.param1);
    if ((source_set >= 1) && (source_set <= 3)) {
        // mavlink command uses range 1 to 3 while ahrs interface accepts 0 to 2
        AP::ahrs().set_posvelyaw_source_set((AP_NavEKF_Source::SourceSetSelection)(source_set-1));
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_DENIED;
}
#endif

#if AP_GRIPPER_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_do_gripper(const mavlink_command_int_t &packet)
{
    AP_Gripper &gripper = AP::gripper();

    // param1 : gripper number (ignored)
    // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
    if(!gripper.enabled()) {
        return MAV_RESULT_FAILED;
    }

    MAV_RESULT result = MAV_RESULT_ACCEPTED;

    switch ((uint8_t)packet.param2) {
    case GRIPPER_ACTION_RELEASE:
        gripper.release();
        break;
    case GRIPPER_ACTION_GRAB:
        gripper.grab();
        break;
    default:
        result = MAV_RESULT_FAILED;
        break;
    }

    return result;
}
#endif  // AP_GRIPPER_ENABLED

#if HAL_SPRAYER_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_do_sprayer(const mavlink_command_int_t &packet)
{
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return MAV_RESULT_FAILED;
    }

    if (is_equal(packet.param1, 1.0f)) {
        sprayer->run(true);
    } else if (is_zero(packet.param1)) {
        sprayer->run(false);
    }

    return MAV_RESULT_ACCEPTED;
}
#endif

#if AP_LANDINGGEAR_ENABLED
/*
  handle MAV_CMD_AIRFRAME_CONFIGURATION for landing gear control
 */
MAV_RESULT GCS_MAVLINK::handle_command_airframe_configuration(const mavlink_command_int_t &packet)
{
    // Param 1: Select which gear, not used in ArduPilot
    // Param 2: 0 = Deploy, 1 = Retract
    // For safety, anything other than 1 will deploy
    AP_LandingGear *lg = AP_LandingGear::get_singleton();
    if (lg == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }
    switch ((uint8_t)packet.param2) {
    case 1:
        lg->set_position(AP_LandingGear::LandingGear_Retract);
        return MAV_RESULT_ACCEPTED;
    default:
        lg->set_position(AP_LandingGear::LandingGear_Deploy);
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}
#endif

#if HAL_INS_ACCELCAL_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_accelcal_vehicle_pos(const mavlink_command_int_t &packet)
{
    if (AP::ins().get_acal() == nullptr ||
        !AP::ins().get_acal()->gcs_vehicle_position(packet.param1)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif  // HAL_INS_ACCELCAL_ENABLED

#if HAL_MOUNT_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }
    return mount->handle_command(packet, msg);
}
#endif  // HAL_MOUNT_ENABLED

#if AP_ARMING_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_component_arm_disarm(const mavlink_command_int_t &packet)
{
    if (is_equal(packet.param1,1.0f)) {
        if (AP::arming().is_armed()) {
            return MAV_RESULT_ACCEPTED;
        }
        // run pre_arm_checks and arm_checks and display failures
        const bool do_arming_checks = !is_equal(packet.param2,magic_force_arm_value) && !is_equal(packet.param2,magic_force_arm_disarm_value);
        if (AP::arming().arm(AP_Arming::Method::MAVLINK, do_arming_checks)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }
    if (is_zero(packet.param1))  {
        if (!AP::arming().is_armed()) {
            return MAV_RESULT_ACCEPTED;
        }
        const bool forced = is_equal(packet.param2, magic_force_arm_disarm_value);
        // note disarm()'s second parameter is "do_disarm_checks"
        if (AP::arming().disarm(AP_Arming::Method::MAVLINK, !forced)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

    return MAV_RESULT_UNSUPPORTED;
}
#endif // AP_ARMING_ENABLED

bool GCS_MAVLINK::location_from_command_t(const mavlink_command_int_t &in, Location &out)
{
    if (!command_long_stores_location((MAV_CMD)in.command)) {
        return false;
    }

    // integer storage imposes limits on the altitudes we can accept:
    if (isnan(in.z) || fabsf(in.z) > LOCATION_ALT_MAX_M) {
        return false;
    }

    Location::AltFrame frame;
    if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)in.frame, frame)) {
        // unknown coordinate frame
        return false;
    }

    out.lat = in.x;
    out.lng = in.y;

    out.set_alt_m(in.z, frame);

    return true;
}

bool GCS_MAVLINK::command_long_stores_location(const MAV_CMD command)
{
    switch(command) {
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_SET_ROI_LOCATION:
    // case MAV_CMD_NAV_TAKEOFF:  // technically yes, but we don't do lat/lng
    // case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_DO_REPOSITION:
    case MAV_CMD_EXTERNAL_POSITION_ESTIMATE:
    case MAV_CMD_DO_SET_GLOBAL_ORIGIN:
        return true;
    default:
        return false;
    }
    return false;
}

#if AP_MAVLINK_COMMAND_LONG_ENABLED
// when conveyed via COMMAND_LONG, a command doesn't come with an
// explicit frame.  When conveying a location they do have an assumed
// frame in ArduPilot, and this function returns that frame.
bool GCS_MAVLINK::mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const
{
    static const struct {
        uint32_t command;
        MAV_FRAME frame;
    } frame_map[] {
        { MAV_CMD_FIXED_MAG_CAL_YAW, MAV_FRAME_GLOBAL_RELATIVE_ALT },
        { MAV_CMD_DO_SET_ROI, MAV_FRAME_GLOBAL_RELATIVE_ALT },
        { MAV_CMD_DO_SET_ROI_LOCATION, MAV_FRAME_GLOBAL_RELATIVE_ALT },
        { MAV_CMD_DO_SET_HOME, MAV_FRAME_GLOBAL },
    };

    // map from command to frame:
    for (const auto &map : frame_map) {
        if (map.command != packet_command) {
            continue;
        }
        frame = map.frame;
        return true;
    }

    return false;
}

MAV_RESULT GCS_MAVLINK::try_command_long_as_command_int(const mavlink_command_long_t &packet, const mavlink_message_t &msg)
{
    MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    if (command_long_stores_location((MAV_CMD)packet.command)) {
        // we must be able to supply a frame for the location:
        if (!mav_frame_for_command_long(frame, (MAV_CMD)packet.command)) {
            return MAV_RESULT_UNSUPPORTED;
        }
    }

    // convert and run the command
    mavlink_command_int_t command_int;
    convert_COMMAND_LONG_to_COMMAND_INT(packet, command_int, frame);

    return handle_command_int_packet(command_int, msg);
}

// returns a value suitable for COMMAND_INT.x or y based on a value
// coming in from COMMAND_LONG.p5 or p6:
static int32_t convert_COMMAND_LONG_loc_param(float param, bool stores_location)
{
    if (isnan(param)) {
        return 0;
    }

    if (stores_location) {
        return param *1e7;
    }

    return param;
}

void GCS_MAVLINK::convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame)
{
    out = {};
    out.target_system = in.target_system;
    out.target_component = in.target_component;
    out.frame = frame;
    out.command = in.command;
    out.current = 0;
    out.autocontinue = 0;
    out.param1 = in.param1;
    out.param2 = in.param2;
    out.param3 = in.param3;
    out.param4 = in.param4;
    const bool stores_location = command_long_stores_location((MAV_CMD)in.command);
    out.x = convert_COMMAND_LONG_loc_param(in.param5, stores_location);
    out.y = convert_COMMAND_LONG_loc_param(in.param6, stores_location);
    out.z = in.param7;
}

void GCS_MAVLINK::handle_command_long(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_command_long_t packet;
    mavlink_msg_command_long_decode(&msg, &packet);

#if AP_SCRIPTING_ENABLED
    AP_Scripting *scripting = AP_Scripting::get_singleton();
    if (scripting != nullptr && scripting->is_handling_command(packet.command)) {
        // Scripting has registered to receive this command, do not procces it internaly
        return;
    }
#endif

    hal.util->persistent_data.last_mavlink_cmd = packet.command;

    const MAV_RESULT result = try_command_long_as_command_int(packet, msg);

    // send ACK or NAK
    mavlink_msg_command_ack_send(chan, packet.command, result,
                                 0, 0,
                                 msg.sysid,
                                 msg.compid);

#if HAL_LOGGING_ENABLED
    // log the packet:
    mavlink_command_int_t packet_int;
    convert_COMMAND_LONG_to_COMMAND_INT(packet, packet_int);
    AP::logger().Write_Command(packet_int, msg.sysid, msg.compid, result, true);
#endif

    hal.util->persistent_data.last_mavlink_cmd = 0;
}

#else
void GCS_MAVLINK::handle_command_long(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_command_long_t packet;
    mavlink_msg_command_long_decode(&msg, &packet);

    // send ACK or NAK
    mavlink_msg_command_ack_send(
        chan,
        packet.command,
        MAV_RESULT_COMMAND_INT_ONLY,
        0,
        0,
        msg.sysid,
        msg.compid
   );

}
#endif  // AP_MAVLINK_COMMAND_LONG_ENABLED

MAV_RESULT GCS_MAVLINK::handle_command_do_set_roi(const Location &roi_loc)
{
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    // sanity check location
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }

    if (!roi_loc.initialised()) {
        mount->clear_roi_target();
    } else {
        mount->set_roi_target(roi_loc);
    }
    return MAV_RESULT_ACCEPTED;
#else
    return MAV_RESULT_UNSUPPORTED;
#endif
}


void GCS_MAVLINK::handle_landing_target(const mavlink_message_t &msg)
{
    mavlink_landing_target_t m;
    mavlink_msg_landing_target_decode(&msg, &m);
    // correct offboard timestamp
    const uint32_t corrected_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, PAYLOAD_SIZE(chan, LANDING_TARGET));
    handle_landing_target(m, corrected_ms);
}


#if AP_HOME_ENABLED
bool GCS_MAVLINK::set_home_to_current_location(bool _lock)
{
#if AP_VEHICLE_ENABLED
    return AP::vehicle()->set_home_to_current_location(_lock);
#else
    return false;
#endif
}

bool GCS_MAVLINK::set_home(const Location& loc, bool _lock) {
#if AP_VEHICLE_ENABLED
    return AP::vehicle()->set_home(loc, _lock);
#else
    return false;
#endif
}
#endif  // AP_HOME_ENABLED

#if AP_HOME_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_do_set_home(const mavlink_command_int_t &packet)
{
    if (is_equal(packet.param1, 1.0f) || (packet.x == 0 && packet.y == 0)) {
        // param1 is 1 (or both lat and lon are zero); use current location
        if (!set_home_to_current_location(true)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }
    // ensure param1 is zero
    if (!is_zero(packet.param1)) {
        return MAV_RESULT_FAILED;
    }
    Location new_home_loc;
    if (!location_from_command_t(packet, new_home_loc)) {
        return MAV_RESULT_DENIED;
    }
    if (!set_home(new_home_loc, true)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif  // AP_HOME_ENABLED

#if AP_AHRS_POSITION_RESET_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_int_external_position_estimate(const mavlink_command_int_t &packet)
{
    if ((packet.frame != MAV_FRAME_GLOBAL && packet.frame != MAV_FRAME_GLOBAL_INT) ||
        !isnan(packet.z)) {
        // we only support global frame without altitude
        return MAV_RESULT_DENIED;
    }

    // cope with the NaN when convering to Location
    Location loc;
    mavlink_command_int_t p2 = packet;
    p2.z = 0;

    if (!location_from_command_t(p2, loc)) {
        return MAV_RESULT_DENIED;
    }
    uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(uint64_t(p2.param1*1e6), PAYLOAD_SIZE(chan, COMMAND_INT));
    const uint32_t processing_ms = p2.param2*1e3;
    const float pos_accuracy = p2.param3;
    if (timestamp_ms > processing_ms) {
        timestamp_ms -= processing_ms;
    }
    if (!AP::ahrs().handle_external_position_estimate(loc, pos_accuracy, timestamp_ms)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif // AP_AHRS_POSITION_RESET_ENABLED

#if AP_AHRS_EXTERNAL_WIND_ESTIMATE_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_int_external_wind_estimate(const mavlink_command_int_t &packet)
{
    if (packet.param1 < 0) {
        return MAV_RESULT_DENIED;
    }
    if (packet.param3 < 0 || packet.param3 > 360) {
        return MAV_RESULT_DENIED;
    }

    AP::ahrs().set_external_wind_estimate(packet.param1, packet.param3);
    return MAV_RESULT_ACCEPTED;
}
#endif // AP_AHRS_EXTERNAL_WIND_ESTIMATE_ENABLED

MAV_RESULT GCS_MAVLINK::handle_command_do_set_roi(const mavlink_command_int_t &packet)
{
    // be aware that this method is called for both MAV_CMD_DO_SET_ROI
    // and MAV_CMD_DO_SET_ROI_LOCATION.  If you intend to support any
    // of the extra fields in the former then you will need to split
    // off support for MAV_CMD_DO_SET_ROI_LOCATION (which doesn't
    // support the extra fields).

    // param1 : /* Region of interest mode (not used)*/
    // param2 : /* MISSION index/ target ID (not used)*/
    // param3 : /* ROI index (not used)*/
    // param4 : /* empty */
    // x : lat
    // y : lon
    // z : alt
    Location roi_loc;
    if (!location_from_command_t(packet, roi_loc)) {
        return MAV_RESULT_DENIED;
    }
    return handle_command_do_set_roi(roi_loc);
}

#if AP_FILESYSTEM_FORMAT_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_storage_format(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (!is_equal(packet.param1, 1.0f) ||
        !is_equal(packet.param2, 1.0f)) {
        return MAV_RESULT_UNSUPPORTED;
    }
    GCS_MAVLINK_InProgress *task = GCS_MAVLINK_InProgress::get_task(MAV_CMD_STORAGE_FORMAT, GCS_MAVLINK_InProgress::Type::SD_FORMAT, msg.sysid, msg.compid, chan);
    if (task == nullptr) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }
    if (!AP::FS().format()) {
        task->abort();
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_IN_PROGRESS;
}
#endif

MAV_RESULT GCS_MAVLINK::handle_do_set_safety_switch_state(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch ((SAFETY_SWITCH_STATE)packet.param1) {
    case SAFETY_SWITCH_STATE_DANGEROUS:
        // turn safety off (pwm outputs flow to the motors)
        hal.rcout->force_safety_off();
        return MAV_RESULT_ACCEPTED;
    case SAFETY_SWITCH_STATE_SAFE:
        // turn safety on (no pwm outputs to the motors)
        if (hal.rcout->force_safety_on()) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    default:
        return MAV_RESULT_DENIED;
    }
}

#if AP_MAVLINK_FOLLOW_HANDLING_ENABLED
MAV_RESULT GCS_MAVLINK::handle_command_do_follow(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    AP_Follow *follow = AP_Follow::get_singleton();
    if (follow == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    // param1: sysid of target to follow
    if ((packet.param1 > 0) && (packet.param1 <= 255)) {
        follow->set_target_sysid((uint8_t)packet.param1);
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_DENIED;
}
#endif  // AP_MAVLINK_FOLLOW_HANDLING_ENABLED

MAV_RESULT GCS_MAVLINK::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {

#if HAL_INS_ACCELCAL_ENABLED
    case MAV_CMD_ACCELCAL_VEHICLE_POS:
        return handle_command_accelcal_vehicle_pos(packet);
#endif

#if AP_LANDINGGEAR_ENABLED
    case MAV_CMD_AIRFRAME_CONFIGURATION:
        return handle_command_airframe_configuration(packet);
#endif

#if AP_BATTERY_ENABLED
    case MAV_CMD_BATTERY_RESET:
        return handle_command_battery_reset(packet);
#endif

#if AP_MAVLINKCAN_ENABLED
    case MAV_CMD_CAN_FORWARD:
        return handle_can_forward(packet, msg);
#endif  // AP_MAVLINKCAN_ENABLED

#if HAL_HIGH_LATENCY2_ENABLED
    case MAV_CMD_CONTROL_HIGH_LATENCY:
        return handle_control_high_latency(packet);
#endif // HAL_HIGH_LATENCY2_ENABLED

    case MAV_CMD_DEBUG_TRAP:
        return handle_command_debug_trap(packet);

#if HAL_ADSB_ENABLED
    case MAV_CMD_DO_ADSB_OUT_IDENT:
        if ((AP::ADSB() != nullptr) && AP::ADSB()->ident_start()) {
            return MAV_RESULT_ACCEPTED;
        }
        return  MAV_RESULT_FAILED;
#endif

#if AP_AHRS_ENABLED
    case MAV_CMD_DO_SET_GLOBAL_ORIGIN:
        return handle_command_do_set_global_origin(packet);
#endif  // AP_AHRS_ENABLED

#if AP_RC_CHANNEL_ENABLED
    case MAV_CMD_DO_AUX_FUNCTION:
        return handle_command_do_aux_function(packet);
#endif

#if AP_FENCE_ENABLED
    case MAV_CMD_DO_FENCE_ENABLE:
        return handle_command_do_fence_enable(packet);
#endif

    case MAV_CMD_DO_FLIGHTTERMINATION:
        return handle_flight_termination(packet);

#if AP_GRIPPER_ENABLED
    case MAV_CMD_DO_GRIPPER:
        return handle_command_do_gripper(packet);
#endif

#if AP_MISSION_ENABLED
    case MAV_CMD_DO_JUMP_TAG:
        return handle_command_do_jump_tag(packet);

    case MAV_CMD_DO_SET_MISSION_CURRENT:
        return handle_command_do_set_mission_current(packet);
#endif

    case MAV_CMD_DO_SET_MODE:
        return handle_command_do_set_mode(packet);

#if HAL_SPRAYER_ENABLED
    case MAV_CMD_DO_SPRAYER:
        return handle_command_do_sprayer(packet);
#endif

#if AP_CAMERA_ENABLED
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_SET_CAMERA_ZOOM:
    case MAV_CMD_SET_CAMERA_FOCUS:
    case MAV_CMD_SET_CAMERA_SOURCE:
    case MAV_CMD_IMAGE_START_CAPTURE:
    case MAV_CMD_IMAGE_STOP_CAPTURE:
    case MAV_CMD_CAMERA_TRACK_POINT:
    case MAV_CMD_CAMERA_TRACK_RECTANGLE:
    case MAV_CMD_CAMERA_STOP_TRACKING:
    case MAV_CMD_VIDEO_START_CAPTURE:
    case MAV_CMD_VIDEO_STOP_CAPTURE:
        return handle_command_camera(packet);
#endif

    case MAV_CMD_DO_SET_ROI_NONE: {
        const Location zero_loc;
        return handle_command_do_set_roi(zero_loc);
    }

    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_SET_ROI_LOCATION:
        return handle_command_do_set_roi(packet);

#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_SET_ROI_SYSID:
    case MAV_CMD_DO_MOUNT_CONFIGURE:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
    case MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
        return handle_command_mount(packet, msg);
#endif  // HAL_MOUNT_ENABLED

    case MAV_CMD_DO_SEND_BANNER:
        send_banner();
        return MAV_RESULT_ACCEPTED;

#if AP_HOME_ENABLED
    case MAV_CMD_DO_SET_HOME:
        return handle_command_do_set_home(packet);
#endif

#if AP_AHRS_POSITION_RESET_ENABLED
    case MAV_CMD_EXTERNAL_POSITION_ESTIMATE:
        return handle_command_int_external_position_estimate(packet);
#endif
#if AP_AHRS_EXTERNAL_WIND_ESTIMATE_ENABLED
    case MAV_CMD_EXTERNAL_WIND_ESTIMATE:
        return handle_command_int_external_wind_estimate(packet);
#endif
#if AP_ARMING_ENABLED
    case MAV_CMD_COMPONENT_ARM_DISARM:
        return handle_command_component_arm_disarm(packet);
#endif
#if AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
    case MAV_CMD_FIXED_MAG_CAL_YAW:
        return handle_command_fixed_mag_cal_yaw(packet);
#endif
#if COMPASS_CAL_ENABLED
    case MAV_CMD_DO_START_MAG_CAL:
    case MAV_CMD_DO_ACCEPT_MAG_CAL:
    case MAV_CMD_DO_CANCEL_MAG_CAL:
        return handle_command_mag_cal(packet);
#endif

#if AP_BOOTLOADER_FLASHING_ENABLED
    case MAV_CMD_FLASH_BOOTLOADER:
        return handle_command_flash_bootloader(packet);
#endif

#if AP_AHRS_ENABLED
    case MAV_CMD_GET_HOME_POSITION:
        return handle_command_get_home_position(packet);
#endif

    case MAV_CMD_PREFLIGHT_CALIBRATION:
        return handle_command_preflight_calibration(packet, msg);

    case MAV_CMD_PREFLIGHT_STORAGE:
        if (is_equal(packet.param1, 2.0f)) {
            AP_Param::erase_all();
            send_text(MAV_SEVERITY_WARNING, "All parameters reset, reboot board");
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;

    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        return handle_preflight_reboot(packet, msg);

    case MAV_CMD_DO_SET_SAFETY_SWITCH_STATE:
        return handle_do_set_safety_switch_state(packet, msg);

#if AP_MAVLINK_SERVO_RELAY_ENABLED
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_RELAY:
        return handle_servorelay_message(packet);
#endif

#if AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        return handle_command_request_autopilot_capabilities(packet);
#endif

#if AP_ARMING_ENABLED
    case MAV_CMD_RUN_PREARM_CHECKS:
        return handle_command_run_prearm_checks(packet);
#endif

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_SCRIPTING:
        {
            AP_Scripting *scripting = AP_Scripting::get_singleton();
            if (scripting == nullptr) {
                return MAV_RESULT_UNSUPPORTED;
            }
            return scripting->handle_command_int_packet(packet);
        }
#endif // AP_SCRIPTING_ENABLED

#if AP_AHRS_ENABLED
    case MAV_CMD_SET_EKF_SOURCE_SET:
        return handle_command_set_ekf_source_set(packet);
#endif

#if AP_RCPROTOCOL_ENABLED
    case MAV_CMD_START_RX_PAIR:
        return handle_START_RX_PAIR(packet);
#endif

#if AP_FILESYSTEM_FORMAT_ENABLED
    case MAV_CMD_STORAGE_FORMAT:
        return handle_command_storage_format(packet, msg);
#endif

    // support for dealing with streamrate for a specific message and
    // requesting a message instance:
    case MAV_CMD_SET_MESSAGE_INTERVAL:
        return handle_command_set_message_interval(packet);

    case MAV_CMD_GET_MESSAGE_INTERVAL:
        return handle_command_get_message_interval(packet);

    case MAV_CMD_REQUEST_MESSAGE:
        return handle_command_request_message(packet);

#if AP_MAVLINK_FOLLOW_HANDLING_ENABLED
    case MAV_CMD_DO_FOLLOW:
        return handle_command_do_follow(packet, msg);
#endif
    }

    return MAV_RESULT_UNSUPPORTED;
}

void GCS_MAVLINK::handle_command_int(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_command_int_t packet;
    mavlink_msg_command_int_decode(&msg, &packet);

#if AP_SCRIPTING_ENABLED
    AP_Scripting *scripting = AP_Scripting::get_singleton();
    if (scripting != nullptr && scripting->is_handling_command(packet.command)) {
        // Scripting has registered to receive this command, do not procces it internaly
        return;
    }
#endif

    hal.util->persistent_data.last_mavlink_cmd = packet.command;

    const MAV_RESULT result = handle_command_int_packet(packet, msg);

    // send ACK or NAK
    mavlink_msg_command_ack_send(chan, packet.command, result,
                                 0, 0,
                                 msg.sysid,
                                 msg.compid);

#if HAL_LOGGING_ENABLED
    AP::logger().Write_Command(packet, msg.sysid, msg.compid, result);
#endif

    hal.util->persistent_data.last_mavlink_cmd = 0;
}

void GCS::try_send_queued_message_for_type(MAV_MISSION_TYPE type) const {
    MissionItemProtocol *prot = get_prot_for_mission_type(type);
    if (prot == nullptr) {
        return;
    }
    prot->queued_request_send();
}

bool GCS_MAVLINK::try_send_mission_message(const enum ap_message id)
{
    switch (id) {
#if AP_MISSION_ENABLED
    case MSG_CURRENT_WAYPOINT:
    {
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        AP_Mission *mission = AP::mission();
        if (mission != nullptr) {
            send_mission_current(*mission, mission->get_current_nav_index());
        }
        break;
    }
    case MSG_MISSION_ITEM_REACHED:
        CHECK_PAYLOAD_SIZE(MISSION_ITEM_REACHED);
        mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
        break;
    case MSG_NEXT_MISSION_REQUEST_WAYPOINTS:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs().try_send_queued_message_for_type(MAV_MISSION_TYPE_MISSION);
        break;
#endif
#if HAL_RALLY_ENABLED
    case MSG_NEXT_MISSION_REQUEST_RALLY:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs().try_send_queued_message_for_type(MAV_MISSION_TYPE_RALLY);
        break;
#endif
#if AP_FENCE_ENABLED
    case MSG_NEXT_MISSION_REQUEST_FENCE:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs().try_send_queued_message_for_type(MAV_MISSION_TYPE_FENCE);
        break;
#endif
    default:
        break;
    }
    return true;
}

#if AP_MAVLINK_MSG_HWSTATUS_ENABLED
void GCS_MAVLINK::send_hwstatus()
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        0);
}
#endif  // AP_MAVLINK_MSG_HWSTATUS_ENABLED

#if AP_RPM_ENABLED
void GCS_MAVLINK::send_rpm() const
{
    AP_RPM *rpm = AP::rpm();
    if (rpm == nullptr) {
        return;
    }

    if (!rpm->enabled(0) && !rpm->enabled(1)) {
        return;
    }

    float rpm1 = -1, rpm2 = -1;

    rpm->get_rpm(0, rpm1);
    rpm->get_rpm(1, rpm2);

    mavlink_msg_rpm_send(
        chan,
        rpm1,
        rpm2);
}
#endif  // AP_RPM_ENABLED

void GCS_MAVLINK::send_sys_status()
{
    // send extended status only once vehicle has been initialised
    // to avoid unnecessary errors being reported to user
    if (!gcs().vehicle_initialised()) {
        return;
    }
#if AP_BATTERY_ENABLED
    const AP_BattMonitor &battery = AP::battery();
    float battery_current;
    const int8_t battery_remaining = battery_remaining_pct(AP_BATT_PRIMARY_INSTANCE);

    if (battery.healthy() && battery.current_amps(battery_current)) {
        battery_current = constrain_float(battery_current * 100,-INT16_MAX,INT16_MAX);
    } else {
        battery_current = -1;
    }
#endif

    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    gcs().get_sensor_status_flags(control_sensors_present, control_sensors_enabled, control_sensors_health);

    const uint32_t errors = AP::internalerror().errors();
    const uint16_t errors1 = errors & 0xffff;
    const uint16_t errors2 = (errors>>16) & 0xffff;
    const uint16_t errors4 = AP::internalerror().count() & 0xffff;
    uint16_t errors_comm = 0;
    mavlink_status_t *ms = mavlink_get_channel_status(chan);
    if (ms) {
        errors_comm = ms->packet_rx_drop_count;
    }

#if HAL_LOGGING_ENABLED
    const uint16_t dropped_logmessage_count = AP::logger().num_dropped();
#else
    const uint16_t dropped_logmessage_count = UINT16_MAX;
#endif  // HAL_LOGGING_ENABLED

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
#if AP_SCHEDULER_ENABLED
        static_cast<uint16_t>(AP::scheduler().load_average() * 1000),
#else
        0,
#endif
#if AP_BATTERY_ENABLED
        battery.gcs_voltage() * 1000,  // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
#else
        0,
        -1,
        -1,
#endif
        0,  // comm drops %,
        errors_comm,  // comm drops in pkts,
        errors1,
        errors2,
        dropped_logmessage_count,  // errors3
        errors4); // errors4
}

void GCS_MAVLINK::send_extended_sys_state() const
{
    mavlink_msg_extended_sys_state_send(chan, vtol_state(), landed_state());
}

void GCS_MAVLINK::send_attitude() const
{
#if AP_AHRS_ENABLED
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        ahrs.get_roll_rad(),
        ahrs.get_pitch_rad(),
        ahrs.get_yaw_rad(),
        omega.x,
        omega.y,
        omega.z);
#endif
}

void GCS_MAVLINK::send_attitude_quaternion() const
{
#if AP_AHRS_ENABLED
    const AP_AHRS &ahrs = AP::ahrs();
    Quaternion quat;
    if (!ahrs.get_quaternion(quat)) {
        return;
    }
    const Vector3f omega = ahrs.get_gyro();
    const float repr_offseq_q[] {0,0,0,0};  // unused, but probably should correspond to the AHRS view?
    mavlink_msg_attitude_quaternion_send(
        chan,
        AP_HAL::millis(),
        quat.q1,
        quat.q2,
        quat.q3,
        quat.q4,
        omega.x, // rollspeed
        omega.y, // pitchspeed
        omega.z, // yawspeed
        repr_offseq_q
        );
#endif
}

int32_t GCS_MAVLINK::global_position_int_alt() const {
    return global_position_current_loc.alt * 10UL;
}
int32_t GCS_MAVLINK::global_position_int_relative_alt() const {
#if AP_AHRS_ENABLED
    float posD;
    AP::ahrs().get_relative_position_D_home(posD);
    posD *= -1000.0f; // change from down to up and metres to millimeters
    return posD;
#else
    return 0;
#endif
}

void GCS_MAVLINK::send_global_position_int()
{
#if AP_AHRS_ENABLED
    AP_AHRS &ahrs = AP::ahrs();

    UNUSED_RESULT(ahrs.get_location(global_position_current_loc)); // return value ignored; we send stale data

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }

    mavlink_msg_global_position_int_send(
        chan,
        AP_HAL::millis(),
        global_position_current_loc.lat, // in 1E7 degrees
        global_position_current_loc.lng, // in 1E7 degrees
        global_position_int_alt(),       // millimeters above ground/sea level
        global_position_int_relative_alt(), // millimeters above home
        vel.x * 100,                     // X speed cm/s (+ve North)
        vel.y * 100,                     // Y speed cm/s (+ve East)
        vel.z * 100,                     // Z speed cm/s (+ve Down)
        ahrs.yaw_sensor);                // compass heading in 1/100 degree
#endif  // AP_AHRS_ENABLED
}

#if HAL_MOUNT_ENABLED
void GCS_MAVLINK::send_gimbal_device_attitude_status() const
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->send_gimbal_device_attitude_status(chan);
}

void GCS_MAVLINK::send_gimbal_manager_information() const
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->send_gimbal_manager_information(chan);
}

void GCS_MAVLINK::send_gimbal_manager_status() const
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->send_gimbal_manager_status(chan);
}
#endif

void GCS_MAVLINK::send_set_position_target_global_int(uint8_t target_system, uint8_t target_component, const Location& loc)
{

    const uint16_t type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | \
                               POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | \
                               POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    // convert altitude to relative to home
    int32_t rel_alt;
    if (!loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, rel_alt)) {
        return;
    }

    mavlink_msg_set_position_target_global_int_send(
            chan,
            AP_HAL::millis(),
            target_system,
            target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            type_mask,
            loc.lat,
            loc.lng,
            rel_alt,
            0,0,0,  // vx, vy, vz
            0,0,0,  // ax, ay, az
            0,0);   // yaw, yaw_rate
}

#if HAL_GENERATOR_ENABLED
void GCS_MAVLINK::send_generator_status() const
{
    AP_Generator *generator = AP::generator();
    if (generator == nullptr) {
        return;
    }
    generator->send_generator_status(*this);
}
#endif

#if HAL_ADSB_ENABLED
void GCS_MAVLINK::send_uavionix_adsb_out_status() const
{
    AP_ADSB *adsb = AP::ADSB();
    if (adsb != nullptr) {
        adsb->send_adsb_out_status(chan);
    }
}
#endif

#if AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
bool GCS_MAVLINK::send_relay_status() const
{
    AP_Relay *relay = AP::relay();
    if (relay == nullptr) {
        // must only return false if out of space:
        return true;
    }

    return relay->send_relay_status(*this);
}
#endif  // AP_MAVLINK_MSG_RELAY_STATUS_ENABLED

void GCS_MAVLINK::send_autopilot_state_for_gimbal_device() const
{
#if AP_AHRS_ENABLED
    // get attitude
    const AP_AHRS &ahrs = AP::ahrs();
    Quaternion quat;
    if (!ahrs.get_quaternion(quat)) {
        return;
    }
    const float repr_offseq_q[] = {quat.q1, quat.q2, quat.q3, quat.q4};

    // get velocity
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }

    // get vehicle earth-frame rotation rate targets
    Vector3f rate_ef_targets;
#if AP_VEHICLE_ENABLED
    const AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->get_rate_ef_targets(rate_ef_targets);
    }
#endif

    // get estimator flags
    uint16_t est_status_flags = 0;
    nav_filter_status nav_filt_status;
    if (ahrs.get_filter_status(nav_filt_status)) {
        est_status_flags = (uint16_t)(nav_filt_status.value & 0xFFFF);
    }

    mavlink_msg_autopilot_state_for_gimbal_device_send(
        chan,
        mavlink_system.sysid,   // target system (this autopilot's gimbal)
        0,                  // target component (anything)
        AP_HAL::micros(),   // time boot us
        repr_offseq_q,  // attitude as quaternion
        0,      // attitude estimated delay in micros
        vel.x,  // x speed in NED (m/s)
        vel.y,  // y speed in NED (m/s)
        vel.z,  // z speed in NED (m/s)
        0,      // velocity estimated delay in micros
        rate_ef_targets.z,  // feed forward angular velocity z
        est_status_flags,   // estimator status
        0,      // landed_state (see MAV_LANDED_STATE)
        AP::ahrs().get_yaw_rate_earth());   // [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown
#endif  // AP_AHRS_ENABLED
}

#if AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
void GCS_MAVLINK::send_flight_information()
{
    const uint64_t time_boot_micros = AP_HAL::micros64();
    const uint32_t time_boot_ms = static_cast<uint32_t>(time_boot_micros / 1000);

    // This field is misnamed as `arming_time_utc` in MAVLink. However, it is
    // not a UTC time, it is the microseconds since boot.
    const uint64_t arm_time_us = AP::arming().arm_time_us();

    const MAV_LANDED_STATE current_landed_state = landed_state();
    if (flight_info.last_landed_state != current_landed_state) {
        switch (current_landed_state) {
            case MAV_LANDED_STATE_IN_AIR:
            case MAV_LANDED_STATE_TAKEOFF:
            case MAV_LANDED_STATE_LANDING:
                if (!flight_info.takeoff_time_us) {
                    flight_info.takeoff_time_us = time_boot_micros;
                }
                break;

            case MAV_LANDED_STATE_ON_GROUND: 
                flight_info.takeoff_time_us = 0;
                break;

            case MAV_LANDED_STATE_UNDEFINED:
            case MAV_LANDED_STATE_ENUM_END:
                break;
        }

        flight_info.last_landed_state = current_landed_state;
    }

    // This field is misnamed as `takeoff_time_utc` in MAVLink. However, it is
    // not a UTC time, it is the microseconds since boot.
    uint64_t takeoff_time_us = flight_info.takeoff_time_us;

    // This field is misnamed as `flight_uuid` in MAVLink.
    const uint64_t flight_number = 0;

    mavlink_msg_flight_information_send(
        chan,
        time_boot_ms,
        arm_time_us,
        takeoff_time_us,
        flight_number
    );
}
#endif // AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED

void GCS_MAVLINK::send_received_message_deprecation_warning(const char * message)
{
    // we're not expecting very many of these ever, so a tiny bit of
    // de-duping is probably OK:
    if (message == last_deprecation_message) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (last_deprecation_warning_send_time_ms - now_ms < 30000) {
        return;
    }
    last_deprecation_warning_send_time_ms = now_ms;
    last_deprecation_message = message;

    send_text(MAV_SEVERITY_INFO, "Received message (%s) is deprecated", message);
}

bool GCS_MAVLINK::send_available_modes()
{
    if (!available_modes.should_send) {
        // must only return false if out of space
        return true;
    }

    CHECK_PAYLOAD_SIZE(AVAILABLE_MODES);

    // Zero is a special case for send all.
    const bool send_all = available_modes.requested_index == 0;
    uint8_t request_index;
    if (!send_all) {
        // Single request
        request_index = available_modes.requested_index;
        available_modes.should_send = false;

    } else {
        // Request all modes
        request_index = available_modes.next_index;
        available_modes.next_index += 1;
    }

    const uint8_t mode_count = send_available_mode(request_index);

    if (send_all && (available_modes.next_index > mode_count)) {
        // Sending all and just sent the last
        available_modes.should_send = false;
    }

    return true;
}

bool GCS_MAVLINK::send_available_mode_monitor()
{
    CHECK_PAYLOAD_SIZE(AVAILABLE_MODES_MONITOR);

    mavlink_msg_available_modes_monitor_send(
        chan,
        gcs().get_available_modes_sequence()
    );

    return true;
}

bool GCS_MAVLINK::try_send_message(const enum ap_message id)
{
    bool ret = true;

    switch(id) {

#if AP_AHRS_ENABLED
    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude();
        break;

    case MSG_ATTITUDE_QUATERNION:
        CHECK_PAYLOAD_SIZE(ATTITUDE_QUATERNION);
        send_attitude_quaternion();
        break;
#endif

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        send_heartbeat();
        break;

#if AP_MAVLINK_MSG_HWSTATUS_ENABLED
    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus();
        break;
#endif  // AP_MAVLINK_MSG_HWSTATUS_ENABLED

#if AP_AHRS_ENABLED
    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_global_position_int();
        break;

    case MSG_HOME:
        CHECK_PAYLOAD_SIZE(HOME_POSITION);
        send_home_position();
        break;

    case MSG_ORIGIN:
        CHECK_PAYLOAD_SIZE(GPS_GLOBAL_ORIGIN);
        send_gps_global_origin();
        break;
#endif  // AP_AHRS_ENABLED

#if AP_RPM_ENABLED
    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        send_rpm();
        break;
#endif

    case MSG_CURRENT_WAYPOINT:
    case MSG_MISSION_ITEM_REACHED:
    case MSG_NEXT_MISSION_REQUEST_WAYPOINTS:
    case MSG_NEXT_MISSION_REQUEST_RALLY:
    case MSG_NEXT_MISSION_REQUEST_FENCE:
        ret = try_send_mission_message(id);
        break;

#if COMPASS_CAL_ENABLED
    case MSG_MAG_CAL_PROGRESS:
        ret = AP::compass().send_mag_cal_progress(*this);
        break;
    case MSG_MAG_CAL_REPORT:
        ret = AP::compass().send_mag_cal_report(*this);
        break;
#endif

#if AP_BATTERY_ENABLED
    case MSG_BATTERY_STATUS:
        send_battery_status();
        break;
#endif // AP_BATTERY_ENABLED

#if AP_AHRS_ENABLED
    case MSG_EKF_STATUS_REPORT:
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        AP::ahrs().send_ekf_status_report(*this);
        break;
#endif

    case MSG_MEMINFO:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

#if AP_FENCE_ENABLED
    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        send_fence_status();
        break;
#endif

#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder();
        break;
#endif  // AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED

    case MSG_DISTANCE_SENSOR:
        send_distance_sensor();
        break;

#if AP_CAMERA_ENABLED
    case MSG_CAMERA_FEEDBACK:
    case MSG_CAMERA_INFORMATION:
    case MSG_CAMERA_SETTINGS:
#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
    case MSG_CAMERA_FOV_STATUS:
#endif
    case MSG_CAMERA_CAPTURE_STATUS:
#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
    case MSG_CAMERA_THERMAL_RANGE:
#endif
#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    case MSG_VIDEO_STREAM_INFORMATION:
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
        {
            AP_Camera *camera = AP::camera();
            if (camera == nullptr) {
                break;
            }
            return camera->send_mavlink_message(*this, id);
        }
#endif  // AP_CAMERA_ENABLED

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time();
        break;

#if AP_GPS_GPS_RAW_INT_SENDING_ENABLED
    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        AP::gps().send_mavlink_gps_raw(chan);
        break;
#endif  // AP_GPS_GPS_RAW_INT_SENDING_ENABLED
#if AP_GPS_GPS_RTK_SENDING_ENABLED
    case MSG_GPS_RTK:
        CHECK_PAYLOAD_SIZE(GPS_RTK);
        AP::gps().send_mavlink_gps_rtk(chan, 0);
        break;
#endif  // AP_GPS_GPS_RTK_SENDING_ENABLED
#if AP_GPS_GPS2_RAW_SENDING_ENABLED
    case MSG_GPS2_RAW:
        CHECK_PAYLOAD_SIZE(GPS2_RAW);
        AP::gps().send_mavlink_gps2_raw(chan);
        break;
#endif  // AP_GPS_GPS2_RAW_SENDING_ENABLED
#if AP_GPS_GPS2_RTK_SENDING_ENABLED
    case MSG_GPS2_RTK:
        CHECK_PAYLOAD_SIZE(GPS2_RTK);
        AP::gps().send_mavlink_gps_rtk(chan, 1);
        break;
#endif  // AP_GPS_GPS2_RTK_SENDING_ENABLED

#if AP_AHRS_ENABLED
    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position();
        break;
#endif

#if HAL_MOUNT_ENABLED
    case MSG_GIMBAL_DEVICE_ATTITUDE_STATUS:
        CHECK_PAYLOAD_SIZE(GIMBAL_DEVICE_ATTITUDE_STATUS);
        send_gimbal_device_attitude_status();
        break;
    case MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE:
        CHECK_PAYLOAD_SIZE(AUTOPILOT_STATE_FOR_GIMBAL_DEVICE);
        send_autopilot_state_for_gimbal_device();
        break;
    case MSG_GIMBAL_MANAGER_INFORMATION:
        CHECK_PAYLOAD_SIZE(GIMBAL_MANAGER_INFORMATION);
        send_gimbal_manager_information();
        break;
    case MSG_GIMBAL_MANAGER_STATUS:
        CHECK_PAYLOAD_SIZE(GIMBAL_MANAGER_STATUS);
        send_gimbal_manager_status();
        break;
#endif  // HAL_MOUNT_ENABLED

#if AP_OPTICALFLOW_ENABLED
    case MSG_OPTICAL_FLOW:
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        send_opticalflow();
        break;
#endif

    case MSG_ATTITUDE_TARGET:
        CHECK_PAYLOAD_SIZE(ATTITUDE_TARGET);
        send_attitude_target();
        break;

    case MSG_POSITION_TARGET_GLOBAL_INT:
        CHECK_PAYLOAD_SIZE(POSITION_TARGET_GLOBAL_INT);
        send_position_target_global_int();
        break;

    case MSG_POSITION_TARGET_LOCAL_NED:
        CHECK_PAYLOAD_SIZE(POSITION_TARGET_LOCAL_NED);
        send_position_target_local_ned();
        break;

    case MSG_POWER_STATUS:
        CHECK_PAYLOAD_SIZE(POWER_STATUS);
        send_power_status();
        break;

#if HAL_WITH_MCU_MONITORING
    case MSG_MCU_STATUS:
        CHECK_PAYLOAD_SIZE(MCU_STATUS);
        send_mcu_status();
        break;
#endif

#if AP_RC_CHANNEL_ENABLED
    case MSG_RC_CHANNELS:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS);
        send_rc_channels();
        break;

#if AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
    case MSG_RC_CHANNELS_RAW:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_rc_channels_raw();
        break;
#endif  // AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED

#endif

    case MSG_RAW_IMU:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu();
        break;

    case MSG_SCALED_IMU:
        CHECK_PAYLOAD_SIZE(SCALED_IMU);
        send_scaled_imu(0, mavlink_msg_scaled_imu_send);
        break;

    case MSG_SCALED_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_IMU2);
        send_scaled_imu(1, mavlink_msg_scaled_imu2_send);
        break;

    case MSG_SCALED_IMU3:
        CHECK_PAYLOAD_SIZE(SCALED_IMU3);
        send_scaled_imu(2, mavlink_msg_scaled_imu3_send);
        break;
#if AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
    case MSG_HIGHRES_IMU:
        CHECK_PAYLOAD_SIZE(HIGHRES_IMU);
        send_highres_imu();
        break;
#endif

    case MSG_SCALED_PRESSURE:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure();
        break;

    case MSG_SCALED_PRESSURE2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE2);
        send_scaled_pressure2();
        break;

    case MSG_SCALED_PRESSURE3:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE3);
        send_scaled_pressure3();
        break;

#if AP_AIRSPEED_ENABLED
    case MSG_AIRSPEED:
        CHECK_PAYLOAD_SIZE(AIRSPEED);
        send_airspeed();
        break;
#endif

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_servo_output_raw();
        break;

#if AP_SIM_ENABLED
    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate();
        break;

    case MSG_SIM_STATE:
        CHECK_PAYLOAD_SIZE(SIM_STATE);
        send_sim_state();
        break;
#endif

    case MSG_SYS_STATUS:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_sys_status();
        break;

#if AP_AHRS_ENABLED
    case MSG_AHRS2:
        CHECK_PAYLOAD_SIZE(AHRS2);
        send_ahrs2();
        break;
#endif

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        send_pid_tuning();
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output();
        break;

#if AP_AHRS_ENABLED
    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs();
        break;
#endif

    case MSG_EXTENDED_SYS_STATE:
        CHECK_PAYLOAD_SIZE(EXTENDED_SYS_STATE);
        send_extended_sys_state();
        break;

#if AP_AHRS_ENABLED
    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud();
        break;
#endif

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration();
        break;

#if HAL_GENERATOR_ENABLED
    case MSG_GENERATOR_STATUS:
    	CHECK_PAYLOAD_SIZE(GENERATOR_STATUS);
    	send_generator_status();
    	break;
#endif

    case MSG_AUTOPILOT_VERSION:
        CHECK_PAYLOAD_SIZE(AUTOPILOT_VERSION);
        send_autopilot_version();
        break;

#if HAL_WITH_ESC_TELEM
    case MSG_ESC_TELEMETRY:
        AP::esc_telem().send_esc_telemetry_mavlink(uint8_t(chan));
        break;
#endif

#if HAL_EFI_ENABLED
    case MSG_EFI_STATUS: {
        CHECK_PAYLOAD_SIZE(EFI_STATUS);
        AP_EFI *efi = AP::EFI();
        if (efi) {
            efi->send_mavlink_status(chan);
        }
        break;
    }
#endif

#if AP_WINCH_ENABLED
    case MSG_WINCH_STATUS:
        CHECK_PAYLOAD_SIZE(WINCH_STATUS);
        send_winch_status();
        break;
#endif

#if HAL_HIGH_LATENCY2_ENABLED
    case MSG_HIGH_LATENCY2:
        CHECK_PAYLOAD_SIZE(HIGH_LATENCY2);
        send_high_latency2();
        break;
#endif // HAL_HIGH_LATENCY2_ENABLED

#if AP_AIS_ENABLED
    case MSG_AIS_VESSEL: {
        AP_AIS *ais = AP_AIS::get_singleton();
        if (ais) {
            ais->send(chan);
        }
        break;
    }
#endif

#if AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
    case MSG_UAVIONIX_ADSB_OUT_STATUS:
        CHECK_PAYLOAD_SIZE(UAVIONIX_ADSB_OUT_STATUS);
        send_uavionix_adsb_out_status();
        break;
#endif

#if AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
    case MSG_RELAY_STATUS:
        ret = send_relay_status();
        break;
#endif

    case MSG_AVAILABLE_MODES:
        ret = send_available_modes();
        break;

    case MSG_AVAILABLE_MODES_MONITOR:
        ret = send_available_mode_monitor();
        break;

#if AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
    case MSG_FLIGHT_INFORMATION:
        CHECK_PAYLOAD_SIZE(FLIGHT_INFORMATION);
        send_flight_information();
        break;
#endif

        // terrain request and report shouldn't be here; the vehicles
        // should be doing better in terms of factoring behaviour up
#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
    case MSG_TERRAIN_REPORT:
#endif  // AP_TERRAIN_AVAILABLE
#if AP_AIRSPEED_HYGROMETER_ENABLE
        // hygrometer should be in its down library, not called via
        // Plane's functions
    case MSG_HYGROMETER:
#endif
        // ADSB should be moved down into here
    case MSG_ADSB_VEHICLE:
        break;

    default:
        // try_send_message must always at some stage return true for
        // a message, or we will attempt to infinitely retry the
        // message as part of send_message.
        // This message will be sent out at the same rate as the
        // unknown message, so should be safe.
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Sending unknown message (%u)", id);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Sending unknown ap_message %u", id);
#endif
        break;
    }

    return ret;
}

uint16_t GCS_MAVLINK::get_interval_for_stream(GCS_MAVLINK::streams id) const
{
    const int16_t frate = streamRates[id].get();
    if (frate == 0) {
        return 0;
    }
    const uint32_t ret = 1000/frate;
    if (ret > 60000) {
        return 60000;
    }
    return ret;
}

void GCS_MAVLINK::initialise_message_intervals_for_stream(GCS_MAVLINK::streams id)
{
    for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
        const GCS_MAVLINK::stream_entries &entries = all_stream_entries[i];
        if (entries.stream_id != id) {
            continue;
        }
        // found it!
        const uint16_t interval_ms = get_interval_for_stream(id);
        for (uint8_t j=0; j<entries.num_ap_message_ids; j++) {
            set_ap_message_interval(entries.ap_message_ids[j], interval_ms);
        }
        break;
    }
}

#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
// open and read contents of path, setting message intervals from each
// line
DefaultIntervalsFromFiles::DefaultIntervalsFromFiles(uint16_t max_num)
{
    if (max_num == 0) {
        return;
    }
    _intervals = NEW_NOTHROW from_file_default_interval[max_num];
    _max_intervals = max_num;
}

DefaultIntervalsFromFiles::~DefaultIntervalsFromFiles()
{
    delete[] (_intervals);
}

void DefaultIntervalsFromFiles::set(ap_message id, uint16_t interval)
{
    if (_intervals == nullptr) {
        return;
    }

    // update any existing interval (last-one-in wins)
    for (uint8_t i=0; i<_num_intervals; i++) {
        if (_intervals[i].id == id) {
            _intervals[i].interval = interval;
            return;
        }
    }

    // store an interval we've not seen before:
    if (_num_intervals == _max_intervals) {
        return;
    }

    _intervals[_num_intervals].id = id;
    _intervals[_num_intervals].interval = interval;
    _num_intervals++;
}

bool DefaultIntervalsFromFiles::get_interval_for_ap_message_id(ap_message id, uint16_t &interval) const
{
    for (uint16_t i=0; i<_num_intervals; i++) {
        if (_intervals[i].id == id) {
            interval = _intervals[i].interval;
            return true;
        }
    }
    return false;
}

ap_message DefaultIntervalsFromFiles::id_at(uint8_t ofs) const
{
    if (_intervals == nullptr || ofs >= _num_intervals) {
        return MSG_LAST;
    }
    return _intervals[ofs].id;
}

uint16_t DefaultIntervalsFromFiles::interval_at(uint8_t ofs) const
{
    if (_intervals == nullptr || ofs >= _num_intervals) {
        return -1;  // unsigned-integer wrap
    }
    return _intervals[ofs].interval;
}

void GCS_MAVLINK::get_intervals_from_filepath(const char *path, DefaultIntervalsFromFiles &intervals)
{
    const int f = AP::FS().open(path, O_RDONLY);
    if (f == -1) {
        return;
    }

    char line[20];
    while (AP::FS().fgets(line, sizeof(line)-1, f)) {
        char *saveptr = nullptr;
        const char *mavlink_id_str = strtok_r(line, " ", &saveptr);
        if (mavlink_id_str == nullptr || strlen(mavlink_id_str) == 0) {
            continue;
        }
        const uint32_t mavlink_id = atoi(mavlink_id_str);

        const ap_message msg_id = mavlink_id_to_ap_message_id(mavlink_id);
        if (msg_id == MSG_LAST) {
            continue;
        }

        const char *interval_str = strtok_r(nullptr, "\r\n", &saveptr);
        if (interval_str == nullptr || strlen(interval_str) == 0) {
            continue;
        }
        const uint16_t interval = atoi(interval_str);

        intervals.set(msg_id, interval);
    }

    AP::FS().close(f);
}

void GCS_MAVLINK::initialise_message_intervals_from_config_files()
{
    static const char *path_templates[] {
        "@ROMFS/message-intervals-chan%u.txt",
        "message-intervals-chan%u.txt"
    };

    // don't do anything at all if no files exist:
    bool exists = false;
    for (const char * path_template : path_templates) {
        struct stat stats;
        char *path;
        if (asprintf(&path, path_template, chan) == -1) {
            continue;
        }
        if (AP::FS().stat(path, &stats) < 0) {
            free(path);
            continue;
        }
        free(path);
        if (stats.st_size == 0) {
            continue;
        }
        exists = true;
        break;
    }
    if (!exists) {
        return;
    }

    // first over-allocate:
    DefaultIntervalsFromFiles *overallocated = NEW_NOTHROW DefaultIntervalsFromFiles(128);
    if (overallocated == nullptr) {
        return;
    }
    for (const char * path_template : path_templates) {
        char *path;
        if (asprintf(&path, path_template, chan) == -1) {
            continue;
        }
        get_intervals_from_filepath(path, *overallocated);
        free(path);
    }

    // then allocate just the right number and redo all of the work:
    const uint16_t num_required = overallocated->num_intervals();
    delete overallocated;
    overallocated = nullptr;

    default_intervals_from_files = NEW_NOTHROW DefaultIntervalsFromFiles(num_required);
    if (default_intervals_from_files == nullptr) {
        return;
    }
    for (const char * path_template : path_templates) {
        char *path;
        if (asprintf(&path, path_template, chan) == -1) {
            continue;
        }
        get_intervals_from_filepath(path, *default_intervals_from_files);
        free(path);
    }

    // now actually initialise the intervals:
    for (uint8_t i=0; i<default_intervals_from_files->num_intervals(); i++) {
        const ap_message id = default_intervals_from_files->id_at(i);
        if (id == MSG_LAST) {
            // internal error
            break;
        }
        const uint16_t interval = default_intervals_from_files->interval_at(i);
        set_ap_message_interval(id, interval);
    }
}
#endif

void GCS_MAVLINK::initialise_message_intervals_from_streamrates()
{
#if HAL_HIGH_LATENCY2_ENABLED
    if (!is_high_latency_link) {
        // this is O(n^2), but it's once at boot and across a 10-entry list...
        for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
            initialise_message_intervals_for_stream(all_stream_entries[i].stream_id);
        }
        set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT, 1000);
    } else {
        set_mavlink_message_id_interval(MAVLINK_MSG_ID_HIGH_LATENCY2, 5000);
    }
#else
    // this is O(n^2), but it's once at boot and across a 10-entry list...
    for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
        initialise_message_intervals_for_stream(all_stream_entries[i].stream_id);
    }
    set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT, 1000);
#endif
}

bool GCS_MAVLINK::get_default_interval_for_ap_message(const ap_message id, uint16_t &interval) const
{
    if (id == MSG_HEARTBEAT) {
        // handle heartbeat requests as a special case because heartbeat is not "streamed"
        interval = 1000;
        return true;
    }

#if HAL_HIGH_LATENCY2_ENABLED
    if (id == MSG_HIGH_LATENCY2) {
        // handle HL2 requests as a special case because HL2 is not "streamed"
        interval = 5000;
        return true;
    }
#endif

#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
    // a user can specify default rates in files, which are read close
    // to vehicle startup
    if (default_intervals_from_files != nullptr &&
        default_intervals_from_files->get_interval_for_ap_message_id(id, interval)) {
        return true;
    }
#endif

    // find which stream this ap_message is in
    for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
        const GCS_MAVLINK::stream_entries &entries = all_stream_entries[i];
        for (uint8_t j=0; j<entries.num_ap_message_ids; j++) {
            if (entries.ap_message_ids[j] == id) {
                interval = get_interval_for_stream(all_stream_entries[i].stream_id);
                return true;
            }
        }
    }
    return false;
}

/*
  correct an offboard timestamp in microseconds into a local timestamp
  since boot in milliseconds. See the JitterCorrection code for details

  Return a value in milliseconds since boot (for use by the EKF)
 */
uint32_t GCS_MAVLINK::correct_offboard_timestamp_usec_to_ms(uint64_t offboard_usec, uint16_t payload_size)
{
    uint64_t local_us;
    // if the HAL supports it then constrain the latest possible time
    // the packet could have been sent by the uart receive time and
    // the baudrate and packet size.
    uint64_t uart_receive_time = _port->receive_time_constraint_us(payload_size);
    if (uart_receive_time != 0) {
        local_us = uart_receive_time;
    } else {
        local_us = AP_HAL::micros64();
    }
    uint64_t corrected_us = lag_correction.correct_offboard_timestamp_usec(offboard_usec, local_us);

    return corrected_us / 1000U;
}

/*
  return true if we will accept this packet. Used to implement MAV_GCS_ENFRCE
 */
bool GCS_MAVLINK::accept_packet(const mavlink_status_t &status,
                                const mavlink_message_t &msg) const
{
    if (msg.sysid == mavlink_system.sysid) {
        // accept packets from our own components
        // (e.g. mavlink-connected companion computers)
        return true;
    }

    if (gcs().sysid_is_gcs(msg.sysid)) {
        return true;
    }

    if (msg.msgid == MAVLINK_MSG_ID_RADIO ||
        msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        return true;
    }

    if (!gcs().option_is_enabled(GCS::Option::GCS_SYSID_ENFORCE)) {
        return true;
    }

    return false;
}

/*
  update UART pass-thru, if enabled
 */
void GCS::update_passthru(void)
{
#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // examples don't have AP::serialmanager
    return;
#endif

    WITH_SEMAPHORE(_passthru.sem);
    uint32_t now = AP_HAL::millis();
    uint32_t baud1, baud2;
    bool enabled = AP::serialmanager().get_passthru(_passthru.port1, _passthru.port2, _passthru.timeout_s,
                                                    baud1, baud2);
    if (enabled && !_passthru.enabled) {
        _passthru.start_ms = now;
        _passthru.last_ms = 0;
        _passthru.enabled = true;
        _passthru.last_port1_data_ms = now;
        _passthru.baud1 = baud1;
        _passthru.baud2 = baud2;
        _passthru.parity1 = _passthru.port1->get_parity();
        _passthru.parity2 = _passthru.port2->get_parity();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Passthru enabled");
        if (!_passthru.timer_installed) {
            _passthru.timer_installed = true;
            hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&GCS::passthru_timer, void));
        }
    } else if (!enabled && _passthru.enabled) {
        _passthru.enabled = false;
        _passthru.port1->lock_port(0, 0);
        _passthru.port2->lock_port(0, 0);
        // Restore original baudrates
        if (_passthru.baud1 != baud1) {
            _passthru.port1->end();
            _passthru.port1->begin(baud1);
        }
        if (_passthru.baud2 != baud2) {
            _passthru.port2->end();
            _passthru.port2->begin(baud2);
        }
        // Restore original parity
        if (_passthru.parity1 != _passthru.port1->get_parity()) {
            _passthru.port1->configure_parity(_passthru.parity1);
        }
        if (_passthru.parity2 != _passthru.port2->get_parity()) {
            _passthru.port2->configure_parity(_passthru.parity2);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Passthru disabled");
    } else if (enabled &&
               _passthru.timeout_s &&
               now - _passthru.last_port1_data_ms > uint32_t(_passthru.timeout_s)*1000U) {
        // timed out, disable
        _passthru.enabled = false;
        _passthru.port1->lock_port(0, 0);
        _passthru.port2->lock_port(0, 0);
        AP::serialmanager().disable_passthru();
        // Restore original baudrates
        if (_passthru.baud1 != baud1) {
            _passthru.port1->end();
            _passthru.port1->begin(baud1);
        }
        if (_passthru.baud2 != baud2) {
            _passthru.port2->end();
            _passthru.port2->begin(baud2);
        }
        // Restore original parity
        if (_passthru.parity1 != _passthru.port1->get_parity()) {
            _passthru.port1->configure_parity(_passthru.parity1);
        }
        if (_passthru.parity2 != _passthru.port2->get_parity()) {
            _passthru.port2->configure_parity(_passthru.parity2);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Passthru timed out");
    }
}

/*
  called at 1kHz to handle pass-thru between SERIAL_PASS1 and SERIAL_PASS2 ports
 */
void GCS::passthru_timer(void)
{
    WITH_SEMAPHORE(_passthru.sem);

    if (!_passthru.enabled) {
        // it has been disabled after starting
        return;
    }
    if (_passthru.start_ms != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - _passthru.start_ms < 1000) {
            // delay for 1s so the reply for the SERIAL_PASS2 param set can be seen by GCS
            return;
        }
        _passthru.start_ms = 0;
        _passthru.port1->begin(_passthru.baud1);
        _passthru.port2->begin(_passthru.baud2);
    }

    // while pass-thru is enabled lock both ports. They remain
    // locked until disabled again, or reboot
    const uint32_t lock_key = 0x3256AB9F;
    _passthru.port1->lock_port(lock_key, lock_key);
    _passthru.port2->lock_port(lock_key, lock_key);

    // Check for requested Baud rates and parity over USB
    uint32_t baud = _passthru.port1->get_usb_baud();
    uint8_t parity = _passthru.port1->get_usb_parity();
    if (baud != 0) { // port1 is USB
        if (_passthru.baud2 != baud) {
            _passthru.baud2 = baud;
            _passthru.port2->end();
            _passthru.port2->begin_locked(baud, 0, 0, lock_key);
        }

        if (_passthru.parity2 != parity) {
            _passthru.parity2 = parity;
            _passthru.port2->configure_parity(parity);
        }
    }

    baud = _passthru.port2->get_usb_baud();
    parity = _passthru.port2->get_usb_parity();
    if (baud != 0) { // port2 is USB
        if (_passthru.baud1 != baud) {
            _passthru.baud1 = baud;
            _passthru.port1->end();
            _passthru.port1->begin_locked(baud, 0, 0, lock_key);
        }

        if (_passthru.parity1 != parity) {
            _passthru.parity1 = parity;
            _passthru.port1->configure_parity(parity);
        }
    }

    uint8_t buf[64];

    // read from port1, and write to port2
    int16_t nbytes = _passthru.port1->read_locked(buf, MIN(sizeof(buf),_passthru.port2->txspace()), lock_key);
    if (nbytes > 0) {
        _passthru.last_port1_data_ms = AP_HAL::millis();
        _passthru.port2->write_locked(buf, nbytes, lock_key);
    }

    // read from port2, and write to port1
    nbytes = _passthru.port2->read_locked(buf, MIN(sizeof(buf),_passthru.port1->txspace()), lock_key);
    if (nbytes > 0) {
        _passthru.port1->write_locked(buf, nbytes, lock_key);
    }
}

bool GCS_MAVLINK::mavlink_coordinate_frame_to_location_alt_frame(const MAV_FRAME coordinate_frame, Location::AltFrame &frame)
{
    switch (coordinate_frame) {
    case MAV_FRAME_GLOBAL_RELATIVE_ALT: // solo shot manager incorrectly sends RELATIVE_ALT instead of RELATIVE_ALT_INT
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        frame = Location::AltFrame::ABOVE_HOME;
        return true;
    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
        frame = Location::AltFrame::ABOVE_TERRAIN;
        return true;
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_INT:
        frame = Location::AltFrame::ABSOLUTE;
        return true;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Unknown mavlink coordinate frame %u", coordinate_frame);
#endif
        return false;
    }
}

uint64_t GCS_MAVLINK::capabilities() const
{
    uint64_t ret = MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
        MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;

    const auto mavlink_protocol = uartstate->get_protocol();
    if (mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLink2 || mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLinkHL) {
        ret |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    }

#if AP_ADVANCEDFAILSAFE_ENABLED
    AP_AdvancedFailsafe *failsafe = AP::advancedfailsafe();
    if (failsafe != nullptr && failsafe->enabled()) {
        // Copter and Sub may also set this bit as they can always terminate
        ret |= MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
    }
#endif

#if HAL_RALLY_ENABLED
    if (AP::rally()) {
        ret |= MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
    }
#endif

#if AP_FENCE_ENABLED
    if (AP::fence()) {
        ret |= MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
    }
#endif

#if AP_MAVLINK_FTP_ENABLED
    if (!AP_BoardConfig::ftp_disabled()){  //if ftp disable board option is not set
        ret |= MAV_PROTOCOL_CAPABILITY_FTP;
    }
#endif

    return ret;
}


#if AP_RC_CHANNEL_ENABLED
void GCS_MAVLINK::manual_override(RC_Channel *c, int16_t value_in, const uint16_t offset, const float scaler, const uint32_t tnow, const bool reversed)
{
    if (c == nullptr) {
        return;
    }
    int16_t override_value = 0;
    if (value_in != INT16_MAX) {
        const int16_t radio_min = c->get_radio_min();
        const int16_t radio_max = c->get_radio_max();
        if (reversed) {
            value_in *= -1;
        }
        override_value = radio_min + (radio_max - radio_min) * (value_in + offset) / scaler;
    }
    c->set_override(override_value, tnow);
}

void GCS_MAVLINK::handle_manual_control(const mavlink_message_t &msg)
{
    if (!gcs().sysid_is_gcs(msg.sysid)) {
        return; // only accept control from our gcs
    }

    mavlink_manual_control_t packet;
    mavlink_msg_manual_control_decode(&msg, &packet);

    if (packet.target != gcs().sysid_this_mav()) {
        return; // only accept control aimed at us
    }

    uint32_t tnow = AP_HAL::millis();

    handle_manual_control_axes(packet, tnow);

    // a manual control message is considered to be a 'heartbeat'
    // from the ground station for failsafe purposes
    sysid_mygcs_seen(tnow);
}
#endif  // AP_RC_CHANNEL_ENABLED

// called when valid traffic has been seen from our GCS
void GCS_MAVLINK::sysid_mygcs_seen(uint32_t seen_time_ms)
{
    gcs().sysid_mygcs_seen(seen_time_ms);
    _sysid_gcs_last_seen_time_ms = seen_time_ms;
}


#if AP_RSSI_ENABLED
uint8_t GCS_MAVLINK::receiver_rssi() const
{
    AP_RSSI *aprssi = AP::rssi();
    if (aprssi == nullptr) {
        return UINT8_MAX;
    }

    if (!aprssi->enabled()) {
        return UINT8_MAX;
    }

    // scale across the full valid range
    return aprssi->read_receiver_rssi() * 254;
}
#endif

GCS &gcs()
{
    return *GCS::get_singleton();
}

/*
  send HIGH_LATENCY2 message
 */
#if HAL_HIGH_LATENCY2_ENABLED
void GCS_MAVLINK::send_high_latency2() const
{
#if AP_AHRS_ENABLED
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));
#if AP_BATTERY_ENABLED
    const int8_t battery_remaining = battery_remaining_pct(AP_BATT_PRIMARY_INSTANCE);
#endif

    uint16_t current_waypoint = 0;
#if AP_MISSION_ENABLED
    AP_Mission *mission = AP::mission();
    if (mission != nullptr) {
        current_waypoint = mission->get_current_nav_index();
    }
#endif

    uint32_t present;
    uint32_t enabled;
    uint32_t health;
    gcs().get_sensor_status_flags(present, enabled, health);
    // Remap HL_FAILURE_FLAG from system status flags
    static const struct PACKED status_map_t {
        MAV_SYS_STATUS_SENSOR sensor;
        HL_FAILURE_FLAG failure_flag;
    } status_map[] {
        { MAV_SYS_STATUS_SENSOR_GPS, HL_FAILURE_FLAG_GPS },
        { MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE },
        { MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, HL_FAILURE_FLAG_ABSOLUTE_PRESSURE },
        { MAV_SYS_STATUS_SENSOR_3D_ACCEL, HL_FAILURE_FLAG_3D_ACCEL },
        { MAV_SYS_STATUS_SENSOR_3D_GYRO, HL_FAILURE_FLAG_3D_GYRO },
        { MAV_SYS_STATUS_SENSOR_3D_MAG, HL_FAILURE_FLAG_3D_MAG },
        { MAV_SYS_STATUS_TERRAIN, HL_FAILURE_FLAG_TERRAIN },
        { MAV_SYS_STATUS_SENSOR_BATTERY, HL_FAILURE_FLAG_BATTERY },
        { MAV_SYS_STATUS_SENSOR_RC_RECEIVER, HL_FAILURE_FLAG_RC_RECEIVER },
        { MAV_SYS_STATUS_GEOFENCE, HL_FAILURE_FLAG_GEOFENCE },
        { MAV_SYS_STATUS_AHRS, HL_FAILURE_FLAG_ESTIMATOR },
    };

    uint16_t failure_flags = 0;
    for (auto &map_entry : status_map) {
        if ((health & map_entry.sensor) == 0) {
            failure_flags |= map_entry.failure_flag;
        }
    }

    mavlink_msg_high_latency2_send(chan, 
        AP_HAL::millis(), //[ms] Timestamp (milliseconds since boot or Unix epoch)
        gcs().frame_type(), // Type of the MAV (quadrotor, helicopter, etc.)
        MAV_AUTOPILOT_ARDUPILOTMEGA, // Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
        gcs().custom_mode(), // A bitfield for use for autopilot-specific flags (2 byte version).
        global_position_current.lat, // [degE7] Latitude
        global_position_current.lng, // [degE7] Longitude
        global_position_current.alt * 0.01f, // [m] Altitude above mean sea level
        high_latency_target_altitude(), // [m] Altitude setpoint
        uint16_t(ahrs.get_yaw_deg()) / 2, // [deg/2] Heading
        high_latency_tgt_heading(), // [deg/2] Heading setpoint
        high_latency_tgt_dist(), // [dam] Distance to target waypoint or position
        abs(vfr_hud_throttle()), // [%] Throttle
        MIN(vfr_hud_airspeed() * 5, UINT8_MAX), // [m/s*5] Airspeed
        high_latency_tgt_airspeed(), // [m/s*5] Airspeed setpoint
        MIN(ahrs.groundspeed() * 5, UINT8_MAX), // [m/s*5] Groundspeed
        high_latency_wind_speed(), // [m/s*5] Windspeed
        high_latency_wind_direction(), // [deg/2] Wind heading
        0, // [dm] Maximum error horizontal position since last message
        0, // [dm] Maximum error vertical position since last message
        high_latency_air_temperature(), // [degC] Air temperature from airspeed sensor
        0, // [dm/s] Maximum climb rate magnitude since last message
#if AP_BATTERY_ENABLED
        battery_remaining, // [%] Battery level (-1 if field not provided).
#else
        -1,
#endif
        current_waypoint, // Current waypoint number
        failure_flags, // Bitmap of failure flags.
        base_mode(), // Field for custom payload. base mode (arming status) in ArduPilot's case
        0, // Field for custom payload.
        0); // Field for custom payload.
#endif
}

int8_t GCS_MAVLINK::high_latency_air_temperature() const
{
#if AP_AIRSPEED_ENABLED
    // return units are degC
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    float air_temperature;
    if (airspeed != nullptr && airspeed->enabled() && airspeed->get_temperature(air_temperature)) {
        return air_temperature;
    }
#endif

    return INT8_MIN;
}

/*
  handle a MAV_CMD_CONTROL_HIGH_LATENCY command 

  Enable or disable any marked (via SERIALn_PROTOCOL) high latency connections
 */
MAV_RESULT GCS_MAVLINK::handle_control_high_latency(const mavlink_command_int_t &packet)
{
    // high latency mode is enabled if param1=1 or disabled if param1=0
    if (is_equal(packet.param1, 0.0f)) {
        gcs().enable_high_latency_connections(false);
    } else if (is_equal(packet.param1, 1.0f)) {
        gcs().enable_high_latency_connections(true);
    } else {
        return MAV_RESULT_FAILED;
    }

    // send to all other mavlink components with same sysid
    mavlink_command_long_t hl_msg{};
    hl_msg.command = MAV_CMD_CONTROL_HIGH_LATENCY;
    hl_msg.param1 = packet.param1;
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&hl_msg, sizeof(hl_msg));

    return MAV_RESULT_ACCEPTED;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
void GCS_MAVLINK::handle_radio_rc_channels(const mavlink_message_t &msg)
{
    mavlink_radio_rc_channels_t packet;
    mavlink_msg_radio_rc_channels_decode(&msg, &packet);

    AP::RC().handle_radio_rc_channels(&packet);
}
#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#endif  // HAL_GCS_ENABLED
