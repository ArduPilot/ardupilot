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
 */
/*
  DroneCAN support for OpenDroneID
 */

#include "AP_OpenDroneID.h"

#if AP_OPENDRONEID_ENABLED

#if AP_OPENDRONEID_DRONECAN_ENABLED
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <dronecan/remoteid/Location.hpp>
#include <dronecan/remoteid/BasicID.hpp>
#include <dronecan/remoteid/SelfID.hpp>
#include <dronecan/remoteid/OperatorID.hpp>
#include <dronecan/remoteid/System.hpp>
#include <dronecan/remoteid/ArmStatus.hpp>

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

static uavcan::Publisher<dronecan::remoteid::Location>* dc_location[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<dronecan::remoteid::BasicID>* dc_basic_id[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<dronecan::remoteid::SelfID>* dc_self_id[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<dronecan::remoteid::System>* dc_system[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<dronecan::remoteid::OperatorID>* dc_operator_id[HAL_MAX_CAN_PROTOCOL_DRIVERS];

/*
  backport helpers
 */

// send text when we do have a GCS
#ifndef GCS_SEND_TEXT
#define GCS_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)
#endif

/*
  strncpy without the warning for not leaving room for nul termination
 */
static void strncpy_noterm(char *dest, const char *src, size_t n)
{
    size_t len = strnlen(src, n);
    if (len < n) {
        // include nul term if it fits
        len++;
    }
    memcpy(dest, src, len);
}


// handle ArmStatus
UC_REGISTRY_BINDER(ArmStatusCb, dronecan::remoteid::ArmStatus);
static uavcan::Subscriber<dronecan::remoteid::ArmStatus, ArmStatusCb> *arm_status_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

static void handle_arm_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ArmStatusCb &cb);

void AP_OpenDroneID::dronecan_init(AP_UAVCAN *uavcan)
{
    const uint8_t driver_index = uavcan->get_driver_index();
    const uint8_t driver_mask = 1U<<driver_index;
    if (dronecan_done_init & driver_mask) {
        // already initialised
        return;
    }

    dc_location[driver_index] = new uavcan::Publisher<dronecan::remoteid::Location>(*uavcan->get_node());
    if (dc_location[driver_index] == nullptr) {
        goto alloc_failed;
    }
    dc_location[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    dc_location[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    dc_basic_id[driver_index] = new uavcan::Publisher<dronecan::remoteid::BasicID>(*uavcan->get_node());
    if (dc_basic_id[driver_index] == nullptr) {
        goto alloc_failed;
    }
    dc_basic_id[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    dc_basic_id[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    dc_self_id[driver_index] = new uavcan::Publisher<dronecan::remoteid::SelfID>(*uavcan->get_node());
    if (dc_self_id[driver_index] == nullptr) {
        goto alloc_failed;
    }
    dc_self_id[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    dc_self_id[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    dc_system[driver_index] = new uavcan::Publisher<dronecan::remoteid::System>(*uavcan->get_node());
    if (dc_system[driver_index] == nullptr) {
        goto alloc_failed;
    }
    dc_system[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    dc_system[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    dc_operator_id[driver_index] = new uavcan::Publisher<dronecan::remoteid::OperatorID>(*uavcan->get_node());
    if (dc_operator_id[driver_index] == nullptr) {
        goto alloc_failed;
    }
    dc_operator_id[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    dc_operator_id[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    arm_status_listener[driver_index] = new uavcan::Subscriber<dronecan::remoteid::ArmStatus, ArmStatusCb>(*uavcan->get_node());
    if (arm_status_listener[driver_index] == nullptr) {
        goto alloc_failed;
    }
    arm_status_listener[driver_index]->start(ArmStatusCb(uavcan, &handle_arm_status));

    dronecan_done_init |= driver_mask;
    return;

alloc_failed:
    dronecan_init_failed |= driver_mask;
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "OpenDroneID DroneCAN alloc failed");
}

/*
  send pending DroneCAN OpenDroneID packets
 */
void AP_OpenDroneID::dronecan_send(AP_UAVCAN *uavcan)
{
    const uint8_t driver_index = uavcan->get_driver_index();
    const uint8_t driver_mask = 1U<<driver_index;

    if (driver_index+1 != _can_driver) {
        // not enabled for this CAN driver
        return;
    }

    dronecan_init(uavcan);

    if (dronecan_init_failed & driver_mask) {
        return;
    }

    if (need_send_basic_id & driver_mask) {
        WITH_SEMAPHORE(_sem);
        dronecan_send_basic_id(uavcan);
        need_send_basic_id &= ~driver_mask;
    }
    if (need_send_system & driver_mask) {
        WITH_SEMAPHORE(_sem);
        dronecan_send_system(uavcan);
        need_send_system &= ~driver_mask;
    }
    if (need_send_self_id & driver_mask) {
        WITH_SEMAPHORE(_sem);
        dronecan_send_self_id(uavcan);
        need_send_self_id &= ~driver_mask;
    }
    if (need_send_operator_id & driver_mask) {
        WITH_SEMAPHORE(_sem);
        dronecan_send_operator_id(uavcan);
        need_send_operator_id &= ~driver_mask;
    }
    if (need_send_location & driver_mask) {
        WITH_SEMAPHORE(_sem);
        dronecan_send_location(uavcan);
        need_send_location &= ~driver_mask;
    }
}

#define ODID_COPY(name) msg.name = pkt.name
#define ODID_COPY_STR(name) do { for (uint8_t i = 0; i<sizeof(pkt.name) && pkt.name[i]; i++) msg.name.push_back(pkt.name[i]); } while(0)


void AP_OpenDroneID::dronecan_send_location(AP_UAVCAN *uavcan)
{
    dronecan::remoteid::Location msg {};
    const auto &pkt = pkt_location;
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(status);
    ODID_COPY(direction);
    ODID_COPY(speed_horizontal);
    ODID_COPY(speed_vertical);
    ODID_COPY(latitude);
    ODID_COPY(longitude);
    ODID_COPY(altitude_barometric);
    ODID_COPY(altitude_geodetic);
    ODID_COPY(height_reference);
    ODID_COPY(height);
    ODID_COPY(horizontal_accuracy);
    ODID_COPY(vertical_accuracy);
    ODID_COPY(barometer_accuracy);
    ODID_COPY(speed_accuracy);
    ODID_COPY(timestamp);
    ODID_COPY(timestamp_accuracy);
    dc_location[uavcan->get_driver_index()]->broadcast(msg);
}

void AP_OpenDroneID::dronecan_send_basic_id(AP_UAVCAN *uavcan)
{
    dronecan::remoteid::BasicID msg {};
    const auto &pkt = pkt_basic_id;
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(id_type);
    ODID_COPY(ua_type);
    ODID_COPY_STR(uas_id);
    dc_basic_id[uavcan->get_driver_index()]->broadcast(msg);
}

void AP_OpenDroneID::dronecan_send_system(AP_UAVCAN *uavcan)
{
    dronecan::remoteid::System msg {};
    const auto &pkt = pkt_system;
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(operator_location_type);
    ODID_COPY(classification_type);
    ODID_COPY(operator_latitude);
    ODID_COPY(operator_longitude);
    ODID_COPY(area_count);
    ODID_COPY(area_radius);
    ODID_COPY(area_ceiling);
    ODID_COPY(area_floor);
    ODID_COPY(category_eu);
    ODID_COPY(class_eu);
    ODID_COPY(operator_altitude_geo);
    ODID_COPY(timestamp);
    dc_system[uavcan->get_driver_index()]->broadcast(msg);
}

void AP_OpenDroneID::dronecan_send_self_id(AP_UAVCAN *uavcan)
{
    dronecan::remoteid::SelfID msg {};
    const auto &pkt = pkt_self_id;
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(description_type);
    ODID_COPY_STR(description);
    dc_self_id[uavcan->get_driver_index()]->broadcast(msg);
}

void AP_OpenDroneID::dronecan_send_operator_id(AP_UAVCAN *uavcan)
{
    dronecan::remoteid::OperatorID msg {};
    const auto &pkt = pkt_operator_id;
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(operator_id_type);
    ODID_COPY_STR(operator_id);
    dc_operator_id[uavcan->get_driver_index()]->broadcast(msg);
}

/*
  handle ArmStatus message from DroneCAN
 */
static void handle_arm_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ArmStatusCb &cb)
{
    const auto &msg = *cb.msg;
    mavlink_open_drone_id_arm_status_t status {};

    status.status = msg.status;
    strncpy_noterm(status.error, msg.error.c_str(), sizeof(status.error));

    AP::opendroneid().set_arm_status(status);

    // Push DroneCAN Arm Message to GCS
    gcs().send_to_active_channels(MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS,(const char*)&status);
}

// copy arm status for DroneCAN
void AP_OpenDroneID::set_arm_status(mavlink_open_drone_id_arm_status_t &status)
{
    WITH_SEMAPHORE(_sem);
    arm_status = status;
    last_arm_status_ms = AP_HAL::millis();
}

#endif // AP_OPENDRONEID_DRONECAN_ENABLED
#endif // AP_OPENDRONEID_ENABLED
