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

#include "AP_OADatabase.h"

#if !HAL_MINIMIZE_FEATURES

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT
    #define AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT   10
#endif

#ifndef AP_OADATABASE_SIZE_DEFAULT
    #define AP_OADATABASE_SIZE_DEFAULT          100
#endif

#ifndef AP_OADATABASE_QUEUE_SIZE_DEFAULT
    #define AP_OADATABASE_QUEUE_SIZE_DEFAULT 80
#endif


const AP_Param::GroupInfo AP_OADatabase::var_info[] = {

    // @Param: SIZE
    // @DisplayName: OADatabase maximum number of points
    // @Description: OADatabase maximum number of points. Set to 0 to disable the OA Database. Larger means more points but is more cpu intensive to process
    // @Range: 0 10000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SIZE", 1, AP_OADatabase, _database_size_param, AP_OADATABASE_SIZE_DEFAULT),

    // @Param: EXPIRE
    // @DisplayName: OADatabase item timeout
    // @Description: OADatabase item timeout. The time an item will linger without any updates before it expires. Zero means never expires which is useful for a sent-once static environment but terrible for dynamic ones.
    // @Units: s
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXPIRE", 2, AP_OADatabase, _database_expiry_seconds, AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT),

    // @Param: QUEUE_SIZE
    // @DisplayName: OADatabase queue maximum number of points
    // @Description: OADatabase queue maximum number of points. This in an input buffer size. Larger means it can handle larger bursts of incoming data points to filter into the database. No impact on cpu, only RAM. Recommend larger for faster datalinks or for sensors that generate a lot of data.
    // @Range: 1 200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("QUEUE_SIZE", 3, AP_OADatabase, _queue_size_param, AP_OADATABASE_QUEUE_SIZE_DEFAULT),

    // @Param: OUTPUT
    // @DisplayName: OADatabase output level
    // @Description: OADatabase output level to configure which database objects are sent to the ground station. All data is always available internally for avoidance algorithms.
    // @Values: 0:Disabled,1:Send only HIGH importance items,2:Send HIGH and NORMAL importance items,3:Send all items
    // @User: Advanced
    AP_GROUPINFO("OUTPUT", 4, AP_OADatabase, _output_level, (float)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_HIGH),

    AP_GROUPEND
};

AP_OADatabase::AP_OADatabase()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OADatabase must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OADatabase::init()
{
    init_database();
    init_queue();

    if (!healthy()) {
        gcs().send_text(MAV_SEVERITY_INFO, "DB init failed . Sizes queue:%u, db:%u", (unsigned int)_queue.size, (unsigned int)_database.size);
        delete _queue.items;
        delete[] _database.items;
        return;
    }
}

void AP_OADatabase::update()
{
    if (!healthy()) {
        return;
    }

    process_queue();
    optimize_db_filter();
    database_items_remove_all_expired();
}

// push a location into the database
void AP_OADatabase::queue_push(const Location &loc, const uint32_t timestamp_ms, const float distance, const float angle)
{
    if (!healthy()) {
        return;
    }

    AP_OADatabase::OA_DbItemImportance importance = AP_OADatabase::OA_DbItemImportance::Normal;

    if (distance <= 0 || angle < 0 || angle > 360) {
        // sanity check
        importance = AP_OADatabase::OA_DbItemImportance::Normal;

    } else if (distance < 10 && (angle > (360-10) || angle < 10)) {
        // far and directly in front +/- 10deg
        importance = AP_OADatabase::OA_DbItemImportance::High;
    } else if (distance < 5 && (angle > (360-30) || angle < 30)) {
        // kinda far and forward of us +/- 30deg
        importance = AP_OADatabase::OA_DbItemImportance::High;
    } else if (distance < 3 && (angle > (360-90) || angle < 90)) {
        // near and ahead +/- 90deg
        importance = AP_OADatabase::OA_DbItemImportance::High;
    } else if (distance < 1.5) {
        // very close anywhere
        importance = AP_OADatabase::OA_DbItemImportance::High;

    } else if  (distance >= 10) {
        // really far away
        importance = AP_OADatabase::OA_DbItemImportance::Low;
    } else if (distance < 5 && (angle <= (360-90) || angle >= 90)) {
        // kinda far and behind us
        importance = AP_OADatabase::OA_DbItemImportance::Low;
    }

    const OA_DbItem item = {loc, timestamp_ms, 0, 0, importance};
    {
        WITH_SEMAPHORE(_queue.sem);
        _queue.items->push(item);
    }
}

void AP_OADatabase::init_queue()
{
    _queue.size = _queue_size_param;
    if (_queue.size == 0) {
        return;
    }

    _queue.items = new ObjectBuffer<OA_DbItem>(_queue.size);
}

void AP_OADatabase::init_database()
{
    _database.size = _database_size_param;
    if (_database_size_param == 0) {
        return;
    }

    _database.items = new OA_DbItem[_database.size];
}

void AP_OADatabase::optimize_db_filter()
{
    // TODO: check database size and if we're getting full
    // we should grow the database size and/or increase
    // _database_filter_m so less objects go into it and let
    // the existing ones timeout naturally

    const float filter_m_backup = _database.filter_m;

    if (_database.count > (_database.size * 0.90f)) {
        // we're almost full, lets increase the filter size by requiring more
        // spacing between points so less things get put into the database
        _database.filter_m = MIN(_database.filter_m*_database.filter_grow_rate, _database.filter_max_m);

    } else if (_database.count < (_database.size * 0.85f)) {
        // we have some room, lets loosen the filter requirement (smaller object points) to allow more samples
        _database.filter_m = MAX(_database.filter_m*_database.filter_shrink_rate,_database.filter_min_m);
    }

    // recompute the the radius filters
    if (!is_equal(filter_m_backup,_database.filter_m)) {
        _radius_importance_low = MIN(_database.filter_m*4,_database.filter_max_m);
        _radius_importance_normal = _database.filter_m;
        _radius_importance_high = MAX(_database.filter_m*0.25,_database.filter_min_m);
    }
}

// get bitmask of gcs channels item should be sent to based on its importance
// returns 0xFF (send to all channels) if should be sent, 0 if it should not be sent
uint8_t AP_OADatabase::get_send_to_gcs_flags(const OA_DbItemImportance importance)
{
    switch (importance) {
    case OA_DbItemImportance::Low:
        if (_output_level.get() >= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_ALL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::Normal:
        if (_output_level.get() >= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_HIGH_AND_NORMAL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::High:
        if (_output_level.get() >= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_HIGH) {
            return 0xFF;
        }
        break;
    }
    return 0x0;
}

float AP_OADatabase::get_radius(const OA_DbItemImportance importance)
{
    switch (importance) {
    case OA_DbItemImportance::Low:
        return _radius_importance_low;

    default:
    case OA_DbItemImportance::Normal:
        return _radius_importance_normal;

    case OA_DbItemImportance::High:
        return _radius_importance_high;
    }
}

// returns true when there's more work inthe queue to do
bool AP_OADatabase::process_queue()
{
    if (!healthy()) {
        return false;
    }

    // processing queue by moving those entries into the database
    // Using a for with fixed size is better than while(!empty) because the
    // while could get us stuck here longer than expected if we're getting
    // a lot of values pushing into it while we're trying to empty it. With
    // the for we know we will exit at an expected time
    const uint16_t queue_available = MIN(_queue.items->available(), 100U);
    if (queue_available == 0) {
        return false;
    }

    for (uint16_t queue_index=0; queue_index<queue_available; queue_index++) {
        OA_DbItem item;

        bool pop_success;
        {
            WITH_SEMAPHORE(_queue.sem);
            pop_success = _queue.items->pop(item);
        }
        if (!pop_success) {
            return false;
        }

        item.radius = get_radius(item.importance);
        item.send_to_gcs = get_send_to_gcs_flags(item.importance);

        // compare item to all items in database. If found a similar item, update the existing, else add it as a new one
        bool found = false;
        for (uint16_t i=0; i<_database.count; i++) {
            if (is_close_to_item_in_database(i, item)) {
                database_item_refresh(i, item.timestamp_ms, item.radius);
                found = true;
                break;
            }
        }

        if (!found) {
            database_item_add(item);
        }
    }
    return (_queue.items->available() > 0);
}

void AP_OADatabase::database_item_add(const OA_DbItem &item)
{
    if (_database.count >= _database.size) {
        return;
    }
    _database.items[_database.count] = item;
    _database.items[_database.count].send_to_gcs = get_send_to_gcs_flags(_database.items[_database.count].importance);
    _database.count++;
}

void AP_OADatabase::database_item_remove(const uint16_t index)
{
    if (index >= _database.count || _database.count == 0) {
        // index out of range
        return;
    }

    // radius of 0 tells the GCS we don't care about it any more (aka it expired)
    _database.items[index].radius = 0;
    _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);

    _database.count--;
    if (_database.count == 0) {
        return;
    }

    if (index != _database.count) {
        // copy last object in array over expired object
        _database.items[index] = _database.items[_database.count];
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

void AP_OADatabase::database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius)
{
    if (index >= _database.count) {
        // index out of range
        return;
    }

    const bool is_different =
            (!is_equal(_database.items[index].radius, radius)) ||
            (timestamp_ms - _database.items[index].timestamp_ms >= 500);

    if (is_different) {
        // update timestamp and radius on close object so it stays around longer
        // and trigger resending to GCS
        _database.items[index].timestamp_ms = timestamp_ms;
        _database.items[index].radius = radius;
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

void AP_OADatabase::database_items_remove_all_expired()
{
    // calculate age of all items in the _database

    if (_database_expiry_seconds <= 0) {
        // zero means never expire. This is not normal behavior but perhaps you could send a static
        // environment once that you don't want to have to constantly update
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t expiry_ms = (uint32_t)_database_expiry_seconds * 1000;
    uint16_t index = 0;
    while (index < _database.count) {
        if (now_ms - _database.items[index].timestamp_ms > expiry_ms) {
            database_item_remove(index);
        } else {
            // overtime, the item radius will grow in size. If too big, remove it before the timer
            _database.items[index].radius *= _database.radius_grow_rate;
            if (_database.items[index].radius >= _database.filter_max_m) {
                database_item_remove(index);
            } else {
                index++;
            }
        }
    }
}

// returns true if a similar object already exists in database. When true, the object timer is also reset
bool AP_OADatabase::is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const
{
    if (index >= _database.count) {
        // index out of range
        return false;
    }

    return (_database.items[index].loc.get_distance(item.loc) < item.radius);
}

// send ADSB_VEHICLE mavlink messages
void AP_OADatabase::send_adsb_vehicle(mavlink_channel_t chan, uint16_t interval_ms)
{
    // ensure database's send_to_gcs field is large enough
    static_assert(MAVLINK_COMM_NUM_BUFFERS <= sizeof(OA_DbItem::send_to_gcs) * 8,
                  "AP_OADatabase's OA_DBItem.send_to_gcs bitmask must be large enough to hold MAVLINK_COMM_NUM_BUFFERS");

    if ((_output_level.get() <= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_DISABLED) || !healthy()) {
        return;
    }

    const uint8_t chan_as_bitmask = 1 << chan;
    const char callsign[9] = "OA_DB";

    // calculate how many messages we should send
    const uint32_t now_ms = AP_HAL::millis();
    uint16_t num_to_send = 1;
    uint16_t num_sent = 0;
    if ((_last_send_to_gcs_ms[chan] != 0) && (interval_ms > 0)) {
        uint32_t diff_ms = now_ms - _last_send_to_gcs_ms[chan];
        num_to_send = MAX(diff_ms / interval_ms, 1U);
    }
    _last_send_to_gcs_ms[chan] = now_ms;

    // send unsent objects until output buffer is full or have sent enough
    for (uint16_t i=0; i < _database.count; i++) {
        if (!HAVE_PAYLOAD_SPACE(chan, ADSB_VEHICLE) || (num_sent >= num_to_send)) {
            // all done for now
            return;
        }

        const uint16_t idx = _next_index_to_send[chan];

        // prepare to send next object
        _next_index_to_send[chan]++;
        if (_next_index_to_send[chan] >= _database.count) {
            _next_index_to_send[chan] = 0;
        }

        if ((_database.items[idx].send_to_gcs & chan_as_bitmask) == 0) {
            continue;
        }

        mavlink_msg_adsb_vehicle_send(chan,
            idx,
            _database.items[idx].loc.lat,
            _database.items[idx].loc.lng,
            0,                          // altitude_type
            _database.items[idx].loc.alt,
            0,                          // heading
            0,                          // hor_velocity
            0,                          // ver_velocity
            callsign,                   // callsign
            255,                        // emitter_type
            0,                          // tslc
            0,                          // flags
            (uint16_t)(_database.items[idx].radius * 100.f));   // squawk

        // unmark item for sending to gcs
        _database.items[idx].send_to_gcs &= ~chan_as_bitmask;

        // update highest index sent to GCS
        _highest_index_sent[chan] = MAX(idx, _highest_index_sent[chan]);

        // update count sent
        num_sent++;
    }

    // clear expired items in case the database size shrank
    while (_highest_index_sent[chan] > _database.count) {
        if (!HAVE_PAYLOAD_SPACE(chan, ADSB_VEHICLE) || (num_sent >= num_to_send)) {
            // all done for now
            return;
        }

        const uint16_t idx = _highest_index_sent[chan];
        _highest_index_sent[chan]--;

        if (_database.items[idx].importance != OA_DbItemImportance::High) {
            continue;
        }

        mavlink_msg_adsb_vehicle_send(chan,
            idx,        // id
            0,          // latitude
            0,          // longitude
            0,          // altitude_type
            0,          // altitude
            0,          // heading
            0,          // hor_velocity
            0,          // ver_velocity
            callsign,   // callsign
            255,        // emitter_type
            0,          // tslc
            0,          // flags
            0);         // squawk

        // update count sent
        num_sent++;
    }
}

// singleton instance
AP_OADatabase *AP_OADatabase::_singleton;
#endif //!HAL_MINIMIZE_FEATURES

namespace AP {
AP_OADatabase *oadatabase()
{
    return AP_OADatabase::get_singleton();
}

}


