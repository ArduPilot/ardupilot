#include "AP_LocationDB_config.h"

#if AP_LOCATIONDB_ENABLED

#include "AP_LocationDB.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdio.h>

#ifndef AP_LOCATIONDB_CAPACITY_DEFAULT
#define AP_LOCATIONDB_CAPACITY_DEFAULT 100
#endif

#define AP_LOCATIONDB_TIMEOUT_DEFAULT 10

const AP_Param::GroupInfo AP_LocationDB::var_info[] = {
    // @Param: CAPACITY
    // @DisplayName: Location Database maximum number of items
    // @Description: LocationDB maximum number of points. Set to 0 to disable the LocationDB. Larger means more points but is more cpu intensive to process
    // @Range: 0 10000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("CAPACITY", 1, AP_LocationDB, _database_capacity_param, AP_LOCATIONDB_CAPACITY_DEFAULT),

    // @Param: TIMEOUT
    // @DisplayName: Location Database item timeout
    // @Description: Time since last update after which the position of a location database item if invalidated
    // @Range: 0 300
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 2, AP_LocationDB, _item_timeout, AP_LOCATIONDB_TIMEOUT_DEFAULT),

    // @Param: INC_RADIUS
    // @DisplayName: Location database radius of inclusion
    // @Description: Maximum distance of a database item from the vehicle to be included in location database. Beyond this distance, the items are excluded.
    // @Range: 0 10000
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("INC_RADIUS", 4, AP_LocationDB, _inc_radius, 1000),

    AP_GROUPEND
};

AP_LocationDB::AP_LocationDB()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_LocationDB must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

}

// initialise location database
void AP_LocationDB::init()
{
#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // use default value for unit tests
    _capacity = AP_LOCATIONDB_CAPACITY_DEFAULT;
# else
    _capacity = _database_capacity_param;
#endif

    // return immediately if required capacity is zero
    if (_capacity == 0) {
        return;
    }

    // allocate array to store location database items
    _items = new AP_LocationDB_Item[_capacity];
    // this sets the current size to zero
    clear();

    if (!healthy()) {
    // just in case we fail to allocate memory
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LocationDB init failed . DB size: %u", _capacity);
        delete[] _items;
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LocationDB init successful . DB size: %u", _capacity);
}

void AP_LocationDB::update()
{
    if (!healthy()) {
        return;
    }

    // invalidate fields of timed out items
    for (int i=0; i<_size; i++) {
        AP_LocationDB_Item item;
        // move to next item if we fail to retrieve the current item
        if (!get_item_at_index(i, item)) {
            continue;
        }

        const uint32_t time_since_last_update_s = (AP_HAL::millis() - item.get_timestamp_ms()) / 1000;
        if ((time_since_last_update_s > (uint32_t)_item_timeout.get()) && item._populated_fields != 0) {
            IGNORE_RETURN(remove_item_at_index(i));
        }
    }
}

// return true if a new item is succesfully added to location database
bool AP_LocationDB::add_item(const AP_LocationDB_Item item)
{
    // return early if
    // the database if not healthy,
    // the database is full,
    // or the key for the new item is invalid
    if (!healthy() || is_full() || !is_valid_key(item.get_key())) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN) && AP_AHRS_ENABLED
    Vector3f item_pos;
    Vector3f our_pos;
    if (item.get_pos_cm_NEU(item_pos) && AP::ahrs().get_relative_position_NED_origin(our_pos)) {
        our_pos.z = -our_pos.z;
        item_pos = item_pos / 100; // cm to m
        // check if the item is inside the inclusion radius
        if ((item_pos - our_pos).length_squared() > (_inc_radius.get() * _inc_radius.get())) {
            // item very far away from the vehicle
            return false; 
        }
    }
#endif

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        // if the item with given key already exists, update it
        if (get_item_index(item.get_key(), item_idx)) {
            return update_item_at_index(item_idx, item);
        }
        // add new position otherwise
        _items[_size] = item;
        _size += 1;
    }

    return true;
}

// return true if we successfully retrieve the index of item with given key in location database
// the db_sem must be held before calling this method
bool AP_LocationDB::get_item_index(const uint32_t key, uint16_t &index)
{
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

    // sequentially match key with each item
    for (int i=0; i<_size; i++) {
        if (_items[i].get_key() == key) {
            index = i;
            return true;
        }
    }

    return false;
}

// return true if an item with given key exists in the location database
bool AP_LocationDB::item_exists(const uint32_t key)
{
    uint16_t idx;

    {
        WITH_SEMAPHORE(db_sem);
        // make sure we are able to retrive the item and some fields are populated
        // fields are not populated for timed out items
        if (get_item_index(key, idx) && _items[idx]._populated_fields != 0) {
            return true;
        }
    }

    return false;
}

// return true if we are able to retrieve item with given key from location database
bool AP_LocationDB::get_item(const uint32_t key, AP_LocationDB_Item &ret)
{
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        // get index of item with given key from location database
        if (get_item_index(key, item_idx)) {
            // populate the item if the index is found and return true
            ret = _items[item_idx];
            return true;
        }
    }

    return false;
}

// return true if we are able to retrieve item at a given index from the location database
bool AP_LocationDB::get_item_at_index(const uint16_t index, AP_LocationDB_Item &ret)
{
    if (!healthy() || index >= _size) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        // populate ret with the item at given index
        ret = _items[index];
    }

    return true;
}

bool AP_LocationDB::remove_item_at_index(const uint16_t index)
{
    // check if database is healthy and the passed index falls inside the current database size
    if (!healthy() || index >= _size) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        _size -= 1; // decrease the size of database by one

        if (_size != 0) {
            // bring the last element at the place of item to be removed
            // the items with index equal to or beyond database size are not considered to be a part of the database
            _items[index] = _items[_size];
        }
    }

    return true;
}

// return true if the item with given index is successfully removed from the location database
bool AP_LocationDB::remove_item(const uint32_t key)
{
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        // get index of the item to be removed and remove the item
        if (get_item_index(key, item_idx)) {
            return remove_item_at_index(item_idx);
        }
    }

    return false;
}

// return true if we successfully update the item at given index of location database
bool AP_LocationDB::update_item_at_index(const uint16_t index, const AP_LocationDB_Item &new_item)
{
    // check if database is healthy and the key for the new item is valid
    // also check if the passed index falls inside the current database size
    if (!healthy() || !is_valid_key(new_item.get_key()) || index >= _size) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        // copy new item at the place of the old item
        _items[index] = new_item;
    }

    return true;
}

// return true if we are able to update the item
bool AP_LocationDB::update_item(const uint32_t key, const AP_LocationDB_Item &new_item)
{
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN) && AP_AHRS_ENABLED
    Vector3f item_pos;
    Vector3f our_pos;
    if (new_item.get_pos_cm_NEU(item_pos) && AP::ahrs().get_relative_position_NED_origin(our_pos)) {
        our_pos.z = -our_pos.z;
        item_pos = item_pos / 100; // cm to m
        // check if the items is inside the inclusion radius
        if ((item_pos - our_pos).length_squared() > (_inc_radius.get() * _inc_radius.get())) {
            // item very far away from the vehicle
            return false; 
        }
    }
#endif

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        // get index of item to be updated
        // update item if the index is successfully retrieved
        if (get_item_index(key, item_idx)) {
            return update_item_at_index(item_idx, new_item);
        }
    }

    return false;
}

bool AP_LocationDB::is_valid_key(const uint32_t key)
{
    const KeyDomain key_domain = static_cast<KeyDomain>(key >> 24);

    // domain specific checks, if any
    switch (key_domain) {
#if AP_LOCATIONDB_KEYDOMAIN_MAVLINK_ENABLED
        case KeyDomain::MAVLINK:
        {
            bool sysid_ok = (((key >> 16) & 255U) != 0); // sysdid cannot be zero as its broadcast id
            bool short_msgid_ok = (key & 255U) != 255U; // short message id should not be 255, this means the mavlink msgid is not supported 
            return sysid_ok && short_msgid_ok;
        }
#endif
#if AP_LOCATIONDB_KEYDOMAIN_ADSB_ENABLED
        case KeyDomain::ADSB:
            return true;
#endif
#if AP_LOCATIONDB_KEYDOMAIN_SCRIPTING_ENABLED
        case KeyDomain::SCRIPTING:
            return true;
#endif
    }

    // we reach here only if the key is for an invalid domain
    return false;
}

// helper method to construct location database key for a mavlink item
uint32_t AP_LocationDB::construct_key_mavlink(const uint8_t sysid, const uint8_t compid, const uint8_t msgid)
{
    return (uint8_t)KeyDomain::MAVLINK << 24 | sysid << 16 | compid << 8 | short_mav_msg_id(msgid);
}

#if AP_LOCATIONDB_KEYDOMAIN_ADSB_ENABLED
// helper method to construct location database key for an adsb item
uint32_t AP_LocationDB::construct_key_adsb(const uint32_t icao)
{
    // first eight bits represent the domain
    // next 24 bits represent the icao number
    // the leading 8 bits of the icao number are ignored
    return (uint8_t)KeyDomain::ADSB << 24 | (icao & ((1U << 24) - 1U));
}
#endif

#if AP_LOCATIONDB_KEYDOMAIN_SCRIPTING_ENABLED
// helper method to construct location database key for a scripting d item
uint32_t AP_LocationDB::construct_key_scripting(const uint32_t subkey)
{
    // first eight bits represen the domain
    // next 24 bits represent the subkey
    return (uint8_t)KeyDomain::SCRIPTING << 24 | (subkey & ((1U << 24) - 1U));
}
#endif

// return 8 bits short message id for mavlink message
// this is used to accomodate message id in location database item key
// currently we can only support 255 different messages from a mavlink source
uint8_t AP_LocationDB::short_mav_msg_id(const uint32_t msgid)
{
    // sequentially assign short message ids to supported messages
    switch(msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        return 0U;
    case MAVLINK_MSG_ID_FOLLOW_TARGET:
        return 1U;
    };

    // return 255 if message not listed
    // is_key_valid() returns false for keys with this id
    return 255U;
}

// clear location database
void AP_LocationDB::clear()
{
    _size = 0;
}

AP_LocationDB_Item::AP_LocationDB_Item(const uint32_t key, const uint32_t timestamp_ms, const Vector3f pos_cm_NEU, const Vector3f vel_cm_NEU, const Vector3f acc_cm_NEU, const float heading_cdeg, const float radius_cm, const uint8_t populated_fields)
{
    init(key, timestamp_ms, pos_cm_NEU, vel_cm_NEU, acc_cm_NEU, heading_cdeg, radius_cm, populated_fields);
}

// initialise empty location database item with given data
void AP_LocationDB_Item::init(const uint32_t key, const uint32_t timestamp_ms, const Vector3f pos_cm_NEU, const Vector3f vel_cm_NEU, const Vector3f acc_cm_NEU, const float heading_cdeg, const float radius_cm, const uint8_t populated_fields)
{
    _key = key;
    _timestamp_ms = timestamp_ms;
    _pos = pos_cm_NEU;
    _vel = vel_cm_NEU;
    _acc = acc_cm_NEU;
    _heading = heading_cdeg;
    _radius = radius_cm;
    _populated_fields = populated_fields;
}

bool AP_LocationDB_Item::get_pos_cm_NEU(Vector3f &ret) const
{
    if (!field_is_populated(DataField::POS)) {
        return false;
    }

    ret = _pos;
    return true;
}

bool AP_LocationDB_Item::get_vel_cm_NEU(Vector3f &ret) const
{
    if (!field_is_populated(DataField::VEL)) {
        return false;
    }

    ret = _vel;
    return true;
}

bool AP_LocationDB_Item::get_acc_cm_NEU(Vector3f &ret) const
{
    if (!field_is_populated(DataField::ACC)) {
        return false;
    }

    ret = _acc;
    return true;
}

bool AP_LocationDB_Item::get_heading_cdeg(float &ret) const
{
    if (!field_is_populated(DataField::HEADING)) {
        return false;
    }

    ret = _heading;
    return true;
}

bool AP_LocationDB_Item::get_radius_cm(float &ret) const
{
    if (!field_is_populated(DataField::RADIUS)) {
        return false;
    }

    ret = _radius;
    return true;
}

assert_storage_size<AP_LocationDB_Item, 56> _assert_storage_size_AP_LocationDB_Item;

// singleton instance
AP_LocationDB *AP_LocationDB::_singleton;

namespace AP {
    AP_LocationDB *locationdb() {
        return AP_LocationDB::get_singleton();
    }
}

#endif  // AP_LOCATIONDB_ENABLED
