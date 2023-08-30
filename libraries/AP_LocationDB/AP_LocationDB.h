#pragma once

#include "AP_LocationDB_config.h"

#if AP_LOCATIONDB_ENABLED

#include <AP_HAL/Semaphores.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>

class AP_LocationDB_Item {
public:
    friend class AP_Filesystem_LocationDB;
    friend class AP_LocationDB;

    AP_LocationDB_Item() {};
    AP_LocationDB_Item(const uint32_t key, const uint32_t timestamp_ms, const Vector3f pos_cm_NEU, const Vector3f vel_cm_NEU, const Vector3f acc_cm_NEU, const float heading_cdeg, const float radius_cm, const uint8_t populated_fields);

    // to initialise an empty database item with given data
    void init(const uint32_t key, const uint32_t timestamp_ms, const Vector3f pos_cm_NEU, const Vector3f vel_cm_NEU, const Vector3f acc_cm_NEU, const float heading_cdeg, const float radius_cm, const uint8_t populated_fields);
    
    uint32_t get_timestamp_ms() const { return _timestamp_ms; }
    uint32_t get_key() const { return _key; }
    bool get_pos_cm_NEU(Vector3f &ret) const;
    bool get_vel_cm_NEU(Vector3f &ret) const;
    bool get_acc_cm_NEU(Vector3f &ret) const;
    bool get_heading_cdeg(float &heading) const;
    bool get_radius_cm(float &ret) const;

    enum class DataField : uint8_t {
        POS = (1U << 0),
        VEL = (1U << 1),
        ACC = (1U << 2),
        HEADING = (1U << 3),
        RADIUS = (1U << 4),
    };

    enum class Flag : uint8_t {
        AVOID = (1U << 0),
    };

private:
    uint32_t _key;
    uint32_t _timestamp_ms;
    Vector3f _pos;
    Vector3f _vel;
    Vector3f _acc;
    float _heading;
    float _radius;
    uint8_t _flags;

    uint8_t _populated_fields = 0;

    bool field_is_populated(DataField option) const {
        return (_populated_fields & (uint8_t)option) != 0;
    }
};

class AP_LocationDB {
public:
    friend class AP_Filesystem_LocationDB;

    AP_LocationDB();

    CLASS_NO_COPY(AP_LocationDB);

    static AP_LocationDB *get_singleton() {
        return _singleton;
    }

    void init();
    void update();

    enum class KeyDomain : uint8_t {
        // 0 should not be used
#if AP_LOCATIONDB_KEYDOMAIN_MAVLINK_ENABLED
        MAVLINK = 1U,
#endif
#if AP_LOCATIONDB_KEYDOMAIN_ADSB_ENABLED
        ADSB = 2U,
#endif
#if AP_LOCATIONDB_KEYDOMAIN_SCRIPTING_ENABLED
        SCRIPTING = 3U,
#endif
    };

    bool add_item(const AP_LocationDB_Item item); // add item to the location database
    bool get_item(const uint32_t key, AP_LocationDB_Item &ret); // get item with given key from location database
    bool update_item(const uint32_t key, const AP_LocationDB_Item &new_item); // update item with given key in location database
    bool remove_item(const uint32_t key); // remove item with given key from location database
    bool healthy() const { return _items != nullptr; } // returns true when memory is allocated to store database items
    bool is_full() const { return _capacity == _size; }; // returns true when database is full
    void clear(); // clear location database, i.e., remove all items
    uint16_t size() const { return _size; } // return number of items present in location database
    uint16_t capacity() const { return _capacity; } // return the capacity of location database
    bool item_exists(const uint32_t key); // return true when item with given key exists in the location database
    bool get_item_at_index(const uint16_t index, AP_LocationDB_Item &ret); // get item at given index from location database

    static bool is_valid_key(const uint32_t key); // return true if a given key is a valid location database key
    static uint32_t construct_key_mavlink(const uint8_t sysid, const uint8_t compid, const uint8_t msgid); // helper method to construct location database key for a mavlink item
#if AP_LOCATIONDB_KEYDOMAIN_ADSB_ENABLED
    static uint32_t construct_key_adsb(const uint32_t icao); // helper method to construct location database key for a adsb item
#endif
#if AP_LOCATIONDB_KEYDOMAIN_SCRIPTING_ENABLED
    static uint32_t construct_key_scripting(const uint32_t subkey); // helper method to construct location database key for a scripting item
#endif
    static uint8_t short_mav_msg_id(const uint32_t msgid); // shortened 8 bit message id for a mavlink message

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // parameters
    AP_Int16        _database_capacity_param;                   // db capacity
    AP_Int8         _default_flags;                             // default flags on new database item
    AP_Int16        _item_timeout;                              // timeout for db item
    AP_Int16        _inc_radius;                                // inclusion radius

private:

    uint16_t _capacity;
    uint16_t _size;
    AP_LocationDB_Item* _items;
    HAL_Semaphore db_sem;

    bool get_item_index(const uint32_t key, uint16_t &index); // get index of a database item with given key
    bool update_item_at_index(const uint16_t index, const AP_LocationDB_Item &item); // update item at given index in location database
    bool remove_item_at_index(const uint16_t index); // remove item at given index in location database

    static AP_LocationDB *_singleton;
};

namespace AP {
    AP_LocationDB *locationdb();
}

#endif
