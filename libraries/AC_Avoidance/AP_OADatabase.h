#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#if !HAL_MINIMIZE_FEATURES
#include <AP_Param/AP_Param.h>

class AP_OADatabase {
public:

    AP_OADatabase();

    /* Do not allow copies */
    AP_OADatabase(const AP_OADatabase &other) = delete;
    AP_OADatabase &operator=(const AP_OADatabase&) = delete;

    // get singleton instance
    static AP_OADatabase *get_singleton() {
        return _singleton;
    }

    enum OA_DbItemImportance {
        Low, Normal, High
    };

    struct OA_DbItem {
        Location loc;           // location of object. TODO: turn this into Vector2Int to save memory
        uint32_t timestamp_ms;  // system time that object was last updated
        float radius;           // objects radius in meters
        uint8_t send_to_gcs;    // bitmask of mavlink comports to which details of this object should be sent
        OA_DbItemImportance importance;
    };

    void init();
    void update();

    // push a location into the database
    void queue_push(const Location &loc, const uint32_t timestamp_ms, const float distance, const float angle);

    // returns true if database is healthy
    bool healthy() const { return (_queue.items != nullptr) && (_database.items != nullptr); }

    // fetch an item in database. Undefined result when i >= _database.count.
    const OA_DbItem& get_item(uint32_t i) const { return _database.items[i]; }

    // get radius (in meters) of objects in database
    float get_accuracy() const { return _database.filter_m; }

    // get number of items in the database
    uint16_t database_count() const { return _database.count; }

    // empty queue and try and put into database. Return true if there's more work to do
    bool process_queue();

    // send ADSB_VEHICLE mavlink messages
    void send_adsb_vehicle(mavlink_channel_t chan, uint16_t interval_ms);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // initialise
    void init_queue();
    void init_database();

    // check queue and database sizes and adjust filter criteria to optimize use
    void optimize_db_filter();

    // database item management
    void database_item_add(const OA_DbItem &item);
    void database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius);
    void database_item_remove(const uint16_t index);
    void database_items_remove_all_expired();

    // get bitmask of gcs channels item should be sent to based on its importance
    // returns 0xFF (send to all channels) if should be sent or 0 if it should not be sent
    uint8_t get_send_to_gcs_flags(const OA_DbItemImportance importance);

    // used to determine the filter radius
    float get_radius(const OA_DbItemImportance importance);

    // returns true if database item "index" is close to "item"
    bool is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const;

    // enum for use with _OUTPUT parameter
    enum class OA_DbOutputLevel {
        OUTPUT_LEVEL_DISABLED = 0,
        OUTPUT_LEVEL_SEND_HIGH = 1,
        OUTPUT_LEVEL_SEND_HIGH_AND_NORMAL = 2,
        OUTPUT_LEVEL_SEND_ALL = 3
    };

    // parameters
    AP_Int16        _queue_size_param;                      // queue size
    AP_Int16        _database_size_param;                   // db size
    AP_Int8         _database_expiry_seconds;               // objects expire after this timeout
    AP_Int8         _output_level;                          // controls which items should be sent to GCS

    struct {
        ObjectBuffer<OA_DbItem> *items;                     // thread safe incoming queue of points from proximity sensor to be put into database
        uint16_t        size;                               // cached value of _queue_size_param.
        HAL_Semaphore   sem;                                // semaphore for multi-thread use of queue
    } _queue;

    struct {
        OA_DbItem       *items;                             // array of objects in the database
        float           filter_m                = 0.2f;     // object avoidance database optimization level radius. Min distance between each fence point. Larger means lower resolution
        const float     filter_max_m            = 10.0f;    // filter value max size allowed to grow to
        const float     filter_min_m            = 0.011f;   // worst case resolution of int32 lat/lng value at equator is 1.1cm;
        const float     filter_grow_rate        = 1.03f;    // db filter how fast you grow to reduce items getting into dB
        const float     filter_shrink_rate      = 0.99f;    // db filter how fast you shrink to increase items getting into dB
        const float     radius_grow_rate        = 1.10f;    // db item radius growth over time. Resets if refreshed, otherwise decaying items grow
        uint16_t        count;                              // number of objects in the items array
        uint16_t        size;                               // cached value of _database_size_param that sticks after initialized
    } _database;

    uint16_t _next_index_to_send[MAVLINK_COMM_NUM_BUFFERS]; // index of next object in _database to send to GCS
    uint16_t _highest_index_sent[MAVLINK_COMM_NUM_BUFFERS]; // highest index in _database sent to GCS
    uint32_t _last_send_to_gcs_ms[MAVLINK_COMM_NUM_BUFFERS];// system time that send_adsb_vehicle was last called

    float _radius_importance_low = _database.filter_m;
    float _radius_importance_normal = _database.filter_m;
    float _radius_importance_high = _database.filter_m;

    static AP_OADatabase *_singleton;
};
#else
class AP_OADatabase {
public:
    static AP_OADatabase *get_singleton() { return nullptr; }
    void init() {};
    void queue_push(const Location &loc, const uint32_t timestamp_ms, const float distance, const float angle) {};
    bool healthy() const { return false; }
    void send_adsb_vehicle(mavlink_channel_t chan, uint16_t interval_ms) {};
};
#endif // #if !HAL_MINIMIZE_FEATURES

namespace AP {
    AP_OADatabase *oadatabase();
};


