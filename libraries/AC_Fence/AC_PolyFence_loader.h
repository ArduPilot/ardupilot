#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT 1

enum class AC_PolyFenceType {
    END_OF_STORAGE    = 99,
    POLYGON_INCLUSION = 98,
    POLYGON_EXCLUSION = 97,
    CIRCLE_EXCLUSION  = 96,
    RETURN_POINT      = 95,
    CIRCLE_INCLUSION  = 94,
};

// a FenceItem is just a means of passing data about an item into
// and out of the polyfence loader.  It uses a AC_PolyFenceType to
// indicate the item type, assuming each fence type is made up of
// only one sort of item.
// TODO: make this a union (or use subclasses) to save memory
class AC_PolyFenceItem {
public:
    AC_PolyFenceType type;
    Vector2l loc;
    uint8_t vertex_count;
    float radius;
};

class AC_PolyFence_loader
{

public:

    AC_PolyFence_loader(AP_Int8 &total) :
        _total(total) {}

    AC_PolyFence_loader(const AC_PolyFence_loader &other) = delete;
    AC_PolyFence_loader &operator=(const AC_PolyFence_loader&) = delete;

    void init();

    // methods primarily for MissionItemProtocol_Fence to use:
    // return the total number of points stored
    uint16_t num_stored_items() const { return _eeprom_item_count; }
    bool get_item(const uint16_t seq, AC_PolyFenceItem &item) WARN_IF_UNUSED;

    ///
    /// exclusion polygons
    ///
    /// returns number of polygon exclusion zones defined
    uint8_t get_exclusion_polygon_count() const {
        return _num_loaded_exclusion_boundaries;
    }

    /// returns pointer to array of exclusion polygon points and num_points is filled in with the number of points in the polygon
    /// points are offsets in cm from EKF origin in NE frame
    Vector2f* get_exclusion_polygon(uint16_t index, uint16_t &num_points) const;

    /// return system time of last update to the exclusion polygon points
    uint32_t get_exclusion_polygon_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// inclusion polygons
    ///
    /// returns number of polygon inclusion zones defined
    uint8_t get_inclusion_polygon_count() const {
        return _num_loaded_inclusion_boundaries;
    }

    /// returns pointer to array of inclusion polygon points and num_points is filled in with the number of points in the polygon
    /// points are offsets in cm from EKF origin in NE frame
    Vector2f* get_inclusion_polygon(uint16_t index, uint16_t &num_points) const;

    /// return system time of last update to the inclusion polygon points
    uint32_t get_inclusion_polygon_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// exclusion circles
    ///
    /// returns number of exclusion circles defined
    uint8_t get_exclusion_circle_count() const {
        return _num_loaded_circle_exclusion_boundaries;
    }

    /// returns the specified exclusion circle
    /// center is offsets in cm from EKF origin in NE frame, radius is in meters
    bool get_exclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const;

    /// return system time of last update to the exclusion circles
    uint32_t get_exclusion_circle_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// inclusion circles
    ///
    /// returns number of inclusion circles defined
    uint8_t get_inclusion_circle_count() const {
        return _num_loaded_circle_inclusion_boundaries;
    }

    /// returns the specified inclusion circle
    /// center is offsets in cm from EKF origin in NE frame, radius is in meters
    bool get_inclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const;

    // false if margin < fence radius 
    bool check_inclusion_circle_margin(float margin) const;

    ///
    /// mavlink
    ///
    /// handler for polygon fence messages with GCS
    void handle_msg(class GCS_MAVLINK &link, const mavlink_message_t& msg);

    //  breached() - returns true if the vehicle has breached any fence
    bool breached() const WARN_IF_UNUSED;
    //  breached(Location&) - returns true if location is outside the boundary
    bool breached(const Location& loc) const WARN_IF_UNUSED;

    // returns true if a polygonal include fence could be returned
    bool inclusion_boundary_available() const WARN_IF_UNUSED {
        return _num_loaded_inclusion_boundaries != 0;
    }

    // loaded - returns true if the fences have been loaded from
    // storage and are available for use
    bool loaded() const WARN_IF_UNUSED {
        return _load_time_ms != 0;
    };

    // maximum number of fence points we can store in eeprom
    uint16_t max_items() const;

    // write_fence - validate and write count new_items to permanent storage
    bool write_fence(const AC_PolyFenceItem *new_items, uint16_t count)  WARN_IF_UNUSED;

    /*
     * Loaded Fence functionality
     *
     * methods and members to do with fences stored in memory.  The
     * locations are translated into offset-from-origin-in-metres
     */

    // load polygon points stored in eeprom into
    // _loaded_offsets_from_origin and perform validation.  returns
    // true if load successfully completed
    bool load_from_eeprom() WARN_IF_UNUSED;

    // allow threads to lock against AHRS update
    HAL_Semaphore &get_loaded_fence_semaphore(void) {
        return _loaded_fence_sem;
    }

    // call @10Hz to check for fence load being valid
    void update();

private:

    // multi-thread access support
    HAL_Semaphore _loaded_fence_sem;

    // breached(Vector2f&) - returns true of pos_cm (an offset in cm from the EKF origin) breaches any fence
    bool breached(const Vector2f& pos_cm) const WARN_IF_UNUSED;

    /*
     * Fence storage Index related functions
     */
    // FenceIndex - a class used to store information about a fence in
    // fence storage.
    class FenceIndex {
    public:
        AC_PolyFenceType type;
        uint16_t count;
        uint16_t storage_offset;
    };
    // index_fence_count - returns the number of fences of type
    // currently in the index
    uint16_t index_fence_count(const AC_PolyFenceType type);

    // void_index - free resources for the index, forcing a reindex
    // (typically via check_indexed)
    void void_index() {
        delete[] _index;
        _index = nullptr;
        _index_attempted = false;
        _indexed = false;
    }

    // check_indexed - read eeprom and create index if the index does
    // not already exist
    bool check_indexed() WARN_IF_UNUSED;

    // find_first_fence - return first fence in index of specific type
    FenceIndex *find_first_fence(const AC_PolyFenceType type) const;

    // find_index_for_seq - returns true if seq is contained within a
    // fence.  If it is, entry will be the relevant FenceIndex.  i
    // will be the offset within _loaded_offsets_from_origin where the
    // first point in the fence is found
    bool find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const WARN_IF_UNUSED;
    // find_storage_offset_for_seq - uses the index to return an
    // offset into storage for an item
    bool find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const WARN_IF_UNUSED;

    uint16_t sum_of_polygon_point_counts_and_returnpoint();

    /*
     * storage-related methods - dealing with fence_storage
     */

    // new_fence_storage_magic - magic number indicating fence storage
    // has been formatted for use by polygon fence storage code.
    // FIXME: ensure this is out-of-band for old lat/lon point storage
    static const uint8_t new_fence_storage_magic = 235;

    // validate_fence - returns true if new_items look completely valid
    bool validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const WARN_IF_UNUSED;

    // _eos_offset - stores the offset in storage of the
    // end-of-storage marker.  Used by low-level manipulation code to
    // extend storage
    uint16_t _eos_offset;

    // formatted - returns true if the fence storage space seems to be
    // formatted for new-style fence storage
    bool formatted() const WARN_IF_UNUSED;
    // format - format the storage space for use by
    // the new polyfence code
    bool format() WARN_IF_UNUSED;


    /*
     * Loaded Fence functionality
     *
     * methods and members to do with fences stored in memory.  The
     * locations are translated into offset-from-origin-in-metres
     */

    // remove resources dedicated to the transformed fences - for
    // example, in _loaded_offsets_from_origin
    void unload();

    // pointer into _loaded_offsets_from_origin where the return point
    // can be found:
    Vector2f *_loaded_return_point;

    class InclusionBoundary {
    public:
        Vector2f *points; // pointer into the _loaded_offsets_from_origin array
        uint8_t count; // count of points in the boundary
    };
    InclusionBoundary *_loaded_inclusion_boundary;
    uint8_t _num_loaded_inclusion_boundaries;

    class ExclusionBoundary {
    public:
        Vector2f *points; // pointer into the _loaded_offsets_from_origin array
        uint8_t count; // count of points in the boundary
    };
    ExclusionBoundary *_loaded_exclusion_boundary;
    uint8_t _num_loaded_exclusion_boundaries;

    // _loaded_offsets_from_origin - stores x/y offset-from-origin
    // coordinate pairs.  Various items store their locations in this
    // allocation - the polygon boundaries and the return point, for
    // example.
    Vector2f *_loaded_offsets_from_origin;

    class ExclusionCircle {
    public:
        Vector2f pos_cm;
        float radius;
    };
    ExclusionCircle *_loaded_circle_exclusion_boundary;
    uint8_t _num_loaded_circle_exclusion_boundaries;

    class InclusionCircle {
    public:
        Vector2f pos_cm;
        float radius;
    };
    InclusionCircle *_loaded_circle_inclusion_boundary;
    uint8_t _num_loaded_circle_inclusion_boundaries;

    // _load_attempted - true if we have attempted to load the fences
    // from storage into _loaded_circle_exclusion_boundary,
    // _loaded_offsets_from_origin etc etc
    bool _load_attempted;

    // _load_time_ms - from millis(), system time when fence load last
    // succeeded.  Will be zero if fences are not loaded
    uint32_t _load_time_ms;

    // read_scaled_latlon_from_storage - reads a latitude/longitude
    // from offset in permanent storage, transforms them into an
    // offset-from-origin and deposits the result into pos_cm.
    // read_offset is increased by the storage space used by the
    // latitude/longitude
    bool read_scaled_latlon_from_storage(const Location &origin,
                                         uint16_t &read_offset,
                                         Vector2f &pos_cm) WARN_IF_UNUSED;
    // read_polygon_from_storage - reads vertex_count
    // latitude/longitude points from offset in permanent storage,
    // transforms them into an offset-from-origin and deposits the
    // results into next_storage_point.
    bool read_polygon_from_storage(const Location &origin,
                                   uint16_t &read_offset,
                                   const uint8_t vertex_count,
                                   Vector2f *&next_storage_point) WARN_IF_UNUSED;

    /*
     * Upgrade functions - attempt to keep user's fences when
     * upgrading to new firmware
     */
    // convert_to_new_storage - will attempt to change a pre-existing
    // stored fence to the new storage format (so people don't lose
    // their fences when upgrading)
    bool convert_to_new_storage() WARN_IF_UNUSED;
    // load boundary point from eeprom, returns true on successful load
    bool load_point_from_eeprom(uint16_t i, Vector2l& point) WARN_IF_UNUSED;


#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
    /*
     * FENCE_POINT protocol compatability
     */
    void handle_msg_fetch_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    void handle_msg_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    // contains_compatible_fence - returns true if the permanent fence
    // storage contains fences that are compatible with the old
    // FENCE_POINT protocol.
    bool contains_compatible_fence() const WARN_IF_UNUSED;

    // get_or_create_include_fence - returns a point to an include
    // fence to be used for the FENCE_POINT-supplied polygon.  May
    // format the storage appropriately.
    FenceIndex *get_or_create_include_fence();
    // get_or_create_include_fence - returns a point to a return point
    // to be used for the FENCE_POINT-supplied return point.  May
    // format the storage appropriately.
    FenceIndex *get_or_create_return_point();
#endif

    // primitives to write parts of fencepoints out:
    bool write_type_to_storage(uint16_t &offset, AC_PolyFenceType type) WARN_IF_UNUSED;
    bool write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon) WARN_IF_UNUSED;
    bool read_latlon_from_storage(uint16_t &read_offset, Vector2l &latlon) const WARN_IF_UNUSED;

    // methods to write specific types of fencepoint out:
    bool write_eos_to_storage(uint16_t &offset);

#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
    // get_return_point - returns latitude/longitude of return point.
    // This works with storage - the returned vector is absolute
    // lat/lon.
    bool get_return_point(Vector2l &ret) WARN_IF_UNUSED;
#endif

    // _total - reference to FENCE_TOTAL parameter.  This is used
    // solely for compatability with the FENCE_POINT protocol
    AP_Int8 &_total;
    uint8_t _old_total;

    // scan_eeprom - a method that traverses the fence storage area,
    // calling the supplied callback for each fence found.  If the
    // scan fails (for example, the storage is corrupt), then this
    // method will return false.
    FUNCTOR_TYPEDEF(scan_fn_t, void, const AC_PolyFenceType, uint16_t);
    bool scan_eeprom(scan_fn_t scan_fn) WARN_IF_UNUSED;
    // scan_eeprom_count_fences - a static function designed to be
    // massed to scan_eeprom which counts the number of fences and
    // fence items present.  The results of this counting appear in _eeprom_fence_count and _eeprom_item_count
    void scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset);
    uint16_t _eeprom_fence_count;
    uint16_t _eeprom_item_count;

    // scan_eeprom_index_fences - a static function designed to be
    // passed to scan_eeprom.  _index must be a pointer to
    // memory sufficient to hold information about all fences present
    // in storage - so it is expected that scan_eeprom_count_fences
    // has been used to count those fences and the allocation already
    // made.  After this method has been called _index will
    // be filled with information about the fences in the fence
    // storage - type, item counts and storage offset.
    void scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset);
    // array specifying type of each fence in storage (and a count of
    // items in that fence)
    FenceIndex *_index;
    bool _indexed; // true if indexing successful
    bool _index_attempted; // true if we attempted to index the eeprom
    // _num_fences - count of the number of fences in _index.  This
    // should be equal to _eeprom_fence_count
    uint16_t _num_fences;

    // count_eeprom_fences - refresh the count of fences in permanent storage
    bool count_eeprom_fences() WARN_IF_UNUSED;
    // index_eeprom - (re)allocate and fill in _index
    bool index_eeprom() WARN_IF_UNUSED;

    uint16_t fence_storage_space_required(const AC_PolyFenceItem *new_items, uint16_t count);
};
