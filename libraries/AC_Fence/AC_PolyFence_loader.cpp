#include "AC_PolyFence_loader.h"

#if AP_FENCE_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef AC_FENCE_DUMMY_METHODS_ENABLED
#define AC_FENCE_DUMMY_METHODS_ENABLED  (!(APM_BUILD_TYPE(APM_BUILD_Rover) | APM_BUILD_COPTER_OR_HELI | APM_BUILD_TYPE(APM_BUILD_ArduPlane) | APM_BUILD_TYPE(APM_BUILD_ArduSub) | (AP_FENCE_ENABLED == 1)))
#endif

#if !AC_FENCE_DUMMY_METHODS_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AC_Fence/AC_Fence.h>

#include <stdio.h>

#define POLYFENCE_LOADER_DEBUGGING 0

#if POLYFENCE_LOADER_DEBUGGING
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#else
#define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

static StorageAccess fence_storage(StorageManager::StorageFence);

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AC_FENCE_SDCARD_FILENAME "APM/fence.stg"
#else
#define AC_FENCE_SDCARD_FILENAME "fence.stg"
#endif

void AC_PolyFence_loader::init()
{
#if AP_SDCARD_STORAGE_ENABLED
    // check for extra storage on microsd
    const auto *bc = AP::boardConfig();
    if (bc != nullptr) {
        const auto size_kb = bc->get_sdcard_fence_kb();
        if (size_kb > 0) {
            _failed_sdcard_storage = !fence_storage.attach_file(AC_FENCE_SDCARD_FILENAME, size_kb);
            if (_failed_sdcard_storage) {
                // wipe fence if storage not available, but don't
                // save. This allows sdcard error to be fixed and
                // reboot
                _total.set(0);
            }
        }
    }
#endif
    if (!check_indexed()) {
        // tell the user, perhaps?
    }
    _old_total = _total;
}

bool AC_PolyFence_loader::find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const
{
    if (_index == nullptr) {
        return false;
    }

    if (seq > _eeprom_item_count) {
        return false;
    }

    i = 0;
    for (uint16_t j=0; j<_num_fences; j++) {
        entry = &_index[j];
        if (seq < i + entry->count) {
            return true;
        }
        i += entry->count;
    }
    return false;
}

bool AC_PolyFence_loader::find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const
{
    if (_index == nullptr) {
        return false;
    }

    uint16_t i = 0;
    const FenceIndex *entry = nullptr;
    if (!find_index_for_seq(seq, entry, i)) {
        return false;
    }

    if (entry == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }

    const uint16_t delta = seq - i;

    offset = entry->storage_offset;
    type = entry->type;
    offset++; // skip over type
    switch (type) {
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        if (delta != 0) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        }
        break;
    case AC_PolyFenceType::POLYGON_INCLUSION:
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        vertex_count_offset = offset;
        offset += 1; // the count of points in the fence
        offset += (delta * 8);
        break;
    case AC_PolyFenceType::RETURN_POINT:
        if (delta != 0) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        }
        break;
    case AC_PolyFenceType::END_OF_STORAGE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::get_item(const uint16_t seq, AC_PolyFenceItem &item)
{
    if (!check_indexed()) {
        return false;
    }

    uint16_t vertex_count_offset = 0; // initialised to make compiler happy
    uint16_t offset;
    AC_PolyFenceType type;
    if (!find_storage_offset_for_seq(seq, offset, type, vertex_count_offset)) {
        return false;
    }

    item.type = type;

    switch (type) {
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.radius = fence_storage.read_float(offset);
        break;
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.radius = fence_storage.read_uint32(offset);
        // magically change int item into a float item:
        if (type == AC_PolyFenceType::CIRCLE_INCLUSION_INT) {
            item.type = AC_PolyFenceType::CIRCLE_INCLUSION;
        } else {
            item.type = AC_PolyFenceType::CIRCLE_EXCLUSION;
        }
        break;
    case AC_PolyFenceType::POLYGON_INCLUSION:
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.vertex_count = fence_storage.read_uint8(vertex_count_offset);
        break;
    case AC_PolyFenceType::RETURN_POINT:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        break;
    case AC_PolyFenceType::END_OF_STORAGE:
        // read end-of-storage when I should never do so
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::write_type_to_storage(uint16_t &offset, const AC_PolyFenceType type)
{
    fence_storage.write_uint8(offset, (uint8_t)type);
    offset++;
    return true;
}

bool AC_PolyFence_loader::write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon)
{
    fence_storage.write_uint32(offset, latlon.x);
    offset += 4;
    fence_storage.write_uint32(offset, latlon.y);
    offset += 4;
    return true;
}

bool AC_PolyFence_loader::read_latlon_from_storage(uint16_t &read_offset, Vector2l &ret) const
{
    ret.x = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    ret.y = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    return true;
}

bool AC_PolyFence_loader::breached() const
{
    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        return false;
    }

    return breached(loc);
}

// check if a position (expressed as lat/lng) is within the boundary
//   returns true if location is outside the boundary
bool AC_PolyFence_loader::breached(const Location& loc) const
{
    if (!loaded()) {
        return false;
    }

    Vector2l pos;
    pos.x = loc.lat;
    pos.y = loc.lng;

    const uint16_t num_inclusion = _num_loaded_circle_inclusion_boundaries + _num_loaded_inclusion_boundaries;
    uint16_t num_inclusion_outside = 0;

    // check we are inside each inclusion zone:
    for (uint8_t i=0; i<_num_loaded_inclusion_boundaries; i++) {
        const InclusionBoundary &boundary = _loaded_inclusion_boundary[i];
        if (Polygon_outside(pos, boundary.points_lla, boundary.count)) {
            num_inclusion_outside++;
        }
    }

    // check we are outside each exclusion zone:
    for (uint8_t i=0; i<_num_loaded_exclusion_boundaries; i++) {
        const ExclusionBoundary &boundary = _loaded_exclusion_boundary[i];
        if (!Polygon_outside(pos, boundary.points_lla, boundary.count)) {
            return true;
        }
    }

    for (uint8_t i=0; i<_num_loaded_circle_exclusion_boundaries; i++) {
        const ExclusionCircle &circle = _loaded_circle_exclusion_boundary[i];
        Location circle_center;
        circle_center.lat = circle.point.x;
        circle_center.lng = circle.point.y;
        const float diff_cm = loc.get_distance(circle_center)*100.0f;
        if (diff_cm < circle.radius * 100.0f) {
            return true;
        }
    }

    for (uint8_t i=0; i<_num_loaded_circle_inclusion_boundaries; i++) {
        const InclusionCircle &circle = _loaded_circle_inclusion_boundary[i];
        Location circle_center;
        circle_center.lat = circle.point.x;
        circle_center.lng = circle.point.y;
        const float diff_cm = loc.get_distance(circle_center)*100.0f;
        if (diff_cm > circle.radius * 100.0f) {
            num_inclusion_outside++;
        }
    }

    if (AC_Fence::option_enabled(AC_Fence::OPTIONS::INCLUSION_UNION, _options)) {
        // using union of inclusion areas, we are outside the fence if
        // there is at least one inclusion areas and we are outside
        // all of them
        if (num_inclusion > 0 && num_inclusion == num_inclusion_outside) {
            return true;
        }
    } else {
        // using intersection of inclusion areas. We are outside if we
        // are outside any of them
        if (num_inclusion_outside > 0) {
            return true;
        }
    }

    // no fence breached
    return false;
}

bool AC_PolyFence_loader::formatted() const
{
    return (fence_storage.read_uint8(0) == new_fence_storage_magic &&
            fence_storage.read_uint8(1) == 0 &&
            fence_storage.read_uint8(2) == 0 &&
            fence_storage.read_uint8(3) == 0);
}

uint16_t AC_PolyFence_loader::max_items() const
{
    // this is 84 items on PixHawk
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}

bool AC_PolyFence_loader::format()
{
    uint16_t offset = 0;
    fence_storage.write_uint32(offset, 0);
    fence_storage.write_uint8(offset, new_fence_storage_magic);
    offset += 4;
    void_index();
    _eeprom_fence_count = 0;
    _eeprom_item_count = 0;
    return write_eos_to_storage(offset);
}

bool AC_PolyFence_loader::scale_latlon_from_origin(const Location &origin, const Vector2l &point, Vector2f &pos_cm)
{
    Location tmp_loc;
    tmp_loc.lat = point.x;
    tmp_loc.lng = point.y;
    pos_cm = origin.get_distance_NE(tmp_loc) * 100.0f;
    return true;
}

bool AC_PolyFence_loader::read_polygon_from_storage(const Location &origin, uint16_t &read_offset, const uint8_t vertex_count, Vector2f *&next_storage_point, Vector2l *&next_storage_point_lla)
{
    for (uint8_t i=0; i<vertex_count; i++) {
        // read from storage to lat/lon
        if (!read_latlon_from_storage(read_offset, *next_storage_point_lla)) {
            return false;
        }
        // convert lat/lon to position in cm from origin
        if (!scale_latlon_from_origin(origin, *next_storage_point_lla, *next_storage_point)) {
            return false;
        }
        
        next_storage_point_lla++;
        next_storage_point++;
    }
    return true;
}

bool AC_PolyFence_loader::scan_eeprom(scan_fn_t scan_fn)
{
    uint16_t read_offset = 0; // skipping reserved first 4 bytes
    if (!formatted()) {
        return false;
    }
    read_offset += 4;
    bool all_done = false;
    while (!all_done) {
        if (read_offset > fence_storage.size()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("did not find end-of-storage-marker before running out of space");
#endif
            return false;
        }
        const AC_PolyFenceType type = (AC_PolyFenceType)fence_storage.read_uint8(read_offset);
        // validate what we've just pulled back from storage:
        switch (type) {
        case AC_PolyFenceType::END_OF_STORAGE:
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        case AC_PolyFenceType::RETURN_POINT:
            break;
        default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Fence corrupt (offset=%u)", read_offset);
#endif
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fence corrupt");
            return false;
        }

        scan_fn(type, read_offset);
        read_offset++;
        switch (type) {
        case AC_PolyFenceType::END_OF_STORAGE:
            _eos_offset = read_offset-1;
            all_done = true;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION: {
            const uint8_t vertex_count = fence_storage.read_uint8(read_offset);
            read_offset += 1; // for the count we just read
            read_offset += vertex_count*8;
            break;
        }
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            read_offset += 8; // for latlon
            read_offset += 4; // for radius
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            read_offset += 8; // for latlon
            break;
        }
    }
    return true;
}

// note read_offset here isn't const and ALSO is not a reference
void AC_PolyFence_loader::scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset)
{
    if (type == AC_PolyFenceType::END_OF_STORAGE) {
        return;
    }
    _eeprom_fence_count++;
    switch (type) {
    case AC_PolyFenceType::END_OF_STORAGE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
    case AC_PolyFenceType::POLYGON_INCLUSION: {
        const uint8_t vertex_count = fence_storage.read_uint8(read_offset+1); // skip type
        _eeprom_item_count += vertex_count;
        break;
    }
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
    case AC_PolyFenceType::RETURN_POINT:
        _eeprom_item_count++;
        break;
    }
}

bool AC_PolyFence_loader::count_eeprom_fences()
{
    _eeprom_fence_count = 0;
    _eeprom_item_count = 0;
    const bool ret = scan_eeprom(FUNCTOR_BIND_MEMBER(&AC_PolyFence_loader::scan_eeprom_count_fences, void, const AC_PolyFenceType, uint16_t));
    return ret;
}

void AC_PolyFence_loader::scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset)
{
    if (_index == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    if (type == AC_PolyFenceType::END_OF_STORAGE) {
        return;
    }
    FenceIndex &index = _index[_num_fences++];
    index.type = type;
    index.storage_offset = read_offset;
    switch (type) {
    case AC_PolyFenceType::END_OF_STORAGE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
    case AC_PolyFenceType::POLYGON_INCLUSION: {
        const uint8_t vertex_count = fence_storage.read_uint8(read_offset+1);
        index.count = vertex_count;
        break;
    }
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        index.count = 1;
        break;
    case AC_PolyFenceType::RETURN_POINT:
        index.count = 1;
        break;
    }
}

bool AC_PolyFence_loader::index_eeprom()
{
    if (!formatted()) {
        if (!format()) {
            return false;
        }
    }

    if (!count_eeprom_fences()) {
        return false;
    }
    if (_eeprom_fence_count == 0) {
        _load_attempted = false;
        return true;
    }

    void_index();

    Debug("Fence: Allocating %u bytes for index",
          (unsigned)(_eeprom_fence_count*sizeof(FenceIndex)));
    _index = new FenceIndex[_eeprom_fence_count];
    if (_index == nullptr) {
        return false;
    }

    _num_fences = 0;
    if (!scan_eeprom(FUNCTOR_BIND_MEMBER(&AC_PolyFence_loader::scan_eeprom_index_fences, void, const AC_PolyFenceType, uint16_t))) {
        void_index();
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_num_fences != _eeprom_fence_count) {
        AP_HAL::panic("indexed fences not equal to eeprom fences");
    }
#endif

    _load_attempted = false;

    return true;
}

bool AC_PolyFence_loader::check_indexed()
{
    if (!_index_attempted) {
        _indexed = index_eeprom();
        _index_attempted = true;
    }
    return _indexed;
}

void AC_PolyFence_loader::unload()
{
    delete[] _loaded_offsets_from_origin;
    _loaded_offsets_from_origin = nullptr;

    delete[] _loaded_points_lla;
    _loaded_points_lla = nullptr;

    delete[] _loaded_inclusion_boundary;
    _loaded_inclusion_boundary = nullptr;
    _num_loaded_inclusion_boundaries = 0;

    delete[] _loaded_exclusion_boundary;
    _loaded_exclusion_boundary = nullptr;
    _num_loaded_exclusion_boundaries = 0;

    delete[] _loaded_circle_inclusion_boundary;
    _loaded_circle_inclusion_boundary = nullptr;
    _num_loaded_circle_inclusion_boundaries = 0;

    delete[] _loaded_circle_exclusion_boundary;
    _loaded_circle_exclusion_boundary = nullptr;
    _num_loaded_circle_exclusion_boundaries = 0;

    _loaded_return_point = nullptr;
    _loaded_return_point_lla = nullptr;
    _load_time_ms = 0;
}

// return the number of fences of type type in the index:
uint16_t AC_PolyFence_loader::index_fence_count(const AC_PolyFenceType type)
{
    uint16_t ret = 0;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        const FenceIndex &index = _index[i];
        if (index.type == type) {
            ret++;
        }
    }
    return ret;
}

uint16_t AC_PolyFence_loader::sum_of_polygon_point_counts_and_returnpoint()
{
    uint16_t ret = 0;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        const FenceIndex &index = _index[i];
        switch (index.type) {
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            break;
        case AC_PolyFenceType::RETURN_POINT:
            ret += 1;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            ret += index.count;
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
    }
    return ret;
}

bool AC_PolyFence_loader::load_from_eeprom()
{
    if (!check_indexed()) {
        return false;
    }

    if (_load_attempted) {
        return _load_time_ms != 0;
    }

    Location ekf_origin{};
    if (!AP::ahrs().get_origin(ekf_origin)) {
//        Debug("fence load requires origin");
        return false;
    }

    // find indexes of each fence:
    if (!get_loaded_fence_semaphore().take_nonblocking()) {
        return false;
    }

    _load_attempted = true;

    unload();

    if (_eeprom_item_count == 0) {
        get_loaded_fence_semaphore().give();
        _load_time_ms = AP_HAL::millis();
        return true;
    }

    { // allocate array to hold offsets-from-origin
        const uint16_t count = sum_of_polygon_point_counts_and_returnpoint();
        Debug("Fence: Allocating %u bytes for points",
              (unsigned)(count * sizeof(Vector2f)));
        _loaded_offsets_from_origin = new Vector2f[count];
        _loaded_points_lla = new Vector2l[count];
        if (_loaded_offsets_from_origin == nullptr || _loaded_points_lla == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    // FIXME: find some way of factoring out all of these allocation routines.

    { // allocate storage for inclusion polyfences:
        const uint8_t count = index_fence_count(AC_PolyFenceType::POLYGON_INCLUSION);
        Debug("Fence: Allocating %u bytes for inc. fences",
              (unsigned)(count * sizeof(InclusionBoundary)));
        _loaded_inclusion_boundary = new InclusionBoundary[count];
        if (_loaded_inclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    { // allocate storage for exclusion polyfences:
        const uint8_t count = index_fence_count(AC_PolyFenceType::POLYGON_EXCLUSION);
        Debug("Fence: Allocating %u bytes for exc. fences",
              (unsigned)(count * sizeof(ExclusionBoundary)));
        _loaded_exclusion_boundary = new ExclusionBoundary[count];
        if (_loaded_exclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    { // allocate storage for circular inclusion fences:
        uint8_t count = index_fence_count(AC_PolyFenceType::CIRCLE_INCLUSION);
        count += index_fence_count(AC_PolyFenceType::CIRCLE_INCLUSION_INT)
        Debug("Fence: Allocating %u bytes for circ. inc. fences",
              (unsigned)(count * sizeof(InclusionCircle)));
        _loaded_circle_inclusion_boundary = new InclusionCircle[count];
        if (_loaded_circle_inclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    { // allocate storage for circular exclusion fences:
        uint8_t count = index_fence_count(AC_PolyFenceType::CIRCLE_EXCLUSION);
        count += index_fence_count(AC_PolyFenceType::CIRCLE_EXCLUSION_INT)
        Debug("Fence: Allocating %u bytes for circ. exc. fences",
              (unsigned)(count * sizeof(ExclusionCircle)));
        _loaded_circle_exclusion_boundary = new ExclusionCircle[count];
        if (_loaded_circle_exclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    Vector2f *next_storage_point = _loaded_offsets_from_origin;
    Vector2l *next_storage_point_lla = _loaded_points_lla;

    // use index to load fences from eeprom
    bool storage_valid = true;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        if (!storage_valid) {
            break;
        }
        const FenceIndex &index = _index[i];
        uint16_t storage_offset = index.storage_offset;
        storage_offset += 1; // skip type
        switch (index.type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("indexed end of storage found");
#endif
            storage_valid = false;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION: {
            // FIXME: consider factoring this with the EXCLUSION case
            InclusionBoundary &boundary = _loaded_inclusion_boundary[_num_loaded_inclusion_boundaries];
            boundary.points = next_storage_point;
            boundary.points_lla = next_storage_point_lla;
            boundary.count = index.count;
            if (index.count < 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: invalid polygon vertex count %u", index.count);
                storage_valid = false;
                break;
            }
            storage_offset += 1; // skip vertex count
            if (!read_polygon_from_storage(ekf_origin, storage_offset, index.count, next_storage_point, next_storage_point_lla)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: polygon read failed");
                storage_valid = false;
                break;
            }
            _num_loaded_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::POLYGON_EXCLUSION: {
            ExclusionBoundary &boundary = _loaded_exclusion_boundary[_num_loaded_exclusion_boundaries];
            boundary.points = next_storage_point;
            boundary.points_lla = next_storage_point_lla;
            boundary.count = index.count;
            if (index.count < 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: invalid polygon vertex count %u", index.count);
                storage_valid = false;
                break;
            }
            storage_offset += 1; // skip vertex count
            if (!read_polygon_from_storage(ekf_origin, storage_offset, index.count, next_storage_point, next_storage_point_lla)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: polygon read failed");
                storage_valid = false;
                break;
            }
            _num_loaded_exclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            ExclusionCircle &circle = _loaded_circle_exclusion_boundary[_num_loaded_circle_exclusion_boundaries];
            if (!read_latlon_from_storage(storage_offset, circle.point)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            if (!scale_latlon_from_origin(ekf_origin, circle.point, circle.pos_cm)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            // now read the radius
            if (index.type == AC_PolyFenceType::CIRCLE_EXCLUSION_INT) {
                circle.radius = fence_storage.read_uint32(storage_offset);
            } else {
                circle.radius = fence_storage.read_float(storage_offset);
            }
            if (!is_positive(circle.radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: non-positive circle radius");
                storage_valid = false;
                break;
            }
            _num_loaded_circle_exclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_INCLUSION: {
            InclusionCircle &circle = _loaded_circle_inclusion_boundary[_num_loaded_circle_inclusion_boundaries];
            if (!read_latlon_from_storage(storage_offset, circle.point)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            if (!scale_latlon_from_origin(ekf_origin, circle.point, circle.pos_cm)){
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            // now read the radius
            if (index.type == AC_PolyFenceType::CIRCLE_INCLUSION_INT) {
                circle.radius = fence_storage.read_uint32(storage_offset);
            } else {
                circle.radius = fence_storage.read_float(storage_offset);
            }
            if (!is_positive(circle.radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: non-positive circle radius");
                storage_valid = false;
                break;
            }
            _num_loaded_circle_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            if (_loaded_return_point != nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: Multiple return points found");
                storage_valid = false;
                break;
            }
            _loaded_return_point = next_storage_point;
            if (_loaded_return_point_lla != nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: Multiple return points found");
                storage_valid = false;
                break;
            }
            _loaded_return_point_lla = next_storage_point_lla;
            // Read the point from storage
            if (!read_latlon_from_storage(storage_offset, *next_storage_point_lla)) {
                storage_valid = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: latlon read failed");
                break;
            }
            if (!scale_latlon_from_origin(ekf_origin, *next_storage_point_lla, *next_storage_point)) {
                storage_valid = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: latlon read failed");
                break;
            }
            next_storage_point++;
            next_storage_point_lla++;
            break;
        }
    }

    if (!storage_valid) {
        unload();
        get_loaded_fence_semaphore().give();
        return false;
    }

    _load_time_ms = AP_HAL::millis();

    get_loaded_fence_semaphore().give();
    return true;
}

/// returns pointer to array of exclusion polygon points and num_points is filled in with the number of points in the polygon
/// points are offsets in cm from EKF origin in NE frame
Vector2f* AC_PolyFence_loader::get_exclusion_polygon(uint16_t index, uint16_t &num_points) const
{
    if (index >= _num_loaded_exclusion_boundaries) {
        num_points = 0;
        return nullptr;
    }
    const ExclusionBoundary &boundary = _loaded_exclusion_boundary[index];
    num_points = boundary.count;

    return boundary.points;
}

/// returns pointer to array of inclusion polygon points and num_points is filled in with the number of points in the polygon
/// points are offsets in cm from EKF origin in NE frame
Vector2f* AC_PolyFence_loader::get_inclusion_polygon(uint16_t index, uint16_t &num_points) const
{
    if (index >= _num_loaded_inclusion_boundaries) {
        num_points = 0;
        return nullptr;
    }
    const InclusionBoundary &boundary = _loaded_inclusion_boundary[index];
    num_points = boundary.count;

    return boundary.points;
}

/// returns the specified exclusion circle
/// circle center offsets in cm from EKF origin in NE frame, radius is in meters
bool AC_PolyFence_loader::get_exclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const
{
    if (index >= _num_loaded_circle_exclusion_boundaries) {
        return false;
    }
    center_pos_cm = _loaded_circle_exclusion_boundary[index].pos_cm;
    radius =  _loaded_circle_exclusion_boundary[index].radius;
    return true;
}

/// returns the specified inclusion circle
/// circle centre offsets in cm from EKF origin in NE frame, radius is in meters
bool AC_PolyFence_loader::get_inclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const
{
    if (index >= _num_loaded_circle_inclusion_boundaries) {
        return false;
    }
    center_pos_cm = _loaded_circle_inclusion_boundary[index].pos_cm;
    radius =  _loaded_circle_inclusion_boundary[index].radius;
    return true;
}

bool AC_PolyFence_loader::check_inclusion_circle_margin(float margin) const
{
    // check circular includes
    for (uint8_t i=0; i<_num_loaded_circle_inclusion_boundaries; i++) {
        const InclusionCircle &circle = _loaded_circle_inclusion_boundary[i];
        if (circle.radius < margin) {
            // circle radius should never be less than margin
            return false;
        } 
    }
    return true;
}

bool AC_PolyFence_loader::validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const
{
    // validate the fence items...
    AC_PolyFenceType expecting_type = AC_PolyFenceType::END_OF_STORAGE;
    uint16_t expected_type_count = 0;
    uint16_t orig_expected_type_count = 0;
    bool seen_return_point = false;

    for (uint16_t i=0; i<count; i++) {
        bool validate_latlon = false;

        switch (new_items[i].type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("passed in an END_OF_STORAGE");
#endif
            return false;

        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            if (new_items[i].vertex_count < 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid vertex count (%u)", new_items[i].vertex_count);
                return false;
            }
            if (expected_type_count == 0) {
                expected_type_count = new_items[i].vertex_count;
                orig_expected_type_count = expected_type_count;
                expecting_type = new_items[i].type;
            } else {
                if (new_items[i].type != expecting_type) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect vertex type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
                    return false;
                } else if (new_items[i].vertex_count != orig_expected_type_count) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unexpected vertex count want=%u got=%u\n", orig_expected_type_count, new_items[i].vertex_count);
                    return false;
                }
            }
            expected_type_count--;
            validate_latlon = true;
            break;

        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
            // should never have AC_PolyFenceItems of these types
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            if (expected_type_count) {
               GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
               return false;
            }
            if (!is_positive(new_items[i].radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Non-positive circle radius");
                return false;
            }
            validate_latlon = true;
            break;

        case AC_PolyFenceType::RETURN_POINT:
            if (expected_type_count) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
                return false;
            }

            // spec says only one return point allowed
            if (seen_return_point) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Multiple return points");
                return false;
            }
            seen_return_point = true;
            validate_latlon = true;
            // TODO: ensure return point is within all fences and
            // outside all exclusion zones
            break;
        }

        if (validate_latlon) {
            if (!check_latlng(new_items[i].loc[0], new_items[i].loc[1])) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Bad lat or lon");
                return false;
            }
        }
    }

    if (expected_type_count) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Incorrect item count");
        return false;
    }

    return true;
}

uint16_t AC_PolyFence_loader::fence_storage_space_required(const AC_PolyFenceItem *new_items, uint16_t count)
{
    uint16_t ret = 4; // for the format header
    uint16_t i = 0;
    while (i < count) {
        ret += 1; // one byte for type
        switch (new_items[i].type) {
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            ret += 1 + 8 * new_items[i].vertex_count; // 1 count, 4 lat, 4 lon for each point
            i += new_items[i].vertex_count - 1; // i is incremented down below
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
            // should never have AC_PolyFenceItems of these types
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            ret += 12; // 4 radius, 4 lat, 4 lon
            break;
        case AC_PolyFenceType::RETURN_POINT:
            ret += 8; // 4 lat, 4 lon
            break;
        }
        i++;
    }
    return ret;
}

bool AC_PolyFence_loader::write_fence(const AC_PolyFenceItem *new_items, uint16_t count)
{
    if (!validate_fence(new_items, count)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fence validation failed");
        return false;
    }

    if (fence_storage_space_required(new_items, count) > fence_storage.size()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fence exceeds storage size");
        return false;
    }

    if (!format()) {
        return false;
    }

    uint8_t total_vertex_count = 0;
    uint16_t offset = 4; // skipping magic
    uint8_t vertex_count = 0;
    for (uint16_t i=0; i<count; i++) {
        const AC_PolyFenceItem new_item = new_items[i];
        switch (new_item.type) {
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            if (vertex_count == 0) {
                // write out new polygon count
                vertex_count = new_item.vertex_count;
                total_vertex_count += vertex_count;
                if (!write_type_to_storage(offset, new_item.type)) {
                    return false;
                }
                fence_storage.write_uint8(offset, vertex_count);
                offset++;
            }
            vertex_count--;
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("asked to store end-of-storage marker");
#endif
            return false;
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
            // should never have AC_PolyFenceItems of these types
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            total_vertex_count++; // useful to make number of lines in QGC file match FENCE_TOTAL
            const bool store_as_int = (new_item.radius - int(new_item.radius) < 0.001);
            AC_PolyFenceType store_type = new_item.type;
            if (store_as_int) {
                if (new_item.type == AC_PolyFenceType::CIRCLE_INCLUSION) {
                    store_type = AC_PolyFenceType::CIRCLE_INCLUSION_INT;
                } else {
                    store_type = AC_PolyFenceType::CIRCLE_EXCLUSION_INT;
                }
            }

            if (!write_type_to_storage(offset, store_type)) {
                return false;
            }
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            // store the radius.  If the radius is very close to an
            // integer then we store it as an integer so users moving
            // from 4.1 back to 4.0 might be less-disrupted.
            if (store_as_int) {
                fence_storage.write_uint32(offset, new_item.radius);
            } else {
                fence_storage.write_float(offset, new_item.radius);
            }
            offset += 4;
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            if (!write_type_to_storage(offset, new_item.type)) {
                return false;
            }
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            break;
        }
    }
    if (!write_eos_to_storage(offset)) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // sanity-check the EEPROM in SITL to make sure we can read what
    // we've just written.
    if (!index_eeprom()) {
        AP_HAL::panic("Failed to index eeprom");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Fence Indexed OK");
#endif

#if HAL_LOGGING_ENABLED
    // start logger logging new fence
    AP::logger().Write_Fence();
#endif

    void_index();

    // this may be completely bogus total.  If we are storing an
    // advanced fence then the old protocol which relies on this value
    // will error off if the GCS tries to fetch points.  This number
    // should be correct for a "compatible" fence, however.
    uint16_t new_total = 0;
    if (total_vertex_count < 3) {
        new_total = 0;
    } else {
        new_total = total_vertex_count+2;
    }
    _total.set_and_save(new_total);

    return true;
}


bool AC_PolyFence_loader::get_return_point(Vector2l &ret)
{
    if (!check_indexed()) {
        return false;
    }

    const FenceIndex *rp = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (rp != nullptr) {
        uint16_t read_offset = rp->storage_offset + 1;
        return read_latlon_from_storage(read_offset, ret);
    }

    const FenceIndex *inc = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inc == nullptr) {
        return false;
    }

    // we found an inclusion fence but not a return point.  Calculate
    // and return the centroid.  Note that this may not actually be
    // inside all inclusion fences...
    uint16_t offset = inc->storage_offset;
    if ((AC_PolyFenceType)fence_storage.read_uint8(offset) != AC_PolyFenceType::POLYGON_INCLUSION) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("wrong type at offset");
#endif
        return false;
    }
    offset++;
    const uint8_t count = fence_storage.read_uint8(offset);
    if (count < 3) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("invalid count found");
#endif
        return false;
    }
    offset++;
    Vector2l min_loc;
    if (!read_latlon_from_storage(offset, min_loc)) {
        return false;
    }
    if (min_loc.is_zero()) {
        return false;
    }
    Vector2l max_loc = min_loc;
    for (uint8_t i=1; i<count; i++) {
        Vector2l new_loc;
        if (!read_latlon_from_storage(offset, new_loc)) {
            return false;
        }
        if (new_loc.is_zero()) {
            return false;
        }
        if (new_loc.x < min_loc.x) {
            min_loc.x = new_loc.x;
        }
        if (new_loc.y < min_loc.y) {
            min_loc.y = new_loc.y;
        }
        if (new_loc.x > max_loc.x) {
            max_loc.x = new_loc.x;
        }
        if (new_loc.y > max_loc.y) {
            max_loc.y = new_loc.y;
        }
    }

    // Potential for int32_t overflow when longitudes are beyond [-107, 107].
    // As a result, the calculated return point's longitude is calculated using overflowed figure.
    // Dividing first before adding avoids the potential overflow.
    ret.x = (min_loc.x / 2) + (max_loc.x / 2);
    ret.y = (min_loc.y / 2) + (max_loc.y / 2);

    return true;
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::find_first_fence(const AC_PolyFenceType type) const
{
    if (_index == nullptr) {
        return nullptr;
    }
    for (uint8_t i=0; i<_num_fences; i++) {
        if (_index[i].type == type) {
            return &_index[i];
        }
    }
    return nullptr;
}

#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
void AC_PolyFence_loader::handle_msg_fetch_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg)
{
    if (!check_indexed()) {
        return;
    }
    if (!contains_compatible_fence()) {
        link.send_text(MAV_SEVERITY_WARNING, "Vehicle contains advanced fences");
        return;
    }

    if (_total != 0 && _total < 5) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid FENCE_TOTAL");
        return;
    }

    mavlink_fence_fetch_point_t packet;
    mavlink_msg_fence_fetch_point_decode(&msg, &packet);

    if (packet.idx >= _total) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, index past total(%u >= %u)", packet.idx, _total.get());
        return;
    }

    mavlink_fence_point_t ret_packet{};
    ret_packet.target_system = msg.sysid;
    ret_packet.target_component = msg.compid;
    ret_packet.idx = packet.idx;
    ret_packet.count = _total;

    if (packet.idx == 0) {
        // return point
        Vector2l ret;
        if (get_return_point(ret)) {
            ret_packet.lat = ret.x * 1.0e-7f;
            ret_packet.lng = ret.y * 1.0e-7f;
        } else {
            link.send_text(MAV_SEVERITY_WARNING, "Failed to get return point");
        }
    } else {
        // find the inclusion fence:
        const FenceIndex *inclusion_fence = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
        if (inclusion_fence == nullptr) {
            // nothing stored yet; just send back zeroes
            ret_packet.lat = 0;
            ret_packet.lng = 0;
        } else {
            uint8_t fencepoint_offset; // 1st idx is return point
            if (packet.idx == _total-1) {
                // the is the loop closure point - send the first point again
                fencepoint_offset = 0;
            } else {
                fencepoint_offset = packet.idx - 1;
            }
            if (fencepoint_offset >= inclusion_fence->count) {
                // we haven't been given a value for this item yet; we will return zeroes
            } else {
                uint16_t storage_offset = inclusion_fence->storage_offset;
                storage_offset++; // skip over type
                storage_offset++; // skip over count
                storage_offset += 8*fencepoint_offset; // move to point we're interested in
                Vector2l bob;
                if (!read_latlon_from_storage(storage_offset, bob)) {
                    link.send_text(MAV_SEVERITY_WARNING, "Fence read failed");
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    AP_HAL::panic("read failure");
#endif
                    return;
                }
                ret_packet.lat = bob[0] * 1.0e-7f;
                ret_packet.lng = bob[1] * 1.0e-7f;
            }
        }
    }

    link.send_message(MAVLINK_MSG_ID_FENCE_POINT, (const char*)&ret_packet);
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::get_or_create_return_point()
{
    if (!check_indexed()) {
        return nullptr;
    }
    FenceIndex *return_point = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (return_point != nullptr) {
        return return_point;
    }

    // if the inclusion fence exists we will move it in storage to
    // avoid having to continually shift the return point forward as
    // we receive fence points
    uint16_t offset;
    const FenceIndex *inclusion_fence = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inclusion_fence != nullptr) {
        offset = inclusion_fence->storage_offset;
        // the "9"s below represent the size of a return point in storage
        for (uint8_t i=0; i<inclusion_fence->count; i++) {
            // we are shifting the last fence point first - so 'i=0'
            // means the last point stored.
            const uint16_t point_storage_offset = offset + 2 + (inclusion_fence->count-1-i) * 8;
            Vector2l latlon;
            uint16_t tmp_read_offs = point_storage_offset;
            if (!read_latlon_from_storage(tmp_read_offs, latlon)) {
                return nullptr;
            }
            uint16_t write_offset = point_storage_offset + 9;
            if (!write_latlon_to_storage(write_offset, latlon)) {
                return nullptr;
            }
        }
        // read/write the count:
        const uint8_t count = fence_storage.read_uint8(inclusion_fence->storage_offset+1);
        fence_storage.write_uint8(inclusion_fence->storage_offset + 1 + 9, count);
        // read/write the type:
        const uint8_t t = fence_storage.read_uint8(inclusion_fence->storage_offset);
        fence_storage.write_uint8(inclusion_fence->storage_offset + 9, t);

        uint16_t write_offset = offset + 2 + 8*inclusion_fence->count + 9;
        if (!write_eos_to_storage(write_offset)) {
            return nullptr;
        }
    } else {
        if (fence_storage.read_uint8(_eos_offset) != (uint8_t)AC_PolyFenceType::END_OF_STORAGE) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Expected end-of-storage marker at offset=%u",
                          _eos_offset);
#endif
            return nullptr;
        }
        offset = _eos_offset;
    }

    if (!write_type_to_storage(offset, AC_PolyFenceType::RETURN_POINT)) {
        return nullptr;
    }
    if (!write_latlon_to_storage(offset, Vector2l{0, 0})) {
        return nullptr;
    }
    if (inclusion_fence == nullptr) {
        if (!write_eos_to_storage(offset)) {
            return nullptr;
        }
    }

    if (!index_eeprom()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Failed to index eeprom after moving inclusion fence for return point");
#endif
        return nullptr;
    }

    return_point = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (return_point == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Failed to get return point after indexing");
#endif
    }
    return return_point;
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::get_or_create_include_fence()
{
    if (!check_indexed()) {
        return nullptr;
    }
    FenceIndex *inclusion = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inclusion != nullptr) {
        return inclusion;
    }
    if (_total < 5) {
        return nullptr;
    }
    if (!write_type_to_storage(_eos_offset, AC_PolyFenceType::POLYGON_INCLUSION)) {
        return nullptr;
    }
    fence_storage.write_uint8(_eos_offset, 0);
    _eos_offset++;
    if (!write_eos_to_storage(_eos_offset)) {
        return nullptr;
    }

    if (!index_eeprom()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Failed to index eeprom after creating fence");
#endif
        return nullptr;
    }
    AC_PolyFence_loader::FenceIndex *ret = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (ret == nullptr) {
        AP_HAL::panic("Failed to index eeprom after creating fence");
    }
#endif
    return ret;
}

void AC_PolyFence_loader::handle_msg_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg)
{
    if (!check_indexed()) {
        return;
    }

    mavlink_fence_point_t packet;
    mavlink_msg_fence_point_decode(&msg, &packet);

    if (_total != 0 && _total < 5) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid FENCE_TOTAL");
        return;
    }

    if (packet.count != _total) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, bad count (%u vs %u)", packet.count, _total.get());
        return;
    }

    if (packet.idx >= _total) {
        // this is a protocol failure
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, index past total (%u >= %u)", packet.idx, _total.get());
        return;
    }

    if (!check_latlng(packet.lat, packet.lng)) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, bad lat or lng");
        return;
    }

    if (!contains_compatible_fence()) {
        // the GCS has started to upload using the old protocol;
        // ensure we can accept it.  We must be able to index the
        // fence, so it must be valid (minimum number of points)
        if (!format()) {
            return;
        }
    }

    const Vector2l point{
        (int32_t)(packet.lat*1.0e7f),
        (int32_t)(packet.lng*1.0e7f)
    };

    if (packet.idx == 0) {
        // this is the return point; if we have a return point then
        // update it, otherwise create a return point fence thingy
        const FenceIndex *return_point = get_or_create_return_point();
        if (return_point == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Didn't get return point");
#endif
            return;
        }
        uint16_t offset = return_point->storage_offset;
        offset++; // don't overwrite the type!
        if (!write_latlon_to_storage(offset, point)) {
            link.send_text(MAV_SEVERITY_WARNING, "PolyFence: storage write failed");
            return;
        }
    } else if (packet.idx == _total-1) {
        /* this is the fence closing point. We use this to set the vertex
           count of the inclusion fence
        */
        const FenceIndex *inclusion_fence = get_or_create_include_fence();
        if (inclusion_fence == nullptr) {
            return;
        }
        // write type and length
        fence_storage.write_uint8(inclusion_fence->storage_offset, uint8_t(AC_PolyFenceType::POLYGON_INCLUSION));
        fence_storage.write_uint8(inclusion_fence->storage_offset+1, packet.idx-1);
        // and write end of storage marker
        fence_storage.write_uint8(inclusion_fence->storage_offset+2+(packet.idx-1)*8, uint8_t(AC_PolyFenceType::END_OF_STORAGE));
        void_index();
    } else {
        const FenceIndex *inclusion_fence = get_or_create_include_fence();
        if (inclusion_fence == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("no inclusion fences found");
#endif
            return;
        }
        uint16_t offset = inclusion_fence->storage_offset;
        offset++; // skip type
        if (packet.idx > inclusion_fence->count) {
            // expand the storage space
            fence_storage.write_uint8(offset, packet.idx); // remembering that idx[0] is the return point....
        }
        offset++; // move past number of points
        offset += (packet.idx-1)*8;
        if (!write_latlon_to_storage(offset, point)) {
            link.send_text(MAV_SEVERITY_WARNING, "PolyFence: storage write failed");
            return;
        }
        if (_eos_offset < offset) {
            if (!write_eos_to_storage(offset)) {
                return;
            }
        }
        void_index();
    }
}

bool AC_PolyFence_loader::contains_compatible_fence() const
{
    // must contain a single inclusion fence with an optional return point
    if (_index == nullptr) {
        // this indicates no boundary points present
        return true;
    }
    bool seen_return_point = false;
    bool seen_poly_inclusion = false;
    for (uint16_t i=0; i<_num_fences; i++) {
        switch (_index[i].type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("end-of-storage marker found in loaded list");
#endif
            return false;
        case AC_PolyFenceType::POLYGON_INCLUSION:
            if (seen_poly_inclusion) {
                return false;
            }
            seen_poly_inclusion = true;
            break;
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            return false;
        case AC_PolyFenceType::RETURN_POINT:
            if (seen_return_point) {
                return false;
            }
            seen_return_point = true;
            break;
        }
    }
    return true;
}

#endif // AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT

bool AC_PolyFence_loader::write_eos_to_storage(uint16_t &offset)
{
    if (!write_type_to_storage(offset, AC_PolyFenceType::END_OF_STORAGE)) {
        return false;
    }
    _eos_offset = offset - 1; // should point to the marker
    return true;
}

/// handler for polygon fence messages with GCS
void AC_PolyFence_loader::handle_msg(GCS_MAVLINK &link, const mavlink_message_t& msg)
{
    switch (msg.msgid) {
#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
    case MAVLINK_MSG_ID_FENCE_POINT:
        handle_msg_fence_point(link, msg);
        break;
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        handle_msg_fetch_fence_point(link, msg);
        break;
#endif
    default:
        break;
    }
}

void AC_PolyFence_loader::update()
{
#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
    // if an older GCS sets the fence point count to zero then clear the fence
    if (_old_total != _total) {
        _old_total = _total;
        if (_total == 0 && _eeprom_fence_count) {
            if (!format()) {
                // we are in all sorts of trouble
                return;
            }
        }
    }
#endif
    if (!load_from_eeprom()) {
        return;
    }
}

#else  // build type is not appropriate; provide a dummy implementation:

void AC_PolyFence_loader::init() {};

bool AC_PolyFence_loader::get_item(const uint16_t seq, AC_PolyFenceItem &item) { return false; }

Vector2f* AC_PolyFence_loader::get_exclusion_polygon(uint16_t index, uint16_t &num_points) const { return nullptr; }
Vector2f* AC_PolyFence_loader::get_inclusion_polygon(uint16_t index, uint16_t &num_points) const { return nullptr; }

bool AC_PolyFence_loader::get_exclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const { return false; }
bool AC_PolyFence_loader::get_inclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const { return false; }

void AC_PolyFence_loader::handle_msg(GCS_MAVLINK &link, const mavlink_message_t& msg) {};

bool AC_PolyFence_loader::breached() const { return false; }
bool AC_PolyFence_loader::breached(const Location& loc) const { return false; }

uint16_t AC_PolyFence_loader::max_items() const { return 0; }

bool AC_PolyFence_loader::write_fence(const AC_PolyFenceItem *new_items, uint16_t count) { return false; }

void AC_PolyFence_loader::update() {};

#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
bool AC_PolyFence_loader::get_return_point(Vector2l &ret) { return false; }
#endif

#endif // #if AC_FENCE_DUMMY_METHODS_ENABLED
#endif // AP_FENCE_ENABLED
