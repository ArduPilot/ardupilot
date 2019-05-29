#include "AC_PolyFence_loader.h"

extern const AP_HAL::HAL& hal;

static const StorageAccess fence_storage(StorageManager::StorageFence);

/*
  maximum number of fencepoints
 */
uint8_t AC_PolyFence_loader::max_points() const
{
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}

// create buffer to hold copy of eeprom points in RAM
// returns nullptr if not enough memory can be allocated
void* AC_PolyFence_loader::create_point_array(uint8_t element_size)
{
    uint32_t array_size = max_points() * element_size;
    if (hal.util->available_memory() < 100U + array_size) {
        // too risky to enable as we could run out of stack
        return nullptr;
    }

    return calloc(1, array_size);
}

// load boundary point from eeprom, returns true on successful load
bool AC_PolyFence_loader::load_point_from_eeprom(uint16_t i, Vector2l& point)
{
    // sanity check index
    if (i >= max_points()) {
        return false;
    }

    // read fence point
    point.x = fence_storage.read_uint32(i * sizeof(Vector2l));
    point.y = fence_storage.read_uint32(i * sizeof(Vector2l) + sizeof(uint32_t));
    return true;
}

// save a fence point to eeprom, returns true on successful save
bool AC_PolyFence_loader::save_point_to_eeprom(uint16_t i, const Vector2l& point)
{
    // sanity check index
    if (i >= max_points()) {
        return false;
    }

    // write point to eeprom
    fence_storage.write_uint32(i * sizeof(Vector2l), point.x);
    fence_storage.write_uint32(i * sizeof(Vector2l)+sizeof(uint32_t), point.y);
    return true;
}

// validate array of boundary points
//   returns true if boundary is valid
bool AC_PolyFence_loader::calculate_boundary_valid() const
{
    // exit immediate if no points
    if (_boundary == nullptr) {
        return false;
    }

    // start from 2nd point as boundary contains return point (as first point)
    uint8_t start_num = 1;

    // a boundary requires at least 4 point (a triangle and last point equals first)
    if (_boundary_num_points < start_num + 4) {
        return false;
    }

    // point 1 and last point must be the same.  Note: 0th point is reserved as the return point
    if (!Polygon_complete(&_boundary[start_num], _boundary_num_points-start_num)) {
        return false;
    }

    // check return point is within the fence
    if (Polygon_outside(_boundary[0], &_boundary[1], _boundary_num_points-start_num)) {
        return false;
    }

    return true;
}

bool AC_PolyFence_loader::breached()
{
    // check if vehicle is outside the polygon fence
    Vector2f position;
    if (!AP::ahrs().get_relative_position_NE_origin(position)) {
        // we have no idea where we are; can't breach the fence
        return false;
    }

    position = position * 100.0f;  // m to cm
    return breached(position);
}

bool AC_PolyFence_loader::breached(const Location& loc)
{
    Vector2f posNE;
    if (!loc.get_vector_xy_from_origin_NE(posNE)) {
        // not breached if we don't now where we are
        return false;
    }
    return breached(posNE);
}

//   returns true if location is outside the boundary
bool AC_PolyFence_loader::breached(const Vector2f& location)
{
    // check consistency of number of points
    if (_boundary_num_points != _total) {
        // Fence is currently not completely loaded.  Can't breach it?!
        load_from_eeprom();
        return false;
    }
    // exit immediate if no points
    if (_boundary == nullptr) {
        return false;
    }
    if (!_valid) {
        // fence isn't valid - can't breach it?!
        return false;
    }

    // start from 2nd point as boundary contains return point (as first point)
    uint8_t start_num = 1;

    // check location is within the fence
    return Polygon_outside(location, &_boundary[start_num], _boundary_num_points-start_num);
}
