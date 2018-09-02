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

// validate array of boundary points (expressed as either floats or long ints)
//   contains_return_point should be true for plane which stores the return point as the first point in the array
//   returns true if boundary is valid
template <typename T>
bool AC_PolyFence_loader::boundary_valid(uint16_t num_points, const Vector2<T>* points, bool contains_return_point) const
{
    // exit immediate if no points
    if (points == nullptr) {
        return false;
    }

    // start from 2nd point if boundary contains return point (as first point)
    uint8_t start_num = contains_return_point ? 1 : 0;

    // a boundary requires at least 4 point (a triangle and last point equals first)
    if (num_points < start_num + 4) {
        return false;
    }

    // point 1 and last point must be the same.  Note: 0th point is reserved as the return point
    if (!Polygon_complete(&points[start_num], num_points-start_num)) {
        return false;
    }

    // check return point is within the fence
    if (contains_return_point && Polygon_outside(points[0], &points[1], num_points-start_num)) {
        return false;
    }

    return true;
}

// check if a location (expressed as either floats or long ints) is within the boundary
//   contains_return_point should be true for plane which stores the return point as the first point in the array
//   returns true if location is outside the boundary
template <typename T>
bool AC_PolyFence_loader::boundary_breached(const Vector2<T>& location, uint16_t num_points, const Vector2<T>* points,
                                            bool contains_return_point) const
{
    // exit immediate if no points
    if (points == nullptr) {
        return false;
    }

    // start from 2nd point if boundary contains return point (as first point)
    uint8_t start_num = contains_return_point ? 1 : 0;

    // check location is within the fence
    return Polygon_outside(location, &points[start_num], num_points-start_num);
}

// declare type specific methods
template bool AC_PolyFence_loader::boundary_valid<int32_t>(uint16_t num_points, const Vector2l* points, bool contains_return_point) const;
template bool AC_PolyFence_loader::boundary_valid<float>(uint16_t num_points, const Vector2f* points, bool contains_return_point) const;
template bool AC_PolyFence_loader::boundary_breached<int32_t>(const Vector2l& location, uint16_t num_points,
                                                              const Vector2l* points, bool contains_return_point) const;
template bool AC_PolyFence_loader::boundary_breached<float>(const Vector2f& location, uint16_t num_points,
                                                            const Vector2f* points, bool contains_return_point) const;
