#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

class AC_PolyFence_loader
{

public:

    AC_PolyFence_loader(AP_Int8 &total) :
        _total(total) {}

    AC_PolyFence_loader(const AC_PolyFence_loader &other) = delete;
    AC_PolyFence_loader &operator=(const AC_PolyFence_loader&) = delete;

    /// returns pointer to array of polygon points and num_points is
    /// filled in with the total number.  This does not include the
    /// return point or the closing point.
    Vector2f* get_boundary_points(uint16_t& num_points) const;

    /// handler for polygon fence messages with GCS
    void handle_msg(GCS_MAVLINK &link, mavlink_message_t* msg);

    bool breached();
    //   returns true if location is outside the boundary
    bool breached(const Location& loc);

    //   returns true if boundary is valid
    bool valid() const {
        return _valid;
    }

    // returns the system in in millis when the boundary was last updated
    uint32_t update_ms() const {
        return _update_ms;
    }

private:

    bool breached(const Vector2f& location);
    /// load polygon points stored in eeprom into boundary array and
    /// perform validation.  returns true if load successfully
    /// completed
    bool load_from_eeprom();

    // maximum number of fence points we can store in eeprom
    uint8_t max_points() const;

    // create buffer to hold copy of eeprom points in RAM
    // returns nullptr if not enough memory can be allocated
    void* create_point_array(uint8_t element_size);

    // load boundary point from eeprom, returns true on successful load
    bool load_point_from_eeprom(uint16_t i, Vector2l& point);

    // save a fence point to eeprom, returns true on successful save
    bool save_point_to_eeprom(uint16_t i, const Vector2l& point);

    // update the validity flag:
    bool calculate_boundary_valid() const;

    Vector2f *_boundary;          // array of boundary points.  Note: point 0 is the return point
    uint8_t _boundary_num_points; // number of points in the boundary array (should equal _total parameter after load has completed)
    bool   _create_attempted;     // true if we have attempted to create the boundary array
    bool   _valid;                // true if boundary forms a closed polygon
    uint32_t        _update_ms;   // system time of last update to the boundary

    AP_Int8 &_total;
};

