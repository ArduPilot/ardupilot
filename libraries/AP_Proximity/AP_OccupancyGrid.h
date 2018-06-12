#pragma once

#include <stdint.h>
#include <AP_Param/AP_Param.h>

class AP_OccupancyGrid_Cell {

private:
    uint8_t last_full_ms;
};


class AP_OccupancyGrid {

public:

    void init();

    // offset_from_origin is in metres
    void occupied_now(Vector2f offset_from_origin);
    bool occupied(Vector2f offset_from_origin) const;

    // returns a distance (in metres) to the nearest thing along a
    // heading, as seen from a particular offset from the origin.
    // returns Inf in the case that we don't know of anything along
    // that heading.
    float clearance_distance(Vector2f offset_from_origin, float heading) const;

private:

//    AP_Int8 sizex;
//   AP_Int8 sizey;

//    AP_Float cellsizex; // m
//    AP_Float cellsizey; // m

    uint8_t sizex = 128;
    uint8_t sizey = 128;

    float cellsizex; // m
    float cellsizey; // m

    AP_OccupancyGrid_Cell **cells;

};
