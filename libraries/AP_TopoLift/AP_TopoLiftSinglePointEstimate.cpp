#include "AP_TopoLiftSinglePointEstimate.h"
#include <AP_Terrain/AP_Terrain.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/vector2.h>


bool AP_TopoLiftSinglePointEstimate::get_terrain_height(const Location loc, float& height_amsl) {

    AP_Terrain *terrain = AP::terrain();
    if (terrain == nullptr) {
        return false;
    }

    // status() handles enable check.
    if (terrain->status() != AP_Terrain::TerrainStatusOK) {
        return false;
    }

    const auto corrected = true;
    return terrain->height_amsl(loc, height_amsl, corrected);


    return true;
}

bool AP_TopoLiftSinglePointEstimate::get_wind_at_current_loc(Vector3f& wind) {
    const AP_AHRS &ahrs = AP::ahrs();

    return ahrs.wind_estimate(wind);
}

bool AP_TopoLiftSinglePointEstimate::calc_lift(const Location loc, float& lift) {
    float height_amsl;
    if (!get_terrain_height(loc, height_amsl)) {
        return false;
    }

    Vector3f wind;
    if (!get_wind_at_current_loc(wind)) {
        return false;
    }
    
    // TODO calculate gradient of terrain.
    Vector2f terrain_grad;

    // TODO find a way to communicate steepness
    [[maybe_unused]] auto const terrain_steepness = terrain_grad.length();


    // TODO compute lift based on gradient, wind, and altitude above terrain.
    lift = 42.0;

    return true;
}
