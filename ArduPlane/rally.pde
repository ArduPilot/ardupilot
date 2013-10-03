// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * rally point support
 * Michael Day, September 2013
 */

//get a rally point
static RallyLocation get_rally_point_with_index(unsigned i)
{
    uint16_t mem;
    RallyLocation ret;

    if (i > (unsigned) g.rally_total.get()) {
        ret.lat = 0; ret.lng = 0; ret.alt = 0;
        ret.land_dir = 0; ret.flags = 0;
        return ret;
    }

    //read rally point
    mem = RALLY_START_BYTE + (i * RALLY_WP_SIZE);
    ret.lat = hal.storage->read_dword(mem);
    mem += sizeof(uint32_t);
    ret.lng = hal.storage->read_dword(mem);
    mem += sizeof(uint32_t);
    ret.alt = hal.storage->read_word(mem);
    mem += sizeof(int16_t);
    ret.break_alt = hal.storage->read_word(mem);
    mem += sizeof(int16_t);
    ret.land_dir = hal.storage->read_word(mem);
    mem += sizeof(uint16_t);
    ret.flags = hal.storage->read_byte(mem);

    return ret;    
}

//save a rally point
static void set_rally_point_with_index(RallyLocation &rallyLoc, unsigned i)
{
    uint16_t mem;

    if (i >= (unsigned) g.rally_total.get()) {
        //not allowed
        return;
    }

    if (i >= MAX_RALLYPOINTS) {
        //also not allowed
        return;
    }

    mem = RALLY_START_BYTE + (i * RALLY_WP_SIZE);

    hal.storage->write_dword(mem, rallyLoc.lat);
    mem += sizeof(uint32_t);
    hal.storage->write_dword(mem, rallyLoc.lng);
    mem += sizeof(uint32_t);
    hal.storage->write_word(mem, rallyLoc.alt);
    mem += sizeof(int16_t);
    hal.storage->write_word(mem, rallyLoc.break_alt);
    mem += sizeof(int16_t);
    hal.storage->write_word(mem, rallyLoc.land_dir);
    mem += sizeof(uint32_t);
    hal.storage->write_byte(mem, rallyLoc.flags);
}

//'best' means 'closest to my current location' for now.
RallyLocation find_best_rally_point() {
    RallyLocation ret;
    RallyLocation next_rally;
    Location rally_loc;
    float dis;
    float min_dis = 999999999.9f;

    for (unsigned i = 0; i < (unsigned) g.rally_total.get(); i++) {
        next_rally = get_rally_point_with_index(i);
        rally_loc = rally_location_to_location(next_rally);
        dis = get_distance(current_loc, rally_loc);

        if (dis < min_dis) {
            min_dis = dis;
            ret = next_rally;
        }
    }

    return ret;
}

//translate a RallyLocation to a Location
Location rally_location_to_location (const RallyLocation &r_loc) {
    Location ret;

    ret.id = MAV_CMD_NAV_LOITER_UNLIM;
    ret.options = MASK_OPTIONS_RELATIVE_ALT;

    //Currently can't do true AGL on the APM.  Relative altitudes are
    //relative to HOME point's altitude.  Terrain on the board is inbound
    //for the PX4, though.  This line will need to be updated when that happens:
    ret.alt = r_loc.alt + home.alt;

    ret.lat = r_loc.lat;
    ret.lng = r_loc.lng;

    return ret;
}
