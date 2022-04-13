//
// Created by bronislav on 08.04.22.
//
#include "Plane.h"

void Plane::do_naw_user1(const AP_Mission::Mission_Command& cmd)
{
    Location location {};
    location.alt = (cmd.content.k.a | cmd.p1 << 16);
    location.lat = cmd.content.k.lat;
    location.lng = cmd.content.k.lng;
    location.relative_alt = 1;

    set_next_WP(location);




}


bool Plane::verify_user1(const AP_Mission::Mission_Command& cmd)
{
    return false;
}
