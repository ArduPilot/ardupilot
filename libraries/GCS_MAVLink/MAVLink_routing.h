// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	MAVLink_routing.h
/// @brief	handle routing of MAVLink packets by ID

#ifndef __MAVLINK_ROUTING_H
#define __MAVLINK_ROUTING_H

#include <AP_HAL.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>

// 20 routes should be enough for now. This may need to increase as
// we make more extensive use of MAVLink forwarding
#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
#define MAVLINK_MAX_ROUTES 20
#else
#define MAVLINK_MAX_ROUTES 5
#endif

/*
  object to handle MAVLink packet routing
 */
class MAVLink_routing
{
public:
    MAVLink_routing(void);

    /*
      forward a MAVLink message to the right port. This also
      automatically learns the route for the sender if it is not
      already known.

      This returns true if the message should be processed locally
    */
    bool check_and_forward(mavlink_channel_t in_channel, const mavlink_message_t* msg);

private:
    // a simple linear routing table. We don't expect to have a lot of
    // routes, so a scalable structure isn't worthwhile yet.
    uint8_t num_routes;
    struct route {
        uint8_t sysid;
        uint8_t compid;
        mavlink_channel_t channel;
    } routes[MAVLINK_MAX_ROUTES];

    // learn new routes
    void learn_route(mavlink_channel_t in_channel, const mavlink_message_t* msg);

    // extract target sysid and compid from a message
    void get_targets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);

    // special handling for heartbeat messages
    void handle_heartbeat(mavlink_channel_t in_channel, const mavlink_message_t* msg);
};

#endif // __MAVLINK_ROUTING_H

