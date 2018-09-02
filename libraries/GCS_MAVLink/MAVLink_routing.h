/// @file	MAVLink_routing.h
/// @brief	handle routing of MAVLink packets by ID
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"

// 20 routes should be enough for now. This may need to increase as
// we make more extensive use of MAVLink forwarding
#define MAVLINK_MAX_ROUTES 20

/*
  object to handle MAVLink packet routing
 */
class MAVLink_routing
{
    friend class GCS_MAVLINK;
    
public:
    MAVLink_routing(void);

    /*
      forward a MAVLink message to the right port. This also
      automatically learns the route for the sender if it is not
      already known.

      This returns true if the message should be processed locally
    */
    bool check_and_forward(mavlink_channel_t in_channel, const mavlink_message_t* msg);

    /*
      send a MAVLink message to all components with this vehicle's system id
      This is a no-op if no routes to components have been learned
    */
    void send_to_components(const mavlink_message_t* msg);

    /*
      search for the first vehicle or component in the routing table with given mav_type and retrieve it's sysid, compid and channel
      returns true if a match is found
     */
    bool find_by_mavtype(uint8_t mavtype, uint8_t &sysid, uint8_t &compid, mavlink_channel_t &channel);

private:
    // a simple linear routing table. We don't expect to have a lot of
    // routes, so a scalable structure isn't worthwhile yet.
    uint8_t num_routes;
    struct route {
        uint8_t sysid;
        uint8_t compid;
        mavlink_channel_t channel;
        uint8_t mavtype;
    } routes[MAVLINK_MAX_ROUTES];
    
    // a channel mask to block routing as required
    uint8_t no_route_mask;
    
    // learn new routes
    void learn_route(mavlink_channel_t in_channel, const mavlink_message_t* msg);

    // extract target sysid and compid from a message
    void get_targets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);

    // special handling for heartbeat messages
    void handle_heartbeat(mavlink_channel_t in_channel, const mavlink_message_t* msg);
};
