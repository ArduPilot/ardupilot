/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/// @file	MAVLink_routing.h
/// @brief	handle routing of MAVLink packets by sysid/componentid

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS.h"
#include "MAVLink_routing.h"

#include <AP_ADSB/AP_ADSB.h>
#include <AP_Logger/AP_Logger_config.h>

extern const AP_HAL::HAL& hal;

#define ROUTING_DEBUG 0

// constructor
MAVLink_routing::MAVLink_routing(void) : num_routes(0) {}

/*
  forward a MAVLink message to the right port. This also
  automatically learns the route for the sender if it is not
  already known.
  
  This returns true if the message should be processed locally

  Theory of MAVLink routing:

  When a flight controller receives a message it should process it
  locally if any of these conditions hold:

    1a) the message has no target_system field

    1b) the message has a target_system of zero and has no
        target_component field, a target_component of zero or one of
        the flight controllers component IDs

    1c) the message has the flight controllers target system and has no
       target_component field

    1d) the message has the flight controllers target system and has
       one of the flight controllers component IDs as its
       target_component.  As well as its own component ID the flight
       controller answers to the IDs of components it sends messages
       as (e.g. MAV_COMP_ID_LOG)

    1e) the MAV_OPTIONS bit ACCEPT_COMMANDS_FOR_OTHER_COMPONENTS is set
        and the message has a target_system of zero, or has the flight
        controllers target system and the flight controller has not
        seen any messages on any of its links from a system that has
        the messages target_system/target_component combination.  This
        is the historical behaviour; by default a message addressed to
        another component is not processed locally

    1f) the message has a target_system of zero or the flight
        controllers target system, is one of a few safety-of-life
        commands (see message_is_component_agnostic()) and was not
        forwarded to its target_component on another link

  When a flight controller receives a message it should forward it
  onto another different link if any of these conditions hold for that
  link: 

    2a) the message has no target_system field

    2b) the message has a target_system of zero

    2c) the message does not have the flight controllers target_system
        and the flight controller has seen a message from the messages
        target_system on the link

    2d) the message has the flight controllers target_system and has a
        target_component field and the flight controllers has seen a
        message from the target_system/target_component combination on
        the link

Note: This proposal assumes that ground stations will not send command
packets to a non-broadcast destination (sysid/compid combination)
until they have received at least one package from that destination
over the link. This is essential to prevent a flight controller from
acting on a message that is not meant for it. For example, a PARAM_SET
cannot be sent to a specific sysid/compid combination until the GCS
has seen a packet from that sysid/compid combination on the link. 

The GCS must also reset what sysid/compid combinations it has seen on
a link when it sees a SYSTEM_TIME message with a decrease in
time_boot_ms from a particular sysid/compid. That is essential to
detect a reset of the flight controller, which implies a reset of its
routing table.

*/
bool MAVLink_routing::check_and_forward(uint8_t framing_status,
                                        GCS_MAVLINK &in_link,
                                        const mavlink_message_t &msg)
{
    // handle the case of loopback of our own messages, due to
    // incorrect serial configuration.
    if (msg.sysid == mavlink_system.sysid &&
        msg.compid == mavlink_system.compid) {
        return false;  // do not process locally
    }

    switch (framing_status) {
    case MAVLINK_FRAMING_OK:
        break;
    case MAVLINK_FRAMING_BAD_CRC:
        if (in_link.option_enabled(GCS_MAVLINK::Option::FORWARD_BAD_CRC) &&
            msg.msgid != MAVLINK_MSG_ID_RADIO &&
            msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS &&
            msg.msgid != MAVLINK_MSG_ID_ADSB_VEHICLE) {
            forward(in_link, msg);
        }
        return false;  // do not process locally
    case MAVLINK_FRAMING_BAD_SIGNATURE:
    default:
        return false;  // do not process locally
    }

#if HAL_SOLO_GIMBAL_ENABLED
    // check if a Gopro is connected. If yes, we allow the routing
    // of mavlink messages to a private channel (Solo Gimbal case)
    if (!gopro_status_check && (msg.msgid == MAVLINK_MSG_ID_GOPRO_HEARTBEAT)) {
       gopro_status_check = true;
       GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "GoPro in Solo gimbal detected");
    }
#endif // HAL_SOLO_GIMBAL_ENABLED

    // learn new routes including private channels
    // so that find_mav_type works for all channels
    learn_route(in_link, msg);

    if (msg.msgid == MAVLINK_MSG_ID_RADIO ||
        msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        // don't forward RADIO packets
        return true;
    }

    const bool from_private_channel = in_link.is_private();

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        // heartbeat needs special handling
        if (!from_private_channel) {
            handle_heartbeat(in_link, msg);
        }
        return true;
    }

    /*
     * Note that if you are looking at handling routing especially for
     * certain packets here then you may need to add your message to
     * the list of IDs we will not forward *even if* the option to
     * send bad-CRC packets has been set (this is in a switch
     * statement, above).
     */

#if HAL_ADSB_ENABLED
    if (msg.msgid == MAVLINK_MSG_ID_ADSB_VEHICLE) {
        // if enabled ADSB packets are not forwarded, they have their own stream rate
        const AP_ADSB *adsb = AP::ADSB();
        if ((adsb != nullptr) && (adsb->enabled())) {
            return true;
        }
    }
#endif

    return forward(in_link, msg);
}

/*
  return true if compid is one of our component IDs.  As well as the
  autopilot's own component ID we answer to the IDs of components we
  send messages as.

  Note that a message addressed to our system and one of these
  component IDs is taken to be for us alone, so it is not forwarded;
  if something else on another link uses the same component ID (e.g. a
  companion computer's own MAV_COMP_ID_LOG) then it will not see
  messages addressed to that ID.  A message addressed to all systems
  is still forwarded as usual.
 */
static bool compid_is_ours(int16_t compid)
{
    if (compid == mavlink_system.compid) {
        return true;
    }
#if HAL_LOGGING_MAVLINK_ENABLED
    // AP_Logger_MAVLink sends log blocks as MAV_COMP_ID_LOG, so the
    // REMOTE_LOG_BLOCK_STATUS replies are addressed to that component
    if (compid == MAV_COMP_ID_LOG) {
        return true;
    }
#endif
    return false;
}

/*
  return true if this message is a command, filling in the command it
  carries
 */
static bool command_from_message(const mavlink_message_t &msg, uint16_t &command)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        command = mavlink_msg_command_long_get_command(&msg);
        return true;
    case MAVLINK_MSG_ID_COMMAND_INT:
        command = mavlink_msg_command_int_get_command(&msg);
        return true;
    }

    return false;
}

#if AP_MAVLINK_COMMANDS_FOR_OTHER_COMPONENTS_ENABLED
/*
  return true if this message must be acted upon when it is addressed
  to a component of our system other than our own which we have never
  seen a message from.

  These are the last-resort, safety-of-life actions; if a GCS addresses
  one at a component which does not appear to exist then acting on it
  ourselves is much better than discarding it.  If the command is
  forwarded on to the addressed component then we keep out of it.  A
  component we only know of on the link the command arrived on has
  already been sent the command, so it is not forwarded and we act on it
  as well.
 */
static bool message_is_component_agnostic(const mavlink_message_t &msg)
{
    uint16_t command;
    if (!command_from_message(msg, command)) {
        return false;
    }

    switch (command) {
    case MAV_CMD_DO_PARACHUTE:
    case MAV_CMD_DO_FLIGHTTERMINATION:
        return true;
    }

    return false;
}
#endif  // AP_MAVLINK_COMMANDS_FOR_OTHER_COMPONENTS_ENABLED

bool MAVLink_routing::forward(GCS_MAVLINK &in_link,
                              const mavlink_message_t &msg)
{
    // extract the targets for this packet
    int16_t target_system = -1;
    int16_t target_component = -1;
    get_targets(msg, target_system, target_component);

    bool broadcast_system = (target_system == 0 || target_system == -1);
    bool broadcast_component = (target_component == 0 || target_component == -1);
    bool match_system = broadcast_system || (target_system == mavlink_system.sysid);
    bool match_component = match_system && (broadcast_component ||
                                            compid_is_ours(target_component));
    bool process_locally = match_system && match_component;

#if AP_MAVLINK_COMMANDS_FOR_OTHER_COMPONENTS_ENABLED
    // a few commands are acted upon when they are addressed to another
    // component of our system which we have no route to:
    const bool component_agnostic = (match_system && !match_component &&
                                     message_is_component_agnostic(msg));
#else
    const bool component_agnostic = false;
#endif

    // don't ever forward data from a private channel
    // unless a Gopro camera is connected to a Solo gimbal
    const bool from_private_channel = in_link.is_private();
    bool should_process_locally = from_private_channel;
#if HAL_SOLO_GIMBAL_ENABLED
    if (gopro_status_check) {
        should_process_locally = false;
    }
#endif
    if (should_process_locally) {
        // nothing is forwarded from a private channel, so the
        // component-agnostic commands are handled here or not at all.
        // Note that this changes once a GoPro is detected on a Solo
        // gimbal; the channel then forwards like any other, so a
        // command for a component we have a route to stops being
        // handled here.
        if (component_agnostic) {
            process_locally = true;
        }
        if (match_system && !match_component) {
            warn_about_message_for_other_component(msg, process_locally, target_component);
        }
        return process_locally;
    }

    if (process_locally && !broadcast_system && !broadcast_component) {
        // nothing more to do - it can only be for us
        return true;
    }

    // forward on any channels matching the targets
    bool forwarded = false;
    // true if the message was sent to the component of our system it
    // is addressed to; a message for all systems is forwarded to every
    // route, so "forwarded" alone does not tell us that
    bool forwarded_to_component = false;
    // true if the link the message arrived on reaches the component of
    // our system it is addressed to; that component has been sent it
    bool target_component_on_in_link = false;
    bool sent_to_chan[MAVLINK_COMM_NUM_BUFFERS];
    memset(sent_to_chan, 0, sizeof(sent_to_chan));
    // true if the message was sent on the channel, i.e. it fitted in the
    // transmit buffer; never true for the channel it arrived on
    bool queued_on_chan[MAVLINK_COMM_NUM_BUFFERS];
    memset(queued_on_chan, 0, sizeof(queued_on_chan));
    for (uint8_t i=0; i<num_routes; i++) {

        // Skip if channel is private and the target system or component IDs do not match
        GCS_MAVLINK *out_link = gcs().chan(routes[i].channel);
        if (out_link == nullptr) {
            // this is bad
            continue;
        }
        if (out_link->is_private() &&
            (target_system != routes[i].sysid ||
             target_component != routes[i].compid)) {
            continue;
        }

        if (broadcast_system || (target_system == routes[i].sysid &&
                                 (broadcast_component || 
                                  target_component == routes[i].compid ||
                                  !match_system))) {

            if (&in_link != out_link && !sent_to_chan[routes[i].channel]) {
                queued_on_chan[routes[i].channel] = out_link->check_payload_size(msg.len);
                if (queued_on_chan[routes[i].channel]) {
#if ROUTING_DEBUG
                    ::printf("fwd msg %u from chan %u on chan %u sysid=%d compid=%d\n",
                             msg.msgid,
                             (unsigned)in_link.get_chan(),
                             (unsigned)routes[i].channel,
                             (int)target_system,
                             (int)target_component);
#endif
                    _mavlink_resend_uart(routes[i].channel, &msg);
                }
                sent_to_chan[routes[i].channel] = true;
                forwarded = true;
            }
            // a component with the same ID on another system is not the
            // one the message is addressed to, even if the message is
            // for all systems:
            const bool route_to_target_component = (routes[i].sysid == mavlink_system.sysid &&
                                                    routes[i].compid == target_component);
            if (route_to_target_component && queued_on_chan[routes[i].channel]) {
                forwarded_to_component = true;
            }
            if (route_to_target_component && &in_link == out_link) {
                target_component_on_in_link = true;
            }
        }
    }

    if (component_agnostic && !forwarded_to_component) {
        // the component-agnostic commands for another component of our
        // system are handled regardless of MAV_OPTIONS unless we sent
        // them on to that component.  Note that they may still have been
        // forwarded elsewhere, e.g. to every link if for all systems.
        process_locally = true;
    }

    if (!match_component &&
        gcs().option_is_enabled(GCS::Option::ACCEPT_COMMANDS_FOR_OTHER_COMPONENTS) &&
        ((!forwarded && match_system) || broadcast_system)) {
        // the message is explicitly addressed to another component.  By
        // default we do not act on it; ACCEPT_COMMANDS_FOR_OTHER_COMPONENTS
        // restores the historical behaviour of handling it ourselves if
        // it is for our system and we found nowhere to forward it to, or
        // if it is for all systems (whether or not we forwarded it).
        process_locally = true;
    }

    // When we discard a message addressed to another component we
    // deliberately remain absolutely silent towards its sender: no
    // COMMAND_ACK, NACK or any other reply is sent.  Anything we sent
    // would claim to come from us about a message which was never ours,
    // and the component it was addressed to may yet answer it.  The
    // rate-limited STATUSTEXT below is for the user, not a reply.
    //
    // A message we sent on to the component it is addressed to was
    // neither acted on nor discarded, so there is nothing to warn about.
    // Nor is there when we discard a message for a component which the
    // link it arrived on reaches (e.g. a GCS and a camera sharing a
    // companion computer's link to us), as that component has been sent
    // it too.  We do still warn about acting on one:
    if (match_system && !match_component &&
        (process_locally || !(forwarded_to_component || target_component_on_in_link))) {
        warn_about_message_for_other_component(msg, process_locally, target_component);
    }

    return process_locally;
}

/*
  tell the user about a message addressed at a component other than our
  own which we are either acting on or discarding; the sender is talking
  to something which is not us, and that is worth knowing about.  If
  MAV_OPTIONS says to act on such messages then acting on one is
  expected, so is not warned about.  A GCS may well send such messages
  continuously, so warn at most once every 10 seconds.  Acting on and
  discarding are rate limited separately so that a stream of messages
  being discarded cannot hide our acting on a parachute or flight
  termination command.
 */
void MAVLink_routing::warn_about_message_for_other_component(const mavlink_message_t &msg,
                                                             bool process_locally,
                                                             int16_t target_component)
{
    if (process_locally &&
        gcs().option_is_enabled(GCS::Option::ACCEPT_COMMANDS_FOR_OTHER_COMPONENTS)) {
        return;
    }
    uint32_t &last_warning_ms = process_locally ? last_acting_on_warning_ms : last_ignoring_warning_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (last_warning_ms != 0 &&
        now_ms - last_warning_ms < 10000) {
        return;
    }
    last_warning_ms = now_ms;
    const char *action = process_locally ? "acting on" : "ignoring";
    // for a command the command ID is much more useful than the message ID:
    uint16_t command;
    if (command_from_message(msg, command)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MAV: %s cmd %u for compid %d",
                      action, command, target_component);
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MAV: %s msg %u for compid %d",
                  action, (unsigned)msg.msgid, target_component);
}

/*
  send a MAVLink message to all components with this vehicle's system id

  This is a no-op if no routes to components have been learned
*/
void MAVLink_routing::send_to_components(uint32_t msgid, const char *pkt, uint8_t pkt_len)
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) {
        return;
    }
    send_to_components(pkt, entry, pkt_len);
}

void MAVLink_routing::send_to_components(const char *pkt, const mavlink_msg_entry_t *entry, const uint8_t pkt_len)
{
    bool sent_to_chan[MAVLINK_COMM_NUM_BUFFERS] {};

    // check learned routes
    for (uint8_t i=0; i<num_routes; i++) {
        if (routes[i].sysid != mavlink_system.sysid) {
            // our system ID hasn't been seen on this link
            continue;
        }
        if (sent_to_chan[routes[i].channel]) {
            // we've already send it on this link
            continue;
        }
        if (comm_get_txspace(routes[i].channel) <
            ((uint16_t)entry->max_msg_len) + GCS_MAVLINK::packet_overhead_chan(routes[i].channel)) {
            // it doesn't fit on this channel
            continue;
        }
#if ROUTING_DEBUG
        ::printf("send msg %u on chan %u sysid=%u compid=%u\n",
                 entry->msgid,
                 (unsigned)routes[i].channel,
                 (unsigned)routes[i].sysid,
                 (unsigned)routes[i].compid);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (entry->max_msg_len > pkt_len) {
            AP_HAL::panic("Passed packet message length (%u > %u)",
                          entry->max_msg_len, pkt_len);
        }
#endif
        _mav_finalize_message_chan_send(routes[i].channel,
                                        entry->msgid,
                                        pkt,
                                        entry->min_msg_len,
                                        MIN(entry->max_msg_len, pkt_len),
                                        entry->crc_extra);
        sent_to_chan[routes[i].channel] = true;
    }
}

/*
  search for the first vehicle or component in the routing table with given mav_type and retrieve it's sysid, compid and channel
  returns true if a match is found
 */
bool MAVLink_routing::find_by_mavtype(uint8_t mavtype, uint8_t &sysid, uint8_t &compid, mavlink_channel_t &channel)
{
    // check learned routes
    for (uint8_t i=0; i<num_routes; i++) {
        if (routes[i].mavtype == mavtype) {
            sysid = routes[i].sysid;
            compid = routes[i].compid;
            channel = routes[i].channel;
            return true;
        }
    }

    // if we've reached we have not found the component
    return false;
}

/*
  search for the first vehicle or component in the routing table with given mav_type and component id and retrieve its sysid and channel
  returns true if a match is found
 */
bool MAVLink_routing::find_by_mavtype_and_compid(uint8_t mavtype, uint8_t compid, uint8_t &sysid, mavlink_channel_t &channel) const
{
    for (uint8_t i=0; i<num_routes; i++) {
        if ((routes[i].mavtype == mavtype) && (routes[i].compid == compid)) {
            sysid = routes[i].sysid;
            channel = routes[i].channel;
            return true;
        }
    }
    return false;
}

/*
  see if the message is for a new route and learn it
*/
void MAVLink_routing::learn_route(GCS_MAVLINK &in_link, const mavlink_message_t &msg)
{
    uint8_t i;
    if (msg.sysid == 0) {
        // don't learn routes to the broadcast system
        return;
    }
    if (msg.sysid == mavlink_system.sysid &&
        msg.compid == mavlink_system.compid) {
        // don't learn routes to ourself.  We know where we are.
        return;
    }
    if (msg.sysid == mavlink_system.sysid &&
        msg.compid == MAV_COMP_ID_ALL) {
        // don't learn routes to the broadcast component ID for our
        // own system id.  We should still broadcast these, but we
        // should also process them locally.
        return;
    }
    const mavlink_channel_t in_channel = in_link.get_chan();
    for (i=0; i<num_routes; i++) {
        if (routes[i].sysid == msg.sysid &&
            routes[i].compid == msg.compid &&
            routes[i].channel == in_channel) {
            if (routes[i].mavtype == 0 && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                routes[i].mavtype = mavlink_msg_heartbeat_get_type(&msg);
            }
            break;
        }
    }
    if (i == num_routes && i<MAVLINK_MAX_ROUTES) {
        routes[i].sysid = msg.sysid;
        routes[i].compid = msg.compid;
        routes[i].channel = in_channel;
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            routes[i].mavtype = mavlink_msg_heartbeat_get_type(&msg);
        }
        num_routes++;
#if ROUTING_DEBUG
        ::printf("learned route %u %u via %u\n",
                 (unsigned)msg.sysid,
                 (unsigned)msg.compid,
                 (unsigned)in_channel);
#endif
    }
}


/*
  special handling for heartbeat messages. To ensure routing
  propagation heartbeat messages need to be forwarded on all channels
  except channels where the sysid/compid of the heartbeat could come from
*/
void MAVLink_routing::handle_heartbeat(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() & ~GCS_MAVLINK::private_channel_mask();

    const mavlink_channel_t in_channel = link.get_chan();

    // don't send on the incoming channel. This should only matter if
    // the routing table is full
    mask &= ~(1U<<(in_channel-MAVLINK_COMM_0));
    
    // mask out channels that do not want the heartbeat to be forwarded
    mask &= ~no_route_mask;
    
    // mask out channels that are known sources for this sysid/compid
    for (uint8_t i=0; i<num_routes; i++) {
        if (routes[i].sysid == msg.sysid && routes[i].compid == msg.compid) {
            mask &= ~(1U<<((unsigned)(routes[i].channel-MAVLINK_COMM_0)));
        }
    }

    if (mask == 0) {
        // nothing to send to
        return;
    }

    // send on the remaining channels
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (mask & (1U<<i)) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            if (comm_get_txspace(channel) >= ((uint16_t)msg.len) +
                GCS_MAVLINK::packet_overhead_chan(channel)) {
#if ROUTING_DEBUG
                ::printf("fwd HB from chan %u on chan %u from sysid=%u compid=%u\n",
                         (unsigned)in_channel,
                         (unsigned)channel,
                         (unsigned)msg.sysid,
                         (unsigned)msg.compid);
#endif
                _mavlink_resend_uart(channel, &msg);
            }
        }
    }
}


/*
  extract target sysid and compid from a message. int16_t is used so
  that the caller can set them to -1 and know when a sysid or compid
  target is found in the message
*/
void MAVLink_routing::get_targets(const mavlink_message_t &msg, int16_t &sysid, int16_t &compid)
{
    const mavlink_msg_entry_t *msg_entry = mavlink_get_msg_entry(msg.msgid);
    if (msg_entry == nullptr) {
        return;
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
        sysid = _MAV_RETURN_uint8_t(&msg,  msg_entry->target_system_ofs);
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
        compid = _MAV_RETURN_uint8_t(&msg,  msg_entry->target_component_ofs);
    }
}

#endif  // HAL_GCS_ENABLED
