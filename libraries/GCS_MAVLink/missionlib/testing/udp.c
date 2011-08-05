/*******************************************************************************
 
 Copyright (C) 2011 Lorenz Meier lm ( a t ) inf.ethz.ch
                and Bryan Godbolt godbolt ( a t ) ualberta.ca
 
 adapted from example written by Bryan Godbolt godbolt ( a t ) ualberta.ca
 
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
 
 ****************************************************************************/
/*
 This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.
 
 
 I compiled this program sucessfully on Ubuntu 10.04 with the following command
 
 gcc -I ../../pixhawk/mavlink/include -o udp-server udp.c
 
 the rt library is needed for the clock_gettime on linux
 */
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();


// FIXME XXX - TO BE MOVED TO XML
enum MAVLINK_WPM_STATES
{
    MAVLINK_WPM_STATE_IDLE = 0,
    MAVLINK_WPM_STATE_SENDLIST,
    MAVLINK_WPM_STATE_SENDLIST_SENDWPS,
    MAVLINK_WPM_STATE_GETLIST,
    MAVLINK_WPM_STATE_GETLIST_GETWPS,
    MAVLINK_WPM_STATE_GETLIST_GOTALL,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES
{
    MAVLINK_WPM_CODE_OK = 0,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
    MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
    MAVLINK_WPM_CODE_ENUM_END
};


/* WAYPOINT MANAGER - MISSION LIB */

#define MAVLINK_WPM_MAX_WP_COUNT 100
#define MAVLINK_WPM_CONFIG_IN_FLIGHT_UPDATE  ///< Enable double buffer and in-flight updates
#define MAVLINK_WPM_TEXT_FEEDBACK            ///< Report back status information as text
#define MAVLINK_WPM_SYSTEM_ID 1
#define MAVLINK_WPM_COMPONENT_ID 1

struct _mavlink_wpm_storage {
	mavlink_waypoint_t waypoints[MAVLINK_WPM_MAX_WP_COUNT];      ///< Currently active waypoints
#ifdef MAVLINK_WPM_CONFIG_IN_FLIGHT_UPDATE
	mavlink_waypoint_t rcv_waypoints[MAVLINK_WPM_MAX_WP_COUNT];  ///< Receive buffer for next waypoints
#endif
	uint16_t count;
	MAVLINK_WPM_STATES current_state;
} mavlink_wpm_storage;


void mavlink_wpm_init(mavlink_wpm_storage* state)
{
	// Set all waypoints to zero
	
	// Set count to zero
	state->count = 0;
	state->current_state = MAVLINK_WPM_STATE_IDLE;
}


PX_WAYPOINTPLANNER_STATES current_state = PX_WPP_IDLE;
uint16_t protocol_current_wp_id = 0;
uint16_t protocol_current_count = 0;
uint8_t protocol_current_partner_systemid = 0;
uint8_t protocol_current_partner_compid = 0;
uint64_t protocol_timestamp_lastaction = 0;

uint64_t timestamp_last_send_setpoint = 0;


/*
 *  @brief Sends an waypoint ack message
 */
void mavlink_wpm_send_waypoint_ack(uint8_t target_systemid, uint8_t target_compid, uint8_t type)
{
    mavlink_message_t msg;
    mavlink_waypoint_ack_t wpa;
	
    wpa.target_system = target_systemid;
    wpa.target_component = target_compid;
    wpa.type = type;
	
    mavlink_msg_waypoint_ack_encode(systemid, compid, &msg, &wpa);
    mavlink_message_t_publish(lcm, "MAVLINK", &msg);
	
    usleep(paramClient->getParamValue("PROTOCOLDELAY"));
	
    if (verbose) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);
}

/*
 *  @brief Broadcasts the new target waypoint and directs the MAV to fly there
 *
 *  This function broadcasts its new active waypoint sequence number and
 *  sends a message to the controller, advising it to fly to the coordinates
 *  of the waypoint with a given orientation
 *
 *  @param seq The waypoint sequence number the MAV should fly to.
 */
void mavlink_wpm_send_waypoint_current(uint16_t seq)
{
    if(seq < waypoints->size())
    {
        mavlink_waypoint_t *cur = waypoints->at(seq);
		
        mavlink_message_t msg;
        mavlink_waypoint_current_t wpc;
		
        wpc.seq = cur->seq;
		
        mavlink_msg_waypoint_current_encode(systemid, compid, &msg, &wpc);
        mavlink_message_t_publish(lcm, "MAVLINK", &msg);
		
        usleep(paramClient->getParamValue("PROTOCOLDELAY"));
		
        if (verbose) printf("Broadcasted new current waypoint %u\n", wpc.seq);
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds\n");
    }
}

/*
 *  @brief Directs the MAV to fly to a position
 *
 *  Sends a message to the controller, advising it to fly to the coordinates
 *  of the waypoint with a given orientation
 *
 *  @param seq The waypoint sequence number the MAV should fly to.
 */
void mavlink_wpm_send_setpoint(uint16_t seq)
{
    if(seq < waypoints->size())
    {
        mavlink_waypoint_t *cur = waypoints->at(seq);
		
        mavlink_message_t msg;
        mavlink_local_position_setpoint_set_t PControlSetPoint;
		
        // send new set point to local IMU
        if (cur->frame == 1)
        {
            PControlSetPoint.target_system = systemid;
            PControlSetPoint.target_component = MAV_COMP_ID_IMU;
            PControlSetPoint.x = cur->x;
            PControlSetPoint.y = cur->y;
            PControlSetPoint.z = cur->z;
            PControlSetPoint.yaw = cur->param4;
			
            mavlink_msg_local_position_setpoint_set_encode(systemid, compid, &msg, &PControlSetPoint);
            mavlink_message_t_publish(lcm, "MAVLINK", &msg);
			
            usleep(paramClient->getParamValue("PROTOCOLDELAY"));
        }
        else
        {
            if (verbose) printf("No new set point sent to IMU because the new waypoint %u had no local coordinates\n", cur->seq);
        }
		
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
        timestamp_last_send_setpoint = now;
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds\n");
    }
}

void mavlink_wpm_send_waypoint_count(uint8_t target_systemid, uint8_t target_compid, uint16_t count)
{
    mavlink_message_t msg;
    mavlink_waypoint_count_t wpc;
	
    wpc.target_system = target_systemid;
    wpc.target_component = target_compid;
    wpc.count = count;
	
    mavlink_msg_waypoint_count_encode(systemid, compid, &msg, &wpc);
    mavlink_message_t_publish(lcm, "MAVLINK", &msg);
	
    if (verbose) printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);
	
    usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

void mavlink_wpm_send_waypoint(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
    if (seq < waypoints->size())
    {
        mavlink_message_t msg;
        mavlink_waypoint_t *wp = waypoints->at(seq);
        wp->target_system = target_systemid;
        wp->target_component = target_compid;
        mavlink_msg_waypoint_encode(systemid, compid, &msg, wp);
        mavlink_message_t_publish(lcm, "MAVLINK", &msg);
        if (verbose) printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);
		
        usleep(paramClient->getParamValue("PROTOCOLDELAY"));
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds\n");
    }
}

void mavlink_wpm_send_waypoint_request(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
    if (seq < waypoints->size())
    {
        mavlink_message_t msg;
        mavlink_waypoint_request_t wpr;
        wpr.target_system = target_systemid;
        wpr.target_component = target_compid;
        wpr.seq = seq;
        mavlink_msg_waypoint_request_encode(systemid, compid, &msg, &wpr);
        mavlink_message_t_publish(lcm, "MAVLINK", &msg);
        if (verbose) printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);
		
        usleep(paramClient->getParamValue("PROTOCOLDELAY"));
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds\n");
    }
}

/*
 *  @brief emits a message that a waypoint reached
 *
 *  This function broadcasts a message that a waypoint is reached.
 *
 *  @param seq The waypoint sequence number the MAV has reached.
 */
void mavlink_wpm_send_waypoint_reached(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_waypoint_reached_t wp_reached;
	
    wp_reached.seq = seq;
	
    mavlink_msg_waypoint_reached_encode(systemid, compid, &msg, &wp_reached);
    mavlink_message_t_publish(lcm, "MAVLINK", &msg);
	
    if (verbose) printf("Sent waypoint %u reached message\n", wp_reached.seq);
	
    usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

float mavlink_wpm_distance_to_segment(uint16_t seq, float x, float y, float z)
{
    if (seq < waypoints->size())
    {
        mavlink_waypoint_t *cur = waypoints->at(seq);
		
        const PxVector3 A(cur->x, cur->y, cur->z);
        const PxVector3 C(x, y, z);
		
        // seq not the second last waypoint
        if ((uint16_t)(seq+1) < waypoints->size())
        {
            mavlink_waypoint_t *next = waypoints->at(seq+1);
            const PxVector3 B(next->x, next->y, next->z);
            const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
            if (r >= 0 && r <= 1)
            {
                const PxVector3 P(A + r*(B-A));
                return (P-C).length();
            }
            else if (r < 0.f)
            {
                return (C-A).length();
            }
            else
            {
                return (C-B).length();
            }
        }
        else
        {
            return (C-A).length();
        }
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds\n");
    }
    return -1.f;
}

float mavlink_wpm_distance_to_point(uint16_t seq, float x, float y, float z)
{
    if (seq < waypoints->size())
    {
        mavlink_waypoint_t *cur = waypoints->at(seq);
		
        const PxVector3 A(cur->x, cur->y, cur->z);
        const PxVector3 C(x, y, z);
		
        return (C-A).length();
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds\n");
    }
    return -1.f;
}


static void mavlink_wpm_mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
    // Handle param messages
    paramClient->handleMAVLinkPacket(msg);
	
    //check for timed-out operations
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
    if (now-protocol_timestamp_lastaction > paramClient->getParamValue("PROTOCOLTIMEOUT") && current_state != PX_WPP_IDLE)
    {
        if (verbose) printf("Last operation (state=%u) timed out, changing state to PX_WPP_IDLE\n", current_state);
        current_state = PX_WPP_IDLE;
        protocol_current_count = 0;
        protocol_current_partner_systemid = 0;
        protocol_current_partner_compid = 0;
        protocol_current_wp_id = -1;
		
        if(waypoints->size() == 0)
        {
            current_active_wp_id = -1;
        }
    }
	
    if(now-timestamp_last_send_setpoint > paramClient->getParamValue("SETPOINTDELAY") && current_active_wp_id < waypoints->size())
    {
        send_setpoint(current_active_wp_id);
    }
	
    switch(msg->msgid)
    {
		case MAVLINK_MSG_ID_ATTITUDE:
        {
            if(msg->sysid == systemid && current_active_wp_id < waypoints->size())
            {
                mavlink_waypoint_t *wp = waypoints->at(current_active_wp_id);
                if(wp->frame == 1)
                {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(msg, &att);
                    float yaw_tolerance = paramClient->getParamValue("YAWTOLERANCE");
                    //compare current yaw
                    if (att.yaw - yaw_tolerance >= 0.0f && att.yaw + yaw_tolerance < 2.f*M_PI)
                    {
                        if (att.yaw - yaw_tolerance <= wp->param4 && att.yaw + yaw_tolerance >= wp->param4)
                            yawReached = true;
                    }
                    else if(att.yaw - yaw_tolerance < 0.0f)
                    {
                        float lowerBound = 360.0f + att.yaw - yaw_tolerance;
                        if (lowerBound < wp->param4 || wp->param4 < att.yaw + yaw_tolerance)
                            yawReached = true;
                    }
                    else
                    {
                        float upperBound = att.yaw + yaw_tolerance - 2.f*M_PI;
                        if (att.yaw - yaw_tolerance < wp->param4 || wp->param4 < upperBound)
                            yawReached = true;
                    }
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_LOCAL_POSITION:
        {
            if(msg->sysid == systemid && current_active_wp_id < waypoints->size())
            {
                mavlink_waypoint_t *wp = waypoints->at(current_active_wp_id);
				
                if(wp->frame == 1)
                {
                    mavlink_local_position_t pos;
                    mavlink_msg_local_position_decode(msg, &pos);
                    if (debug) printf("Received new position: x: %f | y: %f | z: %f\n", pos.x, pos.y, pos.z);
					
                    posReached = false;
					
                    // compare current position (given in message) with current waypoint
                    float orbit = wp->param1;
					
                    float dist;
                    if (wp->param2 == 0)
                    {
                        dist = distanceToSegment(current_active_wp_id, pos.x, pos.y, pos.z);
                    }
                    else
                    {
                        dist = distanceToPoint(current_active_wp_id, pos.x, pos.y, pos.z);
                    }
					
                    if (dist >= 0.f && dist <= orbit && yawReached)
                    {
                        posReached = true;
                    }
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_CMD: // special action from ground station
        {
            mavlink_cmd_t action;
            mavlink_msg_cmd_decode(msg, &action);
            if(action.target == systemid)
            {
                if (verbose) std::cerr << "Waypoint: received message with action " << action.action << std::endl;
                switch (action.action)
                {
						//				case MAV_ACTION_LAUNCH:
						//					if (verbose) std::cerr << "Launch received" << std::endl;
						//					current_active_wp_id = 0;
						//					if (waypoints->size()>0)
						//					{
						//						setActive(waypoints[current_active_wp_id]);
						//					}
						//					else
						//						if (verbose) std::cerr << "No launch, waypointList empty" << std::endl;
						//					break;
						
						//				case MAV_ACTION_CONTINUE:
						//					if (verbose) std::c
						//					err << "Continue received" << std::endl;
						//					idle = false;
						//					setActive(waypoints[current_active_wp_id]);
						//					break;
						
						//				case MAV_ACTION_HALT:
						//					if (verbose) std::cerr << "Halt received" << std::endl;
						//					idle = true;
						//					break;
						
						//				default:
						//					if (verbose) std::cerr << "Unknown action received with id " << action.action << ", no action taken" << std::endl;
						//					break;
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_ACK:
        {
            mavlink_waypoint_ack_t wpa;
            mavlink_msg_waypoint_ack_decode(msg, &wpa);
			
            if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wpa.target_system == systemid && wpa.target_component == compid))
            {
                protocol_timestamp_lastaction = now;
				
                if (current_state == PX_WPP_SENDLIST || current_state == PX_WPP_SENDLIST_SENDWPS)
                {
                    if (protocol_current_wp_id == waypoints->size()-1)
                    {
                        if (verbose) printf("Received Ack after having sent last waypoint, going to state PX_WPP_IDLE\n");
                        current_state = PX_WPP_IDLE;
                        protocol_current_wp_id = 0;
                    }
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
        {
            mavlink_waypoint_set_current_t wpc;
            mavlink_msg_waypoint_set_current_decode(msg, &wpc);
			
            if(wpc.target_system == systemid && wpc.target_component == compid)
            {
                protocol_timestamp_lastaction = now;
				
                if (current_state == PX_WPP_IDLE)
                {
                    if (wpc.seq < waypoints->size())
                    {
                        if (verbose) printf("Received MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT\n");
                        current_active_wp_id = wpc.seq;
                        uint32_t i;
                        for(i = 0; i < waypoints->size(); i++)
                        {
                            if (i == current_active_wp_id)
                            {
                                waypoints->at(i)->current = true;
                            }
                            else
                            {
                                waypoints->at(i)->current = false;
                            }
                        }
                        if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
                        yawReached = false;
                        posReached = false;
                        send_waypoint_current(current_active_wp_id);
                        send_setpoint(current_active_wp_id);
                        timestamp_firstinside_orbit = 0;
                    }
                    else
                    {
                        if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT: Index out of bounds\n");
                    }
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
        {
            mavlink_waypoint_request_list_t wprl;
            mavlink_msg_waypoint_request_list_decode(msg, &wprl);
            if(wprl.target_system == systemid && wprl.target_component == compid)
            {
                protocol_timestamp_lastaction = now;
				
                if (current_state == PX_WPP_IDLE || current_state == PX_WPP_SENDLIST)
                {
                    if (waypoints->size() > 0)
                    {
                        if (verbose && current_state == PX_WPP_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u changing state to PX_WPP_SENDLIST\n", msg->sysid);
                        if (verbose && current_state == PX_WPP_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST again from %u staying in state PX_WPP_SENDLIST\n", msg->sysid);
                        current_state = PX_WPP_SENDLIST;
                        protocol_current_wp_id = 0;
                        protocol_current_partner_systemid = msg->sysid;
                        protocol_current_partner_compid = msg->compid;
                    }
                    else
                    {
                        if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
                    }
                    protocol_current_count = waypoints->size();
                    send_waypoint_count(msg->sysid,msg->compid, protocol_current_count);
                }
                else
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST because i'm doing something else already (state=%i).\n", current_state);
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
        {
            mavlink_waypoint_request_t wpr;
            mavlink_msg_waypoint_request_decode(msg, &wpr);
            if(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid && wpr.target_system == systemid && wpr.target_component == compid)
            {
                protocol_timestamp_lastaction = now;
				
                //ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
                if ((current_state == PX_WPP_SENDLIST && wpr.seq == 0) || (current_state == PX_WPP_SENDLIST_SENDWPS && (wpr.seq == protocol_current_wp_id || wpr.seq == protocol_current_wp_id + 1) && wpr.seq < waypoints->size()))
                {
                    if (verbose && current_state == PX_WPP_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u changing state to PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
                    if (verbose && current_state == PX_WPP_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id + 1) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u staying in state PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
                    if (verbose && current_state == PX_WPP_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u (again) from %u staying in state PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
					
                    current_state = PX_WPP_SENDLIST_SENDWPS;
                    protocol_current_wp_id = wpr.seq;
                    send_waypoint(protocol_current_partner_systemid, protocol_current_partner_compid, wpr.seq);
                }
                else
                {
                    if (verbose)
                    {
                        if (!(current_state == PX_WPP_SENDLIST || current_state == PX_WPP_SENDLIST_SENDWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because i'm doing something else already (state=%i).\n", current_state); break; }
                        else if (current_state == PX_WPP_SENDLIST)
                        {
                            if (wpr.seq != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
                        }
                        else if (current_state == PX_WPP_SENDLIST_SENDWPS)
                        {
                            if (wpr.seq != protocol_current_wp_id && wpr.seq != protocol_current_wp_id + 1) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, protocol_current_wp_id, protocol_current_wp_id+1);
                            else if (wpr.seq >= waypoints->size()) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
                        }
                        else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST - FIXME: missed error description\n");
                    }
                }
            }
            else
            {
                //we we're target but already communicating with someone else
                if((wpr.target_system == systemid && wpr.target_component == compid) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid))
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, protocol_current_partner_systemid);
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_COUNT:
        {
            mavlink_waypoint_count_t wpc;
            mavlink_msg_waypoint_count_decode(msg, &wpc);
            if(wpc.target_system == systemid && wpc.target_component == compid)
            {
                protocol_timestamp_lastaction = now;
				
                if (current_state == PX_WPP_IDLE || (current_state == PX_WPP_GETLIST && protocol_current_wp_id == 0))
                {
                    if (wpc.count > 0)
                    {
                        if (verbose && current_state == PX_WPP_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) from %u changing state to PX_WPP_GETLIST\n", wpc.count, msg->sysid);
                        if (verbose && current_state == PX_WPP_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) again from %u\n", wpc.count, msg->sysid);
						
                        current_state = PX_WPP_GETLIST;
                        protocol_current_wp_id = 0;
                        protocol_current_partner_systemid = msg->sysid;
                        protocol_current_partner_compid = msg->compid;
                        protocol_current_count = wpc.count;
						
                        printf("clearing receive buffer and readying for receiving waypoints\n");
                        while(waypoints_receive_buffer->size() > 0)
                        {
                            delete waypoints_receive_buffer->back();
                            waypoints_receive_buffer->pop_back();
                        }
						
                        send_waypoint_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
                    }
                    else if (wpc.count == 0)
                    {
                        printf("got waypoint count of 0, clearing waypoint list and staying in state PX_WPP_IDLE\n");
                        while(waypoints_receive_buffer->size() > 0)
                        {
                            delete waypoints->back();
                            waypoints->pop_back();
                        }
                        current_active_wp_id = -1;
                        yawReached = false;
                        posReached = false;
                        break;
						
                    }
                    else
                    {
                        if (verbose) printf("Ignoring MAVLINK_MSG_ID_WAYPOINT_COUNT from %u with count of %u\n", msg->sysid, wpc.count);
                    }
                }
                else
                {
                    if (verbose && !(current_state == PX_WPP_IDLE || current_state == PX_WPP_GETLIST)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm doing something else already (state=%i).\n", current_state);
                    else if (verbose && current_state == PX_WPP_GETLIST && protocol_current_wp_id != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm already receiving waypoint %u.\n", protocol_current_wp_id);
                    else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT - FIXME: missed error description\n");
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT:
        {
            mavlink_waypoint_t wp;
            mavlink_msg_waypoint_decode(msg, &wp);
			
            if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wp.target_system == systemid && wp.target_component == compid))
            {
                protocol_timestamp_lastaction = now;
				
                //ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
                if ((current_state == PX_WPP_GETLIST && wp.seq == 0) || (current_state == PX_WPP_GETLIST_GETWPS && wp.seq == protocol_current_wp_id && wp.seq < protocol_current_count))
                {
                    if (verbose && current_state == PX_WPP_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u changing state to PX_WPP_GETLIST_GETWPS\n", wp.seq, msg->sysid);
                    if (verbose && current_state == PX_WPP_GETLIST_GETWPS && wp.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u\n", wp.seq, msg->sysid);
                    if (verbose && current_state == PX_WPP_GETLIST_GETWPS && wp.seq-1 == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u (again) from %u\n", wp.seq, msg->sysid);
					
                    current_state = PX_WPP_GETLIST_GETWPS;
                    protocol_current_wp_id = wp.seq + 1;
                    mavlink_waypoint_t* newwp = new mavlink_waypoint_t;
                    memcpy(newwp, &wp, sizeof(mavlink_waypoint_t));
                    waypoints_receive_buffer->push_back(newwp);
					
                    if (verbose) printf ("Added new waypoint to list. X= %f\t Y= %f\t Z= %f\t Yaw= %f\n", newwp->x, newwp->y, newwp->z, newwp->param4);
					
                    if(protocol_current_wp_id == protocol_current_count && current_state == PX_WPP_GETLIST_GETWPS)
                    {
                        if (verbose) printf("Got all %u waypoints, changing state to PX_WPP_IDLE\n", protocol_current_count);
						
                        send_waypoint_ack(protocol_current_partner_systemid, protocol_current_partner_compid, 0);
						
                        if (current_active_wp_id > waypoints_receive_buffer->size()-1)
                        {
                            current_active_wp_id = waypoints_receive_buffer->size() - 1;
                        }
						
                        // switch the waypoints list
                        std::vector<mavlink_waypoint_t*>* waypoints_temp = waypoints;
                        waypoints = waypoints_receive_buffer;
                        waypoints_receive_buffer = waypoints_temp;
						
                        //get the new current waypoint
                        uint32_t i;
                        for(i = 0; i < waypoints->size(); i++)
                        {
                            if (waypoints->at(i)->current == 1)
                            {
                                current_active_wp_id = i;
                                //if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
                                yawReached = false;
                                posReached = false;
                                send_waypoint_current(current_active_wp_id);
                                send_setpoint(current_active_wp_id);
                                timestamp_firstinside_orbit = 0;
                                break;
                            }
                        }
						
                        if (i == waypoints->size())
                        {
                            current_active_wp_id = -1;
                            yawReached = false;
                            posReached = false;
                            timestamp_firstinside_orbit = 0;
                        }
						
                        current_state = PX_WPP_IDLE;
                    }
                    else
                    {
                        send_waypoint_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
                    }
                }
                else
                {
                    if (current_state == PX_WPP_IDLE)
                    {
                        //we're done receiving waypoints, answer with ack.
                        send_waypoint_ack(protocol_current_partner_systemid, protocol_current_partner_compid, 0);
                        printf("Received MAVLINK_MSG_ID_WAYPOINT while state=PX_WPP_IDLE, answered with WAYPOINT_ACK.\n");
                    }
                    if (verbose)
                    {
                        if (!(current_state == PX_WPP_GETLIST || current_state == PX_WPP_GETLIST_GETWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u because i'm doing something else already (state=%i).\n", wp.seq, current_state); break; }
                        else if (current_state == PX_WPP_GETLIST)
                        {
                            if(!(wp.seq == 0)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the first waypoint ID (%u) was not 0.\n", wp.seq);
                            else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
                        }
                        else if (current_state == PX_WPP_GETLIST_GETWPS)
                        {
                            if (!(wp.seq == protocol_current_wp_id)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was not the expected %u.\n", wp.seq, protocol_current_wp_id);
                            else if (!(wp.seq < protocol_current_count)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was out of bounds.\n", wp.seq);
                            else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
                        }
                        else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
                    }
                }
            }
            else
            {
                //we we're target but already communicating with someone else
                if((wp.target_system == systemid && wp.target_component == compid) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && current_state != PX_WPP_IDLE)
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, protocol_current_partner_systemid);
                }
                else if(wp.target_system == systemid && wp.target_component == compid)
                {
                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
        {
            mavlink_waypoint_clear_all_t wpca;
            mavlink_msg_waypoint_clear_all_decode(msg, &wpca);
			
            if(wpca.target_system == systemid && wpca.target_component == compid && current_state == PX_WPP_IDLE)
            {
                protocol_timestamp_lastaction = now;
				
                if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
                while(waypoints->size() > 0)
                {
                    delete waypoints->back();
                    waypoints->pop_back();
                }
                current_active_wp_id = -1;
                yawReached = false;
                posReached = false;
            }
            else if (wpca.target_system == systemid && wpca.target_component == compid && current_state != PX_WPP_IDLE)
            {
                if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, current_state);
            }
            break;
        }
			
		default:
        {
            if (debug) std::cerr << "Waypoint: received message of unknown type" << std::endl;
            break;
        }
    }
	
    //check if the current waypoint was reached
    if ((posReached && /*yawReached &&*/ !idle))
    {
        if (current_active_wp_id < waypoints->size())
        {
            mavlink_waypoint_t *cur_wp = waypoints->at(current_active_wp_id);
			
            if (timestamp_firstinside_orbit == 0)
            {
                // Announce that last waypoint was reached
                if (verbose) printf("*** Reached waypoint %u ***\n", cur_wp->seq);
                send_waypoint_reached(cur_wp->seq);
                timestamp_firstinside_orbit = now;
            }
			
            // check if the MAV was long enough inside the waypoint orbit
            //if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))
            if(now-timestamp_firstinside_orbit >= cur_wp->param2*1000)
            {
                if (cur_wp->autocontinue)
                {
                    cur_wp->current = 0;
                    if (current_active_wp_id == waypoints->size() - 1 && waypoints->size() > 1)
                    {
                        //the last waypoint was reached, if auto continue is
                        //activated restart the waypoint list from the beginning
                        current_active_wp_id = 1;
                    }
                    else
                    {
                        if ((uint16_t)(current_active_wp_id + 1) < waypoints->size())
                            current_active_wp_id++;
                    }
					
                    // Fly to next waypoint
                    timestamp_firstinside_orbit = 0;
                    send_waypoint_current(current_active_wp_id);
                    send_setpoint(current_active_wp_id);
                    waypoints->at(current_active_wp_id)->current = true;
                    posReached = false;
                    yawReached = false;
                    if (verbose) printf("Set new waypoint (%u)\n", current_active_wp_id);
                }
            }
        }
    }
    else
    {
        timestamp_lastoutside_orbit = now;
    }
}








int main(int argc, char* argv[])
{
	
	char help[] = "--help";
	
	
	char target_ip[100];
	
	float position[6] = {};
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in gcAddr; 
	struct sockaddr_in locAddr;
	//struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen;
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	//int success = 0;
	unsigned int temp = 0;
	
	// Check if --help flag was used
	if ((argc == 2) && (strcmp(argv[1], help) == 0))
    {
		printf("\n");
		printf("\tUsage:\n\n");
		printf("\t");
		printf("%s", argv[0]);
		printf(" <ip address of QGroundControl>\n");
		printf("\tDefault for localhost: udp-server 127.0.0.1\n\n");
		exit(EXIT_FAILURE);
    }
	
	
	// Change the target ip if parameter was given
	strcpy(target_ip, "127.0.0.1");
	if (argc == 2)
    {
		strcpy(target_ip, argv[1]);
    }
	
	
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);
	
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    } 
	
	/* Attempt to make it non blocking */
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }
	
	
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);
	
	
	printf("MAVLINK MISSION LIBRARY EXAMPLE PROCESS INITIALIZATION DONE, RUNNING..\n");
	
	
	for (;;) 
    {
		
		/*Send Heartbeat */
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_CLASS_GENERIC);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		/* Send Status */
		mavlink_msg_sys_status_pack(1, 200, &msg, MAV_MODE_GUIDED, MAV_NAV_HOLD, MAV_STATE_ACTIVE, 500, 7500, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
		
		/* Send Local Position */
		mavlink_msg_local_position_pack(1, 200, &msg, microsSinceEpoch(), 
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		/* Send attitude */
		mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			
			printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				}
			}
			printf("\n");
		}
		memset(buf, 0, BUFFER_LENGTH);
		sleep(1); // Sleep one second
    }
}


/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 100000 + time.tv_nsec/1000;
	
	return micros;
}
#else
uint64_t microsSinceEpoch()
{
	
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}
#endif