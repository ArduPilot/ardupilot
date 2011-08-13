/*******************************************************************************
 
 Copyright (C) 2011 Lorenz Meier lm ( a t ) inf.ethz.ch
 
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

#include "waypoints.h"
#include <math.h>

bool debug = true;
bool verbose = true;

extern mavlink_system_t mavlink_system;
extern mavlink_wpm_storage wpm;

extern void mavlink_wpm_send_message(mavlink_message_t* msg);
extern void mavlink_wpm_send_gcs_string(const char* string);
extern uint64_t mavlink_wpm_get_system_timestamp();


#define MAVLINK_WPM_NO_PRINTF

void mavlink_wpm_init(mavlink_wpm_storage* state)
{
	// Set all waypoints to zero
	
	// Set count to zero
	state->size = 0;
	state->max_size = MAVLINK_WPM_MAX_WP_COUNT;
	state->current_state = MAVLINK_WPM_STATE_IDLE;
	state->current_partner_sysid = 0;
	state->current_partner_compid = 0;
	state->timestamp_lastaction = 0;
	state->timestamp_last_send_setpoint = 0;
	state->timeout = MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT;
	state->delay_setpoint = MAVLINK_WPM_SETPOINT_DELAY_DEFAULT;
	state->idle = false;      				///< indicates if the system is following the waypoints or is waiting
	state->current_active_wp_id = -1;		///< id of current waypoint
	state->yaw_reached = false;						///< boolean for yaw attitude reached
	state->pos_reached = false;						///< boolean for position reached
	state->timestamp_lastoutside_orbit = 0;///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
	state->timestamp_firstinside_orbit = 0;///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value
	
}

/*
 *  @brief Sends an waypoint ack message
 */
void mavlink_wpm_send_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
    mavlink_message_t msg;
    mavlink_waypoint_ack_t wpa;
	
    wpa.target_system = wpm.current_partner_sysid;
    wpa.target_component = wpm.current_partner_compid;
    wpa.type = type;
	
    mavlink_msg_waypoint_ack_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wpa);
    mavlink_wpm_send_message(&msg);
	
    // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
	
    if (MAVLINK_WPM_TEXT_FEEDBACK)
	{
#ifdef MAVLINK_WPM_NO_PRINTF
    	mavlink_wpm_send_gcs_string("Sent waypoint ACK");
#else
		if (MAVLINK_WPM_VERBOSE) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);
#endif
		mavlink_wpm_send_gcs_string("Sent waypoint ACK");
	}
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
    if(seq < wpm.size)
    {
        mavlink_waypoint_t *cur = &(wpm.waypoints[seq]);
		
        mavlink_message_t msg;
        mavlink_waypoint_current_t wpc;
		
        wpc.seq = cur->seq;
		
        mavlink_msg_waypoint_current_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wpc);
        mavlink_wpm_send_message(&msg);
		
        // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
		
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("Broadcasted new current waypoint\n"); //// printf("Broadcasted new current waypoint %u\n", wpc.seq);
    }
    else
    {
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("ERROR: index out of bounds\n");
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
    if(seq < wpm.size)
    {
        mavlink_waypoint_t *cur = &(wpm.waypoints[seq]);
		
        mavlink_message_t msg;
        mavlink_local_position_setpoint_set_t position_control_set_point;
		
        // Send new NED or ENU setpoint to onbaord autopilot
        if (cur->frame == MAV_FRAME_LOCAL_NED || cur->frame == MAV_FRAME_LOCAL_ENU)
        {
            position_control_set_point.target_system = mavlink_system.sysid;
            position_control_set_point.target_component = MAV_COMP_ID_IMU;
            position_control_set_point.x = cur->x;
            position_control_set_point.y = cur->y;
            position_control_set_point.z = cur->z;
            position_control_set_point.yaw = cur->param4;
			
            mavlink_msg_local_position_setpoint_set_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &position_control_set_point);
            mavlink_wpm_send_message(&msg);
			
            // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
        }
        else
        {
            if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("No new setpoint set because of invalid coordinate frame of waypoint");//// if (verbose) // printf("No new set point sent to IMU because the new waypoint %u had no local coordinates\n", cur->seq);
        }
		
        wpm.timestamp_last_send_setpoint = mavlink_wpm_get_system_timestamp();
    }
    else
    {
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("ERROR: Waypoint index out of bounds\n"); //// if (verbose) // printf("ERROR: index out of bounds\n");
    }
}

void mavlink_wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
    mavlink_message_t msg;
    mavlink_waypoint_count_t wpc;
	
    wpc.target_system = wpm.current_partner_sysid;
    wpc.target_component = wpm.current_partner_compid;
    wpc.count = count;
	
    mavlink_msg_waypoint_count_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wpc);
    mavlink_wpm_send_message(&msg);
	
    if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("Sent waypoint count"); //// if (verbose) // printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);
	
    // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

void mavlink_wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq)
{
    if (seq < wpm.size)
    {
        mavlink_message_t msg;
        mavlink_waypoint_t *wp = &(wpm.waypoints[seq]);
        wp->target_system = wpm.current_partner_sysid;
        wp->target_component = wpm.current_partner_compid;
        mavlink_msg_waypoint_encode(mavlink_system.sysid, mavlink_system.compid, &msg, wp);
        mavlink_wpm_send_message(&msg);
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("Sent waypoint"); //// if (verbose) // printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);
		
        // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
    }
    else
    {
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("ERROR: Waypoint index out of bounds\n");
    }
}

void mavlink_wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
    if (seq < wpm.max_size)
    {
        mavlink_message_t msg;
        mavlink_waypoint_request_t wpr;
        wpr.target_system = wpm.current_partner_sysid;
        wpr.target_component = wpm.current_partner_compid;
        wpr.seq = seq;
        mavlink_msg_waypoint_request_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wpr);
        mavlink_wpm_send_message(&msg);
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("Sent waypoint request"); //// if (verbose) // printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);
		
        // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
    }
    else
    {
        if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("ERROR: Waypoint index exceeds list capacity\n");
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
	
    mavlink_msg_waypoint_reached_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &wp_reached);
    mavlink_wpm_send_message(&msg);
	
    if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_wpm_send_gcs_string("Sent waypoint reached message"); //// if (verbose) // printf("Sent waypoint %u reached message\n", wp_reached.seq);
	
    // FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

//float mavlink_wpm_distance_to_segment(uint16_t seq, float x, float y, float z)
//{
//    if (seq < wpm.size)
//    {
//        mavlink_waypoint_t *cur = waypoints->at(seq);
//		
//        const PxVector3 A(cur->x, cur->y, cur->z);
//        const PxVector3 C(x, y, z);
//		
//        // seq not the second last waypoint
//        if ((uint16_t)(seq+1) < wpm.size)
//        {
//            mavlink_waypoint_t *next = waypoints->at(seq+1);
//            const PxVector3 B(next->x, next->y, next->z);
//            const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
//            if (r >= 0 && r <= 1)
//            {
//                const PxVector3 P(A + r*(B-A));
//                return (P-C).length();
//            }
//            else if (r < 0.f)
//            {
//                return (C-A).length();
//            }
//            else
//            {
//                return (C-B).length();
//            }
//        }
//        else
//        {
//            return (C-A).length();
//        }
//    }
//    else
//    {
//        // if (verbose) // printf("ERROR: index out of bounds\n");
//    }
//    return -1.f;
//}

float mavlink_wpm_distance_to_point(uint16_t seq, float x, float y, float z)
{
	//    if (seq < wpm.size)
	//    {
	//        mavlink_waypoint_t *cur = waypoints->at(seq);
	//		
	//        const PxVector3 A(cur->x, cur->y, cur->z);
	//        const PxVector3 C(x, y, z);
	//		
	//        return (C-A).length();
	//    }
	//    else
	//    {
	//        // if (verbose) // printf("ERROR: index out of bounds\n");
	//    }
    return -1.f;
}


void mavlink_wpm_message_handler(const mavlink_message_t* msg)
{
    // Handle param messages
    //paramClient->handleMAVLinkPacket(msg);
	
    //check for timed-out operations
    uint64_t now = mavlink_wpm_get_system_timestamp();
    if (now-wpm.timestamp_lastaction > wpm.timeout && wpm.current_state != MAVLINK_WPM_STATE_IDLE)
    {
#ifdef MAVLINK_WPM_NO_PRINTF
    	mavlink_wpm_send_gcs_string("Operation timeout switching -> IDLE");
#else
		if (MAVLINK_WPM_VERBOSE) printf("Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE\n", wpm.current_state);
#endif
		wpm.current_state = MAVLINK_WPM_STATE_IDLE;
        wpm.current_count = 0;
        wpm.current_partner_sysid = 0;
        wpm.current_partner_compid = 0;
        wpm.current_wp_id = -1;
		
        if(wpm.size == 0)
        {
            wpm.current_active_wp_id = -1;
        }
    }
	
    if(now-wpm.timestamp_last_send_setpoint > wpm.delay_setpoint && wpm.current_active_wp_id < wpm.size)
    {
        mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
    }
	
    switch(msg->msgid)
    {
		case MAVLINK_MSG_ID_ATTITUDE:
        {
            if(msg->sysid == mavlink_system.sysid && wpm.current_active_wp_id < wpm.size)
            {
                mavlink_waypoint_t *wp = &(wpm.waypoints[wpm.current_active_wp_id]);
                if(wp->frame == MAV_FRAME_LOCAL_ENU || wp->frame == MAV_FRAME_LOCAL_NED)
                {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(msg, &att);
                    float yaw_tolerance = wpm.accept_range_yaw;
                    //compare current yaw
                    if (att.yaw - yaw_tolerance >= 0.0f && att.yaw + yaw_tolerance < 2.f*M_PI)
                    {
                        if (att.yaw - yaw_tolerance <= wp->param4 && att.yaw + yaw_tolerance >= wp->param4)
                            wpm.yaw_reached = true;
                    }
                    else if(att.yaw - yaw_tolerance < 0.0f)
                    {
                        float lowerBound = 360.0f + att.yaw - yaw_tolerance;
                        if (lowerBound < wp->param4 || wp->param4 < att.yaw + yaw_tolerance)
                            wpm.yaw_reached = true;
                    }
                    else
                    {
                        float upperBound = att.yaw + yaw_tolerance - 2.f*M_PI;
                        if (att.yaw - yaw_tolerance < wp->param4 || wp->param4 < upperBound)
                            wpm.yaw_reached = true;
                    }
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_LOCAL_POSITION:
        {
            if(msg->sysid == mavlink_system.sysid && wpm.current_active_wp_id < wpm.size)
            {
                mavlink_waypoint_t *wp = &(wpm.waypoints[wpm.current_active_wp_id]);
				
                if(wp->frame == MAV_FRAME_LOCAL_ENU || MAV_FRAME_LOCAL_NED)
                {
                    mavlink_local_position_t pos;
                    mavlink_msg_local_position_decode(msg, &pos);
                    //// if (debug) // printf("Received new position: x: %f | y: %f | z: %f\n", pos.x, pos.y, pos.z);
					
                    wpm.pos_reached = false;
					
                    // compare current position (given in message) with current waypoint
                    float orbit = wp->param1;
					
                    float dist;
                    if (wp->param2 == 0)
                    {
						// FIXME segment distance
                        //dist = mavlink_wpm_distance_to_segment(current_active_wp_id, pos.x, pos.y, pos.z);
                    }
                    else
                    {
                        dist = mavlink_wpm_distance_to_point(wpm.current_active_wp_id, pos.x, pos.y, pos.z);
                    }
					
                    if (dist >= 0.f && dist <= orbit && wpm.yaw_reached)
                    {
                        wpm.pos_reached = true;
                    }
                }
            }
            break;
        }
			
			//		case MAVLINK_MSG_ID_CMD: // special action from ground station
			//        {
			//            mavlink_cmd_t action;
			//            mavlink_msg_cmd_decode(msg, &action);
			//            if(action.target == mavlink_system.sysid)
			//            {
			//                // if (verbose) std::cerr << "Waypoint: received message with action " << action.action << std::endl;
			//                switch (action.action)
			//                {
			//						//				case MAV_ACTION_LAUNCH:
			//						//					// if (verbose) std::cerr << "Launch received" << std::endl;
			//						//					current_active_wp_id = 0;
			//						//					if (wpm.size>0)
			//						//					{
			//						//						setActive(waypoints[current_active_wp_id]);
			//						//					}
			//						//					else
			//						//						// if (verbose) std::cerr << "No launch, waypointList empty" << std::endl;
			//						//					break;
			//						
			//						//				case MAV_ACTION_CONTINUE:
			//						//					// if (verbose) std::c
			//						//					err << "Continue received" << std::endl;
			//						//					idle = false;
			//						//					setActive(waypoints[current_active_wp_id]);
			//						//					break;
			//						
			//						//				case MAV_ACTION_HALT:
			//						//					// if (verbose) std::cerr << "Halt received" << std::endl;
			//						//					idle = true;
			//						//					break;
			//						
			//						//				default:
			//						//					// if (verbose) std::cerr << "Unknown action received with id " << action.action << ", no action taken" << std::endl;
			//						//					break;
			//                }
			//            }
			//            break;
			//        }
			
		case MAVLINK_MSG_ID_WAYPOINT_ACK:
        {
            mavlink_waypoint_ack_t wpa;
            mavlink_msg_waypoint_ack_decode(msg, &wpa);
			
            if((msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_system.compid*/))
            {
                wpm.timestamp_lastaction = now;
				
                if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST || wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)
                {
                    if (wpm.current_wp_id == wpm.size-1)
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("Got last WP ACK state -> IDLE");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Received ACK after having sent last waypoint, going to state MAVLINK_WPM_STATE_IDLE\n");
#endif
						wpm.current_state = MAVLINK_WPM_STATE_IDLE;
                        wpm.current_wp_id = 0;
                    }
                }
            }
			else
			{
#ifdef MAVLINK_WPM_NO_PRINTF
				mavlink_wpm_send_gcs_string("REJ. WP CMD: curr partner id mismatch");
#else
				if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
#endif
			}
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
        {
            mavlink_waypoint_set_current_t wpc;
            mavlink_msg_waypoint_set_current_decode(msg, &wpc);
			
            if(wpc.target_system == mavlink_system.sysid /*&& wpc.target_component == mavlink_system.compid*/)
            {
                wpm.timestamp_lastaction = now;
				
                if (wpm.current_state == MAVLINK_WPM_STATE_IDLE)
                {
                    if (wpc.seq < wpm.size)
                    {
                        // if (verbose) // printf("Received MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT\n");
                        wpm.current_active_wp_id = wpc.seq;
                        uint32_t i;
                        for(i = 0; i < wpm.size; i++)
                        {
                            if (i == wpm.current_active_wp_id)
                            {
                                wpm.waypoints[i].current = true;
                            }
                            else
                            {
                                wpm.waypoints[i].current = false;
                            }
                        }
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("NEW WP SET");
#else
						if (MAVLINK_WPM_VERBOSE) printf("New current waypoint %u\n", wpm.current_active_wp_id);
#endif
                        wpm.yaw_reached = false;
                        wpm.pos_reached = false;
                        mavlink_wpm_send_waypoint_current(wpm.current_active_wp_id);
                        mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
                        wpm.timestamp_firstinside_orbit = 0;
                    }
                    else
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("IGN WP CURR CMD: Not in list");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT: Index out of bounds\n");
#endif
                    }
                }
				else
				{
#ifdef MAVLINK_WPM_NO_PRINTF
					mavlink_wpm_send_gcs_string("IGN WP CURR CMD: Busy");
#else
					if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE NOT IN IDLE STATE\n");
#endif
				}
            }
			else
			{
#ifdef MAVLINK_WPM_NO_PRINTF
				mavlink_wpm_send_gcs_string("REJ. WP CMD: target id mismatch");
#else
				if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
#endif
			}
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
        {
            mavlink_waypoint_request_list_t wprl;
            mavlink_msg_waypoint_request_list_decode(msg, &wprl);
            if(wprl.target_system == mavlink_system.sysid /*&& wprl.target_component == mavlink_system.compid*/)
            {
                wpm.timestamp_lastaction = now;
				
                if (wpm.current_state == MAVLINK_WPM_STATE_IDLE || wpm.current_state == MAVLINK_WPM_STATE_SENDLIST)
                {
                    if (wpm.size > 0)
                    {
                        if (verbose && wpm.current_state == MAVLINK_WPM_STATE_IDLE) // printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u changing state to MAVLINK_WPM_STATE_SENDLIST\n", msg->sysid);
							if (verbose && wpm.current_state == MAVLINK_WPM_STATE_SENDLIST) // printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST again from %u staying in state MAVLINK_WPM_STATE_SENDLIST\n", msg->sysid);
								wpm.current_state = MAVLINK_WPM_STATE_SENDLIST;
                        wpm.current_wp_id = 0;
                        wpm.current_partner_sysid = msg->sysid;
                        wpm.current_partner_compid = msg->compid;
                    }
                    else
                    {
                        // if (verbose) // printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
                    }
                    wpm.current_count = wpm.size;
                    mavlink_wpm_send_waypoint_count(msg->sysid,msg->compid, wpm.current_count);
                }
                else
                {
                    // if (verbose) // printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST because i'm doing something else already (state=%i).\n", wpm.current_state);
                }
            }
			else
			{
				// if (verbose) // printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT MISMATCH\n");
			}
			
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
        {
            mavlink_waypoint_request_t wpr;
            mavlink_msg_waypoint_request_decode(msg, &wpr);
            if(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid && wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_system.compid*/)
            {
                wpm.timestamp_lastaction = now;
				
                //ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
                if ((wpm.current_state == MAVLINK_WPM_STATE_SENDLIST && wpr.seq == 0) || (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && (wpr.seq == wpm.current_wp_id || wpr.seq == wpm.current_wp_id + 1) && wpr.seq < wpm.size))
                {
                    if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST)
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("GOT WP REQ, state -> SEND");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u changing state to MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
#endif
                    }
                    if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm.current_wp_id + 1)
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("GOT 2nd WP REQ");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
#endif
                    }
                    if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm.current_wp_id)
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("GOT 2nd WP REQ");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u (again) from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
#endif
                    }
					
                    wpm.current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;
                    wpm.current_wp_id = wpr.seq;
                    mavlink_wpm_send_waypoint(wpm.current_partner_sysid, wpm.current_partner_compid, wpr.seq);
                }
                else
                {
                    // if (verbose)
                    {
                        if (!(wpm.current_state == MAVLINK_WPM_STATE_SENDLIST || wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS))
						{
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_wpm_send_gcs_string("REJ. WP CMD: Busy");
#else
							if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because i'm doing something else already (state=%i).\n", wpm.current_state);
#endif
							break;
						}
                        else if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST)
                        {
                            if (wpr.seq != 0)
							{
#ifdef MAVLINK_WPM_NO_PRINTF
								mavlink_wpm_send_gcs_string("REJ. WP CMD: First id != 0");
#else
								if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
#endif
							}
                        }
                        else if (wpm.current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)
                        {
                            if (wpr.seq != wpm.current_wp_id && wpr.seq != wpm.current_wp_id + 1)
							{
#ifdef MAVLINK_WPM_NO_PRINTF
								mavlink_wpm_send_gcs_string("REJ. WP CMD: Req. WP was unexpected");
#else
								if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, wpm.current_wp_id, wpm.current_wp_id+1);
#endif
							}
							else if (wpr.seq >= wpm.size)
							{
#ifdef MAVLINK_WPM_NO_PRINTF
								mavlink_wpm_send_gcs_string("REJ. WP CMD: Req. WP not in list");
#else
								if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
#endif
							}
                        }
                        else
						{
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_wpm_send_gcs_string("REJ. WP CMD: ?");
#else
							if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST - FIXME: missed error description\n");
#endif
						}
                    }
                }
            }
            else
            {
                //we we're target but already communicating with someone else
                if((wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_system.compid*/) && !(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid))
                {
#ifdef MAVLINK_WPM_NO_PRINTF
					mavlink_wpm_send_gcs_string("REJ. WP CMD: Busy");
#else
					if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, wpm.current_partner_sysid);
#endif
                }
				else
				{
#ifdef MAVLINK_WPM_NO_PRINTF
					mavlink_wpm_send_gcs_string("REJ. WP CMD: target id mismatch");
#else
					if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
#endif
				}
				
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_COUNT:
        {
            mavlink_waypoint_count_t wpc;
            mavlink_msg_waypoint_count_decode(msg, &wpc);
            if(wpc.target_system == mavlink_system.sysid/* && wpc.target_component == mavlink_system.compid*/)
            {
                wpm.timestamp_lastaction = now;
				
                if (wpm.current_state == MAVLINK_WPM_STATE_IDLE || (wpm.current_state == MAVLINK_WPM_STATE_GETLIST && wpm.current_wp_id == 0))
                {
                    if (wpc.count > 0)
                    {
                        if (wpm.current_state == MAVLINK_WPM_STATE_IDLE)
                        {
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_wpm_send_gcs_string("WP CMD OK: state -> GETLIST");
#else
							if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) from %u changing state to MAVLINK_WPM_STATE_GETLIST\n", wpc.count, msg->sysid);
#endif
                        }
                        if (wpm.current_state == MAVLINK_WPM_STATE_GETLIST)
                        {
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_wpm_send_gcs_string("WP CMD OK AGAIN");
#else
							if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) again from %u\n", wpc.count, msg->sysid);
#endif
                        }
						
                        wpm.current_state = MAVLINK_WPM_STATE_GETLIST;
                        wpm.current_wp_id = 0;
                        wpm.current_partner_sysid = msg->sysid;
                        wpm.current_partner_compid = msg->compid;
                        wpm.current_count = wpc.count;
						
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("CLR RCV BUF: READY");
#else
						if (MAVLINK_WPM_VERBOSE) printf("clearing receive buffer and readying for receiving waypoints\n");
#endif
						wpm.rcv_size = 0;
                        //while(waypoints_receive_buffer->size() > 0)
						//                        {
						//                            delete waypoints_receive_buffer->back();
						//                            waypoints_receive_buffer->pop_back();
						//                        }
						
                        mavlink_wpm_send_waypoint_request(wpm.current_partner_sysid, wpm.current_partner_compid, wpm.current_wp_id);
                    }
                    else if (wpc.count == 0)
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("COUNT 0");
#else
						if (MAVLINK_WPM_VERBOSE) printf("got waypoint count of 0, clearing waypoint list and staying in state MAVLINK_WPM_STATE_IDLE\n");
#endif
						wpm.rcv_size = 0;
                        //while(waypoints_receive_buffer->size() > 0)
						//                        {
						//                            delete waypoints->back();
						//                            waypoints->pop_back();
						//                        }
                        wpm.current_active_wp_id = -1;
                        wpm.yaw_reached = false;
                        wpm.pos_reached = false;
                        break;
						
                    }
                    else
                    {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("IGN WP CMD");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Ignoring MAVLINK_MSG_ID_WAYPOINT_COUNT from %u with count of %u\n", msg->sysid, wpc.count);
#endif
                    }
                }
                else
                {
                    if (!(wpm.current_state == MAVLINK_WPM_STATE_IDLE || wpm.current_state == MAVLINK_WPM_STATE_GETLIST))
					{
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("REJ. WP CMD: Busy");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm doing something else already (state=%i).\n", wpm.current_state);
#endif
					}
                    else if (wpm.current_state == MAVLINK_WPM_STATE_GETLIST && wpm.current_wp_id != 0)
					{
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("REJ. WP CMD: Busy");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm already receiving waypoint %u.\n", wpm.current_wp_id);
#endif
					}
                    else
					{
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_wpm_send_gcs_string("REJ. WP CMD: ?");
#else
						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT - FIXME: missed error description\n");
#endif
					}
                }
            }
			else
			{
#ifdef MAVLINK_WPM_NO_PRINTF
				mavlink_wpm_send_gcs_string("REJ. WP CMD: target id mismatch");
#else
				if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
#endif
			}
            
        }
			break;
			
		case MAVLINK_MSG_ID_WAYPOINT:
        {
            mavlink_waypoint_t wp;
            mavlink_msg_waypoint_decode(msg, &wp);
			
			// if (verbose) // printf("GOT WAYPOINT!");
			
            if((msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && (wp.target_system == mavlink_system.sysid /*&& wp.target_component == mavlink_system.compid*/))
            {
                wpm.timestamp_lastaction = now;
				
                //ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
                if ((wpm.current_state == MAVLINK_WPM_STATE_GETLIST && wp.seq == 0) || (wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq == wpm.current_wp_id && wp.seq < wpm.current_count))
                {
                    if (verbose && wpm.current_state == MAVLINK_WPM_STATE_GETLIST) // printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u changing state to MAVLINK_WPM_STATE_GETLIST_GETWPS\n", wp.seq, msg->sysid);
						if (verbose && wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq == wpm.current_wp_id) // printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u\n", wp.seq, msg->sysid);
							if (verbose && wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq-1 == wpm.current_wp_id) // printf("Got MAVLINK_MSG_ID_WAYPOINT %u (again) from %u\n", wp.seq, msg->sysid);
								
								wpm.current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;
                    mavlink_waypoint_t* newwp = &(wpm.rcv_waypoints[wp.seq]);
                    memcpy(newwp, &wp, sizeof(mavlink_waypoint_t));
					
					wpm.current_wp_id = wp.seq + 1;
					
                    // if (verbose) // printf ("Added new waypoint to list. X= %f\t Y= %f\t Z= %f\t Yaw= %f\n", newwp->x, newwp->y, newwp->z, newwp->param4);
					
                    if(wpm.current_wp_id == wpm.current_count && wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS)
                    {
                        // if (verbose) // printf("Got all %u waypoints, changing state to MAVLINK_WPM_STATE_IDLE\n", wpm.current_count);
						
                        mavlink_wpm_send_waypoint_ack(wpm.current_partner_sysid, wpm.current_partner_compid, 0);
						
                        if (wpm.current_active_wp_id > wpm.rcv_size-1)
                        {
                            wpm.current_active_wp_id = wpm.rcv_size-1;
                        }
						
                        // switch the waypoints list
						// FIXME CHECK!!!
						for (int i = 0; i < wpm.current_count; ++i)
						{
							wpm.waypoints[i] = wpm.rcv_waypoints[i];
						}
						wpm.size = wpm.current_count;
						
                        //get the new current waypoint
                        uint32_t i;
                        for(i = 0; i < wpm.size; i++)
                        {
                            if (wpm.waypoints[i].current == 1)
                            {
                                wpm.current_active_wp_id = i;
                                //// if (verbose) // printf("New current waypoint %u\n", current_active_wp_id);
                                wpm.yaw_reached = false;
                                wpm.pos_reached = false;
								mavlink_wpm_send_waypoint_current(wpm.current_active_wp_id);
                                mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
								wpm.timestamp_firstinside_orbit = 0;
                                break;
                            }
                        }
						
                        if (i == wpm.size)
                        {
                            wpm.current_active_wp_id = -1;
                            wpm.yaw_reached = false;
                            wpm.pos_reached = false;
                            wpm.timestamp_firstinside_orbit = 0;
                        }
						
                        wpm.current_state = MAVLINK_WPM_STATE_IDLE;
                    }
                    else
                    {
                        mavlink_wpm_send_waypoint_request(wpm.current_partner_sysid, wpm.current_partner_compid, wpm.current_wp_id);
                    }
                }
                else
                {
                    if (wpm.current_state == MAVLINK_WPM_STATE_IDLE)
                    {
                        //we're done receiving waypoints, answer with ack.
                        mavlink_wpm_send_waypoint_ack(wpm.current_partner_sysid, wpm.current_partner_compid, 0);
                        // printf("Received MAVLINK_MSG_ID_WAYPOINT while state=MAVLINK_WPM_STATE_IDLE, answered with WAYPOINT_ACK.\n");
                    }
                    // if (verbose)
                    {
                        if (!(wpm.current_state == MAVLINK_WPM_STATE_GETLIST || wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS))
						{
							// printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u because i'm doing something else already (state=%i).\n", wp.seq, wpm.current_state);
							break;
						}
                        else if (wpm.current_state == MAVLINK_WPM_STATE_GETLIST)
                        {
                            if(!(wp.seq == 0))
							{
								// printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the first waypoint ID (%u) was not 0.\n", wp.seq);
							}
                            else
							{
								// printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
							}
                        }
                        else if (wpm.current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS)
                        {
                            if (!(wp.seq == wpm.current_wp_id))
							{
								// printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was not the expected %u.\n", wp.seq, wpm.current_wp_id);
							}
                            else if (!(wp.seq < wpm.current_count))
							{
								// printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was out of bounds.\n", wp.seq);
							}
                            else
							{
								// printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
							}
                        }
                        else
						{
							// printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
						}
                    }
                }
            }
            else
            {
                //we we're target but already communicating with someone else
                if((wp.target_system == mavlink_system.sysid /*&& wp.target_component == mavlink_system.compid*/) && !(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && wpm.current_state != MAVLINK_WPM_STATE_IDLE)
                {
                    // if (verbose) // printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, wpm.current_partner_sysid);
                }
                else if(wp.target_system == mavlink_system.sysid /* && wp.target_component == mavlink_system.compid*/)
                {
                    // if (verbose) // printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
                }
            }
            break;
        }
			
		case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
        {
            mavlink_waypoint_clear_all_t wpca;
            mavlink_msg_waypoint_clear_all_decode(msg, &wpca);
			
            if(wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_system.compid */ && wpm.current_state == MAVLINK_WPM_STATE_IDLE)
            {
                wpm.timestamp_lastaction = now;
				
                // if (verbose) // printf("Got MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
                // Delete all waypoints
				wpm.size = 0;
                wpm.current_active_wp_id = -1;
                wpm.yaw_reached = false;
                wpm.pos_reached = false;
            }
            else if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_system.compid */ && wpm.current_state != MAVLINK_WPM_STATE_IDLE)
            {
                // if (verbose) // printf("Ignored MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, wpm.current_state);
            }
            break;
        }
			
		default:
        {
            // if (debug) // printf("Waypoint: received message of unknown type");
            break;
        }
    }
	
    //check if the current waypoint was reached
    if (wpm.pos_reached /*wpm.yaw_reached &&*/ && !wpm.idle)
    {
        if (wpm.current_active_wp_id < wpm.size)
        {
            mavlink_waypoint_t *cur_wp = &(wpm.waypoints[wpm.current_active_wp_id]);
			
            if (wpm.timestamp_firstinside_orbit == 0)
            {
                // Announce that last waypoint was reached
                // if (verbose) // printf("*** Reached waypoint %u ***\n", cur_wp->seq);
                mavlink_wpm_send_waypoint_reached(cur_wp->seq);
                wpm.timestamp_firstinside_orbit = now;
            }
			
            // check if the MAV was long enough inside the waypoint orbit
            //if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))
            if(now-wpm.timestamp_firstinside_orbit >= cur_wp->param2*1000)
            {
                if (cur_wp->autocontinue)
                {
                    cur_wp->current = 0;
                    if (wpm.current_active_wp_id == wpm.size - 1 && wpm.size > 1)
                    {
                        //the last waypoint was reached, if auto continue is
                        //activated restart the waypoint list from the beginning
                        wpm.current_active_wp_id = 1;
                    }
                    else
                    {
                        if ((uint16_t)(wpm.current_active_wp_id + 1) < wpm.size)
                            wpm.current_active_wp_id++;
                    }
					
                    // Fly to next waypoint
                    wpm.timestamp_firstinside_orbit = 0;
                    mavlink_wpm_send_waypoint_current(wpm.current_active_wp_id);
                    mavlink_wpm_send_setpoint(wpm.current_active_wp_id);
                    wpm.waypoints[wpm.current_active_wp_id].current = true;
                    wpm.pos_reached = false;
                    wpm.yaw_reached = false;
                    // if (verbose) // printf("Set new waypoint (%u)\n", wpm.current_active_wp_id);
                }
            }
        }
    }
    else
    {
        wpm.timestamp_lastoutside_orbit = now;
    }
}

