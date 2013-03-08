/*
 Version: 1.0

 
 Authors:
       Swaroop Hangal
       Bharat Tak
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


/* This file contains all the implementation functions that handle the 
// interaircraft data.
*/



	/* This is the function that sets the recevied leader location data as the 
	// guided waypoint for the follower. It does this only when its a legitimate 
	// new location packet. Also, after setting the guided waypoint it sends the 
	// particular location packet to debugg xbee(6666), as a note of confirmation.  		
	*/
void set_leader_wp(uint8_t *wpp){ 
  send_debugg_data();//debugg
if(check_same_packet() == 1){
  hex_to_data();
  set_guided_WP();
  copy_packet();
  send_debugg_data();
    
  }
}

	/* This is the function used in case of orbit follower. A swarm of orbit followers
	// constantly exchange location information. For a circular exchange of information
	// like 1 -> 2 -> 3 -> 1, this function will decode and save the recevied location 
	// data in the struch recevied_wp for further use(like mantaining distance etc)
	*/
void store_recevied_wp(uint8_t *wpp){ 
if(check_same_packet() == 1){
  hex_to_data();//cehra hai ya chaand khila
  copy_packet();
  send_debugg_data();
  }
}


	/* This function takes the interaircraft data bytes from WPpacket, decodes the bytes to 
	// int32_t and saves them in the struct guided_WP. It modifies lat, long and alt.
	*/
void hex_to_data(void){
#if AIRCRAFT_TYPE_FOLLOWER == ENABLED    
  guided_WP.lat = bytes_to_int(&WPpacket[0]);
  guided_WP.lng = bytes_to_int(&WPpacket[4]);
  guided_WP.alt = bytes_to_int(&WPpacket[8]);
#endif

#if AIRCRAFT_TYPE_ORBIT_FOLLOWER == ENABLED    
  recevied_WP.lat = bytes_to_int(&WPpacket[0]);
  recevied_WP.lng = bytes_to_int(&WPpacket[4]);
  recevied_WP.alt = bytes_to_int(&WPpacket[8]);
#endif

}



int check_same_packet(void){
  int i=0;
  while(i<12){
    if(WPpacket[i] != PWPpacket[i]){
      return 1;
     }
    else{
    i = i+1;
    } 
  }
  return 0;
}

void copy_packet(void){
  for(int i = 0; i<12; i++){
  PWPpacket[i] = WPpacket[i];
  }
}


void send_debugg_data(void){
  Follower_print_recieved(WPpacket);
}

void send_evnt_debugg_data(void){
  Follower_print_recieved(evnt_WPpacket);
}

int check_same_evnt_packet(void){
  int i=0;
  while(i<12){
    if(evnt_WPpacket[i] != evnt_PWPpacket[i]){
      return 1;
     }
    else{
    i = i+1;
    } 
  }
  return 0;
}


void copy_evnt_packet(void){
  for(int i = 0; i<12; i++){
  evnt_PWPpacket[i] = evnt_WPpacket[i];
  }
}



	/* Sets the orbit center and shares it with other aircafts(shares only if center is set through GCS). 
	// Center could either be set using GCS guided waypoint(fly to here command) or automatically 
	// through interaircaft data packet 'evnt_WPpacket'as shared by some other aircraft
	// in the system.    
	*/
void set_new_orbit_center(void){  
  if(check_same_evnt_packet() == 1){
   guided_WP.lat = bytes_to_int(&evnt_WPpacket[0]);//cehra hai ya chaand khila
   guided_WP.lng = bytes_to_int(&evnt_WPpacket[4]);//zulf ghaneri shaam hai kya
   guided_WP.alt = bytes_to_int(&evnt_WPpacket[8]);//saagar jaisi aankhon waali, ye to bata tera naam hai kya ?
   set_guided_WP();
   p_guided_WP = guided_WP; 
   xbee_flag = 0;
   copy_evnt_packet();
   send_evnt_debugg_data();
   }
  
  
  if (check_same_GWP() == 1){
   p_guided_WP = guided_WP; 
    
   XBee_sendFollower(0,&guided_WP, 1);
   XBee_sendFollower(2,&guided_WP, 1); 
  }
  
}


int check_same_GWP(void){
    if(p_guided_WP.lat != guided_WP.lat){
      return 1;
     }
    else if(p_guided_WP.lng != guided_WP.lng){
     return 1;
    }
    else if(p_guided_WP.alt != guided_WP.alt){
     return 1;
    }
    else{
     return 0; 
    }
}

	/* Sets the orbit center, its parameters like radius, approach gain etc.
	*/
void set_orbit_parameters(void){
orbit_center = guided_WP;
orbit_radius = 100;
orbit_gain_factor = 0.02;
}

	/* Function to set orbit following in motion. Aircraft follows an
	// orbit around the set orbit center. When far away from the center, heading is 
	// almost directed toward the center, as it approaches closer, it slips in orbit
	// as set using the orbit_gain_factor.
	*/
void follow_orbit(void){
if(control_mode == GUIDED){
  float orbit_distance = get_distance(&current_loc, &orbit_center);
  	if (orbit_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		//Serial.println(wp_distance,DEC);
		return;
	}
   target_bearing_cd = get_bearing_cd(&orbit_center, &current_loc) - 9000 - atan2(orbit_gain_factor*(orbit_distance - orbit_radius), 1) * 5729.57795;
    nav_bearing_cd = target_bearing_cd;
    loiter_delta = (target_bearing_cd - old_target_bearing_cd)/100;

    old_target_bearing_cd = target_bearing_cd;

    if (loiter_delta > 180) loiter_delta -= 360;
    if (loiter_delta < -180) loiter_delta += 360;
    loiter_sum += abs(loiter_delta);
    update_navigation();
  }
}



/****************************************************************************************************************************
//        orbit follower SETUP
//plan:
//	-- Do orbit following in guided mode
//		CENTER DATA	
//		-- take input for orbit center lat,long,alt from the guided mode's 'fly to here' command on leader
//		-- give provision to hard code a constant lat long alt for orbit following
//		-- send the orbit centers data to the two followers 
//		
//		RADIUS r
//		-- Take radius input from 'loiter_radius' parameter on leader and send to followers everytime its changed
//		-- give provision for hardcoding orbit radius
//
//		GAIN FACTOR K
//		-- hard code gain factor k
//		-- check if you can use some parameter in parameter list to set k on the go
//
//              DISTANCE d
//              -- use get_distance(&current_loc, &guided_WP);
//
//		HEADING
//		-- use gps heading to calculate your heading
//		-- check if you can directly set heading without setting a waypoint
//			-- else create waypoints on the go with required headings and altitude
//
//              SET HEADING
//              -- gamma_D = gamma - (pi/2) - atan(K*(d - r)) 
//                      -- gamma_d = desired heading (wrt North)
//                      -- gamma   = current heading (wrt North)
//                      -- K       = Gain factor, decides how the aircraft gets into the orbit
//                      -- r       = Radius of the orbit to follow
//                      -- d       = distance between aircraft location and center location
****************************************************************************************************************************/

//function to set the center, radius, gain factor etc of the orbit to follow


