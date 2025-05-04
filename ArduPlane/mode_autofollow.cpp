#include "mode.h"
#include "Plane.h"
#include "GCS_Mavlink.h"
#include "math.h"

bool ModeAutoFollow::_enter()
{
    // plane.throttle_allows_nudging = true;
    // plane.auto_throttle_mode = true;
    // plane.auto_navigation_mode = true;
    

    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();
if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif


     return true;
}


void ModeAutoFollow::_exit()
{
    // plane.next_WP_loc.relative_alt = 1;
    // plane.next_WP_loc.alt = 100;
    // plane.set_guided_WP(plane.next_WP_loc);
    //plane.guided_WP_loc.lat = -353472656L;
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Best number ever: %d", plane.guided_WP_loc.lat);
    

   // const AP_AHRS &ahrs = AP::ahrs();

    // float lat = plane.current_loc.lat * (0.0000001) + (0.009*sinf((3.141593/2)-ahrs.yaw));//pow(10,-7) + (0.009* sin((3.141593/2)-ahrs.yaw ));
    // float lng = plane.current_loc.lng * (0.0000001) + (0.009*cosf((3.141593/2)-ahrs.yaw));//pow(10,-7) + (0.009* cos((3.141593/2)-ahrs.yaw));


    // plane.guided_WP_loc.lat = (int)(lat * (10000000));//* pow(10,7));
    // plane.guided_WP_loc.lng = (int)(lng * (10000000));//* pow(10,7));
    
    // plane.guided_WP_loc.alt = plane.current_loc.alt;

    // plane.next_WP_loc.lat = (int)(lat * (10000000));//* pow(10,7));
    // plane.next_WP_loc.lng = (int)(lng * (10000000));//* pow(10,7));
    
    // plane.next_WP_loc.relative_alt = 1;
    // plane.next_WP_loc.alt = 100;
   // plane.next_WP_loc.alt = plane.current_loc.alt;
    




    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%d",plane.guided_WP_loc.alt) ; 
    // plane.set_guided_WP();  
     // plane.set_guided_WP(plane.next_WP_loc);
       // plane.set_mode(plane.mode_auto, ModeReason::MISSION_END);
   
    // if(plane.g.auto_follow_enable == 0 && plane.adjusted_relative_altitude_cm()/100 >= plane.g.follow_min_alt){
     // plane.set_mode(plane.mode_auto, ModeReason::MISSION_END);
     // }
}

void ModeAutoFollow::update()
{   

       // if(!plane.g.auto_follow_enable || plane.g.follow_min_alt > plane.adjusted_relative_altitude_cm()/100){
          //  plane.set_mode(plane.mode_guided, ModeReason::MISSION_END);
       // }

        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();


        //plane.next_WP_loc.alt = (int32_t)plane.g.follow_pitch;
        // plane.next_WP_loc.set_alt_cm((int32_t)plane.g.follow_pitch ,  plane.next_WP_loc.get_alt_frame() );
        plane.next_WP_loc.alt = ((int32_t)plane.g.follow_pitch*100);
        plane.next_WP_loc.relative_alt = 1;
        plane.set_guided_WP(plane.next_WP_loc);
       
             
        // plane.set_next_WP(plane.next_WP_loc);
         
        //plane.set_target_altitude_proportion((int32_t)plane.g.follow_pitch);
        
        //plane.calc_nav_yaw_coordinated(0);
       //const AP_AHRS &ahrs = AP::ahrs();
    
}

void ModeAutoFollow::navigate()
{
      if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}

bool ModeAutoFollow::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

bool ModeAutoFollow::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}
