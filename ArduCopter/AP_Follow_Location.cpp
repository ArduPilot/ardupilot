#include "AP_Follow_Location.h"
#include "Copter.h"
#include <AP_Common/Location.h>


AP_Follow_Location::AP_Follow_Location(){

}

void AP_Follow_Location::_init(){
   StartLoc.alt = 13;
   StartLoc.lat = 570138730;
   StartLoc.lng = 99874617;

   NewLoc.alt = StartLoc.alt;
   NewLoc.lat = StartLoc.lat;
   NewLoc.lng = StartLoc.lng;
}

bool AP_Follow_Location::get_location(){
   // This function gets the location from the Link module

   /*
   if (received_location()){
      NewLoc.alt = receivedLoc.alt;
      NewLoc.lat = receivedLoc.lat;
      NewLoc.lng = receivedLoc.lng;
      return true;
   } else{
      return false;
   }
   */

   // Temporary location for testing
   // NewLoc.alt = StartLoc.alt;
   // NewLoc.lat = StartLoc.lat;
   // NewLoc.lng = StartLoc.lng;
   
   if (count != previous_count) { //whenever the count variable changes, add 5 to the lattitude and longitude
      NewLoc.lat -= 500;
      NewLoc.lng -= 500;
      previous_count = count;
   }
   return true;
}

bool AP_Follow_Location::check_location(){
   if (!check_latlng(NewLoc.lat, NewLoc.lng)) {
      // failed as the new location is not valid
      return false;
   } else if(NewLoc.sanitize(copter.current_loc)){ // if the location wasn't already sane don't load it
      // failed as the current location is not valid
      return false;
   } else{
      return true;
   }
}

void AP_Follow_Location::update_velocity(){
   range = 100; //The range of the allowed follow mode
   kp = 1; //The proportional gain for the velocity controller

   x1 = copter.current_loc.get_distance_NE(NewLoc).x;
   y1 = copter.current_loc.get_distance_NE(NewLoc).y;
   z1 = copter.current_loc.get_alt_cm(NewLoc.get_alt_frame(),NewLoc.alt);
   wp_len = sqrt(x1*x1 + y1*y1);

   // if the drone is out of range, it should land
   if (wp_len > range){
      copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
   } else if (wp_len > 7.5){
      vel.x = (x1*100/wp_len)*kp;  // x1 is in meters and vel.x should be in cm/s. Therefroe, we multiply by 100
      vel.y = (y1*100/wp_len)*kp;  // y1 is in meters and vel.y should be in cm/s. Therefroe, we multiply by 100
      vel.z = z1*100/wp_len;
   } else {
      vel.x = 0;   // x, y, and z are set to 0 for hovering
      vel.y = 0;
      vel.z = 0;

      count += 1; // counter to change the location
   }
   copter.mode_guided.set_velocity(vel);
}


void AP_Follow_Location::update_destination(){
   // we need to do this first, as we don't want to change the flight mode unless we can also set the target
   if (!copter.mode_guided.set_destination(NewLoc, false, 0, false, 0)) {
      //copter.mode_guided.set_speed_xy(100*100);
      DoesItWork = false;
      return; 
   }

   if (!copter.flightmode->in_guided_mode()) {
      if (!copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
         DoesItWork = false;
         return;  
      }
      // the position won't have been loaded if we had to change the flight mode, so load it again
      if (!copter.mode_guided.set_destination(NewLoc, false, 0, false, 0)) {
         DoesItWork = false;
         return;  
      }
   }
}

