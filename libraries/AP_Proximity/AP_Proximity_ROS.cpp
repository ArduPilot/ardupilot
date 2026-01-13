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

 #include "AP_Proximity_config.h"

 #if AP_PROXIMITY_ROS_ENABLED
 
 #include "AP_Proximity_ROS.h"
 #include "AP_Proximity/AP_Proximity.h"
 
 extern const AP_HAL::HAL& hal;
 
 #define PROXIMITY_ROS_TIMEOUT_MS    200 // distance messages must arrive within this many milliseconds
 
 void AP_Proximity_ROS::handle_laser_scan(const sensor_msgs_msg_LaserScan& msg)
 {
     const float angle_min = msg.angle_min;
     const float angle_increment = msg.angle_increment;
     const float range_min = msg.range_min;
     const float range_max = msg.range_max;
 
     for (uint32_t i = 0; i < msg.ranges_size; ++i) {
         const float range = msg.ranges[i];
         if (range >= range_min && range <= range_max) {
             const float angle = angle_min + i * angle_increment;
             const float angle_deg = wrap_360(degrees(-angle));
 
             const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
 
             if (face != _last_face) {
                 // distance is for a new face, the previous one can be updated now
                 if (_last_distance_valid) {
                     frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
                 } else {
                     // reset distance from last face
                     frontend.boundary.reset_face(face, state.instance);
                 }
 
                 // initialize the new face
                 _last_face = face;
                 _last_distance_valid = false;
             }
             if (range > frontend.distance_min()) {
                 // update shortest distance
                 if (!_last_distance_valid || (range < _last_distance_m)) {
                     _last_distance_m = range;
                     _last_distance_valid = true;
                     _last_angle_deg = angle_deg;
                 }
                 // update OA database
                 database_push(_last_angle_deg, _last_distance_m);
             }
         }
     }
     _last_update_ms = AP_HAL::millis();
 }
 
 void AP_Proximity_ROS::update()
 {
     // タイムアウト時間を200msに設定
     if (AP_HAL::millis() - _last_update_ms > PROXIMITY_ROS_TIMEOUT_MS) {
         set_status(AP_Proximity::Status::NoData);
     } else {
         set_status(AP_Proximity::Status::Good);
     }
 }
 
 #endif // AP_PROXIMITY_ROS_ENABLED
 