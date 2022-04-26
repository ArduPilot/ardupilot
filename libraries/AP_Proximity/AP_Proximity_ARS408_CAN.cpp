#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_ARS408_CAN.h"
#include <ctype.h>
#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <utility>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

#define RP_DEBUG_LEVEL 0

#if RP_DEBUG_LEVEL
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

 AP_Proximity_ARS408_CAN::AP_Proximity_ARS408_CAN(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state):
    CANSensor("RS408"),
    AP_Proximity_Backend(_frontend,_state)
    {
        register_driver(AP_CANManager::Driver_Type_ARS408);
        radar_cfg_msg.data.RadarCfg_MaxDistance_valid = 0;
        radar_cfg_msg.data.RadarCfg_SensorID_valid = 0;
        radar_cfg_msg.data.RadarCfg_RadarPower_valid = 0;
        radar_cfg_msg.data.RadarCfg_OutputType_valid = 0;
        radar_cfg_msg.data.RadarCfg_SendQuality_valid = 0;
        radar_cfg_msg.data.RadarCfg_SendExtInfo_valid = 0;
        radar_cfg_msg.data.RadarCfg_SortIndex_valid = 0;
        radar_cfg_msg.data.RadarCfg_CtrlRelay_valid = 0;
        radar_cfg_msg.data.RadarCfg_StoreInNVM_valid = 0;
        radar_cfg_msg.data.RadarCfg_RCS_Threshold_valid = 0;
    }


// update state
void AP_Proximity_ARS408_CAN::update(void)
{
    if ((_last_reading_ms == 0) || (AP_HAL::millis() - _last_reading_ms > 3000)) {
        set_status(AP_Proximity::Status::NoData);
    }

    if(!initialized_){
        initialized_ = true;

        // Store in NVM
        set_store_in_nvm(true);

        // Set output type [NOTE:clusters(0x02);objects(0x01)]
        set_output_type(0x01);

        // Set output sort
        set_sort_index(1);

        // Set max distance
        set_max_distance(200);

        // Set sensor id
        set_sensor_id(0);

        // Set radar power
        set_radar_power(0);

        // Set quality output[true]
        set_send_quality(1);

        // Set ext_info[true]
        set_send_ext_info(1);

        // Set control relay info
        set_ctrl_relay_cfg(0);

        AP_HAL::CANFrame frame;
        frame = {(0x200 & AP_HAL::CANFrame::MaskStdID), radar_cfg_msg.raw_data, sizeof(radar_cfg_msg.raw_data)};
        write_frame(frame, AP_HAL::native_micros64() + 1000);

        return;
    }
     // Send YWA_RATE and SPEED message
    send_yaw_rate_message();
    send_speed_message();
}

void AP_Proximity_ARS408_CAN::send_speed_message()
{
    float speed = 0.0f;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return;
    }

    // Set speed
    uint32_t radar_speed = static_cast<uint32_t>(fabs(speed) / 0.02);
    speed_information speed_information_msg;
    speed_information_msg.data.RadarDevice_Speed1 = static_cast<uint64_t>(radar_speed >> 8);
    speed_information_msg.data.RadarDevice_Speed2 = static_cast<uint64_t>(radar_speed & 255);
    speed_information_msg.data.RadarDevice_SpeedDirection = RadarDevice_SpeedDirection::STANDSTILL;
    if(speed > 0.2){speed_information_msg.data.RadarDevice_SpeedDirection  = RadarDevice_SpeedDirection::FORWARD;}
    if(speed < -0.2){speed_information_msg.data.RadarDevice_SpeedDirection = RadarDevice_SpeedDirection::BACKWARD;}

    // construct AP_HAL::CANframe
    AP_HAL::CANFrame speed_frame;
    speed_frame = {(0x300 & AP_HAL::CANFrame::MaskStdID), speed_information_msg.raw_data, sizeof(speed_information_msg.raw_data)};
    write_frame(speed_frame, AP_HAL::native_micros64() + 1000);

}

// get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
bool AP_Proximity_ARS408_CAN::get_forward_speed(float &speed) const
{
    Vector3f velocity;
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.get_velocity_NED(velocity)) {
        // use less accurate GPS, assuming entire length is along forward/back axis of vehicle
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            if (abs(wrap_180_cd(_ahrs.yaw_sensor - AP::gps().ground_course_cd())) <= 9000) {
                speed = AP::gps().ground_speed();
            } else {
                speed = -AP::gps().ground_speed();
            }
            return true;
        } else {
            return false;
        }
    }
    // calculate forward speed velocity into body frame
    speed = velocity.x*_ahrs.cos_yaw() + velocity.y*_ahrs.sin_yaw();
    return true;
}


void AP_Proximity_ARS408_CAN::send_yaw_rate_message()
{
    float yaw_rate = -degrees(AP::ahrs().get_yaw_rate_earth());
    uint32_t radar_yaw_rate = static_cast<uint32_t>((yaw_rate + 327.68) / 0.01);

    yaw_rate_information yaw_rate_raw;
    yaw_rate_raw.data.RadarDevice_YawRate1 = static_cast<uint64_t>(radar_yaw_rate >> 8);
    yaw_rate_raw.data.RadarDevice_YawRate2 = static_cast<uint64_t>(radar_yaw_rate & 255);

    // construct CANFrame
    AP_HAL::CANFrame yaw_rate_frame;
    yaw_rate_frame = {(0x301 & AP_HAL::CANFrame::MaskStdID), yaw_rate_raw.raw_data, sizeof(yaw_rate_raw.raw_data)};
    write_frame(yaw_rate_frame, AP_HAL::native_micros64() + 1000);
}


bool AP_Proximity_ARS408_CAN::set_max_distance(uint64_t distance, bool valid) {
  if (distance < 90 || distance > 1000) {
    return false;
  }
  distance /= 2;
  radar_cfg_msg.data.RadarCfg_MaxDistance1 = distance >> 2;
  radar_cfg_msg.data.RadarCfg_MaxDistance2 = distance & 0b11;
  radar_cfg_msg.data.RadarCfg_MaxDistance_valid = static_cast<uint64_t>(valid);
  return true;
}

bool AP_Proximity_ARS408_CAN::set_sensor_id(int id, bool valid) {
  if (id < 0 || id > 7) {
    return false;
  }
  radar_cfg_msg.data.RadarCfg_SensorID = static_cast<uint64_t>(id);
  radar_cfg_msg.data.RadarCfg_SensorID_valid = static_cast<uint64_t>(valid);
  return true;
}

bool AP_Proximity_ARS408_CAN::set_radar_power(int power, bool valid) {
  if (power < 0 || power > 3) {
    return false;
  }
  radar_cfg_msg.data.RadarCfg_RadarPower = static_cast<uint64_t>(power);
  radar_cfg_msg.data.RadarCfg_RadarPower_valid = static_cast<uint64_t>(valid);
  return true;
}

bool AP_Proximity_ARS408_CAN::set_output_type(int output_type, bool valid) {
  if (output_type < 0 || output_type > 2) {
    return false;
  }
  radar_cfg_msg.data.RadarCfg_OutputType = static_cast<uint64_t>(output_type);
  radar_cfg_msg.data.RadarCfg_OutputType_valid = static_cast<uint64_t>(valid);
  return true;
}

void AP_Proximity_ARS408_CAN::set_send_quality(bool quality, bool valid) {
  radar_cfg_msg.data.RadarCfg_SendQuality = static_cast<uint64_t>(quality);
  radar_cfg_msg.data.RadarCfg_SendQuality_valid = static_cast<uint64_t>(valid);
}

void AP_Proximity_ARS408_CAN::set_send_ext_info(bool send_ext, bool valid) {
  radar_cfg_msg.data.RadarCfg_SendExtInfo = static_cast<uint64_t>(send_ext);
  radar_cfg_msg.data.RadarCfg_SendExtInfo_valid = static_cast<uint64_t>(valid);
}

bool AP_Proximity_ARS408_CAN::set_sort_index(int sort_index, bool valid) {
  if (sort_index < 0 || sort_index > 2) {
    return false;
  }
  radar_cfg_msg.data.RadarCfg_SortIndex = static_cast<uint64_t>(sort_index);
  radar_cfg_msg.data.RadarCfg_SortIndex_valid = static_cast<uint64_t>(valid);
  return true;
}

void AP_Proximity_ARS408_CAN::set_ctrl_relay_cfg(bool ctrl_relay, bool valid) {
  radar_cfg_msg.data.RadarCfg_CtrlRelay = static_cast<uint64_t>(ctrl_relay);
  radar_cfg_msg.data.RadarCfg_CtrlRelay_valid = static_cast<uint64_t>(valid);
}

void AP_Proximity_ARS408_CAN::set_store_in_nvm(bool store_in_nvm, bool valid) {
  radar_cfg_msg.data.RadarCfg_StoreInNVM = static_cast<uint64_t>(store_in_nvm);
  radar_cfg_msg.data.RadarCfg_StoreInNVM_valid = static_cast<uint64_t>(valid);
}

bool AP_Proximity_ARS408_CAN::set_rcs_threshold(int rcs_threshold, bool valid) {
  if (rcs_threshold != 0 && rcs_threshold != 1) {
    return false;
  }
  radar_cfg_msg.data.RadarCfg_RCS_Threshold = static_cast<uint64_t>(rcs_threshold);
  radar_cfg_msg.data.RadarCfg_RCS_Threshold_valid = static_cast<uint64_t>(valid);
  return true;
}



// get maximum distance (in meters) of sensor
float AP_Proximity_ARS408_CAN::distance_max() const
{
    return 200.0f;  //200m max range
}

// get minimum distance (in meters) of sensor
float AP_Proximity_ARS408_CAN::distance_min() const
{
    return 0.5f;  // 0.5 min range 
}

// handler for incoming frames
void AP_Proximity_ARS408_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    uint32_t msg_id = frame.id;
    switch(CAN_ID(msg_id)){
        // 0x60A
        case CAN_ID::Object_Status:
        {
             _last_reading_ms = AP_HAL::millis();
             memcpy(obj_status_.raw_data, frame.data, frame.dlc);
             if(obj_status_.data.Object_NofObjects == 0){
                set_status(AP_Proximity::Status::NoData);
             }

            // clear contian list
            obj_general_list_.clear();
            obj_quality_list_.clear();
            obj_extended_list_.clear();

            break;
        }

        // 0x60B
        case CAN_ID::Object_General:
        {
            object_1_general obj1_general_;
            memcpy(obj1_general_.raw_data,frame.data, frame.dlc);
           
            //  Push message into vector
            obj_general_list_.emplace_back(obj1_general_);

            break;
        } 

        // 0x60C
        case CAN_ID::Object_Quality:
        {
            object_2_quality obj_quality;
            memcpy(obj_quality.raw_data,frame.data,frame.dlc);

            // Push message into vecotr
            obj_quality_list_.emplace_back(obj_quality);

            break;
        }

        // 0x60D
        case CAN_ID::Object_Extended:
        {
            object_3_extended obj_extended;
            memcpy(obj_extended.raw_data,frame.data,frame.dlc);

            obj_extended_list_.emplace_back(obj_extended);

            uint64_t id = obj_extended.data.Object_ID;
            float obj_length = obj_extended.data.Object_Length * 0.2f;
            float obj_width  = obj_extended.data.Object_Width  * 0.2f;
            int cls = obj_extended.data.Object_Class;
            if(cls == CONTI_POINT){
              obj_length = 1.0f;
              obj_width  = 1.0f;
            }
            
            if (obj_length * obj_width < 1.0e-4) {
              if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
                obj_length = 4.0f;
                obj_width  = 1.6f;  // vehicle template
              } else {
                obj_length= 1.0f;
                obj_width = 1.0f;
              }
            }

            float obj_radius = 0.5f * sqrtf(sq(obj_length) + sq(obj_width));

            auto itera = std::find_if(
                obj_quality_list_.begin(),obj_quality_list_.end(),
                [&](const object_2_quality & ps){
                    return ps.data.Obj_ID == id;
                });
            
            if(itera != obj_quality_list_.end()){
                // we find it
                 uint16_t obj_meas_state = itera->data.Obj_MeasState;
                 uint16_t obj_prob_exist = itera->data.Obj_ProbOfExist;

                // prob_exist >= 0x03(75%) and obj_meas_state != 0x03(predict)
                if(OBJ_MEAS_STATE(obj_meas_state) == OBJ_MEAS_STATE::MEASURED && OBJ_PROB_EXIST(obj_prob_exist) >= OBJ_PROB_EXIST::PERCENT_90){ 
                    auto index = std::find_if(
                        obj_general_list_.begin(),obj_general_list_.end(),
                        [&](const object_1_general ps){
                            return ps.data.Object_ID == id;
                        });

                    if(index != obj_general_list_.end()){
                        // get object we will deal with
                        const float lon_ds= (index->data.Object_DistLong1 << 5 |
                                            index->data.Object_DistLong2) * 0.2 - 500.0;
                        const float lat_ds = (index->data.Object_DistLat1 << 8 |
                                            index->data.Object_DistLat2) * 0.2 - 204.6;

                        const float angle_deg = wrap_360(degrees(-atan2f(lat_ds,lon_ds)));
                        const float distance_m = norm(lon_ds,lat_ds);
                        set_status(AP_Proximity::Status::Good);

                        const bool range_check = (distance_m > distance_max()) || (distance_m < distance_min());
                        if (range_check || ignore_reading(angle_deg, distance_m)) {
                          return;
                        }
                        
                        const AP_Proximity_Boundary_3D::Face face = boundary.get_face(angle_deg);
                        if (face != _last_face) {
                          // distance is for a new face, the previous one can be updated now
                          if (_last_distance_valid) {
                              boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m);
                          } else {
                              // reset distance from last face
                              boundary.reset_face(face);
                          }
                          // initialize the new face
                          _last_face = face;
                          _last_distance_valid = false;
                        } 

                        // update shortest distance
                        if (!_last_distance_valid || (distance_m < _last_distance_m)) {
                            _last_distance_m = distance_m;
                            _last_distance_valid = true;
                            _last_angle_deg = angle_deg;
                        }
                        // update OA database
                        database_push(angle_deg,distance_m);
                    }
                }
            }

            break;
        }
    }

}


#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
