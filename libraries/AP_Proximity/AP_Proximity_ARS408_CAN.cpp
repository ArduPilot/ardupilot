#include "AP_Proximity_config.h"

#if AP_PROXIMITY_ARS408_CAN_ENABLED 

#include "AP_Proximity_ARS408_CAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>

#include <ctype.h>
#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <utility>

const AP_Param::GroupInfo AP_Proximity_ARS408_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames. A value of zero means all IDs are accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RADAR_ID", 1, AP_Proximity_ARS408_CAN, radar_id, 0),


    AP_GROUPEND
};


ARS408_MultiCAN* AP_Proximity_ARS408_CAN::multican;

AP_Proximity_ARS408_CAN::AP_Proximity_ARS408_CAN(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_Proximity_Params &_params) :
    AP_Proximity_Backend(_frontend, _state, _params)
{
    if (multican == nullptr) {
        multican = new ARS408_MultiCAN();
        if (multican == nullptr) {
            AP_BoardConfig::allocation_error("ARS408_CAN");
        }
    }

    {
        // add to linked list of drivers
        WITH_SEMAPHORE(multican->sem);
        auto *prev = multican->drivers;
        next = prev;
        multican->drivers = this;
    }

    AP_Param::setup_object_defaults(this, var_info);
 
}


AP_Proximity_ARS408_CAN::~AP_Proximity_ARS408_CAN()
{
  
}

 void AP_Proximity_ARS408_CAN::initialize(void)
 {
    if (!_initialized) {
        // Store in NVM
        set_store_in_nvm(true);

        // Set output type [NOTE:clusters(0x02);objects(0x01)]
        set_output_type(0x01);

        // Set output sort
        set_sort_index(1);

        // Set max distance
        set_max_distance(_distance_max);

        // Set sensor id
        set_sensor_id(radar_id.get());

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
        multican->write_frame(frame, AP_HAL::native_micros64() + 1000);

        _initialized = true;
    } else {
        // Send YWA_RATE and SPEED message
        send_yaw_rate_message();
        send_speed_message();
    }
 }

// update state
void AP_Proximity_ARS408_CAN::update(void)
{
    initialize();

    if ((_last_reading_ms == 0) || (AP_HAL::millis() - _last_reading_ms > 3000)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
         set_status(AP_Proximity::Status::Good);
    }  
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
    multican->write_frame(speed_frame, AP_HAL::native_micros64() + 1000);

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
    multican->write_frame(yaw_rate_frame, AP_HAL::native_micros64() + 1000);
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


// handler for incoming frames
void AP_Proximity_ARS408_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    const uint32_t msg_id = (frame.id & AP_HAL::CANFrame::MaskStdID) - radar_id.get() * 0x10;
    switch (msg_id) {
        // 0x60A
        case  Object_Status:
        {
             _last_reading_ms = AP_HAL::millis();
             memcpy(_obj_status.raw_data, frame.data, frame.dlc);

             if(_obj_status.data.Object_NofObjects == 0){
                set_status(AP_Proximity::Status::NoData);
             }

            // clear contian list
            _obj_general_list.clear();
            _obj_quality_list.clear();
            _obj_extended_list.clear();

            break;
        }

        // 0x60B
        case Object_General:
        {
            object_1_general obj1_general;
            memcpy(obj1_general.raw_data, frame.data, frame.dlc);
           
            //  Push message into vector
            _obj_general_list.emplace_back(obj1_general);

            break;
        } 

        // 0x60C
        case Object_Quality:
        {
            object_2_quality obj_quality;
            memcpy(obj_quality.raw_data, frame.data, frame.dlc);

            // Push message into vecotr
            _obj_quality_list.emplace_back(obj_quality);

            break;
        }

        // 0x60D
        case Object_Extended:
        {
            object_3_extended obj_extended;
            memcpy(obj_extended.raw_data, frame.data, frame.dlc);

            _obj_extended_list.emplace_back(obj_extended);

            uint64_t id = obj_extended.data.Object_ID;
          
            auto itera = std::find_if(_obj_quality_list.begin(),_obj_quality_list.end(),
                                      [&](const object_2_quality & ps){ return ps.data.Obj_ID == id; });
            
            if (itera != _obj_quality_list.end()) {
                // we find it
                 uint16_t obj_meas_state = itera->data.Obj_MeasState;
                 uint16_t obj_prob_exist = itera->data.Obj_ProbOfExist;

                // prob_exist >= 0x03(75%) and obj_meas_state != 0x03(predict)
                if(OBJ_MEAS_STATE(obj_meas_state) == OBJ_MEAS_STATE::MEASURED && OBJ_PROB_EXIST(obj_prob_exist) >= OBJ_PROB_EXIST::PERCENT_90){ 
                    auto index = std::find_if(_obj_general_list.begin(),_obj_general_list.end(),
                                             [&](const object_1_general ps){ return ps.data.Object_ID == id; });

                    if(index != _obj_general_list.end()){
                        // get object we will deal with
                        const float lon_ds= (index->data.Object_DistLong1 << 5 |
                                            index->data.Object_DistLong2) * 0.2 - 500.0;
                        const float lat_ds = (index->data.Object_DistLat1 << 8 |
                                            index->data.Object_DistLat2) * 0.2 - 204.6;

                        const float object_angle_deg = wrap_360(degrees(-atan2f(lat_ds,lon_ds)));
                        const float angle_deg = correct_angle_for_orientation(object_angle_deg);
                        const float distance_m = norm(lon_ds, lat_ds);

                        const bool range_check = (distance_m > distance_max()) || (distance_m < distance_min());
                        if (range_check || ignore_reading(angle_deg, distance_m)) {
                          return;
                        }

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

                        // update shortest distance
                        if (!_last_distance_valid || (distance_m < _last_distance_m)) {
                            _last_distance_m = distance_m;
                            _last_distance_valid = true;
                            _last_angle_deg = angle_deg;
                        }

                        // update OA database
                        database_push(angle_deg, distance_m);
                    }
                }
            }

            break;
        }


        // clusters
        case Cluster_Status:
        {   
            cluster_0_status cluster_0_status_msg;
             _last_reading_ms = AP_HAL::millis();
             memcpy(cluster_0_status_msg.raw_data, frame.data, frame.dlc);

             if (cluster_0_status_msg.data.Cluster_NofClustersNear == 0 &&
                cluster_0_status_msg.data.Cluster_NofClustersFar == 0) {
                set_status(AP_Proximity::Status::NoData);
             }

             break;
        }

        case Cluster_General:
        {
            cluster_1_general cluster_1_general_msg;
            memcpy(cluster_1_general_msg.raw_data,frame.data, frame.dlc);

            const float lon_ds= (cluster_1_general_msg.data.Cluster_DistLong1 << 5 |
                                cluster_1_general_msg.data.Cluster_DistLong2) * 0.2 - 500.0;
            const float lat_ds = (cluster_1_general_msg.data.Cluster_DistLat1 << 8 |
                                 cluster_1_general_msg.data.Cluster_DistLat2) * 0.2 - 102.3;

            const float object_angle_deg  = wrap_360(degrees(-atan2f(lat_ds, lon_ds)));
            const float angle_deg = correct_angle_for_orientation(object_angle_deg);
            const float distance_m = norm(lon_ds, lat_ds);

            if (!ignore_reading(angle_deg, distance_m)) {
                // Get location on 3-D boundary based on angle to the object
                const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

               if ((distance_m <= distance_max()) && (distance_m >= distance_min())) {
                    frontend.boundary.set_face_attributes(face, angle_deg, distance_m, state.instance);
                    // update OA database
                    database_push(angle_deg, distance_m);
                } else {
                    // invalidate distance of face
                    frontend.boundary.reset_face(face, state.instance);
                }
            }
              break;
        } 
    }

}


// handle frames from CANSensor, passing to the drivers
void ARS408_MultiCAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    for (auto *d = drivers; d; d=d->next) {
        d->handle_frame(frame);
    }
}

#endif