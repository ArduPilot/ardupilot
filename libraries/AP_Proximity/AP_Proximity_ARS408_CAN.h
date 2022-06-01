#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

class AP_Proximity_ARS408_CAN : public CANSensor, public AP_Proximity_Backend {
public:
    AP_Proximity_ARS408_CAN(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    enum class CAN_ID:uint32_t{
    
        Cluster_Status  = 0x600,
        Cluster_General = 0x701,
  
        Object_Status   = 0x60A,
        Object_General  = 0x60B,
        Object_Quality  = 0x60C,
        Object_Extended = 0x60D,
    };

    enum class OBJ_MEAS_STATE:uint16_t{
        DELETE      = 0x00,
        NEW         = 0x01,
        MEASURED    = 0x02,
        PREDICTED   = 0x03,
        DELETE_FOR_MERGE = 0x04,
        NEW_FOR_MERGE    = 0x05,
    };

    enum class OBJ_PROB_EXIST:uint16_t{
        INVALID       = 0x00,
        PERCENT_25    = 0x01,
        PERCENT_50    = 0x02,
        PERCENT_75    = 0x03,
        PERCENT_90    = 0x04,
        PERCNET_99    = 0x05,
        PERCENT_99_9  = 0x06,
        PERCENT_100   = 0x07,
    };

    enum ContiDynProp {
        CONTI_MOVING = 0,
        CONTI_STATIONARY = 1,
        CONTI_ONCOMING = 2,
        CONTI_STATIONARY_CANDIDATE = 3,
        CONTI_DYNAMIC_UNKNOWN = 4,
        CONTI_CROSSING_STATIONARY = 5,
        CONTI_CROSSING_MOVING = 6,
        CONTI_STOPPED = 7
    };

    enum ContiObjectType {
        CONTI_POINT = 0,
        CONTI_CAR = 1,
        CONTI_TRUCK = 2,
        CONTI_PEDESTRIAN = 3,
        CONTI_MOTOCYCLE = 4,
        CONTI_BICYCLE = 5,
        CONTI_WIDE = 6,
        CONTI_TYPE_UNKNOWN = 7,
        CONTI_MAX_OBJECT_TYPE = 8
    };


    typedef union speed_information {
    struct {
        uint64_t RadarDevice_Speed1:5;
        uint64_t Reserved:1;
        uint64_t RadarDevice_SpeedDirection:2;
        uint64_t RadarDevice_Speed2:8;
    } data = {};

    uint8_t raw_data[2];
    } speed_information;

    typedef enum RadarDevice_SpeedDirection {
        STANDSTILL = 0x0,
        FORWARD = 0x1,
        BACKWARD = 0x2,
    } RadarDevice_SpeedDirection;

    typedef union yaw_rate_information {
    struct {
        uint64_t RadarDevice_YawRate1:8;
        uint64_t RadarDevice_YawRate2:8;
    } data = {};

    uint8_t raw_data[2];
    } yaw_rate_information;

    typedef union object_0_status {
        struct {
            uint64_t Object_NofObjects:8;
            uint64_t Object_MeasCounter1:8;
            uint64_t Object_MeasCounter2:8;
            uint64_t Reserved:4;
            uint64_t Object_InterfaceVersion:4;
        } data = {};

        uint8_t raw_data[4];
    } object_0_status;

    typedef union object_1_general {
        struct {
            uint64_t Object_ID:8;
            uint64_t Object_DistLong1:8;
            uint64_t Object_DistLat1:3;
            uint64_t Object_DistLong2:5;
            uint64_t Object_DistLat2:8;
            uint64_t Object_VrelLong1:8;
            uint64_t Object_VrelLat1:6;
            uint64_t Object_VrelLong2:2;
            uint64_t Object_DynProp:3;
            uint64_t Reserved:2;
            uint64_t Object_VrelLat2:3;
            uint64_t Object_RCS:8;
        } data = {};
        uint8_t raw_data[8];
    } object_1_general;

    typedef union object_2_quality
    {
        struct
        {
            uint64_t Obj_ID : 8;
            uint64_t Obj_DistLat_rms1 : 3;
            uint64_t Obj_DistLong_rms : 5;
            uint64_t Obj_VrelLat_rms1 : 1;
            uint64_t Obj_VrelLong_rms : 5;
            uint64_t Obj_DistLat_rms2 : 2;
            uint64_t Obj_ArelLong_rms1 : 4;
            uint64_t Obj_VrelLat_rms2 : 4;
            uint64_t Obj_Orientation_rms1 : 2;
            uint64_t Obj_ArelLat_rms : 5;
            uint64_t Obj_ArelLong_rms2 : 1;
            uint64_t Reserved1 : 5;
            uint64_t Obj_Orientation_rms2 : 3;
            uint64_t Reserved2 : 2;
            uint64_t Obj_MeasState : 3;
            uint64_t Obj_ProbOfExist : 3;
        } data = {};
        uint8_t raw_data[8];
    }object_2_quality;

    typedef union object_3_extended
    {
        struct
        {
            uint64_t Object_ID : 8;
            uint64_t Object_ArelLong1 : 8;
            uint64_t Object_ArelLat1 : 5;
            uint64_t Object_ArelLong2 : 3;
            uint64_t Object_Class : 3;
            uint64_t Reserved : 1;
            uint64_t Object_ArelLat2 : 4;
            uint64_t Object_OrientationAngle1 : 8;
            uint64_t Reserved2 : 6;
            uint64_t Object_OrientationAngle2 : 2;
            uint64_t Object_Length : 8;
            uint64_t Object_Width : 8;
        } data = {};
        uint8_t raw_data[8];
    }object_3_extended;

    typedef union cluster_0_status {
    struct {
        uint64_t Cluster_NofClustersNear:8;
        uint64_t Cluster_NofClustersFar:8;
        uint64_t Cluster_MeasCounter1:8;
        uint64_t Cluster_MeasCounter2:8;
        uint64_t Reserved:4;
        uint64_t Cluster_InterfaceVersion:4;
    } data = {};

        uint8_t raw_data[5];
    } cluster_0_status;

    typedef union cluster_1_general {
    struct {
        uint64_t Cluster_ID:8;
        uint64_t Cluster_DistLong1:8;
        uint64_t Cluster_DistLat1:2;
        uint64_t Reserved:1;
        uint64_t Cluster_DistLong2:5;
        uint64_t Cluster_DistLat2:8;
        uint64_t Cluster_VrelLong1:8;
        uint64_t Cluster_VrelLat1:6;
        uint64_t Cluster_VrelLong2:2;
        uint64_t Cluster_DynProp:3;
        uint64_t Reserved2:2;
        uint64_t Cluster_VrelLat2:3;
        uint64_t Cluster_RCS:8;
    } data = {};
       uint8_t raw_data[8];
    } cluster_1_general;

    typedef union cluster_2_quality
    {
        struct
        {
            uint64_t Cluster_ID : 8;
            uint64_t Cluster_DistLat_rms1 : 3;
            uint64_t Cluster_DistLong_rms : 5;
            uint64_t Cluster_VrelLat_rms1 : 1;
            uint64_t Cluster_VrelLong_rms : 5;
            uint64_t Cluster_DistLat_rms2 : 2;
            uint64_t Cluster_Pdh0 : 3;
            uint64_t Reserved : 1;
            uint64_t Cluster_VrelLat_rms2 : 4;
            uint64_t Cluster_AmbigState : 3;
            uint64_t Cluster_InvalidState : 5;
        } data = {};
        uint8_t raw_data[5];
    }cluster_2_quality;


typedef union radar_cfg {
  struct {
    uint64_t RadarCfg_MaxDistance_valid:1;
    uint64_t RadarCfg_SensorID_valid:1;
    uint64_t RadarCfg_RadarPower_valid:1;
    uint64_t RadarCfg_OutputType_valid:1;
    uint64_t RadarCfg_SendQuality_valid:1;
    uint64_t RadarCfg_SendExtInfo_valid:1;
    uint64_t RadarCfg_SortIndex_valid:1;
    uint64_t RadarCfg_StoreInNVM_valid:1;
    uint64_t RadarCfg_MaxDistance1:8;
    uint64_t Reserved:6;
    uint64_t RadarCfg_MaxDistance2:2;
    uint64_t Reserved2:8;
    uint64_t RadarCfg_SensorID:3;
    uint64_t RadarCfg_OutputType:2;
    uint64_t RadarCfg_RadarPower:3;
    uint64_t RadarCfg_CtrlRelay_valid:1;
    uint64_t RadarCfg_CtrlRelay:1;
    uint64_t RadarCfg_SendQuality:1;
    uint64_t RadarCfg_SendExtInfo:1;
    uint64_t RadarCfg_SortIndex:3;
    uint64_t RadarCfg_StoreInNVM:1;
    uint64_t RadarCfg_RCS_Threshold_valid:1;
    uint64_t RadarCfg_RCS_Threshold:3;
    uint64_t Reserved3:4;
    uint64_t Reserved4:8;
  } data = {};

  uint8_t raw_data[8];
} radar_cfg;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

protected:
    void send_speed_message();
    void send_yaw_rate_message();
    bool get_forward_speed(float &speed) const;
    bool set_max_distance(uint64_t distance, bool valid = true);

    bool set_sensor_id(int id, bool valid = true);
    bool set_radar_power(int power, bool valid = true);
    bool set_output_type(int output_type, bool valid = true);
    void set_send_quality(bool quality, bool valid = true);
    void set_send_ext_info(bool send_ext, bool valid = true);
    bool set_sort_index(int sort_index, bool valid = true);
    void set_ctrl_relay_cfg(bool ctrl_relay, bool valid = true);
    void set_store_in_nvm(bool store_in_nvm, bool valid = true);
    bool set_rcs_threshold(int rcs_threshold, bool valid = true);

private:
    object_0_status obj_status_;
    std::vector<object_1_general> obj_general_list_;
    std::vector<object_2_quality> obj_quality_list_;
    std::vector<object_3_extended> obj_extended_list_;
    radar_cfg radar_cfg_msg;
    bool initialized_{false};

private:
    uint32_t _last_reading_ms;
    // variables to calculate closest angle and distance for each face
    AP_Proximity_Boundary_3D::Face _last_face;///< last face requested
    float _last_angle_deg;                    ///< yaw angle (in degrees) of _last_distance_m
    float _last_distance_m;                   ///< shortest distance for _last_face
    bool _last_distance_valid;                ///< true if _last_distance_m is valid
};
#endif //HAL_MAX_CAN_PROTOCOL_DRIVERS

