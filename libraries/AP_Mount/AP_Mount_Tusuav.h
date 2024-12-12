/*
 * @Author: Yu Sun,Yiyun Gong
 * @Date: 2024-10
 * @Description: Tusuav, Leading the way in drone innovation!
 
  Tusuav gimbal driver using custom serial protocol
  Packet format (courtesy of Tusuav's SDK document)

  -------------------------------------------------------------------------------------------
  Field     Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  Header    0       1           0x5A
  Length    1       1           total length, n=all bytes from byte5 to checksum, min=6, max=128
  Flag      2       1           bit0~2: board id of the gimbal 0=main controller 1=comm board 2~4=motor pitch/roll/yaw
                                bit3: 1= need reply 0 = no need
                                bit4: 1= is a reply message 0 = now a reply message
                                bit5: 1= is a reading message 0 = is a writing message                                

  Cmd       3       1           see details at below
  Sub Cmd   4       1           see details at below
  Data      5~n+1   n           data in little endian lowest byte first
  Checksum  n+2     1           calculate_crc from mavlink 2.0 of byte from 0 to n+1 (inclusive)
 */

#pragma once

#include "AP_Mount_config.h"
#if HAL_MOUNT_TUSUAV_ENABLED
#include "AP_Mount_Backend_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>

#define AP_MOUNT_TUSUAV_PACKETLEN_MAX  255  // maximum number of bytes in a packet sent to or received from the gimbal

class AP_Mount_Tusuav : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Tusuav);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // return true if this mount accepts roll targets
    bool has_roll_control() const override { return false; };

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return true; };
	
    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t lens) override;

    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    // get rangefinder distance.  returns true on success
    bool get_rangefinder_distance(float& distance_m) const override;

    // enable/disable rangefinder.  returns true on success
    bool set_rangefinder_enable(bool enable) override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;

    // gimbal main commands for controlling gimbal behavior
    enum class GimbalCmd : uint8_t {
        MasCmd_Load   = 0xBC,  // load settings or configurations
        MasCmd_Gimbal = 0xBD,  // gimbal operations
        MasCmd_Device = 0x0B   // device-specific operations
    };

    // gimbal sub-commands for controlling and retrieving gimbal status
    enum class GimbalSubCmd : uint8_t {
        GimbalSubCmd_GimbalAngle           =   0x16,   // retrieve gimbal angles
        GimbalSubCmd_PlaneSensor_Attitude  =   0x18,   // send plane sensor & attitude to gimbal
        GimbalSubCmd_PlaneControlPara      =   0x19,   // control parameters
        GimbalSubCmd_TimeSynchronize       =   0x20,   // synchronize time with gimbal
        GimbalSubCmd_LaserDistance         =   0x21,   // retrieve laser distance measurements
        GimbalSubCmd_PodCtrlReg            =   0x2B,   // control settings for gimbal pod
        GimbalSubCmd_GimbalSn              =   0x34,   // read gimbal serial number
        GimbalSubcmd_LaserState            =   0x59,   // enable/disable laser rangefinder state
        DebugSubCmd_InnerFramePIDPara      =   0x60,   // internal frame PID parameter debugging
        gimbal_single_cmd_stick_control    =   0xA0,   // joystick control pitch&yaw
        gimbal_single_cmd_track_switch     =   0xA2,   // track start and target select
        gimbal_single_cmd_trackpos_set     =   0xA3,   // track target select
        gimbal_single_cmd_record_switch    =   0xA4,   // record start and stop
        gimbal_single_cmd_infrared_display =   0xA5,   // infrared camera color mode
        gimbal_single_cmd_zoomspeed_set    =   0xA6,   // zoom in speed mode
        gimbal_single_cmd_zoomvalue_set    =   0xA7,   // zoom in position mode
        gimbal_single_cmd_trackbox_set     =   0xA8,   // track box size setting
    };

    // device sub-command for retrieving firmware info
    enum class DeviceSubCmd : uint8_t {
        Firmware_Info = 0x03,   // retrieve firmware version information
    };

    // rangefinder command 
    enum class LRFSettings : uint8_t {
        RangeFinder_Off = 0x00,                    // turn off rangefinder
        RangeFinder_Measure_Once = 0x02,           // measure distance once
        RangeFinder_Continuously_Measure = 0x03    // continuously measure distance
    };

    // recording status 
    enum class RecordingCmd : uint8_t {
        No_Action = 0x00,     // no recording action
        Start = 0x01,         // start recording
        Stop = 0x02           // stop recording
    };

    // display output source 
    enum class CameraSource : uint8_t {
        EO = 0x00,             // electro-optical (RGB camera)
        IR = 0x01,             // infrared (thermal camera)
        PIP_EO = 0x02,         // Picture-in-Picture, EO as main and IR in a small window
        PIP_IR = 0x03          // Picture-in-Picture, IR as main and EO in a small window
    };

    // infrared color modes for thermal imaging
    enum class IRColor : uint8_t { 
        White = 0x00,	    	// infrared, white hot 
        Black = 0x01,	     	// infrared, black hot
    };

    // tracking status for gimbal
    enum class TrackingStatus : uint8_t {
        Stopped = 0x00,        // not tracking
        Searching = 0x01,      // searching
        Tracking = 0x02,       // locked onto a target
        Lost = 0x03,           // lost target
    };

    // parsing states for handling incoming gimbal packets
    enum class ParseState : uint8_t { 
        Waiting_For_Header,           // waiting for the start of a packet
        Waiting_For_Total_Length,     // waiting for the total length of the packet
        Waiting_For_Flag,             // waiting for the flag byte
        Waiting_For_Cmd,              // waiting for the command byte
        Waiting_For_Sub_Cmd,          // waiting for the sub-command byte
        Waiting_For_Data,             // waiting for the packet's data bytes
        Waiting_For_CRC               // waiting for the CRC byte to validate the packet
    };

    struct FrameType {
        union {
            uint8_t all;          
            struct {
                uint8_t no_define   : 3;     // reserved/undefined bits
                uint8_t is_need_ack : 1;     // if an acknowledgment is required
                uint8_t is_ack_msg  : 1;     // if this message is an acknowledgment
                uint8_t is_read_cmd : 1;     // if this is a read command
                uint8_t no_define2  : 2;     // reserved/undefined bits
            } bits;              
        } flag;                 
        uint8_t total_len;       // total length of the frame, including header and CRC
        uint8_t data_len;        // length of the data payload
        uint8_t cmd;             // command byte indicating the main command type
        uint8_t sub_cmd;         // sub-command byte indicating the specific action
        uint8_t crc;             // CRC byte for error checking
        uint8_t* data;           // pointer to the data payload
    };

    struct ParsedRxMsg {
        struct FrameType msg;      // the parsed message 
        ParseState state;          // the current state of the message parser
    } _parsed_msg;

    union PodCtrlUnion {
        struct PodCtrlBitField {
            uint8_t torque_on    : 1;        // always 0xB1
            uint8_t display_mode : 2;        // 0 = EO camera, 1 = IR camera, 2 = EO-PIP, 3 = IR-PIP
            uint8_t fpv          : 1;        // 1 if the direction of the vel(x,y)
            uint8_t lock_ef      : 1;        // 1 if gimbal is locked in earth frame mode
            uint8_t in_tracking  : 1;        // 1 if gimbal is in tracking mode
            uint8_t osd_off      : 1;        // 1 if on-screen display (OSD) is off
        } bits;                        
        uint8_t all;                   
    };

    union RecordPhotoSettings {
        struct RecordBitField {
            uint8_t trigger_capture  : 1;     // photo once if this bit turns to 1
            RecordingCmd record      : 2;     // recording command: start, stop, or no action
            uint8_t no_defined       : 5;     // reserved/undefined bits
        } bits;                            
        uint8_t all;                       
    };

    struct PACKED MessageCameraSource {
        IRColor color;              // infrared color mode 
        CameraSource source;        // camera source
    };

    struct PACKED MessageGimbalAngle {
        float gimbal_angle[3];           // gimbal euler angle: pitch, roll, yaw in degrees
        float gimbal_frame_angle[3];     // gimbal frame angle: pitch, roll, yaw in degrees

        // union for tracking status and other gimbal settings
        union TrackingStatusUnion {
            struct StatusBitField {
                uint8_t is_tracking  : 1;        // 1 if gimbal is actively tracking
                uint8_t osd_off      : 1;        // 1 if OSD (on-screen display) is off
                uint8_t recording    : 1;        // 1 if video recording is active
                uint8_t photo_on     : 1;        // 1 if a photo is being captured
                uint8_t display_mode : 2;        // display mode (0: EO, 1: IR, 2: EO-PIP, 3: IR-PIP)
                uint8_t retracking   : 1;        // 1 if gimbal is retracking a target
                uint8_t no_define1   : 1;        // reserved/undefined bit
                uint8_t ai_detect    : 1;        // 1 if AI object detection is active
                uint8_t eis_on       : 1;        // 1 if electronic image stabilization is on
                uint8_t human_detect : 1;        // 1 if human detection is enabled
                uint8_t inf_white    : 1;        // 1 if infrared white-hot mode is active
                uint8_t no_define2   : 1;        // reserved/undefined bit
                uint8_t ai_density   : 1;        // 1 if AI density detection is enabled
                uint8_t ai_vehicle   : 1;        // 1 if AI vehicle detection is enabled
            } bits;                      
            uint16_t all;                
        } status;                         

        enum QualityEnum : uint8_t {
            Track_Quality_None   = 0,       // no tracking quality
            Track_Quality_Weak   = 128,     // weak tracking quality
            Track_Quality_Normal = 255      // normal tracking quality
        } quality;

        uint8_t zoom;  
    };

    // GimbalSubCmd_GimbalSensor
    struct PACKED MessageGimbalSensor{
        int16_t imu_temperature;
        float gimbal_acc[3];    //gimbal body frame NED accelerated in G(9.8m/s^2)
        float gimbal_gyro[3];   //gimbal body frame NED angular speed in degree/sec
    };

    // M_AHRS used to send vehicle attitude and position to gimbal 
    enum class AircraftType : uint8_t {
        Aircraft_Vtol,        // Vertical Take-Off and Landing (VTOL) aircraft
        Aircraft_Copter,      // Multi-rotor copter aircraft
    };
    // GimbalSubCmd_PlaneSensor_Attitude data frame
    struct PACKED MessageAhrs{
        uint8_t gps_update_flag;
        union FcStatus {
            struct {
                uint8_t aircraft_type:3;      
                uint8_t is_slant_fly:1;
                uint8_t no_define:1;
                uint8_t have_taken_off:1;
                uint8_t disable_control_pod:1;
                uint8_t no_define1:1;
            } bits;
            uint8_t all;
        } fc_status;
        float fc_gyro[3];             // fc body frame NED angular speed in degrees/sec
        float fc_acc[3];              // fc body frame NED acceleration in G (9.8m/s^2)
        float vel[3];                 // fc earth coordinate NED velocity in m/s
        float euler[3];               // c euler angle PRY in deree
        float relative_height;        
        float altitude;               
        float longitude;              
        float latitude;               
    };

    //GimbalSubCmd_PlaneSensor_Attitude data frame
    struct PACKED MessageTimeSync{
        struct PACKED DateType{
            uint8_t year;          
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t min;
            uint8_t sec;
        }time;

        struct PACKED LocType {
            int32_t lat;               // latitude *1e7
            int32_t lng;               // longitude
            int32_t altitude_cm;       // altitude (cm)
            uint32_t time_of_week_ms;  // GPS time of the week
        } loc;

    };

    // M_AHRS used to send vehicle attitude and position to gimbal
    enum class GimbalAngleMode : uint8_t {
        Stabilize,
        Earth_Frame_Angle,
        Earth_Frame_Rates,
        Body_Frame_Angle,
    };
    // GimbalSubCmd_PlaneControlPara
    struct PACKED MessageGimbalAngleRates{
        float ef_angle[3];    // gimbal earth frame angle in degree
        float rates[3];       // gimbal earth frame rates in degree/sec
        float bf_angle[3];    // gimbal body frame angle in degree
        union{
            struct PACKED{
                uint8_t pitch:2;
                uint8_t roll:2;
                uint8_t yaw:2;
                uint8_t special:2;
            }bits;
            uint8_t all;
        }mode;
    };
    // reading incoming packets from gimbal and confirm they are of the correct format
    // results are held in the _parsed_msg structure
    void read_incoming_packets();

    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet(const FrameType& frame);

    // send handshake, gimbal will respond with T1_F1_B1_D1 packet that includes current angles
    void send_time_sync();

    // request the current tracking status from the gimbal
    bool request_tracking_status();

    // request the gimbal's firmware version
    bool request_versions();

    // request the gimbal's model name or serial number
    bool request_models();

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    bool set_lock(bool lock);

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    bool send_target_angles_rates(MessageGimbalAngleRates mode);

    // send joystick inputs for controlling the gimbal's pitch and yaw axes
    bool send_stick_pitch_yaw(int16_t pitch,int16_t yaw);

    // send vehicle attitude and position to gimbal
    bool send_m_ahrs();

    // internal variables
    uint8_t _msg_buff[AP_MOUNT_TUSUAV_PACKETLEN_MAX];   // buffer holding latest bytes from gimbal
    uint8_t _msg_buff_len;                              // number of bytes held in msg buff

    // variables for sending packets to gimbal
    uint8_t _gps_update_count;                          // counter for GPS updates sent to the gimbal
    uint32_t _gps_last_update_ms;                       // timestamp of the last GPS update 
    uint32_t _last_time_sync_ms_send;                   // timestamp of the last time synchronization packet sent to the gimbal
    uint32_t _last_update_ms_send;                      // system time (in milliseconds) that angle or rate targets were last sent
    uint32_t _last_update_ms_parse;                     // system time (in milliseconds) that incoming data was last parsed
    Vector3f _current_angle_rad;                        // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;                // system time _current_angle_rad was updated (used for health reporting)
    uint32_t _last_tracking_ms;                         // timestamp of the last tracking status update from the gimbal
    bool _recording;                                    // recording status received from gimbal
    bool _last_lock;                                    // last lock mode sent to gimbal
    TrackingStatus _last_tracking_status;               // last tracking status received from gimbal (used to notify users)
    float _zoom_times;                                  // zoom times received from gimbal
    uint32_t _firmware_version;                         // firmware version from gimbal
    bool _got_firmware_version;                         // true once we have received the firmware version
    uint8_t _model_sn[21] {};                           // model sn received from gimbal
    bool _got_model_name;                               // true once we have received model name
    float _rangefinder_dist_m;                          // latest rangefinder distance (in meters)
    PodCtrlUnion _gimbal_ctrl_settings = {.all = 0x01}; // control settings for the gimbal
    IRColor _IR_color = IRColor::White;                 // current infrared color mode 
    TrackingType _tracking_type = TrackingType::TRK_NONE;  // current tracking type
};

#endif // HAL_MOUNT_TUSUAV_ENABLED