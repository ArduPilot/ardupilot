/*
  ViewPro mount using serial protocol backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Mount_Backend.h"
#include <Filter/Filter.h>

#define AP_MOUNT_VIEWPRO_RESEND_MS   1000    // resend angle targets to gimbal once per second

class AP_Mount_ViewPro : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_ViewPro(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override;

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

   void set_init_cmd();

   void reset_camera();

private:

    void update_target_spd_from_rc();

    void update_zoom_focus_from_rc();

    void enable_follow_yaw();
    void disable_follow_yaw();


    void send_targeting_cmd();

    void send_zoom();

   // void set_to_follow(bool follow_en);

    void turn_motors_off(bool en);

    // send read data request
    void get_angles();

    // read_incoming
    void read_incoming();
    void parse_reply();

    void command_gimbal();
    bool yaw_center();
    void cmd_look_down();
    void zoom_camera();
    void zero_zoom();
    void full_zoom();
    void camera_state(int camera_state_cmd);

    void toggle_pip();
    void toggle_color();
    void default_pip_color();
    void cmd_flip_image_EO();
    void cmd_flip_image_IR();


    //void camera_state_trackingCam(int camera_state_cmd);

    void record();


    enum ReplyType {
        ReplyType_UNKNOWN = 0,
        ReplyType_Zoom_DATA,
		ReplyType_angle_DATA,
		ReplyType_Rec_State_DATA,
    };

    //void add_next_reply(ReplyType reply_type);
    uint8_t get_reply_size(ReplyType reply_type);
    bool can_send(bool with_control);

   // LowPassFilterFloat yaw_follow_filter;

    struct PACKED ViewPro_reply_angle_struct {
        uint8_t header1;
        uint8_t header2;
        uint8_t header3;
        uint8_t header4;
        int16_t roll_ang;
        uint8_t roll_RC_target_ang_1;
        uint8_t roll_RC_target_ang_2;
        uint8_t roll_rel_ang_1;
        uint8_t roll_rel_ang_2;
        uint8_t roll_rel_ang_3;
        uint8_t roll_rel_ang_4;
        uint8_t zero_1a;
        uint8_t zero_2a;
        uint8_t zero_3a;
        uint8_t zero_4a;
        uint8_t zero_5a;
        uint8_t zero_6a;
        uint8_t zero_7a;
        uint8_t zero_8a;
        uint8_t zero_9a;
        uint8_t zero_10a;
        int16_t pitch_ang;
        uint8_t pitch_RC_target_ang_1;
        uint8_t pitch_RC_target_ang_2;
        int32_t pitch_rel_ang;
        uint8_t zero_1b;
        uint8_t zero_2b;
        uint8_t zero_3b;
        uint8_t zero_4b;
        uint8_t zero_5b;
        uint8_t zero_6b;
        uint8_t zero_7b;
        uint8_t zero_8b;
        uint8_t zero_9b;
        uint8_t zero_10b;
        int16_t yaw_ang;
        uint8_t yaw_RC_target_ang_1;
        uint8_t yaw_RC_target_ang_2;
        int32_t yaw_rel_ang;
        uint8_t crc;
    };


    struct PACKED ViewPro_reply_video_state{
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t state;
        uint8_t byte6;

    };



    struct PACKED ViewPro_reply_zoom_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    };



    struct PACKED cmd_9_byte_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
        uint8_t byte8;
        uint8_t byte9;
    };



    struct PACKED cmd_set_angles_struct {
        uint8_t header1;
        uint8_t header2;
        uint8_t header3;
        uint8_t header4;
        uint8_t RM;
        uint8_t PM;
        uint8_t YM;
        int16_t Rs;
        int16_t Ra;
        int16_t Ps;
        int16_t Pa;
        int16_t Ys;
        int16_t Ya;
        uint8_t crc;
    };


    struct PACKED cmd_5_byte_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
    };


    struct PACKED cmd_6_byte_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
    };


    struct PACKED cmd_7_byte_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    };



    struct PACKED cmd_11_byte_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
        uint8_t byte8;
        uint8_t byte9;
        uint8_t byte10;
        uint8_t byte11;
    };


    struct PACKED cmd_48_byte_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
        uint8_t byte8;
        uint8_t byte9;
        uint8_t byte10;
        uint8_t byte11;
        uint8_t byte12;
        uint8_t byte13;
        uint8_t byte14;
        uint8_t byte15;
        uint8_t byte16;
        uint8_t byte17;
        uint8_t byte18;
        uint8_t byte19;
        uint8_t byte20;
        uint8_t byte21;
        uint8_t byte22;
        uint8_t byte23;
        uint8_t byte24;
        uint8_t byte25;
        uint8_t byte26;
        uint8_t byte27;
        uint8_t byte28;
        uint8_t byte29;
        uint8_t byte30;
        uint8_t byte31;
        uint8_t byte32;
        uint8_t byte33;
        uint8_t byte34;
        uint8_t byte35;
        uint8_t byte36;
        uint8_t byte37;
        uint8_t byte38;
        uint8_t byte39;
        uint8_t byte40;
        uint8_t byte41;
        uint8_t byte42;
		uint8_t byte43;
		uint8_t byte44;
		uint8_t byte45;
		uint8_t byte46;
		uint8_t byte47;
		uint8_t byte48;
    };



enum zoom_state{
	ZOOM_IN,
	ZOOM_OUT,
	ZOOM_STOP,
}current_zoom_state;


enum camera_state_cmd{
	TOGGLE_REC,
	TURN_VID_OFF,
	TOGGLE_STATE,
};

enum rec_state{
	VID_REC_ON,
	VID_REC_OFF,
	PIC_MODE,

}current_rec_state;


struct query_flags {
    bool angles;
	bool zoom;
    bool camera_state;
    bool track_state;
    }  query_flags;


    uint8_t pip_state;
    uint8_t color_state;
	bool is_recording;
	bool state_is_video;
	bool pip_color_hold;
	bool image_flip_toggle;


uint16_t _zoom_level;

bool _zooming_state_change;
bool _query_switch;

uint8_t _camera_speed_zoom_out;
uint8_t _camera_speed_zoom_in;


    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal

    uint8_t _reply_length;
    uint8_t _reply_counter;
    ReplyType _reply_type;


    union PACKED viewpro_reply {
        DEFINE_BYTE_ARRAY_METHODS
		ViewPro_reply_zoom_struct zoom_data;
        ViewPro_reply_angle_struct angle_data;
        ViewPro_reply_video_state video_state_data;
    } _buffer;

    // keep the last _current_angle values
    Vector3l _current_angle;
};
