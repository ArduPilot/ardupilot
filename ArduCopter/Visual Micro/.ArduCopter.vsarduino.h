/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Mega 2560 HAL (MPNG), Platform=avr, Package=apm
*/

#define __AVR_ATmega2560__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define CONFIG_HAL_BOARD HAL_BOARD_MPNG
#define EXCLUDECORE
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

//
static void compass_accumulate(void);
static void barometer_accumulate(void);
static void perf_update(void);
//
static void fast_loop();
static void throttle_loop();
static void update_mount();
static void update_batt_compass(void);
static void ten_hz_logging_loop();
static void fifty_hz_logging_loop();
static void three_hz_loop();
static void one_hz_loop();
static void update_optical_flow(void);
static void update_GPS(void);
bool set_yaw_mode(uint8_t new_yaw_mode);
void update_yaw_mode(void);
uint8_t get_wp_yaw_mode(bool rtl);
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode);
void exit_roll_pitch_mode(uint8_t old_roll_pitch_mode);
void update_roll_pitch_mode(void);
static void init_simple_bearing();
void update_simple_mode(void);
void update_super_simple_bearing(bool force_update);
bool throttle_mode_manual(uint8_t thr_mode);
bool set_throttle_mode( uint8_t new_throttle_mode );
void update_throttle_mode(void);
static void set_target_alt_for_reporting(float alt_cm);
static float get_target_alt_for_reporting();
static void read_AHRS(void);
static void update_trig(void);
static void update_altitude();
static void tuning();
void set_home_is_set(bool b);
void set_auto_armed(bool b);
void set_simple_mode(uint8_t b);
static void set_failsafe_radio(bool b);
void set_failsafe_battery(bool b);
static void set_failsafe_gps(bool b);
static void set_failsafe_gcs(bool b);
void set_takeoff_complete(bool b);
void set_land_complete(bool b);
void set_pre_arm_check(bool b);
void set_pre_arm_rc_check(bool b);
static void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in);
static void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out);
static void get_stabilize_roll(int32_t target_angle);
static void get_stabilize_pitch(int32_t target_angle);
static void get_stabilize_yaw(int32_t target_angle);
static void get_acro_level_rates();
static void get_roll_rate_stabilized_bf(int32_t stick_angle);
static void get_pitch_rate_stabilized_bf(int32_t stick_angle);
static void get_yaw_rate_stabilized_bf(int32_t stick_angle);
static void get_roll_rate_stabilized_ef(int32_t stick_angle);
static void get_pitch_rate_stabilized_ef(int32_t stick_angle);
static void get_yaw_rate_stabilized_ef(int32_t stick_angle);
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );
void update_rate_contoller_targets();
void run_rate_controllers();
static int16_t get_rate_roll(int32_t target_rate);
static int16_t get_rate_pitch(int32_t target_rate);
static int16_t get_rate_yaw(int32_t target_rate);
static int32_t get_of_roll(int32_t input_roll);
static int32_t get_of_pitch(int32_t input_pitch);
static void get_circle_yaw();
static void get_look_at_yaw();
static void get_look_ahead_yaw(int16_t pilot_yaw);
static void update_throttle_cruise(int16_t throttle);
static int16_t get_angle_boost(int16_t throttle);
static int16_t get_angle_boost(int16_t throttle);
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost );
void set_throttle_accel_target( int16_t desired_acceleration );
void throttle_accel_deactivate();
static void set_throttle_takeoff();
static int16_t get_throttle_accel(int16_t z_target_accel);
static int16_t get_pilot_desired_throttle(int16_t throttle_control);
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control);
static int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms);
static void get_throttle_rate(float z_target_speed);
static void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
static void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
static void get_throttle_rate_stabilized(int16_t target_rate);
static void get_throttle_land();
static void get_throttle_land();
static void reset_land_detector();
static bool update_land_detector();
static void get_throttle_surface_tracking(int16_t target_rate);
static void reset_I_all(void);
static void reset_rate_I();
static void reset_optflow_I(void);
static void reset_throttle_I(void);
static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle);
static void gcs_send_heartbeat(void);
static void gcs_send_deferred(void);
static NOINLINE void send_heartbeat(mavlink_channel_t chan);
static NOINLINE void send_attitude(mavlink_channel_t chan);
static NOINLINE void send_limits_status(mavlink_channel_t chan);
static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops);
static void NOINLINE send_location(mavlink_channel_t chan);
static void NOINLINE send_nav_controller_output(mavlink_channel_t chan);
static void NOINLINE send_ahrs(mavlink_channel_t chan);
static void NOINLINE send_simstate(mavlink_channel_t chan);
static void NOINLINE send_hwstatus(mavlink_channel_t chan);
static void NOINLINE send_gps_raw(mavlink_channel_t chan);
static void NOINLINE send_system_time(mavlink_channel_t chan);
static void NOINLINE send_servo_out(mavlink_channel_t chan);
static void NOINLINE send_radio_in(mavlink_channel_t chan);
static void NOINLINE send_radio_out(mavlink_channel_t chan);
static void NOINLINE send_vfr_hud(mavlink_channel_t chan);
static void NOINLINE send_raw_imu1(mavlink_channel_t chan);
static void NOINLINE send_raw_imu2(mavlink_channel_t chan);
static void NOINLINE send_raw_imu3(mavlink_channel_t chan);
static void NOINLINE send_current_waypoint(mavlink_channel_t chan);
static void NOINLINE send_statustext(mavlink_channel_t chan);
static bool telemetry_delayed(mavlink_channel_t chan);
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str);
static void mavlink_delay_cb();
static void gcs_send_message(enum ap_message id);
static void gcs_data_stream_send(void);
static void gcs_check_input(void);
static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
static bool print_log_menu(void);
static void do_erase_logs(void);
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp);
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds);
static void Log_Write_Current();
static void Log_Write_Optflow();
static void Log_Write_Nav_Tuning();
static void Log_Write_Control_Tuning();
static void Log_Write_Compass();
static void Log_Write_Performance();
static void Log_Write_Cmd(uint8_t num, const struct Location *wp);
static void Log_Write_Attitude();
static void Log_Write_Mode(uint8_t mode);
static void Log_Write_Startup();
static void Log_Write_Event(uint8_t id);
static void Log_Write_Data(uint8_t id, int16_t value);
static void Log_Write_Data(uint8_t id, uint16_t value);
static void Log_Write_Data(uint8_t id, int32_t value);
static void Log_Write_Data(uint8_t id, uint32_t value);
static void Log_Write_Data(uint8_t id, float value);
static void Log_Write_Camera();
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code);
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
static void start_logging();
static void Log_Write_Startup();
static void Log_Write_Cmd(uint8_t num, const struct Location *wp);
static void Log_Write_Mode(uint8_t mode);
static void Log_Write_IMU();
static void Log_Write_GPS();
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp);
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds);
static void Log_Write_Current();
static void Log_Write_Compass();
static void Log_Write_Attitude();
static void Log_Write_Data(uint8_t id, int16_t value);
static void Log_Write_Data(uint8_t id, uint16_t value);
static void Log_Write_Data(uint8_t id, int32_t value);
static void Log_Write_Data(uint8_t id, uint32_t value);
static void Log_Write_Data(uint8_t id, float value);
static void Log_Write_Event(uint8_t id);
static void Log_Write_Optflow();
static void Log_Write_Nav_Tuning();
static void Log_Write_Control_Tuning();
static void Log_Write_Performance();
static void Log_Write_Camera();
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code);
static void load_parameters(void);
void userhook_init();
void userhook_FastLoop();
void userhook_50Hz();
void userhook_MediumLoop();
void userhook_SlowLoop();
void userhook_SuperSlowLoop();
static void auto_tune_initialise();
static void auto_tune_intra_test_gains();
static void auto_tune_restore_orig_gains();
static void auto_tune_load_tuned_gains();
static void auto_tune_load_test_gains();
static bool auto_tune_start();
static void auto_tune_stop();
static void auto_tune_save_tuning_gains_and_reset();
void auto_tune_update_gcs(uint8_t message_id);
static void get_autotune_roll_pitch_controller(int16_t pilot_roll_angle, int16_t pilot_pitch_angle, int16_t pilot_yaw_command);
static void init_commands();
static struct Location get_cmd_with_index(int i);
static void set_cmd_with_index(struct Location temp, int i);
static int32_t get_RTL_alt();
static void init_home();
static void process_nav_command();
static void process_cond_command();
static void process_now_command();
static bool verify_nav_command();
static bool verify_cond_command();
static void do_RTL(void);
static void do_takeoff();
static void do_nav_wp();
static void do_land(const struct Location *cmd);
static void do_loiter_unlimited();
static void do_circle();
static void do_loiter_time();
static bool verify_takeoff();
static bool verify_land();
static bool verify_nav_wp();
static bool verify_loiter_unlimited();
static bool verify_loiter_time();
static bool verify_circle();
static bool verify_RTL();
static void do_wait_delay();
static void do_change_alt();
static void do_within_distance();
static void do_yaw();
static bool verify_wait_delay();
static bool verify_change_alt();
static bool verify_within_distance();
static bool verify_yaw();
static void do_guided(const struct Location *cmd);
static void do_change_speed();
static void do_jump();
static void do_set_home();
static void do_roi();
static void do_take_picture();
static void change_command(uint8_t cmd_index);
static void update_commands();
static void execute_nav_command(void);
static void verify_commands(void);
static int16_t find_next_nav_index(int16_t search_index);
static void exit_mission();
static void delay(uint32_t ms);
static void mavlink_delay(uint32_t ms);
static uint32_t millis();
static uint32_t micros();
static void pinMode(uint8_t pin, uint8_t output);
static void digitalWrite(uint8_t pin, uint8_t out);
static uint8_t digitalRead(uint8_t pin);
static void read_control_switch();
static uint8_t readSwitch(void);
static void reset_control_switch();
static uint8_t read_3pos_switch(int16_t radio_in);
static void read_aux_switches();
static void init_aux_switches();
static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
static void save_trim();
static void auto_trim();
void crash_check();
static void get_roll_pitch_drift();
static void get_yaw_drift();
static void failsafe_radio_on_event();
static void failsafe_radio_off_event();
static void failsafe_battery_event(void);
static void failsafe_gps_check();
static void failsafe_gps_off_event(void);
static void failsafe_gcs_check();
static void failsafe_gcs_off_event(void);
static void update_events();
void failsafe_enable();
void failsafe_disable();
void failsafe_check();
void fence_check();
static void fence_send_mavlink_status(mavlink_channel_t chan);
void init_flip();
void roll_flip();
static void heli_init();
static int16_t get_pilot_desired_collective(int16_t control_in);
static void check_dynamic_flight(void);
static void heli_integrated_swash_controller(int32_t target_roll_rate, int32_t target_pitch_rate);
static int16_t get_heli_rate_yaw(int32_t target_rate);
static void heli_update_landing_swash();
static void heli_update_rotor_speed_targets();
static void read_inertia();
static void read_inertial_altitude();
static void update_notify();
static void arm_motors_check();
static void auto_disarm_check();
static void init_arm_motors();
static void pre_arm_checks(bool display_failure);
static void pre_arm_rc_checks();
static bool pre_arm_gps_checks(bool display_failure);
static bool arm_checks(bool display_failure);
static void init_disarm_motors();
static void set_servos_4();
static void servo_write(uint8_t ch, uint16_t pwm);
static void run_nav_updates(void);
static void calc_position();
static void calc_distance_and_bearing();
static void run_autopilot();
static bool set_nav_mode(uint8_t new_nav_mode);
static void update_nav_mode();
static void reset_nav_params(void);
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec);
static void circle_set_center(const Vector3f current_position, float heading_in_radians);
static void update_circle();
void perf_info_reset();
void perf_info_check_loop_time(uint32_t time_in_micros);
uint16_t perf_info_get_num_loops();
uint32_t perf_info_get_max_time();
uint16_t perf_info_get_num_long_running();
Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt);
Vector3f pv_location_to_vector(Location loc);
int32_t pv_get_lat(const Vector3f pos_vec);
int32_t pv_get_lon(const Vector3f &pos_vec);
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);
static void default_dead_zones();
static void init_rc_in();
static void init_rc_out();
void output_min();
static void read_radio();
static void set_throttle_and_failsafe(uint16_t throttle_pwm);
void aux_servos_update_fn();
static void trim_radio();
static void init_sonar(void);
static void init_barometer(bool full_calibration);
static int32_t read_barometer(void);
static int16_t read_sonar(void);
static void init_compass();
static void init_optflow();
static void read_battery(void);
void read_receiver_rssi(void);
static void display_compassmot_info(Vector3f& motor_impact, Vector3f& motor_compensation);
static void report_batt_monitor();
static void report_sonar();
static void report_frame();
static void report_radio();
static void report_ins();
static void report_compass();
static void report_flight_modes();
void report_optflow();
static void print_radio_values();
static void print_switch(uint8_t p, uint8_t m, bool b);
static void print_done();
static void zero_eeprom(void);
static void print_accel_offsets_and_scaling(void);
static void print_gyro_offsets(void);
static void print_blanks(int16_t num);
static void print_divider(void);
static void print_enabled(bool b);
static void init_esc();
static void report_version();
static void report_tuning();
static void init_ardupilot();
static void startup_ground(bool force_gyro_cal);
static bool GPS_ok();
static bool mode_requires_GPS(uint8_t mode);
static bool manual_flight_mode(uint8_t mode);
static bool set_mode(uint8_t mode);
static void update_auto_armed();
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud);
static void check_usb_mux(void);
uint16_t board_voltage(void);
static void print_hit_enter();

#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\ArduCopter.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\APM_Config.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\APM_Config_mavlink_hil.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\AP_State.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\Attitude.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\GCS_Mavlink.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\Log.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\Parameters.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\Parameters.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\UserCode.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\UserVariables.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\auto_tune.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\commands.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\commands_logic.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\commands_process.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\compat.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\compat.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\config.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\config_channels.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\control_modes.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\crash_check.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\defines.h"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\drift.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\events.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\failsafe.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\fence.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\flip.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\heli.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\inertia.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\leds.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\motors.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\navigation.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\perf_info.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\position_vector.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\radio.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\sensors.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\setup.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\system.ino"
#include "C:\sketches\ardupilot-ArduCopter-3.1.5\ArduCopter\test.ino"
