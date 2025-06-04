/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger, Tom Pittenger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See https://ardupilot.org/dev for details

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

#include "Plane.h"

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Plane, &plane, func)


/*
  scheduler table - all regular tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table presnet in each of the
  vehicles to determine the order in which tasks are run.  Convenience
  methods SCHED_TASK and SCHED_TASK_CLASS are provided to build
  entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

FAST_TASK entries are run on every loop even if that means the loop
overruns its allotted time
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    FAST_TASK(ahrs_update),
    FAST_TASK(update_control_mode),
    FAST_TASK(stabilize),
    FAST_TASK(set_servos),
    SCHED_TASK(read_radio,             50,    100,   6),
    SCHED_TASK(check_short_failsafe,   50,    100,   9),
    SCHED_TASK(update_speed_height,    50,    200,  12),
    SCHED_TASK(update_throttle_hover, 100,     90,  24),
    SCHED_TASK_CLASS(RC_Channels,     (RC_Channels*)&plane.g2.rc_channels, read_mode_switch,           7,    100, 27),
    SCHED_TASK(update_GPS_50Hz,        50,    300,  30),
    SCHED_TASK(update_GPS_10Hz,        10,    400,  33),
    SCHED_TASK(navigate,               10,    150,  36),
    SCHED_TASK(update_compass,         10,    200,  39),
    SCHED_TASK(calc_airspeed_errors,   10,    100,  42),
    SCHED_TASK(update_alt,             10,    200,  45),
    SCHED_TASK(adjust_altitude_target, 10,    200,  48),
#if AP_ADVANCEDFAILSAFE_ENABLED
    SCHED_TASK(afs_fs_check,           10,    100,  51),
#endif
    SCHED_TASK(ekf_check,              10,     75,  54),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500,  57),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750,  60),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events, 50, 150,  63),
#endif
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read,   10, 300,  66),
    SCHED_TASK_CLASS(AP_Baro, &plane.barometer, accumulate,  50, 150,  69),
    SCHED_TASK(read_rangefinder,       50,    100, 78),
#if AP_ICENGINE_ENABLED
    SCHED_TASK_CLASS(AP_ICEngine,      &plane.g2.ice_control, update,     10, 100,  81),
#endif
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow, &plane.optflow, update,    50,    50,  87),
#endif
    SCHED_TASK(one_second_loop,         1,    400,  90),
    SCHED_TASK(three_hz_loop,           3,     75,  93),
    SCHED_TASK(check_long_failsafe,     3,    400,  96),
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,           &plane.rpm_sensor,     update,     10, 100,  99),
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    SCHED_TASK(airspeed_ratio_update,   1,    100,  102),
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100, 105),
#endif // HAL_MOUNT_ENABLED
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update,      50, 100, 108),
#endif // CAMERA == ENABLED
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100, 111),
#endif
    SCHED_TASK(compass_save,          0.1,    200, 114),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(Log_Write_FullRate,        400,    300, 117),
    SCHED_TASK(update_logging10,        10,    300, 120),
    SCHED_TASK(update_logging25,        25,    300, 123),
#endif
#if HAL_SOARING_ENABLED
    SCHED_TASK(update_soaring,         50,    400, 126),
#endif
    SCHED_TASK(parachute_check,        10,    200, 129),
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200, 132),
#endif // AP_TERRAIN_AVAILABLE
    SCHED_TASK(update_is_flying_5Hz,    5,    100, 135),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,         &plane.logger, periodic_tasks, 50, 400, 138),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins,    periodic,       50,  50, 141),
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update,  10,    100, 144),
#endif
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200, 147),
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button, &plane.button, update, 5, 100, 150),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats, &plane.g2.stats, update, 1, 100, 153),
#endif
#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper, &plane.g2.gripper, update, 10, 75, 156),
#endif
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landing_gear_update, 5, 50, 159),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(precland_update, 400, 50, 160),
#endif
};

void Plane::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

#if HAL_QUADPLANE_ENABLED
constexpr int8_t Plane::_failsafe_priorities[7];
#else
constexpr int8_t Plane::_failsafe_priorities[6];
#endif

// update AHRS system
void Plane::ahrs_update()
{
    arming.update_soft_armed();

    ahrs.update();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
#endif

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = aparm.roll_limit*100;
    pitch_limit_min = aparm.pitch_limit_min;

    bool rotate_limits = true;
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        rotate_limits = false;
    }
#endif
    if (rotate_limits) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min *= fabsf(ahrs.cos_roll());
    }

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

#if HAL_QUADPLANE_ENABLED
    // check if we have had a yaw reset from the EKF
    quadplane.check_yaw_reset();

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update();
#endif

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
#endif
}

//@yu sensor test param
int test_start_status = 0;
int sensor_init = 0;

/*
  update 50Hz speed/height controller
 */
void Plane::update_speed_height(void)
{
    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif
    if (should_run_tecs) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        TECS_controller.update_50hz();
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
#endif

    //@yu for sensor test
    /*
    if(AP::serialmanager().find_portnum(AP_SerialManager::SerialProtocol_None, 6)){
        if(sensor_init == 0){//開機後發送初始化命令
            //hal.serial(6)->write("init\n");
            sensor_init = 1;
        }
        if(arming.is_armed()){
            if(test_start_status == 0){
                hal.serial(6)->write("start\n");
                test_start_status = 1;
            }
        }else{
            if(test_start_status == 1){
                hal.serial(6)->write("end\n");
                test_start_status = 0;
            }
        }
    }*/

}


/*
  read and update compass
 */
void Plane::update_compass(void)
{
    compass.read();
}

//@yu get average airspeed
float average_asp = 0.0;//平均空速，10次平均
float asp_sum = 0.0;//加總暫存
int count_10hz = 0;//10hz計數器
//follow arspd controller output
float now_need_speed = 0;//當前目標空速
float the_speed_error = 0;//當前與最後目標空速的差
float the_last_speed_error = 0;//前一次的當前與最後目標空速差

//@yu ti params
//q_wp up speed control
float copy_wp_up_speed = 0.0;//vtol的wp上升速度複製
int get_wp_up = 0;
int do_param_check_cycle = 3;
float copy_old_qrtl_alt = 0.0;
int qrtl_alt_fresh_cycle = 3;
//auto switch mode when disarm
int ti_able_arm_overtime = 0;
//auto mode roll ff auto change
float copy_old_roll_ff = 0.0;
//timer counter
int _30s_count = 0;
int _10s_count = 0;
int _5s_count = 0;
int _3s_count = 0;
//gps failsafe
int gps_arm_status;
int ti_gps_fs_notice = 0;
int ti_gps_fs_counter = 0;
int is_in_auto;
int copy_qrtl_mode = 0;
int pass = 0;
//rtl change speed
float copy_airspeed = 0;//儲存原始空速設定
//follow airspeed control
float copy_cruise_airspeed = 0.0;//儲存原始空速設定
float dist_to_target = 0.0;//與目標的距離
float last_dist = 0;//前一次的距離
float last_dist_err = 0;//前一次的距離
int is_behind = 1;//1：在長機後
int being_leader = 0;
int wait_for_leader = 0;
int copy_target_sysid = 0;
int foll_target_notice = 0;
float dist_err_sum = 0;
bool target_status;
float this_head;
float err_yaw_to_target;
float bearing_to_target;
bool is_angle_over;
int wait_count = 0;
float dist_err = 0;
float final_speed = 0;
int target_alt_lost_count = 0;
int target_dist_lost_count = 0;
int have_target_ned = 0;
float target_speed_sum = 0;
int record_str = 0;
float need_speed = 0;
float old_need_speed = 0;
float last_need_speed = 0;
float new_need_speed = 0;
int is_stable = 0;
int cycle_copy = 0;
int now_cycle = 0;
int during_time = 0;
//bool need_do_foll_reset = false;
//status report
int link_report_count = 5;
int report_done = 0;
int cycle_during_time = 0;
int altitude_now = 0;
int altitude_last = 0;
int target_heartbeat_count = 0;
//auto change q_trans_decel
float copy_trans_decel = 0;
//rars function
uint8_t copy_old_targetid = 0;
int rars_guided_status = 0;//0:not use ,1:during rars guide ,2:escape rars guide ,3:error status
//ship takeoff
int pos_ready = 0;
int8_t pos_match_status = 0;//0:not work, 1:during, 2:finish or end
uint32_t speed_descend_time = 0;
float init_speed = 0;
float init_angle_rad = 0;
float speed_match_decel_time = 0;
float side_drift_angle_rad = 0;
uint32_t shift_start_time = 0;
uint32_t shift_end_time = 0;
float shift_target_speed = 0;
float now_shift_speed = 0;

#if HAL_LOGGING_ENABLED
/*
  do 10Hz logging
 */
void Plane::update_logging10(void)
{
    bool log_faster = (should_log(MASK_LOG_ATTITUDE_FULLRATE) || should_log(MASK_LOG_ATTITUDE_FAST));
    if (should_log(MASK_LOG_ATTITUDE_MED) && !log_faster) {
        Log_Write_Attitude();
        ahrs.Write_AOA_SSA();
    } else if (log_faster) {
        ahrs.Write_AOA_SSA();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif

    if(count_10hz == 10){//@yu get average airspeed
        average_asp = asp_sum/10;
        count_10hz = 0;
        asp_sum = 0;
    }else{
        asp_sum = asp_sum + airspeed.get_airspeed();
        count_10hz++;
    }

    if(!is_equal(new_need_speed,now_need_speed) && g.ti_group_throttle_type == 2 && g.ti_group_speed_update_cycle > 0){
        if(is_stable == 2){
            now_need_speed = new_need_speed;
            aparm.airspeed_cruise.set(now_need_speed);
        }else{//改用linear_interpolate？
            float limit_range = 0.05f;
            the_speed_error = new_need_speed - now_need_speed;
            //float speed_pid = the_speed_error*g.ti_group_spd_p + (the_speed_error - the_last_speed_error)*g.ti_group_spd_d + now_need_speed;
            float speed_pid = the_speed_error*0.1f + (the_speed_error - the_last_speed_error)*0.1f + now_need_speed;
            now_need_speed = constrain_value(speed_pid,now_need_speed - limit_range,limit_range + now_need_speed);
            //now_need_speed = (float)((int)(now_need_speed*100))/100;//取小數一位
            the_last_speed_error = the_speed_error;
            if(abs(new_need_speed - now_need_speed) < 0.015f){
                now_need_speed = new_need_speed;
            }
            aparm.airspeed_cruise.set(now_need_speed);
        }
        //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "speed = %.2f",now_need_speed);
    }

    if(g2.follow.have_heart_msg()){
        if(target_heartbeat_count < 100){
            target_heartbeat_count++;
        }
    }
    /*
    if(g2.follow.enabled() && g2.follow.rars_is_use() && (control_mode == &mode_rtl || control_mode == &mode_qrtl)){//rars目前基本上只會在rtl時工作
        if(arming.is_armed()){
            float rars_radius;
            if(is_zero(g2.follow.get_rars_work_radius())){
                if(g.rtl_radius == 0){
                    rars_radius = 150.0f;
                }else{
                    rars_radius = (float)g.rtl_radius;
                }
            }else{
                rars_radius = g2.follow.get_rars_work_radius();
            }
            if(auto_state.wp_distance < rars_radius){//當距home點150米以內觸發
                if(copy_old_targetid == 0){
                    copy_old_targetid = g2.follow.get_target_sysid();//紀錄舊目標id
                    g2.follow.set_target_sysid(g2.follow.get_rars_sysid());
                    rars_guided_status = 1;
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target ID Switch to %u(RARS)",g2.follow.get_target_sysid());
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RARS Guide Start!!!");
                }
            }
            if(rars_guided_status == 1 && !g2.follow.have_target()){
                if(g2.follow.get_rars_lost_act() == 1){//act = 1,繼續等待
                rars_guided_status = 2;
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RARS Guide Pause");
                }else{//act = 0,跳回原本的目標(較不精準)
                    rars_guided_status = 3;//
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Escape RARS Guide");
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Switch to origin Target");
                    g2.follow.set_target_sysid(copy_old_targetid);
                    copy_old_targetid = 0;
                }
            }else if(rars_guided_status == 2 && g2.follow.have_target()){//正常後回歸的通知
                rars_guided_status = 1;
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RARS Guide Continue");
            }
        }else{
            if(rars_guided_status > 0){//上鎖後結束導航
                rars_guided_status = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RARS Guide Finish");
                if(copy_old_targetid != 0){
                    g2.follow.set_target_sysid(copy_old_targetid);
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target ID Switch back to ID:%u",g2.follow.get_target_sysid());
                    copy_old_targetid = 0;
                }
            }
        }
    }*/

#if HAL_QUADPLANE_ENABLED
    if(g.ti_ship_vel_match){
        if(arming.is_armed()){
            if(control_mode == &mode_auto && pos_ready  > 1 && pos_match_status != 2){//mode = auto，且位置、速度可用
                if(plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_VTOL_TAKEOFF || plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF){//僅限vtol takeoff時
                    Location origin_next_WP;
                    if(get_target_location(origin_next_WP)){//紀錄初始目標
                        int32_t altitude_above_home; // 獲取高度
                        if(current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_above_home)){//對地高
                            altitude_above_home = (altitude_above_home + 50)/100;//四捨五入，轉公尺
                        }
                        if(g.ti_ship_vel_side_shift_position != 0 && side_shift_done == false && g.ti_ship_vel_side_shift_dist > 0.01){
                            if(altitude_above_home > g.ti_ship_vel_side_shift_min_alt){
                                if(is_zero(side_drift_angle_rad)){
                                    if(g.ti_ship_vel_side_shift_position > 0){//右偏
                                        //side_drift_angle_rad = radians((degrees(init_angle_rad)) + 90);//heading + 90 deg
                                        side_drift_angle_rad = init_angle_rad + radians(90);//heading + 90 deg
                                    }else{//左偏
                                        side_drift_angle_rad = init_angle_rad - radians(90);
                                    }
                                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Start side shift");
                                    shift_start_vel = takeoff_vel;//紀錄初始速度
                                    shift_start_time = AP_HAL::millis();//開始的時間
                                }
                                if(AP_HAL::millis() - shift_start_time > g.ti_ship_vel_side_shift_dist / ((float)g.ti_ship_vel_descend_per_second/2) * 1000.0f + 500.0f){//以距離/速度當作時間的依據
                                    if(shift_end_time == 0){
                                        shift_end_time = AP_HAL::millis();//結束的時間
                                    }
                                    shift_target_speed = linear_interpolate(now_shift_speed,0.0f,(float)AP_HAL::millis(),(float)shift_end_time,(float)(shift_end_time + 2000));//2秒內側向減速到0
                                    if(is_zero(shift_target_speed)){//歸零後才結束
                                        side_shift_done = true;
                                        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Side shift done");
                                    }
                                }else if(AP_HAL::millis() - shift_start_time < 2200){//2200應該可確保萬無一失
                                    float ship_vel_decel_gain;//根據初速調整側移速率
                                    if(init_speed > g.ti_ship_vel_match_low_speed){
                                        ship_vel_decel_gain = (float)g.ti_ship_vel_descend_per_second/2;
                                    }else{
                                        ship_vel_decel_gain = (float)g.ti_ship_vel_descend_per_second;//純側移時提昇加速度
                                    }
                                    shift_target_speed = linear_interpolate(0.0f,ship_vel_decel_gain,(float)AP_HAL::millis(),(float)shift_start_time,(float)(shift_start_time + 2000));//2秒內側向加速到g.ti_ship_vel_descend_per_second/2
                                    now_shift_speed = shift_target_speed;//動態調整減速起始值
                                }
                                takeoff_vel.x = shift_start_vel.x + cosf(side_drift_angle_rad)*shift_target_speed;
                                takeoff_vel.y = shift_start_vel.y + sinf(side_drift_angle_rad)*shift_target_speed;
                                //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Shift speed:%.2f", shift_target_speed);
                            }
                        }else{
                            side_shift_done = true;
                        }
                        if(g.ti_ship_vel_match_min_alt > -1 && side_shift_done == true){//超過某一高度後降低水平速度
                            if(altitude_above_home > g.ti_ship_vel_match_min_alt){
                                //if(init_speed > 0.1){//有初速才做變化，0.1應該能當誤差？
                                    if(speed_descend_time == 0){//紀錄起始時間
                                        speed_descend_time = AP_HAL::millis();
                                        if(g.ti_ship_vel_descend_per_second > 0){//有效值就做計算
                                            speed_match_decel_time = init_speed/g.ti_ship_vel_descend_per_second;//計算煞車時間
                                        }else{//無效值就給最小的
                                            speed_match_decel_time = init_speed/0.1;
                                        }
                                        base_takeoff_vel = takeoff_vel;
                                    }
                                    takeoff_vel = base_takeoff_vel * linear_interpolate(1.0f,0.0f,(float)AP_HAL::millis(),(float)speed_descend_time,(float)(speed_descend_time + (int)(speed_match_decel_time*1000)));//高於目標速度遞減
                                    //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "alt:%u ,vel:%.1f",altitude_above_home,safe_sqrt(sq(takeoff_vel.y) + sq(takeoff_vel.x)));/test message
                                //}
                            }
                        }
                        if(set_velocity_match(takeoff_vel.xy())){//設定初始速度
                            Location shipv_next_pos = takeoff_pos;//新增目標位置
                            shipv_next_pos.alt = origin_next_WP.alt;//將原本的目標高度設給新導航點
                            if(update_target_location(origin_next_WP,shipv_next_pos)){//檢查並更新導航點
                                if(pos_match_status == 0){//回報起始速度及方向
                                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Start match speed:%.1f m/s, angle:%.1f",init_speed,wrap_360(degrees(init_angle_rad)));
                                    pos_match_status = 1;//更新狀態
                                }
                            }
                        }
                    }
                }else{
                    //pos_ready = 0;
                    if(pos_match_status == 1){//脫離則宣告結束
                        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "END match speed");
                        pos_match_status = 2;
                        takeoff_vel.x = 0;
                        takeoff_vel.y = 0;
                        //speed_descend_time = 0;
                    }
                }
            }
            if(pos_match_status == 0){//沒進狀態大概是有啥bug
                if(pos_ready < 1){
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Match speed error, not ready");
                }else{
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Match speed only start in Auto mode Arm");
                }
                pos_match_status = 2;
                //speed_descend_time = 0;
            }
        }else{
            if(ahrs.get_velocity_NED(takeoff_vel) && ahrs.get_location(takeoff_pos)){//更新位置、速度
                takeoff_pos.change_alt_frame(Location::AltFrame::ABSOLUTE);
                pos_ready++;//健康度閥值
                pos_ready = MIN(pos_ready,10);
                init_speed = safe_sqrt(sq(takeoff_vel.y) + sq(takeoff_vel.x));
                init_angle_rad = atan2f(takeoff_vel.y, takeoff_vel.x);
                if(init_speed < g.ti_ship_vel_match_low_speed){
                    init_angle_rad = ahrs.get_yaw();//速度過低時方位角由ahrs給出，幾乎靜止時可用
                    takeoff_vel.x = 0;//太小直接給0
                    takeoff_vel.y = 0;
                }
                //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "heading:%.2f", degrees(init_angle_rad));
            }else{
                pos_ready = 0;
            }
            //disarm後初始化
            pos_match_status = 0;
            side_shift_done = false;
            shift_start_time = 0;
            shift_end_time = 0;
            side_drift_angle_rad = 0;
            speed_descend_time = 0;
            
        }
#if HAL_LOGGING_ENABLED
AP::logger().WriteStreaming("SVM","TimeUS,Match_VX,Match_VY,Spd,Hdg,Status","snnnd-","F?????","Iffffb",
                            AP_HAL::micros64(),
                            takeoff_vel.x,
                            takeoff_vel.y,
                            safe_sqrt(sq(takeoff_vel.y) + sq(takeoff_vel.x)),
                            wrap_360(degrees(atan2f(takeoff_vel.y, takeoff_vel.x))),
                            pos_match_status);
#endif //log end
    }
#endif //speed match end

}

/*
  do 25Hz logging
 */
void Plane::update_logging25(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE logs at 400Hz, MASK_LOG_ATTITUDE_FAST at 25Hz, MASK_LOG_ATTIUDE_MED logs at 10Hz
    // highest rate selected wins
    bool log_faster = should_log(MASK_LOG_ATTITUDE_FULLRATE);
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !log_faster) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_NOTCH_FULLRATE)) {
            AP::ins().write_notch_log_messages();
        }
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        Log_Write_Guided();
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        AP::ins().Write_Vibration();
}
#endif  // HAL_LOGGING_ENABLED

/*
  check for AFS failsafe check
 */
#if AP_ADVANCEDFAILSAFE_ENABLED
void Plane::afs_fs_check(void)
{
    afs.check(failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

void Plane::one_second_loop()
{
    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(&rcmap, g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

#if HAL_ADSB_ENABLED
    adsb.set_stall_speed_cm(aparm.airspeed_min * 100); // convert m/s to cm/s
    adsb.set_max_speed(aparm.airspeed_max);
#endif

    if (flight_option_enabled(FlightOptions::ENABLE_DEFAULT_AIRSPEED)) {
        // use average of min and max airspeed as default airspeed fusion with high variance
        ahrs.writeDefaultAirSpeed((float)((aparm.airspeed_min + aparm.airspeed_max)/2),
                                  (float)((aparm.airspeed_max - aparm.airspeed_min)/2));
    }

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE && HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // update home position if NOT armed and gps position has
    // changed. Update every 5s at most
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            landing.alt_offset = 0;
    }

    // this ensures G_Dt is correct, catching startup issues with constructors
    // calling the scheduler methods
    if (!is_equal(1.0f/scheduler.get_loop_rate_hz(), scheduler.get_loop_period_s()) ||
        !is_equal(G_Dt, scheduler.get_loop_period_s())) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    const float loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.attitude_control->set_notch_sample_rate(loop_rate);
    }
#endif
    rollController.set_notch_sample_rate(loop_rate);
    pitchController.set_notch_sample_rate(loop_rate);
    yawController.set_notch_sample_rate(loop_rate);
/*
    if(g2.follow.have_target()){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Target ID: %u ,Dist: %.2f m ,bearing angle: %.1f",g2.follow.get_target_sysid(),g2.follow.get_distance_to_target(),g2.follow.get_bearing_to_target());
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, " ");
    }
*/
    if(g2.follow.enabled()){
        if(g2.follow.have_target()){
            if(foll_target_notice == 0 && !being_leader){
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Connect Leader, MAV ID:%u",g2.follow.get_target_sysid());
                foll_target_notice = 1;
            }
        }else{
            if(foll_target_notice && !arming.is_armed()){
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Lost connect");
            }
            foll_target_notice = 0;
        }
    }

    if(g.ti_status_report != 0 && !report_done){
        char status_msg[100]; // 用來存儲狀態訊息
        char temp[50]; // 暫時字串

        snprintf(status_msg, sizeof(status_msg), "TI AHRS report ON: ");

        if (g.ti_status_report & (1 << 2)) {
            snprintf(temp, sizeof(temp), "Voltage, ");
            strncat(status_msg, temp, sizeof(status_msg) - strlen(status_msg) - 1);
        }
        if (g.ti_status_report& (1 << 1)) {
            snprintf(temp, sizeof(temp), "Airspeed, ");
            strncat(status_msg, temp, sizeof(status_msg) - strlen(status_msg) - 1);
        }
        if (g.ti_status_report & (1 << 0)) {
            snprintf(temp, sizeof(temp), "Altitude ");
            strncat(status_msg, temp, sizeof(status_msg) - strlen(status_msg) - 1);
        }

        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "%s",status_msg);
        report_done = 1;
    }
/*
    if(target_heartbeat_count > 0){//傳輸測試
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Have target heartbeat");
        target_heartbeat_count = 0;
    }*/

    if(arming.is_armed()){//-------------------------------------------Arming-------------------------------------------------------------------

        if(!is_zero(g.ti_arspd2up_min)){//風大時自動調整上升速率  ＃改下降速率會導致控制器異常，且變更下降速率有可能會剎不住，故不做任何調整
            if(quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) || quadplane.guided_takeoff){//檢測是否在auto takeoff
                if(!get_wp_up){
                    copy_wp_up_speed = quadplane.wp_nav->get_default_speed_up();//wp_nav位於AC庫，AP無法直接調用，所以自己加個函式
                    get_wp_up = 1;
                }
                if(average_asp > g.ti_max_safe_arspd){
                    float target_speed = get_target_speed(average_asp,copy_wp_up_speed,g.ti_arspd2up_min);//計算目標速度，原本計畫上下都會用到，故寫成函式
                    float now_up_speed = quadplane.wp_nav->get_default_speed_up();
                    if((target_speed - now_up_speed) > 5){//上下限
                        target_speed = now_up_speed + 5;
                        if(target_speed > copy_wp_up_speed){
                            target_speed = copy_wp_up_speed;
                        }
                    }else if((target_speed - now_up_speed) < 5){
                        target_speed = now_up_speed - 5;
                        if(target_speed < g.ti_arspd2up_min){
                            target_speed = g.ti_arspd2up_min;
                        }
                    }
                    quadplane.wp_nav->set_wp_speed_up(target_speed);
                }
            }else{
                if(get_wp_up == 1){
                    quadplane.wp_nav->set_wp_speed_up(copy_wp_up_speed);
                    get_wp_up = 0;
                }
            }
        }

        if(g.ti_auto_fix_alt > 0 && quadplane.rtl_mode != 0 && control_mode == &mode_rtl){//當rtl時，自動同步調整qrtl_alt
            if(is_zero(copy_old_qrtl_alt)){
                copy_old_qrtl_alt = quadplane.qrtl_alt;
                quadplane.qrtl_alt.set_and_notify((current_loc.alt - home.alt)/100 - 5);
            }/*else{
                if(_3s_count == qrtl_alt_fresh_cycle){
                    quadplane.qrtl_alt.set_and_notify((current_loc.alt - home.alt)/100 - 5);
                }
            }*/
        }

        if(g.ti_gps_fs > 0 && gps_arm_status == 1){//gps 故障保護
            
            if(gps.status() < 3){

                if(control_mode == &mode_qrtl || control_mode == &mode_rtl || control_mode == &mode_auto || control_mode == &mode_guided || control_mode == &mode_takeoff){//auto才動作
                    is_in_auto = 1;
                }else{
                    is_in_auto = 0;
                }

                if((g.ti_gps_fs_option & (1 << 0)) == is_in_auto || (g.ti_gps_fs_option & (1 << 0)) == 0){
                    pass = 1;
                }else{
                    pass = 0;
                }

                if(ti_gps_fs_counter == g.ti_gps_fs_overtime && pass){
                    if(g.ti_gps_fs == 2){
                        set_mode(mode_qhover,ModeReason::GPS_GLITCH);
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS Failsafe ON: QHOVER");
                    }else if(g.ti_gps_fs == 1){
                        if(g.ti_gps_fs_option & (1 << 1)){
                            if(quadplane.rtl_mode != 0 && copy_qrtl_mode == 0){
                                copy_qrtl_mode = quadplane.rtl_mode;
                            }
                            quadplane.rtl_mode.set(0);//禁止Qmode降落
                            set_mode(mode_rtl,ModeReason::GPS_GLITCH);
                        }else{
                            if(control_mode == &mode_qhover && control_mode == &mode_qstabilize && control_mode == &mode_qloiter && control_mode == &mode_qacro){
                                set_mode(mode_qrtl,ModeReason::GPS_GLITCH);
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS Failsafe ON: QRTL");
                            }else{
                                set_mode(mode_rtl,ModeReason::GPS_GLITCH);
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS Failsafe ON: RTL");
                            }
                        }
                    }
                }
                
                if(ti_gps_fs_counter <= g.ti_gps_fs_overtime){
                    ti_gps_fs_counter++;
                }

            }else{
                if(ti_gps_fs_counter  > 0 && pass){
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS Failsafe OFF");
                }
                ti_gps_fs_counter = 0;
                pass = 0;
            }
            
        }else if(g.ti_gps_fs > 0 && gps_arm_status == 0){}

        if((control_mode == &mode_rtl || control_mode == &mode_qrtl) && pass == 0 && g.ti_rtl_spd_dn != 0 && is_zero(copy_airspeed)){
            if(is_zero(copy_airspeed)){
                copy_airspeed = aparm.airspeed_cruise;
            }
            float diff_spd_comp = copy_airspeed - 21;
            if(diff_spd_comp < 0){
                diff_spd_comp = 0;
            }
            if(auto_state.wp_distance < 500 + diff_spd_comp*50){//依當前速度變更減速所需距離
                aparm.airspeed_cruise.set(aparm.airspeed_min + 2);
            }
        }
/**/
        int32_t altitude_for_foll; // 獲取高度
        int rep_alt_for_foll;
        bool alt_is_enough = true;
        if(current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_for_foll)){
            altitude_for_foll = (altitude_for_foll + 50)/100;//四捨五入
            rep_alt_for_foll = (int)altitude_for_foll;//飛機相對高，米
            if(g.ti_group_min_safe_altitude != 0){
                if(g.ti_group_min_safe_altitude > rep_alt_for_foll){//若啟用高度限制，超過才為真，使飛機在接近巡航高度後再啟用編隊，動動力不是那麼足的飛機會很有效
                    alt_is_enough = false;
                }
            }
        }
        
        if(g2.follow.enabled() && g.ti_group_enable && alt_is_enough){
            //if(g.sysid_this_mav > 1){//id=1,不啟用;=>不限定ID:1，但沒有變換目標 
                if(control_mode == &mode_auto && !quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) && plane.mission.get_current_nav_cmd().id != MAV_CMD_NAV_LOITER_UNLIM){

                    if(is_zero(copy_cruise_airspeed)){
                        copy_cruise_airspeed = aparm.airspeed_cruise;
                        if(is_zero(g.ti_group_spd_m)){
                            final_speed = copy_cruise_airspeed;
                        }else{
                            final_speed = g.ti_group_spd_m;
                        }
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Start follow Leader");
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "D = distance, E D = error distance");
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "T S = target speed, B = behind, C = cycle");
                        new_need_speed =  final_speed;
                        need_speed = new_need_speed;
                        old_need_speed = need_speed;
                        last_need_speed = new_need_speed;
                        now_need_speed = new_need_speed;
                    }else{
                        if(!is_zero(g.ti_group_spd_m)){
                            final_speed = g.ti_group_spd_m;
                        }else{
                            final_speed = copy_cruise_airspeed;
                        }
                    }

                    if(have_target_ned){
                        int different_alt = 0;
                        int32_t altitude_tar; // 獲取高度
                        if(current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, altitude_tar)){//海拔高,對應target gps alt
                            altitude_tar = (altitude_tar + 50)/100;//四捨五入
                        }
                        different_alt = abs(g2.follow.get_target_alt() - altitude_tar);
                        if(different_alt > g.ti_group_error_alt_max && g.ti_group_error_alt_max > 0){
                            target_alt_lost_count++;
                        }else{
                            target_alt_lost_count = 0;
                        }
                    }else{
                        target_alt_lost_count = 0;
                    }

                    if(dist_to_target > g.ti_group_target_lost_distance && !is_zero(g.ti_group_target_lost_distance)){
                        target_dist_lost_count++;
                    }else{
                        target_dist_lost_count = 0;
                    }

                    //if((g2.follow.have_heart_msg() && !g2.follow.is_target_in_auto()) || (g2.follow.have_gps_msg() && !g2.follow.is_target_gps_fix_good()) || !g2.follow.have_target() || target_alt_lost_count > 5 || target_dist_lost_count > 5){//有心跳或gps封包就加入作為判斷依據
                    //if((g2.follow.have_gps_msg() && !g2.follow.is_target_gps_fix_good()) || !g2.follow.have_target() || target_alt_lost_count > 5 || target_dist_lost_count > 5){
                    if(!g2.follow.have_target() || target_alt_lost_count > 5 || target_dist_lost_count > 5){
                        target_status = false;
                    }else{
                        target_status = true;
                    }
                    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "gps msg = %u, fix type = %u, fix good =%u",g2.follow.have_gps_msg(),g2.follow.get_target_gps_fix_type(),g2.follow.is_target_gps_fix_good());
                    if(!target_status && being_leader == 0){
                        if(wait_count == 0){//失連類別
                            if(!g2.follow.have_target()){
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Leader msg lost");
                            }else if(target_dist_lost_count > 5){
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Leader distance lost");
                            }else if(target_alt_lost_count > 5){
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Leader altitude lost");
                            }else if(g2.follow.have_gps_msg() && !g2.follow.is_target_gps_fix_good()){
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Leader's GPS Unhealthy");
                            }
                        }

                        if(g.ti_group_type == 0){//要馬跳開，要馬繼續等
                            being_leader = 1;
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Continue mission alone");
                        }else if(g.ti_group_type != 0){
                            wait_for_leader = 1;
                            if(wait_count == 0){
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Wait for Leader ...");
                            }
                            if(wait_for_leader && g.ti_group_wait_long_time > 0){
                                wait_count++;
                                if(wait_count > (int)g.ti_group_wait_long_time){
                                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Waiting overtime ..., continue mission alone");
                                    wait_for_leader = 0;
                                    wait_count = 0;
                                    being_leader = 1;
                                }
                                wait_count = constrain_value(wait_count,0,MIN(1000,g.ti_group_wait_long_time + 10));
                            }else{
                                wait_count = 1;
                            }
                        }/*else if(g.ti_group_type == 2){
                            int new_id = g2.follow.get_target_sysid() + 1;
                            if(!wait_for_leader){
                                if(new_id < g.sysid_this_mav){//沒有就順延到下一個
                                    if(!copy_target_sysid){
                                        copy_target_sysid = g2.follow.get_target_sysid();
                                    }
                                    g2.follow.set_target_sysid(new_id);
                                    wait_for_leader = 1;
                                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Search next Leader, MAV ID:%u...",g2.follow.get_target_sysid());
                                }else if(new_id == g.sysid_this_mav){
                                    being_leader = 1;
                                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Become Leader, leave follow");
                                }
                            }
                            if(wait_for_leader){
                                wait_count++;
                                if(wait_count > 10){
                                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MAV ID:%u no respond",g2.follow.get_target_sysid());
                                    wait_for_leader = 0;
                                    wait_count = 0;
                                }
                            }
                        }*/
                    }else{
                        if(wait_for_leader == 1){
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Leader's Signal back");
                            wait_for_leader = 0;
                        }
                        wait_count = 0;
                    }
                    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "now target = %u ,wait  = %u, count  = %u",g2.follow.get_target_sysid(),wait_for_leader,wait_count);

                    Vector3f dist_vec;  // vector to lead vehicle
                    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
                    Vector3f vel_of_target;  // velocity of lead vehicle 用這個求出行徑方向
                    Vector3f this_vel;

                    float ti_target_airspeed = 0;
                    //int follow_source = 0;//確認誤差來源

                    if(g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target) || gps.status() > 2){//呼叫函數取得與目標間的距離
                        dist_to_target = g2.follow.get_distance_to_target();
                    
                        float target_head = 0;
                        float target_head_msg = 0;

                        if(safe_sqrt(sq(vel_of_target.y) + sq(vel_of_target.x)) > 3){//目標靜止時使用頭向，地速超過3時使用行徑方向
                            target_head = wrap_360(degrees(atan2f(vel_of_target.y, vel_of_target.x)));//計算航向，角度(+-180) -> 0~360
                        }else{
                            if (g2.follow.get_target_heading_deg(target_head_msg)) {
                                target_head = target_head_msg;
                            }
                        }

                        if(ahrs.get_velocity_NED(this_vel)){//靜止時使用頭向，速度超過q_assist時使用行徑方向
                            if(safe_sqrt(sq(this_vel.y) + sq(this_vel.x)) > 3 || quadplane.assist_speed < average_asp){
                                this_head = wrap_360(degrees(atan2f(this_vel.y, this_vel.x)));//本機航向，角度(+-180) -> 0~360
                            }else{
                                this_head = wrap_360(degrees(ahrs.get_yaw()));//本機頭向，角度(+-180) -> 0~360
                            }
                        }

                        err_yaw_to_target = abs(wrap_180(target_head - this_head));//路徑點夾角，度，絕對值,0~180
                        bearing_to_target = wrap_360(g2.follow.get_bearing_to_target());//朝向目標的方向(+-180) -> 0~360
                        float err_bearing_to_target = abs(wrap_180(bearing_to_target - this_head));//航向與目標方向的夾角,0~180

    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target head = %.2f, this head = %.2f, bearing = %.2f, head err = %.2f,bear err = %.2f",target_head,this_head,bearing_to_target,err_yaw_to_target,err_bearing_to_target);
                        
                        float group_tri_dist = safe_sqrt(sq(g.ti_group_tri_h) + sq(g.ti_group_tri_v));//原本的三角隊形斜邊距離
                        float diff_dis2angle = abs(degrees(atan2f((float)g.ti_group_tri_v, (float)g.ti_group_tri_h)));//若角度小於某個值(30?)，直接用角度那套

                        if(!being_leader){//防止切換成leader之後還跳警告
                        /*
                            if(g2.follow.get_target_sysid() == 1 && diff_dis2angle > 30){//target id = 1 & xy夾角小於30度, 使用距離做控制

                                if(err_yaw_to_target < 120.0){//+-90度的彎可正常判斷；若大於90，
                                    if(is_angle_over == 1){
                                        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "angle back");
                                        is_angle_over = 0;
                                    }
                                }else{
                                    if(is_angle_over == 0){
                                        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "over angle");
                                        is_angle_over = 1;
                                    }
                                }

                                is_behind = 1;
                                if(err_bearing_to_target < 91){//長機應該都會在前方+-90deg內
                                    dist_err = dist_to_target - group_tri_dist;//實際誤差
                                    follow_source = 0;
                                }else{
                                    is_behind = 0;
                                    dist_err = -1 * (group_tri_dist -(float)g.ti_group_tri_v) + 2.0f*(90.0f - err_bearing_to_target);//實際誤差，反方向為負值
                                    follow_source = 1;
                                }

            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"base dist = %.2f, yaw gain = %.2f, behind = %u",base_target_dist,cosf(radians(err_yaw_to_target)),is_behind);

                            }else{//純使用角度做判斷  
                                if(err_yaw_to_target < 120.0){//+-90度的彎可正常判斷
                                    if(g2.follow.get_target_sysid() != 1){
                                        dist_err = 2.0f*(90.0f - err_bearing_to_target);//實際誤差(與平行的夾角)
                                    }else{
                                        dist_err = 2.0f*((90.0f - diff_dis2angle) - err_bearing_to_target);//實際誤差(與平行的夾角)
                                    }
                                    if(is_angle_over == 1){
                                        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "angle back");
                                        is_angle_over = 0;
                                    }
                                }else{
                                    if(is_angle_over == 0){
                                        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "over angle");
                                        is_angle_over = 1;
                                    }
                                }
                                follow_source = 1;
                            }*/
                            is_behind = 1;
                            if(diff_dis2angle > 30){//以到長機與水平的夾角，以30度為基準切換運算邏輯
                                if(err_bearing_to_target < 90){//長機在前方+-90deg內，且距離大於斜邊，使用距離計算
                                    dist_err = dist_to_target - group_tri_dist;//實際誤差
                                    //follow_source = 0;
                                }else{
                                    dist_err = -1 * (group_tri_dist -(float)g.ti_group_tri_v) + 2.0f*(90.0f - err_bearing_to_target);//超前後改以角度差做運算
                                    is_behind = 0;
                                    //follow_source = 1;
                                }
                            }else{
                                float angle_revise = 0;
                                if((abs(90.0f - diff_dis2angle) - err_bearing_to_target) < 30){//依距離調整線性變化度
                                    angle_revise = 0.5;
                                }else if((abs(90.0f - diff_dis2angle) - err_bearing_to_target) < 60){
                                    angle_revise = 1;
                                }else{
                                    angle_revise = 2;
                                }
                                dist_err = angle_revise*((90.0f - diff_dis2angle) - err_bearing_to_target);//實際誤差(與平行的夾角)
                                if(diff_dis2angle > 2){
                                    if(err_bearing_to_target < 91){
                                        is_behind = 0;
                                    }
                                }
                                //follow_source = 1;
                            }
                        }
                        have_target_ned = 1;
                    }else{//這下面好像不需要？
                        have_target_ned = 0;
                        dist_err -= (average_asp - final_speed);
                        if(_5s_count == 5){
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No Leader NED INFO");
                        }
                    }

                    float arspd_max = aparm.airspeed_max;//目標空速上下限
                    float arspd_min = aparm.airspeed_min;
                    //if((dist_err * last_dist_err) < 0 && dist_err > 0){//當減速過需求距離的零點時，直接重置積分項，減少遲滯
                    //    dist_err_sum = 0;
                    //}
                    
                    float xkpe = g.ti_group_spd_p * dist_err;
                    float xkdde = g.ti_group_spd_d * (dist_err - last_dist_err);
                    float xkiie = g.ti_group_spd_i * dist_err_sum;
                    
                    float K = g.ti_group_speed_rate;//減緩加速，加快減速
                    K = constrain_value(K,0.1f,0.9f);//增益比率的上下限
                    if(dist_err < 0){
                        K = 1.0f - K;
                    }
                    K = K * g.ti_group_spd_gain;//總增益

                    ti_target_airspeed = K * (xkpe + xkdde + xkiie) + final_speed;//基於位置誤差的速度pid控制器
                    //ti_target_airspeed = constrain_value(ti_target_airspeed,arspd_min,arspd_max);//範圍限制

                    if(is_equal(ti_target_airspeed,arspd_max) || is_equal(ti_target_airspeed,arspd_min)){//控制積分，飽和後緩降
                        dist_err_sum = dist_err_sum * 0.7;
                        ti_target_airspeed = K * (xkpe + xkdde + (g.ti_group_spd_i * dist_err_sum)) + final_speed;//飽和才重新計算
                    }else{
                        dist_err_sum += dist_err;
                    }

                    last_dist_err = dist_err;

                    if(wait_for_leader || being_leader || is_angle_over){//等待時使用預設速度
                        ti_target_airspeed = final_speed;
                    }else{
                        //if(_30s_count && g.ti_status_report == 0){
                        //    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "MAV is now following");
                        //}
                    }
                    
                    ti_target_airspeed = constrain_value(ti_target_airspeed,arspd_min,arspd_max);//範圍限制
                    if(g.ti_group_speed_range_limit > -1){
                        float speed_rng = (float)g.ti_group_speed_range_limit;
                        ti_target_airspeed = constrain_value(ti_target_airspeed, final_speed - speed_rng, final_speed + speed_rng);//調速範圍限制
                    }
                    //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "target asp = %f,wfl = %u,bl = %u,iao =  %u",ti_target_airspeed,wait_for_leader,being_leader,is_angle_over);
                    if(g.ti_group_speed_update_cycle > 0){//大於0才做限制

                        float speed_new_limit = 3.0f;
                        if(g.ti_group_speed_update_cycle > 1){//隨週期改變上下限
                            speed_new_limit = constrain_value((float)g.ti_group_speed_update_cycle, 2.0f, 5.0f);
                        }
                        ti_target_airspeed = constrain_value(ti_target_airspeed,(new_need_speed - speed_new_limit),(new_need_speed + speed_new_limit));//單次變化限制

                        target_speed_sum += ti_target_airspeed;
                        record_str++;
                        if(is_stable == 1){//變更更新頻率回原本的
                            g.ti_group_speed_update_cycle.set(record_str);
                            is_stable--;
                        }else if(is_stable == 0 && cycle_copy != 0){
                            g.ti_group_speed_update_cycle.set(cycle_copy);
                            cycle_copy = 0;
                        }

                        if(record_str > g.ti_group_speed_update_cycle - 1){//減少更新的頻率/平滑更新量，減少油門跳變
                            last_need_speed = new_need_speed;
                            new_need_speed = target_speed_sum/g.ti_group_speed_update_cycle;
                            new_need_speed = (float)((int)(new_need_speed*10))/10;//取小數一位
                            need_speed = new_need_speed;
                            target_speed_sum = 0;
                            record_str = 0;
                            if(g.ti_group_throttle_type == 0){//更新後才調整
                                need_speed = constrain_value(need_speed,arspd_min,arspd_max);//最後的保險
                                aparm.airspeed_cruise.set(need_speed);
                            }
                        }
                        if(g.ti_group_throttle_type == 1){//依據上次更新值，以cycle次數做分量平滑輸出
                            need_speed = (new_need_speed - last_need_speed)/g.ti_group_speed_update_cycle*record_str + last_need_speed;
                            need_speed = (float)((int)(need_speed*10))/10;
                            need_speed = constrain_value(need_speed,arspd_min,arspd_max);//最後的保險
                            aparm.airspeed_cruise.set(need_speed);
                        }

                        if(abs(ti_target_airspeed - last_need_speed) < 0.3f){
                            during_time++;
                            during_time = MIN(g.ti_group_speed_update_cycle * 20,during_time);
                        }else{
                            during_time = 0;
                        }
                        if(during_time > g.ti_group_speed_update_cycle * 10 && cycle_copy == 0){
                            cycle_copy = g.ti_group_speed_update_cycle;
                            g.ti_group_speed_update_cycle.set(cycle_copy * 5);
                            is_stable = 2;
                        }else{                            
                            if(is_stable == 2 && during_time == 0){
                                is_stable--;
                            }
                        }
                    }else{
                        last_need_speed = new_need_speed;
                        new_need_speed = (float)((int)(ti_target_airspeed*10))/10;
                        need_speed = new_need_speed;
                        need_speed = constrain_value(need_speed,arspd_min,arspd_max);//最後的保險
                        aparm.airspeed_cruise.set(need_speed);
                    }

                    if(g.ti_group_status_report && !being_leader){
                        //int show_cycle = g.ti_group_speed_update_cycle;
                        //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "cycle = %u",show_cycle);
                        int c_report = g.ti_group_speed_update_cycle;
                        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "D = %.1f, E D = %.1f, T S = %.1f, B = %u, C = %u",dist_to_target,dist_err,need_speed,is_behind,c_report);
                        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "  ");
                    }
                    
                }else{
                    if(cycle_copy > 0){
                        g.ti_group_speed_update_cycle.set(cycle_copy);
                        cycle_copy = 0;
                    }
                    record_str = 0;
                    is_stable = 0;
                    target_speed_sum = 0;
                    dist_err_sum = 0;
                    last_dist_err = 0;
                }
            //}
        }else{
            if(!is_zero(copy_cruise_airspeed)){//途中關閉復原空速
                aparm.airspeed_cruise.set_and_notify(copy_cruise_airspeed);
                copy_cruise_airspeed = 0;
            }
            if(cycle_copy > 0){//之後可能會將狀態重置單獨設為另外的函數
                g.ti_group_speed_update_cycle.set(cycle_copy);
                cycle_copy = 0;
            }
            record_str = 0;
            is_stable = 0;
            target_speed_sum = 0;
            dist_err_sum = 0;
            last_dist_err = 0;
        }
        
        if(g.ti_status_report != 0 && plane.is_flying()){
            int batt_index = g.ti_status_report_batt_index - 1;
            float volt = battery.voltage(batt_index);
            int32_t altitude; // 獲取高度
            int rep_alt;
            if(current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude)){
                altitude = (altitude + 50)/100;//四捨五入
            }else{
                altitude = (int)barometer.get_altitude(); // 獲取高度
            }
            rep_alt = (int)altitude;
            bool during_vtol = false;//判斷是否垂起
            bool in_vtol_auto = false;
            if((quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) || quadplane.guided_takeoff) || ((quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION2) && control_mode == &mode_qrtl) || control_mode == &mode_qstabilize || control_mode == &mode_qhover || control_mode == &mode_qloiter){
                during_vtol = true;
                if((quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) || quadplane.guided_takeoff) ||((quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION2) && control_mode == &mode_qrtl)){
                    in_vtol_auto = true;
                }
            }

            char status_msg[100]; // 用來存儲狀態訊息
            char temp[50]; // 暫時字串
            snprintf(status_msg, sizeof(status_msg), " "); // 初始化字串
            
            if(!in_vtol_auto){
                if (g.ti_status_report & (1 << 0)) {
                    snprintf(temp, sizeof(temp), "Altitude: %i ", rep_alt);
                    strncat(status_msg, temp, sizeof(status_msg) - strlen(status_msg) - 1);
                }
                if (g.ti_status_report & (1 << 1) && !during_vtol) {
                    snprintf(temp, sizeof(temp), "Airspeed: %i ", (int)average_asp);
                    strncat(status_msg, temp, sizeof(status_msg) - strlen(status_msg) - 1);
                }
                if (g.ti_status_report & (1 << 2)) {
                    snprintf(temp, sizeof(temp), "Voltage: %.1f ", volt);
                    strncat(status_msg, temp, sizeof(status_msg) - strlen(status_msg) - 1);
                }
            }

            if(during_vtol){
                if(in_vtol_auto){//auto takeoff 第一次報"高度"+數值 之後只報數值
                    altitude_now = (rep_alt + 1)/5;
                    if(altitude_now != altitude_last){
                        if(altitude_last == 0){
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Altitude: %i ", rep_alt);
                        }else{
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%i",rep_alt);
                        }
                        altitude_last = altitude_now;
                    }
                }else{
                    if(cycle_during_time == 0 || cycle_during_time > g.ti_status_report_cycle/2){
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s",status_msg);
                        cycle_during_time = 0;
                    }
                }
            }else{
                if(cycle_during_time == 0 || cycle_during_time > g.ti_status_report_cycle){
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s",status_msg);
                    cycle_during_time = 0;
                }
                altitude_last = 0;
                altitude_now = 0;
            }
            cycle_during_time++;
        }

    }else{//-------------------------------------------------------------DisArming-----------------------------------------------------

        if(do_param_check_cycle == _3s_count){//每3秒檢查一次
            if(quadplane.wp_nav->get_default_speed_up() < g.ti_arspd2up_min){
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TI_AS2QUP_MIN(cm) too high");
                g.ti_arspd2up_min.set_and_notify(quadplane.wp_nav->get_default_speed_up() - 1);//參數值太大就自動變更為max-1
            }else if(g.ti_arspd2up_min < 50 && !is_zero(g.ti_arspd2up_min)){
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TI_AS2QUP_MIN(cm) too low(<50) ,please retry");
                g.ti_arspd2up_min.set_and_notify(0);
            }
        }

        if(control_mode == &mode_rtl || control_mode == &mode_qrtl || control_mode == &mode_qland){//exit rtl and qrtl when disarm
            if(ti_able_arm_overtime > 10){
                set_mode(g.initial_mode,ModeReason::AUTO_RTL_EXIT);
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Change to INITIAL MODE to be able Arm");
                ti_able_arm_overtime = 0;
            }
            ti_able_arm_overtime++;
        }

        if(gps.status() < 3){
            gps_arm_status = 0;
        }else{
            gps_arm_status = 1;
        }

    }

    //當處在auto模式下，且q_guided enable以及任務項目為可停懸的loiter時，變更q_trans_decel，以防煞車反應不如預期  P.S.這大概率跟空速縮放值有關，建議先調整那個
    if(control_mode == &mode_auto && !is_zero(g.ti_q_transition_decel_in_mission) && plane.quadplane.guided_mode_enabled() && ((plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_UNLIM) || (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_TIME) || (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_TO_ALT))){
        if(arming.is_armed()){
            if(is_zero(copy_trans_decel) || !is_equal((float)plane.quadplane.transition_decel,(float)g.ti_q_transition_decel_in_mission)){
                copy_trans_decel = plane.quadplane.transition_decel;
                float new_trans_decel = g.ti_q_transition_decel_in_mission;
                if(new_trans_decel > 5){//參數檢查
                    new_trans_decel = 5;
                    g.ti_q_transition_decel_in_mission.set_and_notify(new_trans_decel);
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "TI_WP_Q_T_DECEL out of range");
                }else if(new_trans_decel < 0.2){
                    new_trans_decel = 0.2;
                    g.ti_q_transition_decel_in_mission.set_and_notify(new_trans_decel);
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "TI_WP_Q_T_DECEL out of range");
                }
                plane.quadplane.transition_decel.set_and_notify(new_trans_decel);
            }
        }
    }else{
        if(!is_zero(copy_trans_decel)){//有改過就復原
            plane.quadplane.transition_decel.set_and_notify(copy_trans_decel);
            copy_trans_decel = 0;
        }
    }

    if(g2.follow.enabled()){
        /*if(arming.is_armed()){//暫時先全開
            if(link_report_count == 0){
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target MAV ID:%u, This MAV ID:%u",g2.follow.get_target_sysid(),(int)g.sysid_this_mav);
                link_report_count = 5;
            }
        }else{*/
            if(link_report_count == 0){
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target MAV ID:%u, This MAV ID:%u",g2.follow.get_target_sysid(),(int)g.sysid_this_mav);
            }
            if(link_report_count < 5){
                if(g2.follow.have_target()){
                    Vector3f dist_vec1;  // vector to lead vehicle
                    Vector3f dist_vec_offs1;  // vector to lead vehicle + offset
                    Vector3f vel_of_target1;  // velocity of lead vehicle 用這個求出行徑方向

                    if(g2.follow.get_target_dist_and_vel_ned(dist_vec1, dist_vec_offs1, vel_of_target1)){//呼叫函數取得與目標間的距離
                        float dist_to_target1 = g2.follow.get_distance_to_target();//與目標的距離
                        float target_head1 = wrap_360(degrees(atan2f(vel_of_target1.y, vel_of_target1.x)));//計算航向，角度(+-180) -> 0~360
                        float bearing_to_target1 = wrap_360(g2.follow.get_bearing_to_target());//朝向目標的方向(+-180) -> 0~360
                        
                        if(g2.follow.have_gps_msg()){
                            if(target_heartbeat_count > 0){
                                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target dist:%.2f, head:%.2f, bear angle:%.2f,gps fix type %u, have heartbeat",dist_to_target1,target_head1,bearing_to_target1,g2.follow.get_target_gps_fix_type());
                            }else{
                                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target dist:%.2f, head:%.2f, bear angle:%.2f,gps fix type %u, no heartbeat",dist_to_target1,target_head1,bearing_to_target1,g2.follow.get_target_gps_fix_type());
                            }
                        }else{
                            if(target_heartbeat_count > 0){
                                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target dist:%.2f, head:%.2f, bear angle:%.2f, have heartbeat",dist_to_target1,target_head1,bearing_to_target1);
                            }else{
                                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Target dist:%.2f, head:%.2f, bear angle:%.2f, no heartbeat",dist_to_target1,target_head1,bearing_to_target1);
                            }
                        }   
                    }
                }else if(!g2.follow.have_target()){
                    if(target_heartbeat_count > 0){
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No NED MSG, have heartbeat");
                    }else{
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No NED MSG, no heartbeat");
                    }
                }
                link_report_count++;
                target_heartbeat_count = 0;//重置讀數
            }
        //}
        if(g.ti_group_target_link_test == 1){
            link_report_count = 0;
            g.ti_group_target_link_test.set_and_notify(0);
        }else if(g.ti_group_target_link_test == 2){
            link_report_count = 1;
        }
    }

    if(ti_gps_fs_notice == 0 && g.ti_gps_fs > 0){
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "TI GPS FS will only effect on VTOL, please check your plane type");
        ti_gps_fs_notice = 1;
    }

    if(control_mode == &mode_rtl || control_mode == &mode_auto || control_mode == &mode_guided){//auto 時更改滾角速率增益
        if(is_zero(copy_old_roll_ff)){
            copy_old_roll_ff = rollController.get_origin_ff();//rollController調用AC庫，AP無法直接呼叫roll_rate_ff，所以自己加個函式
        }
        float roll_ff = g.ti_wp_roll_rate_ff;
        if(!is_zero(g.ti_wp_roll_rate_ff) && !is_equal(rollController.get_origin_ff(),roll_ff)){
            rollController.do_change_ff(g.ti_wp_roll_rate_ff);//跟上面那個同理
        }
    }else{
        if(!is_zero(g.ti_wp_roll_rate_ff) && !is_zero(copy_old_roll_ff)){
            rollController.do_change_ff(copy_old_roll_ff);
            copy_old_roll_ff = 0.0;
        }
    }

    if(!arming.is_armed() || control_mode != &mode_auto){
        if(copy_target_sysid){
            g2.follow.set_target_sysid(copy_target_sysid);
            copy_target_sysid = 0;
        }
        if(being_leader || wait_for_leader){
            being_leader = 0;
            wait_for_leader = 0;
        }
    }
    
    if(((control_mode != &mode_rtl && control_mode != &mode_qrtl) || !arming.is_armed()) && !is_zero(copy_old_qrtl_alt) && g.ti_auto_fix_alt >= 0){//脫離rtl就恢復原本的數值
        if(copy_old_qrtl_alt > quadplane.land_final_alt){
            quadplane.qrtl_alt.set_and_notify(copy_old_qrtl_alt);
        }else{
            quadplane.qrtl_alt.set_and_notify(quadplane.land_final_alt + 10);
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Should check QRTL ALT");
        }
        copy_old_qrtl_alt = 0;
    }

    if((!arming.is_armed() || (control_mode != &mode_rtl && control_mode != &mode_qrtl) || (g.ti_gps_fs_option & (1 << 1)) == 0) && copy_qrtl_mode != 0){//gps fs 復原用
        quadplane.rtl_mode.set_and_notify(copy_qrtl_mode); 
    }

    if((!arming.is_armed() || (control_mode != &mode_rtl && control_mode != &mode_qrtl)) && !is_zero(copy_airspeed)){//rtl 變速復原用
        aparm.airspeed_cruise.set(copy_airspeed);
        copy_airspeed = 0;
    }

    if(control_mode != &mode_auto && !is_zero(copy_cruise_airspeed)){//follow 變速復原用
        aparm.airspeed_cruise.set(copy_cruise_airspeed);
        copy_cruise_airspeed = 0;
    }

    if(_10s_count == 10){//10s counter
        _10s_count = 1;
    }else{_10s_count++;}

    _5s_count = _10s_count%5 + 1;//5s counter

    if(_3s_count == 3){//3s counter
        _3s_count = 1;
    }else{_3s_count++;}

    if(_10s_count == 10 && _3s_count == 3){
        _30s_count = 1;
    }else{_30s_count = 0;}
}

float Plane::get_target_speed(float asp,float origin_up_speed,float min_up_speed)
{
    float arspd_diff_rate = 0;
    arspd_diff_rate = (asp - g.ti_max_safe_arspd)/(quadplane.assist_speed - g.ti_max_safe_arspd);
    if(arspd_diff_rate > 1){
        arspd_diff_rate = 1;
    }
    float cal_rate = cosf(3.14/2*arspd_diff_rate);//90/180=2
    float target_speed = (origin_up_speed - min_up_speed)*cal_rate + min_up_speed;

    return target_speed;
}

int voyage_acc_status = 0;//紀錄狀態,0:無/結束,1:進行中,2:暫停
float total_accumulate_voyage = 0;//總里程
uint32_t voyage_time_in_auto_air = 0;//總時數

void Plane::three_hz_loop()
{
#if AP_FENCE_ENABLED
    fence_check();
#endif
//@yu calculate total voyage in auto
    if(g.ti_calculate_voyage_in_auto){
        if(arming.is_armed()){//arm且處於auto FW(含rtl)時才計算里程
            if((control_mode == &mode_auto && (!quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id) && plane.mission.get_current_nav_cmd().id != MAV_CMD_NAV_VTOL_LAND)) 
            || control_mode == &mode_rtl || ((quadplane.poscontrol.get_state() < QuadPlane::QPOS_POSITION2) && control_mode == &mode_qrtl)){
                if(voyage_acc_status == 0){
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Start Log Voyage");
                }else if(voyage_acc_status == 2){
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Continue Log Voyage");
                }
                if(time_last_update_in_auto_air != 0){//若有初始值則開始紀錄
                    voyage_time_in_auto_air += AP_HAL::millis() - time_last_update_in_auto_air;
                }
                //if(voyage_acc_status != 1){//頭一次進來才刷新
                time_last_update_in_auto_air = AP_HAL::millis();
                //}
                voyage_acc_status = 1;//計算中
                Location voy_current_loc;//當前位置
                Vector3f dist_old2new;//三軸位移
                if (ahrs.get_location(voy_current_loc)){//取得當前位置
                    if(last_vehical_location.lat != 0 && last_vehical_location.lng != 0){//第一次更新過後才開始計算
                        dist_old2new = voy_current_loc.get_distance_NED(last_vehical_location);//計算三圍距離
                        total_accumulate_voyage += safe_sqrt(sq(dist_old2new.x) + sq(dist_old2new.y));//計算水平距離並加總
                    }
                    last_vehical_location = voy_current_loc;//更新最後的位置
                }
            }else{
                if(voyage_acc_status == 1){
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Pause Log Voyage");
                    last_vehical_location.lat = 0;//暫停時初始化位置，以防瞬移
                    last_vehical_location.lng = 0;
                    voyage_acc_status = 2;//暫停狀態
                    time_last_update_in_auto_air = 0;//計時器重置
                    //voyage_time_in_auto_air += AP_HAL::millis() - time_begin_in_auto_air;//途中結算
                }
            }
        }else{//disarm時結算
            if(voyage_acc_status != 0){
                //if(voyage_acc_status == 1){
                //    voyage_time_in_auto_air += AP_HAL::millis() - time_begin_in_auto_air;//最後結算
                //}
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Total Voyage : %.1f m, Time : %u s",total_accumulate_voyage,(int)voyage_time_in_auto_air/1000);//(int) for 6X
                last_vehical_location.lat = 0;//結束時初始化位置
                last_vehical_location.lng = 0;
                total_accumulate_voyage = 0;
                voyage_acc_status = 0;//結束狀態
                voyage_time_in_auto_air = 0;//歸零飛行時間
            }
        }
#if HAL_LOGGING_ENABLED
    float voyage_time_u2f = ((float)voyage_time_in_auto_air)/1000;
    AP::logger().WriteStreaming("VOYG","TimeUS,TimeFW,VOYAGE","ssm","F??","Iff",
                                AP_HAL::micros64(),
                                voyage_time_u2f,
                                total_accumulate_voyage);
#endif
    }
}

void Plane::compass_save()
{
    if (AP::compass().available() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
         */
        compass.save_offsets();
    }
}

#if AP_AIRSPEED_AUTOCAL_ENABLE
/*
  once a second update the airspeed calibration ratio
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max*100 ||
        ahrs.pitch_sensor < pitch_limit_min*100) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    update_current_loc();
}

/*
  read update GPS position - 10Hz update
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else if (!hal.util->was_watchdog_reset()) {
                if (!set_home_persistently(gps.location())) {
                    // silently ignore failure...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main control mode dependent update code
 */
void Plane::update_control_mode(void)
{
    if (control_mode != &mode_auto) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    update_fly_forward();

    control_mode->update();
}


void Plane::update_fly_forward(void)
{
    // ensure we are fly-forward when we are flying as a pure fixed
    // wing aircraft. This helps the EKF produce better state
    // estimates as it can make stronger assumptions
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() &&
        quadplane.tailsitter.is_in_fw_flight()) {
        ahrs.set_fly_forward(true);
        return;
    }

    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        ahrs.set_fly_forward(false);
        return;
    }
#endif

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
        return;
    }

    ahrs.set_fly_forward(true);
}

/*
  set the flight stage
 */
void Plane::set_flight_stage(AP_FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    landing.handle_flight_stage_change(fs == AP_FixedWing::FlightStage::LAND);

    if (fs == AP_FixedWing::FlightStage::ABORT_LANDING) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
#if HAL_LOGGING_ENABLED
    Log_Write_Status();
#endif
}

void Plane::update_alt()
{
    barometer.update();

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if PARACHUTE == ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif

    update_flight_stage();

#if AP_SCRIPTING_ENABLED
    if (nav_scripting_active()) {
        // don't call TECS while we are in a trick
        return;
    }
#endif

    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif
    
    if (should_run_tecs && !throttle_suppressed) {

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_FixedWing::FlightStage::LAND &&
            current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        tecs_target_alt_cm = relative_target_altitude_cm();

        if (control_mode == &mode_rtl && !rtl.done_climb && (g2.rtl_climb_min > 0 || (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)))) {
            // ensure we do the initial climb in RTL. We add an extra
            // 10m in the demanded height to push TECS to climb
            // quickly
            tecs_target_alt_cm = MAX(tecs_target_alt_cm, prev_WP_loc.alt - home.alt) + (g2.rtl_climb_min+10)*100;
        }

        TECS_controller.update_pitch_throttle(tecs_target_alt_cm,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor,
                                                 g.pitch_trim.get());
    }
}

/*
  recalculate the flight_stage
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (control_mode->does_auto_throttle() && !throttle_suppressed) {
        if (control_mode == &mode_auto) {
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
                return;
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Landing aborted via throttle");
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else {
                    set_flight_stage(AP_FixedWing::FlightStage::LAND);
                }
                return;
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        } else if (control_mode != &mode_takeoff) {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        }
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        set_flight_stage(AP_FixedWing::FlightStage::VTOL);
        return;
    }
#endif
    set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
}




/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires

    only called from AP_Landing, when the landing library is ready to disarm
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm(AP_Arming::Method::AUTOLANDED)) {
                gcs().send_text(MAV_SEVERITY_INFO,"Auto disarmed");
            }
        }
    }
}

bool Plane::trigger_land_abort(const float climb_to_alt_m)
{
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        return quadplane.abort_landing();
    }
#endif

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool is_in_landing = (plane.flight_stage == AP_FixedWing::FlightStage::LAND) ||
        plane.is_land_command(mission_id);
    if (is_in_landing) {
        // fly a user planned abort pattern if available
        if (plane.mission.jump_to_abort_landing_sequence()) {
            return true;
        }

        // only fly a fixed wing abort if we aren't doing quadplane stuff, or potentially
        // shooting a quadplane approach
#if HAL_QUADPLANE_ENABLED
        const bool attempt_go_around =
            (!plane.quadplane.available()) ||
            ((!plane.quadplane.in_vtol_auto()) &&
                (!plane.quadplane.landing_with_fixed_wing_spiral_approach()));
#else
        const bool attempt_go_around = true;
#endif
        if (attempt_go_around) {
            // Initiate an aborted landing. This will trigger a pitch-up and
            // climb-out to a safe altitude holding heading then one of the
            // following actions will occur, check for in this order:
            // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
            //      increment mission index to execute it
            // - else if DO_LAND_START is available, jump to it
            // - else decrement the mission index to repeat the landing approach

            if (!is_zero(climb_to_alt_m)) {
                plane.auto_state.takeoff_altitude_rel_cm = climb_to_alt_m * 100;
            }
            if (plane.landing.request_go_around()) {
                plane.auto_state.next_wp_crosstrack = false;
                return true;
            }
        }
    }
    return false;
}


/*
  the height above field elevation that we pass to TECS
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;
    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        hgt_afe = height_above_target();
        hgt_afe -= rangefinder_correction();
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

// vehicle specific waypoint info helpers
bool Plane::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        distance = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_distance_to_destination() * 0.01 : 0;
        return true;
    }
#endif
    distance = auto_state.wp_distance;
    return true;
}

bool Plane::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        bearing = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_bearing_to_destination() : 0;
        return true;
    }
#endif
    bearing = nav_controller->target_bearing_cd() * 0.01;
    return true;
}

bool Plane::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        xtrack_error = quadplane.using_wp_nav() ? quadplane.wp_nav->crosstrack_error() : 0;
        return true;
    }
#endif
    xtrack_error = nav_controller->crosstrack_error();
    return true;
}

#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
// set target location (for use by external control and scripting)
bool Plane::set_target_location(const Location &target_loc)
{
    Location loc{target_loc};

    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    // add home alt if needed
    if (loc.relative_alt) {
        loc.alt += plane.home.alt;
        loc.relative_alt = 0;
    }
    plane.set_guided_WP(loc);
    return true;
}
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
// set target location (for use by scripting)
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
#endif
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

/*
  update_target_location() works in all auto navigation modes
 */
bool Plane::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      by checking the caller has provided the correct old target
      location we prevent a race condition where the user changes mode
      or commands a different target in the controlling lua script
     */
    if (!old_loc.same_loc_as(next_WP_loc) ||
        old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }
    next_WP_loc = new_loc;

#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qland || control_mode == &mode_qloiter) {
        mode_qloiter.last_target_loc_set_ms = AP_HAL::millis();
    }
#endif

    return true;
}

// allow for velocity matching in VTOL
bool Plane::set_velocity_match(const Vector2f &velocity)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_land_sequence()) {
        quadplane.poscontrol.velocity_match = velocity;
        quadplane.poscontrol.last_velocity_match_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

// allow for override of land descent rate
bool Plane::set_land_descent_rate(float descent_rate)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() ||
        control_mode == &mode_qland) {
        quadplane.poscontrol.override_descent_rate = descent_rate;
        quadplane.poscontrol.last_override_descent_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}
#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing.
bool Plane::is_landing() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_land_descent()) {
        return true;
    }
#endif
    return control_mode->is_landing();
}

// returns true if vehicle is taking off.
bool Plane::is_taking_off() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_takeoff()) {
        return true;
    }
#endif
    return control_mode->is_taking_off();
}

// correct AHRS pitch for PTCH_TRIM_DEG in non-VTOL modes, and return VTOL view in VTOL
void Plane::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        pitch = quadplane.ahrs_view->pitch;
        roll = quadplane.ahrs_view->roll;
        return;
    }
#endif
    pitch = ahrs.get_pitch();
    roll = ahrs.get_roll();
    if (!(flight_option_enabled(FlightOptions::OSD_REMOVE_TRIM_PITCH))) {  // correct for PTCH_TRIM_DEG
        pitch -= g.pitch_trim * DEG_TO_RAD;
    }
}

/*
  update current_loc Location
 */
void Plane::update_current_loc(void)
{
    have_position = plane.ahrs.get_location(plane.current_loc);

    // re-calculate relative altitude
    ahrs.get_relative_position_D_home(plane.relative_altitude);
    relative_altitude *= -1.0f;
}

// check if FLIGHT_OPTION is enabled
bool Plane::flight_option_enabled(FlightOptions flight_option) const
{
    return g2.flight_options & flight_option;
}

#if AC_PRECLAND_ENABLED
void Plane::precland_update(void)
{
    // alt will be unused if we pass false through as the second parameter:
    return g2.precland.update(rangefinder_state.height_estimate*100, rangefinder_state.in_range);
}
#endif

AP_HAL_MAIN_CALLBACKS(&plane);
