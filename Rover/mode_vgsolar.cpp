#include "Rover.h"
#include <AP_Math/AP_Math.h>
#include <AP_Brush/AP_Brush.h>

#if MODE_VGSOLAR_ENABLED

const AP_Param::GroupInfo ModeVGSolar::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: VG Solar mode enable
    // @Description: Enable VG Solar cleaning robot mode
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, ModeVGSolar, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: KP_YAW
    // @DisplayName: VG Solar yaw P gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("KP_YAW", 2, ModeVGSolar, _kp_yaw, 1.0f),

    // @Param: KP_SPEED
    // @DisplayName: VG Solar speed P gain
    // @Range: 0.1 5.0
    // @User: Advanced
    AP_GROUPINFO("KP_SPEED", 3, ModeVGSolar, _kp_speed, 0.5f),

    // @Param: CRUISE_SPD
    // @DisplayName: VG Solar default cruise speed
    // @Description: Default cruise speed for position navigation (m/s)
    // @Range: 0.1 5.0
    // @User: Advanced
    AP_GROUPINFO("CRUISE_SPD", 4, ModeVGSolar, _cruise_speed_default, 1.0f),

    // @Param: TURN_TO
    // @DisplayName: VG Solar turn timeout
    // @Description: Maximum time for turn sequence in seconds
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("TURN_TO", 5, ModeVGSolar, _turn_timeout, 15.0f),

    // @Param: TURN_SPD
    // @DisplayName: VG Solar turn speed
    // @Description: Max speed during moving turn (m/s)
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("TURN_SPD", 6, ModeVGSolar, _turn_max_speed, 0.3f),

    AP_GROUPEND
};

ModeVGSolar::ModeVGSolar(void) :
    _vg_submode(VGSubMode::STANDBY),
    _target_speed_ms(0.0f),
    _target_yaw_cd(0.0f),
    _target_yaw_rate_cds(0.0f),
    _fault_flags(0),
    _last_ncu_cmd_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeVGSolar::_enter()
{
    if (!_enabled) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: mode disabled");
        return false;
    }

    if (!ModeGuided::_enter()) {
        return false;
    }

    _vg_submode = VGSubMode::STANDBY;
    _target_speed_ms = 0.0f;
    _last_ncu_cmd_ms = 0;
    _fault_flags = 0;

    rover.companion_computer.stop_brushes();
    AP::brush().set_active(true);

    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: entered");
    return true;
}

void ModeVGSolar::_exit()
{
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    rover.companion_computer.stop_brushes();
    AP::brush().set_active(false);
    rover.companion_computer.reset_mode_status();

    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: exited");
}

void ModeVGSolar::update()
{
    check_ncu_timeout();
    read_companion_commands();

    if (_vg_submode == VGSubMode::ESTOP) {
        update_estop();
        return;
    }

    switch (_vg_submode) {
    case VGSubMode::STANDBY:
        update_standby();
        break;
    case VGSubMode::YAW:
        update_yaw();
        break;
    case VGSubMode::YAWRATE:
        update_yawrate();
        break;
    default:
        break;
    }
}

void ModeVGSolar::publish_status_feedback()
{
    uint8_t control_mode = uint8_t(ControlMode::STANDBY);
    switch (_vg_submode) {
    case VGSubMode::STANDBY:
        control_mode = uint8_t(ControlMode::STANDBY);
        break;
    case VGSubMode::YAW:
        control_mode = uint8_t(ControlMode::YAW);
        break;
    case VGSubMode::YAWRATE:
        control_mode = uint8_t(ControlMode::YAWRATE);
        break;
    case VGSubMode::ESTOP:
        control_mode = uint8_t(ControlMode::STANDBY);
        break;
    }

    const bool estop = (_vg_submode == VGSubMode::ESTOP);
    rover.companion_computer.update_mode_status(control_mode, estop, false, _fault_flags);
}

void ModeVGSolar::publish_nav_status_feedback()
{
    // Phase 2: NAV 子模式与 0xBB 0x04 导航反馈
}

void ModeVGSolar::read_companion_commands()
{
    auto &cc = rover.companion_computer;

    if (cc.is_new_system_ctrl()) {
        cc.clear_new_system_flag();
        const SystemCtrlData &cmd = cc.get_latest_system_ctrl();
        _last_ncu_cmd_ms = AP_HAL::millis();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        switch (cmd.command) {
        case SYS_CMD_ESTOP:
            _vg_submode = VGSubMode::ESTOP;
            rover.companion_computer.stop_brushes();
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: ESTOP");
            break;
        case SYS_CMD_ESTOP_CLEAR:
            if (_vg_submode == VGSubMode::ESTOP) {
                _vg_submode = VGSubMode::STANDBY;
                gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: ESTOP cleared");
            }
            break;
        case SYS_CMD_REBOOT:
            hal.scheduler->reboot(false);
            break;
        default:
            break;
        }
    }

    if (_vg_submode == VGSubMode::ESTOP) {
        return;
    }

    // Phase 2: 转弯序列
    if (cc.is_new_turn()) {
        cc.clear_new_turn_flag();
        _last_ncu_cmd_ms = AP_HAL::millis();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN not implemented");
        return;
    }

    // Phase 2: 位置导航
    if (cc.is_new_position()) {
        cc.clear_new_position_flag();
        _last_ncu_cmd_ms = AP_HAL::millis();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;
        cc.send_position_ack(CMD_ACK_FAILED);
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: NAV not implemented");
        return;
    }

    if (cc.is_new_speed_ctrl()) {
        cc.clear_new_speed_flag();
        const SpeedCtrlData &cmd = cc.get_latest_speed_ctrl();
        _last_ncu_cmd_ms = AP_HAL::millis();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        _target_speed_ms = cmd.velocity * 0.01f;

        if (cmd.control_mode == SPEED_MODE_YAW) {
            _vg_submode = VGSubMode::YAW;
            _target_yaw_cd = cmd.yaw_data;
        } else if (cmd.control_mode == SPEED_MODE_YAWRATE) {
            _vg_submode = VGSubMode::YAWRATE;
            _target_yaw_rate_cds = cmd.yaw_data;
        }
    }
}

void ModeVGSolar::update_standby()
{
    stop_vehicle();
}

void ModeVGSolar::update_yaw()
{
    // // 复用 ModeGuided 的 HeadingAndSpeed 能力：
    // // 设置目标航向和速度，更新 ModeGuided 内态，然后调用 ModeGuided::update()
    // set_desired_heading_and_speed(_target_yaw_cd, _target_speed_ms);
    // ModeGuided::update();

   
    // calc_steering_to_heading(_target_yaw_cd);

    // gcs().send_text(MAV_SEVERITY_INFO, "YAW DEBUG: speed=%.2f, direct set_throttle=50", _target_speed_ms);
    // g2.motors.set_throttle(50.0f);   // 硬编码 50%

    calc_steering_to_heading(_target_yaw_cd);

    // 直接速度→油门映射：目标速度 ÷ 最大速度 × 100
    float speed_max = calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
    float throttle_pct = 100.0f * (_target_speed_ms / speed_max);
    
    if (throttle_pct > 100.0f)  throttle_pct = 100.0f;
    if (throttle_pct < -100.0f) throttle_pct = -100.0f;

    g2.motors.set_throttle(throttle_pct);       

}

void ModeVGSolar::update_yawrate()
{
    // // 复用 ModeGuided 的 TurnRateAndSpeed 能力
    // set_desired_turn_rate_and_speed(_target_yaw_rate_cds, _target_speed_ms);
    // ModeGuided::update();

    float current_yaw_rate = ahrs.get_yaw_rate_earth();  //陀螺仪实测角速率
    float target_yaw_rate  = radians(_target_yaw_rate_cds * 0.01f);  //上位机发来的目标角速率
    float error = target_yaw_rate - current_yaw_rate;
    float turn_rate_rads = _kp_yaw.get() * error;

    calc_steering_from_turn_rate(turn_rate_rads);

    float speed_max = calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
    float throttle_pct = 100.0f * (_target_speed_ms / speed_max);
   
    if (throttle_pct > 100.0f)  throttle_pct = 100.0f;
    if (throttle_pct < -100.0f) throttle_pct = -100.0f;

    g2.motors.set_throttle(throttle_pct);
}

void ModeVGSolar::update_estop()
{
    stop_vehicle();
    rover.companion_computer.stop_brushes();
}

void ModeVGSolar::check_ncu_timeout()
{
    if (_last_ncu_cmd_ms == 0) {
        return;
    }

    const bool ncu_timed_out = AP_HAL::millis() - _last_ncu_cmd_ms > NCU_HEARTBEAT_TIMEOUT_MS;
    if (!ncu_timed_out) {
        return;
    }

    _fault_flags |= FAULT_COMM_TIMEOUT;
    rover.companion_computer.stop_brushes();

    if (_vg_submode == VGSubMode::ESTOP) {
        return;
    }

    if (_vg_submode != VGSubMode::STANDBY) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: NCU timeout, auto stop");
        _vg_submode = VGSubMode::STANDBY;
    }
    stop_vehicle();
}

float ModeVGSolar::get_distance_to_destination() const
{
    return 0.0f;
}

#endif  // MODE_VGSOLAR_ENABLED
