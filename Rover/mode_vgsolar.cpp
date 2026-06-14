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
    _submode_before_turn(VGSubMode::STANDBY),
    _turn_phase(TurnPhase::IDLE),
    _target_speed_ms(0.0f),
    _target_yaw_cd(0.0f),
    _target_yaw_rate_cds(0.0f),
    _cruise_speed_ms(1.0f),
    _turn_direction(TURN_DIR_LEFT),
    _turn_mode_type(TURN_MODE_SPOT),
    _turn_target_angle_deg(0.0f),
    _turn_angular_vel_dps(30.0f),
    _turn_start_yaw_deg(0.0f),
    _turn_accumulated_deg(0.0f),
    _last_turn_yaw_deg(0.0f),
    _nav_coord_mode(0),
    _fault_flags(0),
    _nav_report_state(NavReportState::NONE),
    _nav_phase(NavPhase::CRUISE),
    _ned_origin_valid(false),
    _arrival_radius_m(0.0f),
    _arrival_yaw_required(false),
    _arrival_yaw_raw_cd(0),
    _arrival_yaw_target_cd(0.0f),
    _last_ncu_cmd_ms(0),
    _turn_phase_start_ms(0)
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
    _turn_phase = TurnPhase::IDLE;
    _target_speed_ms = 0.0f;
    _last_ncu_cmd_ms = 0;
    _fault_flags = 0;
    _nav_phase = NavPhase::CRUISE;
    // 记录 NED 全局原点，供后续 NED 导航换算经纬度
    capture_ned_origin();

    rover.companion_computer.stop_brushes();
    AP::brush().set_active(true);

    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: entered");
    return true;
}

void ModeVGSolar::_exit()
{
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;

    rover.companion_computer.stop_brushes();
    AP::brush().set_active(false);
    rover.companion_computer.reset_mode_status();

    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: exited");
}

void ModeVGSolar::update()
{
    // 低电压：强制关刷（台架无电池时注释掉，上车后恢复）
    uint8_t batt_pct = 0;
    if (AP::battery().capacity_remaining_pct(batt_pct, 1) && batt_pct <= 20) {
        rover.companion_computer.stop_brushes();
    }

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
    case VGSubMode::TURN:
        update_turn();
        break;
    case VGSubMode::NAV:
        update_nav();
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
    case VGSubMode::TURN:
        control_mode = uint8_t(ControlMode::TURN);
        break;
    case VGSubMode::NAV:
        control_mode = (_nav_coord_mode == NAV_MODE_BODY)
            ? uint8_t(ControlMode::NAV_BODY)
            : uint8_t(ControlMode::NAV_GPS);
        break;
    case VGSubMode::ESTOP:
        control_mode = uint8_t(ControlMode::STANDBY);
        break;
    }

    const bool estop = (_vg_submode == VGSubMode::ESTOP);
    const bool turning = (_vg_submode == VGSubMode::TURN);

    rover.companion_computer.update_mode_status(control_mode, estop, turning, _fault_flags);
}

void ModeVGSolar::publish_nav_status_feedback()
{
    NavStatusData data {};
    bool send_nav = false;

    if (_nav_report_state == NavReportState::ARRIVED) {
        data.nav_state = NAV_STATE_ARRIVED;
        data.coord_mode = _nav_coord_mode;
        send_nav = true;
        _nav_report_state = NavReportState::NONE;
    } else if (_nav_report_state == NavReportState::FAILED) {
        data.nav_state = NAV_STATE_FAILED;
        data.coord_mode = _nav_coord_mode;
        send_nav = true;
        _nav_report_state = NavReportState::NONE;
    } else if (_nav_report_state == NavReportState::CANCELLED) {
        data.nav_state = NAV_STATE_CANCELLED;
        data.coord_mode = _nav_coord_mode;
        send_nav = true;
        _nav_report_state = NavReportState::NONE;
    } else if (_vg_submode == VGSubMode::NAV) {
        data.nav_state = NAV_STATE_ACTIVE;
        data.coord_mode = _nav_coord_mode;

        // protocol 5.4: 距目标 uint32 cm
        const float dist_m = get_distance_to_destination();
        data.distance_to_target = (uint32_t)constrain_float(dist_m * 100.0f, 0.0f, 4294967295.0f);

        const float yaw_deg = ahrs.yaw_sensor * 0.01f;
        float err_deg;
        if (_nav_phase == NavPhase::YAW_ALIGN) {
            // 对航向阶段：偏差相对 arrival_yaw，而非航点方位
            err_deg = wrap_180(yaw_deg - _arrival_yaw_target_cd * 0.01f);
        } else {
            const float bearing_deg = wp_bearing();
            // 航向偏差，正值=目标在左侧
            err_deg = wrap_180(yaw_deg - bearing_deg);
        }
        data.heading_error = constrain_int16(int16_t(lroundf(err_deg * 100.0f)), -32767, 32767);

        send_nav = true;
    }

    rover.companion_computer.set_nav_status(data, send_nav);
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
            // 导航中急停：停车并上报 nav_state=已取消
            if (_vg_submode == VGSubMode::NAV) {
                stop_vehicle();
                g2.wp_nav.set_reversed(false);
                _nav_phase = NavPhase::CRUISE;
                _nav_report_state = NavReportState::CANCELLED;
            }
            _vg_submode = VGSubMode::ESTOP;
            cc.stop_brushes();
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

    if (cc.is_new_turn()) {
        cc.clear_new_turn_flag();
        const TurnData &cmd = cc.get_latest_turn();
        _last_ncu_cmd_ms = AP_HAL::millis();
        start_turn(cmd);
        _fault_flags &= ~FAULT_COMM_TIMEOUT;
        return;
    }

    if (cc.is_new_position()) {
        cc.clear_new_position_flag();
        const PositionData &cmd = cc.get_latest_position();
        _last_ncu_cmd_ms = AP_HAL::millis();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        if (cmd.nav_mode == NAV_MODE_CANCEL) {
            cancel_navigation();
            cc.send_position_ack(CMD_ACK_SUCCESS);
            return;
        }

        Location target_loc;
        Location current_loc;
        if (!ahrs.get_location(current_loc)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: no position, nav rejected");
            _nav_coord_mode = cmd.nav_mode;
            _nav_report_state = NavReportState::FAILED;
            cc.send_position_ack(CMD_ACK_FAILED);
            return;
        }

        switch (cmd.nav_mode) {
        case NAV_MODE_GPS:
            target_loc.lat = cmd.target_y;
            target_loc.lng = cmd.target_x;
            break;

        case NAV_MODE_NED: {
            // 以进入 VGSL 时记录的全局原点为基准，非当前位置
            if (!_ned_origin_valid) {
                capture_ned_origin();
            }
            if (!_ned_origin_valid) {
                gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: no NED origin, nav rejected");
                _nav_coord_mode = cmd.nav_mode;
                _nav_report_state = NavReportState::FAILED;
                cc.send_position_ack(CMD_ACK_FAILED);
                return;
            }
            const float north_m = cmd.target_x * 0.01f;
            const float east_m = cmd.target_y * 0.01f;
            target_loc = _ned_origin;
            target_loc.offset(north_m, east_m);
            break;
        }

        case NAV_MODE_BODY: {
            // 以指令接收时刻的当前位置+航向为基准，换算为 NED 偏移
            const float yaw_rad = radians(ahrs.yaw_sensor * 0.01f);
            const float forward_cm = cmd.target_x;
            const float right_cm = cmd.target_y;
            const float north_m = (forward_cm * cosf(yaw_rad) - right_cm * sinf(yaw_rad)) * 0.01f;
            const float east_m = (forward_cm * sinf(yaw_rad) + right_cm * cosf(yaw_rad)) * 0.01f;
            target_loc = current_loc;
            target_loc.offset(north_m, east_m);
            break;
        }

        default:
            cc.send_position_ack(CMD_ACK_FAILED);
            return;
        }

        if (!set_desired_location(target_loc)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: set destination failed");
            _nav_coord_mode = cmd.nav_mode;
            _nav_report_state = NavReportState::FAILED;
            cc.send_position_ack(CMD_ACK_FAILED);
            _fault_flags |= FAULT_NAV_FAILED;
            return;
        }
        // ModeGuided 不初始化 _distance_to_destination；wp_nav 在设点后已有正确距离
        _distance_to_destination = g2.wp_nav.get_distance_to_destination();

        _nav_coord_mode = cmd.nav_mode;
        _nav_report_state = NavReportState::NONE;
        _nav_phase = NavPhase::CRUISE;
        _fault_flags &= ~FAULT_NAV_FAILED;

        // 巡航速度：0 用参数默认；负值倒车（见 apply_nav_speed / set_reversed）
        if (cmd.cruise_speed != 0) {
            _cruise_speed_ms = cmd.cruise_speed * 0.01f;
        } else {
            _cruise_speed_ms = _cruise_speed_default;
        }

        // 到达半径：NCU 指定则优先；0 则回退 WP_RADIUS
        _arrival_radius_m = (cmd.arrival_radius > 0) ? cmd.arrival_radius * 0.01f : 0.0f;

        // 到达航向：0xFFFF 不指定；Body 模式在到达位置后再换算为绝对航向
        const uint16_t arrival_yaw_u16 = (uint16_t)cmd.arrival_yaw;
        if (arrival_yaw_u16 == NAV_YAW_UNSPECIFIED) {
            _arrival_yaw_required = false;
        } else {
            _arrival_yaw_required = true;
            _arrival_yaw_raw_cd = arrival_yaw_u16;
        }

        g2.wp_nav.set_reversed(is_negative(_cruise_speed_ms));
        apply_nav_speed();

        _vg_submode = VGSubMode::NAV;
        cc.send_position_ack(CMD_ACK_SUCCESS);

        gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV start, mode=%d", cmd.nav_mode);
        return;
    }

    if (cc.is_new_speed_ctrl()) {
        cc.clear_new_speed_flag();
        const SpeedCtrlData &cmd = cc.get_latest_speed_ctrl();
        _last_ncu_cmd_ms = AP_HAL::millis();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        if (_vg_submode == VGSubMode::NAV || _vg_submode == VGSubMode::TURN) {
            return;
        }

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
    // 复用 ModeGuided 的 HeadingAndSpeed 能力：
    set_desired_heading_and_speed(_target_yaw_cd, _target_speed_ms);
    ModeGuided::update();
}

void ModeVGSolar::update_yawrate()
{
    // 复用 ModeGuided 的 TurnRateAndSpeed 能力
    set_desired_turn_rate_and_speed(_target_yaw_rate_cds, _target_speed_ms);
    ModeGuided::update();
}

void ModeVGSolar::update_nav()
{
    //位置已到达，原地旋转到 arrival_yaw
    if (_nav_phase == NavPhase::YAW_ALIGN) {
        calc_steering_to_heading(_arrival_yaw_target_cd);
        g2.motors.set_throttle(0.0f);

        const float err_deg = fabsf(wrap_180(_arrival_yaw_target_cd * 0.01f - ahrs.yaw_sensor * 0.01f));
        if (err_deg <= 2.0f) {  // 2° 容差内视为对航向完成
            complete_nav_arrived();
        }
        return;
    }

    //沿航点导航至目标区域（巡航速度在收到导航指令时已设置）
    ModeGuided::update();

    if (nav_position_reached()) {
        if (_arrival_yaw_required) {
            stop_vehicle();
            if (_nav_coord_mode == NAV_MODE_BODY) {
                // Body 模式：arrival_yaw 相对车头，到达时刻叠加当前航向
                _arrival_yaw_target_cd = wrap_360_cd(ahrs.yaw_sensor + _arrival_yaw_raw_cd);
            } else {
                // GPS/NED：arrival_yaw 为相对正北的绝对航向
                _arrival_yaw_target_cd = _arrival_yaw_raw_cd;
            }
            _nav_phase = NavPhase::YAW_ALIGN;
            gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV position reached, aligning yaw");
            return;
        }
        complete_nav_arrived();
    }
}

void ModeVGSolar::capture_ned_origin()
{
    // 优先用当前 GPS 位置；无定位时退回到 Home 点
    Location loc;
    if (ahrs.get_location(loc)) {
        _ned_origin = loc;
        _ned_origin_valid = true;
        return;
    }

    if (ahrs.home_is_set()) {
        _ned_origin = ahrs.get_home();
        _ned_origin_valid = true;
    }
}

bool ModeVGSolar::nav_position_reached() const
{
    if (_arrival_radius_m > 0.0f) {
        // NCU 下发的到达半径优先于 WP_RADIUS
        return get_distance_to_destination() <= _arrival_radius_m;
    }
    return reached_destination();
}

void ModeVGSolar::complete_nav_arrived()
{
    stop_vehicle();
    g2.wp_nav.set_reversed(false);
    _nav_phase = NavPhase::CRUISE;
    _nav_report_state = NavReportState::ARRIVED;  // 下一周期 publish 一次 0xBB 0x04
    _vg_submode = VGSubMode::STANDBY;
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV arrived");
}

void ModeVGSolar::apply_nav_speed()
{
    // set_desired_speed 只接受正值；倒车方向由 wp_nav.set_reversed 控制
    set_desired_speed(fabsf(_cruise_speed_ms));
}

void ModeVGSolar::update_estop()
{
    stop_vehicle();
    rover.companion_computer.stop_brushes();
}

void ModeVGSolar::start_turn(const TurnData &cmd)
{
    _submode_before_turn = (_vg_submode == VGSubMode::TURN || _vg_submode == VGSubMode::ESTOP)
        ? VGSubMode::STANDBY : _vg_submode;

    _vg_submode = VGSubMode::TURN;
    _turn_direction = cmd.direction;
    _turn_mode_type = cmd.turn_mode;
    _turn_target_angle_deg = cmd.target_angle * 0.01f;
    _turn_angular_vel_dps = MAX(cmd.angular_vel * 0.01f, 1.0f);
    _turn_phase = TurnPhase::STOPPING;
    _turn_accumulated_deg = 0.0f;

    gcs().send_text(MAV_SEVERITY_INFO,
        "VG_SOLAR: TURN start dir=%d mode=%d angle=%.1f",
        _turn_direction, _turn_mode_type, _turn_target_angle_deg);
}

void ModeVGSolar::update_turn()
{
    const uint32_t now = AP_HAL::millis();

    switch (_turn_phase) {

    case TurnPhase::STOPPING: {
        const bool stopped = stop_vehicle();
        if (stopped) {
            _turn_phase = TurnPhase::WAIT_STOPPED;
            _turn_phase_start_ms = now;
        }
        break;
    }

    case TurnPhase::WAIT_STOPPED: {
        if (now - _turn_phase_start_ms > 500) {
            _turn_phase = TurnPhase::LOWER_SUCTION;
            _turn_phase_start_ms = now;
            // TODO: 控制吸盘舵机放下
        }
        break;
    }

    case TurnPhase::LOWER_SUCTION: {
        if (now - _turn_phase_start_ms > 1000) {
            _turn_phase = TurnPhase::WAIT_SUCTION_DN;
            _turn_phase_start_ms = now;
        }
        break;
    }

    case TurnPhase::WAIT_SUCTION_DN: {
        if (now - _turn_phase_start_ms > 500) {
            _turn_phase = TurnPhase::TURNING;
            _turn_phase_start_ms = now;
            _turn_start_yaw_deg = wrap_180(degrees(ahrs.get_yaw()));
            _last_turn_yaw_deg = _turn_start_yaw_deg;
            _turn_accumulated_deg = 0.0f;
        }
        break;
    }

    case TurnPhase::TURNING: {
        const float current_yaw_deg = wrap_180(degrees(ahrs.get_yaw()));
        const float step_deg = wrap_180(current_yaw_deg - _last_turn_yaw_deg);
        _turn_accumulated_deg += fabsf(step_deg);
        _last_turn_yaw_deg = current_yaw_deg;

        if (_turn_accumulated_deg >= _turn_target_angle_deg) {
            stop_vehicle();
            _turn_phase = TurnPhase::RAISE_SUCTION;
            _turn_phase_start_ms = now;
        } else if (now - _turn_phase_start_ms > _turn_timeout * 1000.0f) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN timeout");
            stop_vehicle();
            _turn_phase = TurnPhase::RAISE_SUCTION;
            _turn_phase_start_ms = now;
        } else {
            const float dir_sign = (_turn_direction == TURN_DIR_LEFT) ? 1.0f : -1.0f;
            const float turn_rate_cds = _turn_angular_vel_dps * 100.0f * dir_sign;
            const float speed_ms = (_turn_mode_type == TURN_MODE_SPOT) ? 0.0f : _turn_max_speed;
            set_desired_turn_rate_and_speed(turn_rate_cds, speed_ms);
            ModeGuided::update();
        }
        break;
    }

    case TurnPhase::RAISE_SUCTION: {
        // TODO: 控制吸盘舵机抬起
        if (now - _turn_phase_start_ms > 1000) {
            _turn_phase = TurnPhase::WAIT_SUCTION_UP;
            _turn_phase_start_ms = now;
        }
        break;
    }

    case TurnPhase::WAIT_SUCTION_UP: {
        if (now - _turn_phase_start_ms > 500) {
            _vg_submode = _submode_before_turn;
            _turn_phase = TurnPhase::IDLE;
            gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: TURN complete, restored");
        }
        break;
    }

    case TurnPhase::IDLE:
    default:
        break;
    }
}

void ModeVGSolar::cancel_navigation()
{
    stop_vehicle();
    g2.wp_nav.set_reversed(false);
    _nav_phase = NavPhase::CRUISE;
    _nav_report_state = NavReportState::CANCELLED;
    _vg_submode = VGSubMode::STANDBY;
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV cancelled");
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

    if (_vg_submode == VGSubMode::NAV || _vg_submode == VGSubMode::TURN ||
        _vg_submode == VGSubMode::ESTOP) {
        return;
    }

    if (_vg_submode != VGSubMode::STANDBY) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: NCU timeout, auto stop");
        _vg_submode = VGSubMode::STANDBY;
        _turn_phase = TurnPhase::IDLE;
    }
    stop_vehicle();
}

float ModeVGSolar::get_distance_to_destination() const
{
    if (_vg_submode == VGSubMode::NAV) {
        // 始终以 wp_nav 为准，避免 Mode 层 _distance_to_destination 滞后
        return g2.wp_nav.get_distance_to_destination();
    }
    return 0.0f;
}

void ModeVGSolar::set_brush_control(uint8_t brush_id, bool turn_on)
{
    (void)brush_id;
    if (!turn_on) {
        rover.companion_computer.stop_brushes();
    }
}

#endif  // MODE_VGSOLAR_ENABLED
