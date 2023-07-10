#include <stdlib.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_RSSI/AP_RSSI.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AC_PID/AP_PIDInfo.h>

#include "AP_Logger.h"
#include "AP_Logger_File.h"
#include "AP_Logger_MAVLink.h"
#include "LoggerMessageWriter.h"

extern const AP_HAL::HAL& hal;


/*
  write a structure format to the log - should be in frontend
 */
void AP_Logger_Backend::Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = s->msg_type;
    pkt.length = s->msg_len;
    strncpy_noterm(pkt.name, s->name, sizeof(pkt.name));
    strncpy_noterm(pkt.format, s->format, sizeof(pkt.format));
    strncpy_noterm(pkt.labels, s->labels, sizeof(pkt.labels));
}

/*
  Pack a LogStructure packet into a structure suitable to go to the logfile:
 */
void AP_Logger_Backend::Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_UNITS_MSG;
    pkt.time_us = AP_HAL::micros64();
    pkt.format_type = s->msg_type;
    strncpy_noterm(pkt.units, s->units, sizeof(pkt.units));
    strncpy_noterm(pkt.multipliers, s->multipliers, sizeof(pkt.multipliers));
}

/*
  write a structure format to the log
 */
bool AP_Logger_Backend::Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;
    Fill_Format(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a unit definition
 */
bool AP_Logger_Backend::Write_Unit(const struct UnitStructure *s)
{
    struct log_Unit pkt{
        LOG_PACKET_HEADER_INIT(LOG_UNIT_MSG),
        time_us : AP_HAL::micros64(),
        type    : s->ID,
        unit    : { }
    };
    strncpy_noterm(pkt.unit, s->unit, sizeof(pkt.unit));

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a unit-multiplier definition
 */
bool AP_Logger_Backend::Write_Multiplier(const struct MultiplierStructure *s)
{
    const struct log_Format_Multiplier pkt{
        LOG_PACKET_HEADER_INIT(LOG_MULT_MSG),
        time_us      : AP_HAL::micros64(),
        type         : s->ID,
        multiplier   : s->multiplier,
    };

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write the units for a format to the log
 */
bool AP_Logger_Backend::Write_Format_Units(const struct LogStructure *s)
{
    struct log_Format_Units pkt;
    Fill_Format_Units(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const char *name, float value, float default_val)
{
    struct log_Parameter pkt{
        LOG_PACKET_HEADER_INIT(LOG_PARAMETER_MSG),
        time_us : AP_HAL::micros64(),
        name  : {},
        value : value,
        default_value : default_val
    };
    strncpy_noterm(pkt.name, name, sizeof(pkt.name));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const AP_Param *ap,
                                            const AP_Param::ParamToken &token,
                                            enum ap_var_type type,
                                            float default_val)
{
    char name[16];
    ap->copy_name_token(token, &name[0], sizeof(name), true);
    return Write_Parameter(name, ap->cast_to_float(type), default_val);
}

#if AP_RC_CHANNEL_ENABLED
// Write an RCIN packet
void AP_Logger::Write_RCIN(void)
{
    uint16_t values[16] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));
    const struct log_RCIN pkt{
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : values[0],
        chan2         : values[1],
        chan3         : values[2],
        chan4         : values[3],
        chan5         : values[4],
        chan6         : values[5],
        chan7         : values[6],
        chan8         : values[7],
        chan9         : values[8],
        chan10        : values[9],
        chan11        : values[10],
        chan12        : values[11],
        chan13        : values[12],
        chan14        : values[13]
    };
    WriteBlock(&pkt, sizeof(pkt));

    const uint16_t override_mask = rc().get_override_mask();

    // don't waste logging bandwidth if we haven't seen non-zero
    // channels 15/16:
    if (!should_log_rcin2) {
        if (values[14] || values[15]) {
            should_log_rcin2 = true;
        } else if (override_mask != 0) {
            should_log_rcin2 = true;
        }
    }

    if (!should_log_rcin2) {
        return;
    }

    const struct log_RCIN2 pkt2{
        LOG_PACKET_HEADER_INIT(LOG_RCIN2_MSG),
        time_us       : AP_HAL::micros64(),
        chan15         : values[14],
        chan16         : values[15],
        override_mask  : override_mask,
    };
    WriteBlock(&pkt2, sizeof(pkt2));
}
#endif  // AP_RC_CHANNEL_ENABLED

// Write an SERVO packet
void AP_Logger::Write_RCOUT(void)
{
    const uint32_t enabled_mask = ~SRV_Channels::get_output_channel_mask(SRV_Channel::k_GPIO);

    if ((enabled_mask & 0x3FFF) != 0) {
        uint16_t channels[14] {};
        hal.rcout->read(channels, ARRAY_SIZE(channels));
        const struct log_RCOUT pkt{
            LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
            time_us       : AP_HAL::micros64(),
            chan1         : channels[0],
            chan2         : channels[1],
            chan3         : channels[2],
            chan4         : channels[3],
            chan5         : channels[4],
            chan6         : channels[5],
            chan7         : channels[6],
            chan8         : channels[7],
            chan9         : channels[8],
            chan10        : channels[9],
            chan11        : channels[10],
            chan12        : channels[11],
            chan13        : channels[12],
            chan14        : channels[13]
        };
        WriteBlock(&pkt, sizeof(pkt));
    }

#if NUM_SERVO_CHANNELS >= 15
    if ((enabled_mask & 0x3C000) != 0) {
        const struct log_RCOUT2 pkt2{
            LOG_PACKET_HEADER_INIT(LOG_RCOUT2_MSG),
            time_us       : AP_HAL::micros64(),
            chan15         : hal.rcout->read(14),
            chan16         : hal.rcout->read(15),
            chan17         : hal.rcout->read(16),
            chan18         : hal.rcout->read(17),
        };
        WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif

#if NUM_SERVO_CHANNELS >= 19
    if ((enabled_mask & 0xFFFC0000) != 0) {
        const struct log_RCOUT pkt3{
            LOG_PACKET_HEADER_INIT(LOG_RCOUT3_MSG),
            time_us       : AP_HAL::micros64(),
            chan1         : hal.rcout->read(18),
            chan2         : hal.rcout->read(19),
            chan3         : hal.rcout->read(20),
            chan4         : hal.rcout->read(21),
            chan5         : hal.rcout->read(22),
            chan6         : hal.rcout->read(23),
            chan7         : hal.rcout->read(24),
            chan8         : hal.rcout->read(25),
            chan9         : hal.rcout->read(26),
            chan10        : hal.rcout->read(27),
            chan11        : hal.rcout->read(28),
            chan12        : hal.rcout->read(29),
            chan13        : hal.rcout->read(30),
            chan14        : hal.rcout->read(31)
        };
        WriteBlock(&pkt3, sizeof(pkt3));
    }
#endif

}

// Write an RSSI packet
void AP_Logger::Write_RSSI()
{
    AP_RSSI *rssi = AP::rssi();
    if (rssi == nullptr) {
        return;
    }

    const struct log_RSSI pkt{
        LOG_PACKET_HEADER_INIT(LOG_RSSI_MSG),
        time_us       : AP_HAL::micros64(),
        RXRSSI        : rssi->read_receiver_rssi(),
        RXLQ          : rssi->read_receiver_link_quality()
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Command(const mavlink_command_int_t &packet,
                              uint8_t source_system,
                              uint8_t source_component,
                              const MAV_RESULT result,
                              bool was_command_long)
{
    const struct log_MAVLink_Command pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAVLINK_COMMAND_MSG),
        time_us         : AP_HAL::micros64(),
        target_system   : packet.target_system,
        target_component: packet.target_component,
        source_system   : source_system,
        source_component: source_component,
        frame           : packet.frame,
        command         : packet.command,
        param1          : packet.param1,
        param2          : packet.param2,
        param3          : packet.param3,
        param4          : packet.param4,
        x               : packet.x,
        y               : packet.y,
        z               : packet.z,
        result          : (uint8_t)result,
        was_command_long:was_command_long,
    };
    return WriteBlock(&pkt, sizeof(pkt));
}

bool AP_Logger_Backend::Write_Mission_Cmd(const AP_Mission &mission,
                                              const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_int_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink_int(cmd,mav_cmd);
    const struct log_Cmd pkt{
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        time_us         : AP_HAL::micros64(),
        command_total   : mission.num_commands(),
        sequence        : mav_cmd.seq,
        command         : mav_cmd.command,
        param1          : mav_cmd.param1,
        param2          : mav_cmd.param2,
        param3          : mav_cmd.param3,
        param4          : mav_cmd.param4,
        latitude        : mav_cmd.x,
        longitude       : mav_cmd.y,
        altitude        : mav_cmd.z,
        frame           : mav_cmd.frame
    };
    return WriteBlock(&pkt, sizeof(pkt));
}

#if AP_MISSION_ENABLED
bool AP_Logger_Backend::Write_EntireMission()
{
    // kick off asynchronous write:
    return _startup_messagewriter->writeentiremission();
}
#endif

// Write a text message to the log
bool AP_Logger_Backend::Write_Message(const char *message)
{
    struct log_Message pkt{
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };
    strncpy_noterm(pkt.msg, message, sizeof(pkt.msg));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Power(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    uint8_t safety_and_armed = uint8_t(hal.util->safety_switch_state());
    if (hal.util->get_soft_armed()) {
        // encode armed state in bit 3
        safety_and_armed |= 1U<<2;
    }
    const uint64_t now = AP_HAL::micros64();
    const struct log_POWR powr_pkt{
        LOG_PACKET_HEADER_INIT(LOG_POWR_MSG),
        time_us : now,
#if HAL_HAVE_BOARD_VOLTAGE
        Vcc     : hal.analogin->board_voltage(),
#else
        Vcc     : quiet_nanf(),
#endif
#if HAL_HAVE_SERVO_VOLTAGE
        Vservo  : hal.analogin->servorail_voltage(),
#else
        Vservo  : quiet_nanf(),
#endif
        flags   : hal.analogin->power_status_flags(),
        accumulated_flags   : hal.analogin->accumulated_power_status_flags(),
        safety_and_arm : safety_and_armed,
    };
    WriteBlock(&powr_pkt, sizeof(powr_pkt));

#if HAL_WITH_MCU_MONITORING
    const struct log_MCU mcu_pkt{
        LOG_PACKET_HEADER_INIT(LOG_MCU_MSG),
        time_us : now,
        MCU_temp : hal.analogin->mcu_temperature(),
        MCU_voltage : hal.analogin->mcu_voltage(),
        MCU_voltage_min : hal.analogin->mcu_voltage_min(),
        MCU_voltage_max : hal.analogin->mcu_voltage_max(),
    };
    WriteBlock(&mcu_pkt, sizeof(mcu_pkt));
#endif

#endif
}

void AP_Logger::Write_Radio(const mavlink_radio_t &packet)
{
    const struct log_Radio pkt{
        LOG_PACKET_HEADER_INIT(LOG_RADIO_MSG),
        time_us      : AP_HAL::micros64(),
        rssi         : packet.rssi,
        remrssi      : packet.remrssi,
        txbuf        : packet.txbuf,
        noise        : packet.noise,
        remnoise     : packet.remnoise,
        rxerrors     : packet.rxerrors,
        fixed        : packet.fixed
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance)
{
    const Compass &compass = AP::compass();

    const Vector3f &mag_field = compass.get_field(mag_instance);
    const Vector3f &mag_offsets = compass.get_offsets(mag_instance);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(mag_instance);
    const struct log_MAG pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAG_MSG),
        time_us         : time_us,
        instance        : mag_instance,
        mag_x           : (int16_t)mag_field.x,
        mag_y           : (int16_t)mag_field.y,
        mag_z           : (int16_t)mag_field.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z,
        health          : (uint8_t)compass.healthy(mag_instance),
        SUS             : compass.last_update_usec(mag_instance)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Compass packet
void AP_Logger::Write_Compass()
{
    const uint64_t time_us = AP_HAL::micros64();
    const Compass &compass = AP::compass();
    for (uint8_t i=0; i<compass.get_count(); i++) {
        Write_Compass_instance(time_us, i);
    }
}

// Write a mode packet.
bool AP_Logger_Backend::Write_Mode(uint8_t mode, const ModeReason reason)
{
    static_assert(sizeof(ModeReason) <= sizeof(uint8_t), "Logging expects the ModeReason to fit in 8 bits");
    const struct log_Mode pkt{
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_us  : AP_HAL::micros64(),
        mode     : mode,
        mode_num : mode,
        mode_reason : static_cast<uint8_t>(reason)
    };
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write servo status from CAN servo
 */
void AP_Logger::Write_ServoStatus(uint64_t time_us, uint8_t id, float position, float force, float speed, uint8_t power_pct,
                                  float pos_cmd, float voltage, float current, float mot_temp, float pcb_temp, uint8_t error)
{
    const struct log_CSRV pkt {
        LOG_PACKET_HEADER_INIT(LOG_CSRV_MSG),
        time_us     : time_us,
        id          : id,
        position    : position,
        force       : force,
        speed       : speed,
        power_pct   : power_pct,
        pos_cmd     : pos_cmd,
        voltage     : voltage,
        current     : current,
        mot_temp    : mot_temp,
        pcb_temp    : pcb_temp,
        error       : error,
    };
    WriteBlock(&pkt, sizeof(pkt));
}


// Write a Yaw PID packet
void AP_Logger::Write_PID(uint8_t msg_type, const AP_PIDInfo &info)
{
    const struct log_PID pkt{
        LOG_PACKET_HEADER_INIT(msg_type),
        time_us         : AP_HAL::micros64(),
        target          : info.target,
        actual          : info.actual,
        error           : info.error,
        P               : info.P,
        I               : info.I,
        D               : info.D,
        FF              : info.FF,
        Dmod            : info.Dmod,
        slew_rate       : info.slew_rate,
        limit           : info.limit
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& breadcrumb)
{
    const struct log_SRTL pkt_srtl{
        LOG_PACKET_HEADER_INIT(LOG_SRTL_MSG),
        time_us         : AP_HAL::micros64(),
        active          : active,
        num_points      : num_points,
        max_points      : max_points,
        action          : action,
        N               : breadcrumb.x,
        E               : breadcrumb.y,
        D               : breadcrumb.z
    };
    WriteBlock(&pkt_srtl, sizeof(pkt_srtl));
}

void AP_Logger::Write_Winch(bool healthy, bool thread_end, bool moving, bool clutch, uint8_t mode, float desired_length, float length, float desired_rate, uint16_t tension, float voltage, int8_t temp)
{
    struct log_Winch pkt{
        LOG_PACKET_HEADER_INIT(LOG_WINCH_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : healthy,
        thread_end      : thread_end,
        moving          : moving,
        clutch          : clutch,
        mode            : mode,
        desired_length  : desired_length,
        length          : length,
        desired_rate    : desired_rate,
        tension         : tension,
        voltage         : voltage,
        temp            : temp
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// a convenience function for writing out the position controller PIDs
void AP_Logger::Write_PSCx(LogMessages id, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    const struct log_PSCx pkt{
        LOG_PACKET_HEADER_INIT(id),
            time_us         : AP_HAL::micros64(),
            pos_target    : pos_target * 0.01f,
            pos           : pos * 0.01f,
            vel_desired   : vel_desired * 0.01f,
            vel_target    : vel_target * 0.01f,
            vel           : vel * 0.01f,
            accel_desired : accel_desired * 0.01f,
            accel_target  : accel_target * 0.01f,
            accel         : accel * 0.01f
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_PSCN(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCN_MSG, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

void AP_Logger::Write_PSCE(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCE_MSG, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

void AP_Logger::Write_PSCD(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCD_MSG, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}
