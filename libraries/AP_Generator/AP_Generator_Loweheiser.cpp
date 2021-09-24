#include "AP_Generator_Loweheiser.h"

#if AP_GENERATOR_LOWEHEISER_ENABLED

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define MASK_LOG_ANY                    0xFFFF
#define LOWEHEISER_EFI_COMS_TIMEOUT_MS  1500
#define LOWEHEISER_EFI_VOLT_MIN         11.0
#define LOWEHEISER_EFI_VOLT_MAX         13.5

const AP_Param::GroupInfo AP_Generator_Loweheiser::var_info[] = {

    // Param indexes must be between 10 and 19 to avoid conflict with other generator param tables loaded by pointer

    // @Param: MNT_TIME
    // @DisplayName: Seconds until maintenance required
    // @Description: Seconds until maintenance required
    // @User: Advanced
    AP_GROUPINFO("MNT_TIME", 10, AP_Generator_Loweheiser, time_until_maintenance, 0),

    // @Param: RUNTIME
    // @DisplayName: Total runtime
    // @Description: Total time this generator has run in seconds
    // @User: Advanced
    AP_GROUPINFO("RUNTIME", 11, AP_Generator_Loweheiser, total_runtime, 0),

    // @Param: IDLE_TH_H
    // @DisplayName: High Idle throttle
    // @Description: throttle value to use when warming up or cooling down
    // @User: Advanced
    AP_GROUPINFO("IDLE_TH_H", 12, AP_Generator_Loweheiser, high_idle_throttle, 20),

    // @Param: IDLE_TH
    // @DisplayName: Idle throttle
    // @Description: throttle value to use when idling
    // @User: Advanced
    AP_GROUPINFO("IDLE_TH", 13, AP_Generator_Loweheiser, idle_throttle, 20),

    // @Param: RUN_TEMP
    // @DisplayName: Run Temperature
    // @Description: temperature required for generator to start producing power in deg celsius
    // @User: Advanced
    AP_GROUPINFO("RUN_TEMP", 14, AP_Generator_Loweheiser, temp_required_for_run, 60),

    // @Param: IDLE_TEMP
    // @DisplayName: Idle Temperature
    // @Description: temperature required for generator to return to idle after having run
    // @User: Advanced
    AP_GROUPINFO("IDLE_TEMP", 15, AP_Generator_Loweheiser, temp_required_for_idle, 110),

    // @Param: OVER_TEMP
    // @DisplayName: Cylinder Head Over Temperature Warning Level
    // @Description: threshold temperature for the cylinder head above which the mavlink over temperature message gets sent
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("OVER_TEMP", 16, AP_Generator_Loweheiser, temp_for_overtemp_warning, 205),

    // Param indexes must be between 10 and 19 to avoid conflict with other generator param tables loaded by pointer

    AP_GROUPEND
};

void AP_Generator_Loweheiser::init()
{
    AP_Param::setup_object_defaults(this, var_info);

    _frontend._has_current = true;
    _frontend._has_consumed_energy = true;
    _frontend._has_fuel_remaining = false;

    // nothing in this method may use parameters as AP_Generator loads
    // values from eeprom!
}

// healthy returns true if the generator is not present, or it is
// present, providing telemetry and not indicating an errors.  Check
// several fields as being within-range.
bool AP_Generator_Loweheiser::healthy() const
{
    if (last_packet_received_ms == 0) {
        return true;
    }

    if (AP_HAL::millis() - last_packet_received_ms > LOWEHEISER_EFI_COMS_TIMEOUT_MS) {
        return false;
    }

    // these voltage constants were supplied by the manufacturer:
    if (!isnan(packet.efi_batt) &&
        (packet.efi_batt < LOWEHEISER_EFI_VOLT_MIN || packet.efi_batt > LOWEHEISER_EFI_VOLT_MAX)) {
        return false;
    }

    return true;
}

// runstate_string provides a textual representation for the supplied
// runstate.  Useful in messages to the user.
const char *AP_Generator_Loweheiser::runstate_string(PilotDesiredRunState runstate)
{
    switch (runstate) {
    case PilotDesiredRunState::STOP:
        return "STOP";
    case PilotDesiredRunState::IDLE:
        return "IDLE";
    case PilotDesiredRunState::RUN:
        return "RUN";
    }
    return "?";
}

void AP_Generator_Loweheiser::set_pilot_desired_runstate(PilotDesiredRunState newstate) {
    gcs().send_text(MAV_SEVERITY_INFO, "LH: pilot-desired state to (%s) from (%s)", runstate_string(newstate), runstate_string(pilot_desired_runstate));
    pilot_desired_runstate = newstate;
}

// returns true if the generator should be allowed to move into the
// "run" (high-RPM) state:
bool AP_Generator_Loweheiser::generator_ok_to_run() const
{
    if (isnan(packet.efi_clt)) {
        return false;
    }
    return packet.efi_clt >= temp_required_for_run;
}

// returns true if the generator should be allowed to move into the
// "stop" state:
bool AP_Generator_Loweheiser::generator_ok_to_stop() const
{
    return packet.efi_clt <= temp_required_for_idle;
}

// should_emergency_stop - returns true if the generator must stop
// immediately
bool AP_Generator_Loweheiser::should_emergency_stop()
{
    const char *estop_reason = nullptr;

    if (AP::vehicle()->is_crashed()) {
        estop_reason = "crash";
    }

    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        estop_reason = "safety switch";
    }

    if (estop_reason == nullptr) {
        // not emergency stopped
        estop_reported = false;
        return false;
    }

    // we are currrently emergency-stopped
    if (!estop_reported) {
        gcs().send_text(MAV_SEVERITY_INFO, "LH: %s; stopping generator", estop_reason);
        estop_reported = true;
    }

    return true;
}

// update_runstate updates the commanded runstate - what we are
// telling the geneator to do.  Which state we request the generator
// move to depends on the RC input control, the temperature the
// generator is at and other information
void AP_Generator_Loweheiser::update_runstate()
{
    // if the vehicle crashes then we assume the pilot wants to stop
    // the motor.  This is done as a once-off when the crash is
    // detected to allow the operator to rearm the vehicle, or we end
    // up in a catch-22 situation where we force the stop state on the
    // generator so they can't arm and can't start the generator
    // because the vehicle is crashed.
    if (should_emergency_stop()) {
        commanded_runstate = RunState::STOP;
        return;
    }

    if (hal.util->get_soft_armed()) {
        // don't permit transitions while armed
        return;
    }

    // if we have lost RC input, and the vehicle is disarmed then stop
    // the generator - but allow the thing to cool down....
    if (!rc().has_valid_input() &&
        pilot_desired_runstate != PilotDesiredRunState::STOP) {
        gcs().send_text(MAV_SEVERITY_INFO, "LH: no RC and disarmed; stopping generator");
        pilot_desired_runstate = PilotDesiredRunState::STOP;
    }

    // skip changing the commanded runstate if we're already in the
    // target state.  Note that this block also stops us forcing the
    // runstate to idle when we don't have data from the EFI!
    switch (pilot_desired_runstate) {
    case PilotDesiredRunState::STOP:
        if (commanded_runstate == RunState::STOP) {
            return;
        }
        break;
    case PilotDesiredRunState::IDLE:
        if (commanded_runstate == RunState::IDLE) {
            return;
        }
        break;
    case PilotDesiredRunState::RUN:
        if (commanded_runstate == RunState::RUN) {
            return;
        }
        break;
    }

    if (isnan(packet.efi_clt)) {
        // we don't know what the temperature is..... command idle
        // until we know what the temperature is.  This can happen
        // because the EFI is actually powered off when we command the
        // generator to stop.  Moving to IDLE should start it up so we
        // can get the data we need:
        commanded_runstate = RunState::IDLE;
        return;
    }

    // consider changing the commanded runstate to the pilot desired
    // runstate:
    commanded_runstate = RunState::IDLE;
    switch (pilot_desired_runstate) {
    case PilotDesiredRunState::STOP:
        if (!generator_ok_to_stop()) {
            commanded_runstate = RunState::COOLING_DOWN;
            break;
        }
        commanded_runstate = RunState::STOP;
        break;
    case PilotDesiredRunState::IDLE:
        if (!generator_ok_to_stop()) {
            commanded_runstate = RunState::COOLING_DOWN;
            break;
        }
        commanded_runstate = RunState::IDLE;
        break;
    case PilotDesiredRunState::RUN:
        if (!generator_ok_to_run()) {
            commanded_runstate = RunState::WARMING_UP;
            break;
        }
        commanded_runstate = RunState::RUN;
        break;
    }
}

// update_common_backend_variables changes data in the superclass
// object, reporting our current state
void AP_Generator_Loweheiser::update_common_backend_variables()
{
    if (last_packet_received_ms == 0) {
        // no data from the generator
        return;
    }

    // common backend variables.  These are used by the base class
    // (AP_Generator_Backend) to provide data to the battery monitor
    // library.
    _voltage = packet.volt_batt;
    _current = packet.curr_batt;

    // provide our own aggregate data:
    _consumed_mah = accumulated_consumed_fuel_litres * 1000;

    // packet.efi_rpm_consumed goes to NaN while the EFI is off
    // (which is the case when the generator is in the "off" state).
    // In that case we assume the generator isn't turning.
    if (isnan(packet.efi_rpm)) {
        _rpm = 0;
    } else {
        _rpm = packet.efi_rpm;
    }

    // packet.efi_fuel_consumed goes to NaN while the EFI is off
    // (which is the case when the generator is in the "off" state).
    // In that case we send 0 as the amount of fuel remaining
    if (isnan(packet.fuel_level)) {
        _fuel_remain_l = 0;
    } else {
        _fuel_remain_l = packet.fuel_level;
    }
}

void AP_Generator_Loweheiser::check_second_init()
{
    if (second_init_done) {
        return;
    }
    second_init_done = true;
    if (time_until_maintenance == 0) {
        // manufacturer-recommended maintenance interval, 300 hours.
        // On the off-chance that the user manages to switch the
        // vehicle off at exactly 0 seconds remaining this *will*
        // reset.
        time_until_maintenance.set_and_save(300 * 60 * 60);
    }
}

void AP_Generator_Loweheiser::update()
{
    check_second_init();

    // periodically check the user's configuration of RC input channels:
    check_rc_input_channels();

    // consider our pilot-demanded run state and perhaps change the
    // way we are commanding the generator:
    update_runstate();

    // update the variables the frontend looksat:
    update_common_backend_variables();

    // now ask the frontend to look at that updated state:
    update_frontend();

    // consider sending commands to the generator, which we do at
    // regular intervals:
    command_generator();

    // see if the generator is producing good acks:
    if (last_ack_packet_ms != last_ack_packet_processed_ms) {
        last_ack_packet_processed_ms = last_ack_packet_ms;
        good_ack = ack_packet.result == MAV_RESULT_ACCEPTED;
    }

    // onboard logging
    Log_Write();

    // record runtime statistics (e.g. total runtime) to backing parameters:
    update_stats();
}

void AP_Generator_Loweheiser::check_rc_input_channels()
{
    const uint32_t now_ms = AP_HAL::millis();

    // only check every now and then:
    if (now_ms - last_rc_channel_check < 1000) {
        return;
    }
    last_rc_channel_check = now_ms;

    // update our manual control throttle channel every now and then
    // (for runtime configuration):
    RC_Channel *x = rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOWEHEISER_THROTTLE);
    if (x != rc_channel_manual_throttle) {
        rc_channel_manual_throttle = x;
        if (rc_channel_manual_throttle != nullptr) {
            rc_channel_manual_throttle->set_range(4500);
        }
    }

    // see if the user has defined a starter motor input channel.
    // This allows for manual control of the starter motor
    rc_channel_starter_motor = rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOWEHEISER_STARTER);
}

// command_generator assembles COMMAND_LONG messages to send to the
// generator.  It does this at regular intervals so that in the case
// the MCU dies the generator can undertake sensible failsafe actions.
void AP_Generator_Loweheiser::command_generator()
{
    if (mavlink_channel == nullptr) {
        // we've never seen a generator, so don't bother with any more work
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // these values are appropriate to command the generator to the
    // "STOP state.  Note that these are values which are placed
    // directly into the packet going to the generator.
    float throttle = 0.0;
    uint8_t desired_governor_state = 0;
    uint8_t run_electric_starter = 0;
    uint8_t desired_engine_state = 0;

    switch (commanded_runstate) {
    case RunState::STOP:
        // the variable initialisation above is sufficient
        break;
    case RunState::COOLING_DOWN:
    case RunState::WARMING_UP: {
        // because we can spend a deal of time in this state we
        // comfort the user periodically by giving them a progress
        // message of sorts:
        if (now_ms - last_waiting_temperature_change_ms > 5000) {
            if (isnan(packet.efi_clt)) {
                gcs().send_text(MAV_SEVERITY_INFO, "LH: Waiting for EFI");
            } else if (commanded_runstate == RunState::WARMING_UP) {
                gcs().send_text(MAV_SEVERITY_INFO,
                                "LH: Generator warming up (%f < %f)",
                                packet.efi_clt,
                                temp_required_for_run.get());
            } else {
                gcs().send_text(MAV_SEVERITY_INFO,
                                "LH: Generator cooling down (%f > %f)",
                                packet.efi_clt,
                                temp_required_for_idle.get());
            }
            last_waiting_temperature_change_ms = now_ms;
        }
        desired_engine_state = 1;
        throttle = high_idle_throttle;
        break;
    }
    case RunState::IDLE:
        // consider acting on manual throttle input:
        if (rc_channel_manual_throttle != nullptr &&
            rc().has_valid_input() &&
            !rc_channel_manual_throttle->in_min_dz()) {
            throttle = rc_channel_manual_throttle->percent_input();
            // honour an electric start channel, too:
            if (rc_channel_starter_motor != nullptr) {
                switch (rc_channel_starter_motor->get_aux_switch_pos()) {
                case RC_Channel::AuxSwitchPos::LOW:
                case RC_Channel::AuxSwitchPos::MIDDLE:
                    break;
                case RC_Channel::AuxSwitchPos::HIGH:
                    run_electric_starter = (rc_channel_starter_motor != nullptr);
                }
            }
        } else {
            // no manual throttle in play
            throttle = idle_throttle;
        }
        desired_engine_state = 1;
        break;
    case RunState::RUN:
        desired_engine_state = 1;
        desired_governor_state = 1;
        break;
    }

    // if our desired run state is not "stop", and the RPM is zero,
    // then consider running the starter motor.  Run motor for 5s
    if (commanded_runstate != RunState::STOP) {
        bool configure_for_start = false;
        if (last_start_time_ms == 0) {
            if (is_zero(packet.efi_rpm)) {
                gcs().send_text(MAV_SEVERITY_INFO, "LH: running starter motor");
                last_start_time_ms = now_ms;
                configure_for_start = true;
            }
        } else if (now_ms - last_start_time_ms < 5000) {
            configure_for_start = true;
        } else if (now_ms - last_start_time_ms > 20000) {
            last_start_time_ms = 0;
        }

        if (configure_for_start) {
            run_electric_starter = 1;
            desired_governor_state = 0;
        }
    }

    // actually send the packet:
    mavlink_msg_command_long_send(
        mavlink_channel->get_chan(),
        sysid,
        compid,
        MAV_CMD_LOWEHEISER_SET_STATE,
        0,  // confirmation
        efi_index,     // p1, EFI index
        desired_engine_state,  // p2, desired engine state
        desired_governor_state,  // p3, desired governor state - 1 means governed
        throttle,  // p4, desired manual throttle
        run_electric_starter,  // p5, electric starter
        0,  // p6, empty
        0   // p7, empty
        );

    // log all commands to dataflash:

// @LoggerMessage: LOEC
// @Description: Gathered Loweheiser EFI/Governor telemetry
// @Field: TimeUS: Time since system startup
// @Field: SI: target system ID
// @Field: CI: target component ID
// @Field: C: command
// @Field: I: efi index
// @Field: ES: desired engine state (0:EFI off 1:EFI on)
// @Field: GS: desired governor state (0:Governor off 1:Governor on)
// @Field: Thr: manual throttle value
// @Field: Strtr: desired electric start

    if (AP::logger().should_log(MASK_LOG_ANY)) {
        AP::logger().Write(
            "LOEC",
            "TimeUS," "SI," "CI," "C," "I," "ES," "GS," "Thr," "Strtr",
            "s"       "-"   "-"   "-"  "#"  "-"   "-"   "-"    "-"     ,
            "F"       "-"   "-"   "-"  "-"  "-"   "-"   "-"    "-"     ,
            "Q"       "B"   "B"   "I"  "B"  "B"   "B"   "f"    "B"     ,
            AP_HAL::micros64(),
            sysid,
            compid,
            MAV_CMD_LOWEHEISER_SET_STATE,
            efi_index,
            desired_engine_state,
            desired_governor_state,
            throttle,
            run_electric_starter
            );
    }
}

// update (and perhaps persist) statistics (e.g. total runtime) to
// backing parameters:
void AP_Generator_Loweheiser::update_stats()
{
    bool running = false;
    switch (commanded_runstate) {
    case RunState::STOP:
        running = false;
        break;
    case RunState::IDLE:
    case RunState::RUN:
    case RunState::WARMING_UP:
    case RunState::COOLING_DOWN:
        running = true;
        break;
    }

    if (is_zero(packet.efi_rpm)) {
        running = false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (running) {
        if (was_running) {
            runtime_delta_ms += now_ms - run_start_ms;
            run_start_ms = now_ms;
        } else {
            run_start_ms = now_ms;
            was_running = true;
        }
    } else {
        if (was_running) {
            runtime_delta_ms += now_ms - run_start_ms;
            was_running = false;
        }
    }

    // don't record small runtime changes:
    if (runtime_delta_ms < 1000) {
        return;
    }

    // only save to permanent storage every 30 seconds:
    if (now_ms - last_stats_saved_ms < 30000) {
        return;
    }
    last_stats_saved_ms = now_ms;

    uint32_t seconds = runtime_delta_ms / 1000;
    runtime_delta_ms -= seconds * 1000;

    total_runtime.set_and_save_ifchanged(total_runtime + seconds);
        // avoid resetting time until maintenance to initial value:
    if (seconds > 0 && (signed)seconds == time_until_maintenance) {
        seconds--;
    }
    time_until_maintenance.set_and_save_ifchanged(time_until_maintenance - seconds);
}

// ensure the generator is running and generally working before
// allowing the vehicle to arm:
bool AP_Generator_Loweheiser::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (last_packet_received_ms == 0) {
        // allow optional use of generator
        return true;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_packet_received_ms > 2000) { // we expect @1Hz
        hal.util->snprintf(failmsg, failmsg_len, "LH: no messages in %ums", unsigned(now_ms - last_packet_received_ms));
        return false;
    }

    // we warn the user that maintenance is required, but don't stop
    // them arming and flying:
    if (packet.until_maintenance == 0) {
        hal.util->snprintf(failmsg, failmsg_len, "LH: requires maintenance");
    }

    // check the user is trying to run the generator:
    if (pilot_desired_runstate != PilotDesiredRunState::RUN) {
        hal.util->snprintf(failmsg, failmsg_len, "LH: requested state is not RUN");
        return false;
    }

    // check ArduPilot is trying to run the generator:
    switch (commanded_runstate) {
    case RunState::RUN:
        // this is a good place to be
        break;
    case RunState::STOP:
    case RunState::IDLE:
    case RunState::COOLING_DOWN:
        break;
    case RunState::WARMING_UP:
        if (!isnan(packet.efi_clt)) {
            hal.util->snprintf(failmsg, failmsg_len, "LH: Generator warming up (%.0f%%)", ((packet.efi_clt*100.0) / temp_required_for_run));
        } else {
            hal.util->snprintf(failmsg, failmsg_len, "LH: Generator warming up (waiting for data)");
        }
        return false;
    }

    return true;
}

// handle mavlink packets received from the Loweheiser generator
// control system.  We keep an entire packet around for ease of use.
void AP_Generator_Loweheiser::handle_mavlink_msg(const GCS_MAVLINK &channel, const mavlink_message_t &msg)
{
    if (seen_good_message) {
        // ensure any subsequent messages we accept are only from the
        // sysid/compid tuple we initially saw:
        if (msg.sysid != sysid || msg.compid != compid) {
            return;
        }
    }

    // good_message is set true if we discover a packet from a
    // Loweheiser generator.
    bool good_message = false;
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI: {
        mavlink_msg_loweheiser_gov_efi_decode(&msg, &packet);

        // TODO: ensure from correct efi_number
        efi_index = packet.efi_index;

        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t delta_t_ms = now_ms - last_packet_received_ms;
        if (last_packet_received_ms == 0 ||
            delta_t_ms > 30000) {
            gcs().send_text(MAV_SEVERITY_INFO, "LH: Found Loweheiser Generator");
        } else {
            // update the accumulated fuel, assuming we've been consuming
            // at the new rate since the last packet was received:

            // convert from litres/hour to litres/millisecond:
            if (!isnan(packet.efi_fuel_flow)) {
                const float litres_per_millisecond = packet.efi_fuel_flow/(60*60*1000);
                accumulated_consumed_fuel_litres += litres_per_millisecond*delta_t_ms;
            }
        }
        last_packet_received_ms = now_ms;
        good_message = true;
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
        // only accept acks once we've seen a loweheiser packet:
        if (!seen_good_message) {
            return;
        }
        mavlink_msg_command_ack_decode(&msg, &ack_packet);
        last_ack_packet_ms = AP_HAL::millis();
        break;
    }

    if (!good_message) {
        return;
    }
    if (seen_good_message) {
        return;
    }
    seen_good_message = true;

    mavlink_channel = &channel;
    sysid = msg.sysid;
    compid = msg.compid;
}

// send mavlink generator status
void AP_Generator_Loweheiser::send_generator_status(const GCS_MAVLINK &channel)
{
    if (last_packet_received_ms == 0) {
        // nothing to report
        return;
    }

    uint64_t status = 0;
    switch (commanded_runstate) {
    case RunState::STOP:
        // we checking RPM here but confounded by starter motor running....
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
        break;
    case RunState::COOLING_DOWN:
    case RunState::IDLE:
        if (generator_ok_to_run()) {
            status |= MAV_GENERATOR_STATUS_FLAG_READY;
        }
        break;
    case RunState::RUN:
        status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
        break;
    case RunState::WARMING_UP:
        status |= MAV_GENERATOR_STATUS_FLAG_WARMING_UP;
        break;
    }

    // note that the EFI unit can be turned off on the Loweheiser, in
    // which case we receive NaN values back.  We translate these into
    // MAVLink "unknown" values throughout here:

    if (!isnan(packet.efi_clt) &&
        packet.efi_clt > temp_for_overtemp_warning) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING;
    }

    int16_t rectifier_temp = INT16_MAX;
    if (!isnan(packet.rectifier_temp)) {
        rectifier_temp = packet.rectifier_temp;
    }
    int16_t generator_temp = INT16_MAX;
    if (!isnan(packet.generator_temp)) {
        generator_temp = packet.generator_temp;
    }

    // not all loweheiser firmwares provide runtime and
    // time-until-maintenance, so provide values we have in our
    // parameters for those:
    uint32_t runtime = packet.runtime;
    if (runtime == UINT32_MAX) {
        runtime = total_runtime + runtime_delta_ms/1000;
    }
    uint32_t maint_time = packet.until_maintenance;
    if (maint_time == INT32_MAX) {
        maint_time = time_until_maintenance - runtime_delta_ms/1000;
    }

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,
        _rpm, // generator_speed
        packet.curr_batt,  // Current into/out of battery
        packet.curr_gen, // load_current; Current going to UAV
        (packet.curr_rot * _voltage),               // power_generated (w)
        _voltage, // bus_voltage; Voltage of the bus seen at the generator
        rectifier_temp, // rectifier_temperature
        std::numeric_limits<double>::quiet_NaN(), // bat_current_setpoint; The target battery current
        generator_temp, // generator temperature
        runtime,  // runtime / time since boot,
        maint_time
        );
}

// methods to control the generator state:
bool AP_Generator_Loweheiser::stop()
{
    set_pilot_desired_runstate(PilotDesiredRunState::STOP);
    return true;
}

bool AP_Generator_Loweheiser::idle()
{
    set_pilot_desired_runstate(PilotDesiredRunState::IDLE);
    return true;
}

bool AP_Generator_Loweheiser::run()
{
    set_pilot_desired_runstate(PilotDesiredRunState::RUN);
    return true;
}

// log generator status to the onboard log
void AP_Generator_Loweheiser::Log_Write()
{
    if (!AP::logger().should_log(MASK_LOG_ANY)) {
        return;
    }

    // only log new readings:
    if (last_logged_reading_ms == last_packet_received_ms) {
        return;
    }
    last_logged_reading_ms = last_packet_received_ms;

// @LoggerMessage: LOEG
// @Description: Gathered Loweheiser EFI/Governor telemetry
// @Field: TimeUS: Time since system startup
// @Field: I: EFI/Gov sensor instance number
// @Field: VB: battery voltage
// @Field: CB: battery current
// @Field: CG: generator current
// @Field: Th: throttle input
// @Field: EB: EFI battery voltage
// @Field: RPM: generator RPM
// @Field: PW: EFI pulse-width
// @Field: FF: fuel flow
// @Field: FC: fuel consumed
// @Field: EP: EFI pressure
// @Field: EMT: EFI manifold air temperature
// @Field: CHT: cylinder head temperature
// @Field: TPS: throttle position sensor

    AP::logger().Write(
        "LOEG",
        "TimeUS," "I," "VB," "CB," "CG," "Th," "EB," "RPM," "PW," "FF," "FC," "EP," "EMT," "CHT," "TPS",
        "s"       "#"  "v"   "A"   "A"   "%"   "v"   "q"    "s"   "y"   "l"  "P"   "O"    "O"     "%"  ,
        "F"       "-"  "-"   "-"   "-"   "0"   "0"   "0"    "F"   "0"   "0"  "0"   "0"    "0"     "0"  ,
        "Q"       "B"  "f"   "f"   "f"   "f"   "f"   "f"    "f"   "f"   "f"  "f"   "f"    "f"     "f"  ,
        AP_HAL::micros64(),
        packet.efi_index,
        packet.volt_batt,
        packet.curr_batt,
        packet.curr_gen,
        packet.throttle,
        packet.efi_batt,
        packet.efi_rpm,
        packet.efi_pw,
        packet.efi_fuel_flow,
        packet.efi_fuel_consumed,
        packet.efi_baro,
        packet.efi_mat,
        packet.efi_clt,
        packet.efi_tps
        );
}

#endif
