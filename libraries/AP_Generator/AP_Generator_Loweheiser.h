// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Generator_Backend.h"

#if AP_GENERATOR_LOWEHEISER_ENABLED

#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include <stdint.h>

/*
   Example setup:
 param set SERIAL2_PROTOCOL 2  # Generator protocol
 param set SERIAL2_BAUD 115200
 param set GEN_TYPE 4
 param set EFI_TYPE 4
 param set RC9_OPTION 85  # generator
 param set RC10_OPTION 212  # loweheiser manual throttle
 long SET_MESSAGE_INTERVAL 225 100000  // emit EFI_STATUS at 10Hz
 */

class AP_Generator_Loweheiser : public AP_Generator_Backend
{
    friend class AP_EFI_Loweheiser;

public:
    // constructor
    using AP_Generator_Backend::AP_Generator_Backend;

    // init should be called at vehicle startup to get the generator
    // library ready
    void init(void) override;

    // update should be called regularly to update the generator state
    void update(void) override;

    // methods to control the desired generator state:
    bool stop(void) override;
    bool idle(void) override;
    bool run(void) override;

    // handle loweheiser_gov_efi and related MAVLink messages:
    void handle_mavlink_msg(const class GCS_MAVLINK &channel, const mavlink_message_t &msg);

    // method to send a GENERATOR_STATUS mavlink message, called by
    // the GCS_MAVLink library when it wants the message emitted:
    void send_generator_status(const class GCS_MAVLINK &channel) override;

    // prearm checks to ensure generator is good for arming:
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // healthy returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating an errors:
    bool healthy() const override;

    // parameters for the Loweheiser generator:
    static const struct AP_Param::GroupInfo var_info[];

    const struct AP_Param::GroupInfo *get_var_info() const override {
        return var_info;
    }

    // method to reset the amount of energy consumed by the generator.
    // This typically means someone has refueled the vehicle without
    // powering it off.
    bool reset_consumed_energy() override {
        accumulated_consumed_fuel_litres = 0.0f;
        return true;
    }

    // return the amount of fuel consumed from when capacity was last
    // reset
    float fuel_consumed() {
        if (last_packet_received_ms == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return accumulated_consumed_fuel_litres;
    }

private:

    // second init is called after parameters are loaded from eeprom;
    // init() before.
    bool second_init_done;
    void check_second_init();

    // we store the entirety of the most recent packet for the time being:
    mavlink_loweheiser_gov_efi_t packet;
    uint32_t last_packet_received_ms;

    mavlink_command_ack_t ack_packet;
    uint32_t last_ack_packet_ms;
    // record the timestamp of the most recently processed ack:
    uint32_t last_ack_packet_processed_ms;
    // good_ack is true if the last ack we received from the generator
    // reported success:
    bool good_ack;

    // state recorded from the first Loweheiser message we see so we
    // only process from one source:
    bool seen_good_message;
    const class GCS_MAVLINK *mavlink_channel;
    uint8_t sysid;
    uint8_t compid;
    uint8_t efi_index;

    // methods and state to record pilot desired runstate and actual runstate:
    enum class RunState {
        STOP = 17,
        IDLE = 18,
        WARMING_UP = 19,
        RUN = 20,
        COOLING_DOWN = 21,
    };
    enum class PilotDesiredRunState {
        STOP = 65,
        IDLE = 66,
        RUN = 67,
    };
    PilotDesiredRunState pilot_desired_runstate = PilotDesiredRunState::STOP;
    // last pilot desired runstate we acted on; forced to an invalid
    // value so that we action at least once.
    RunState commanded_runstate = RunState::STOP;  // output is based on this
    void set_pilot_desired_runstate(PilotDesiredRunState newstate);
    void update_runstate();
    const char *runstate_string(PilotDesiredRunState runstate);

    // returns true if the generator should be allowed to move into
    // the "run" (high-RPM) state:
    bool generator_ok_to_run() const;

    // returns true if the generator should be allowed to move into
    // the "stop" state.  This is used to enforce a cool-down period
    // for the device
    bool generator_ok_to_stop() const;

    // timestamp of last time we told the user we're waiting for the
    // generator to change temperature:
    uint32_t last_waiting_temperature_change_ms;

    // estop_reported is true if we do not need to tell the user of an
    // estop event if it occurs:
    bool estop_reported;

    // prepare and send commands to generator:
    void command_generator();

    // log data to onboard storage:
    void Log_Write();
    uint32_t last_logged_reading_ms;

    // state related to loweheuser-specific RC input to the generator:
    RC_Channel *rc_channel_manual_throttle;
    RC_Channel *rc_channel_starter_motor;
    uint32_t last_rc_channel_check;
    uint32_t last_start_time_ms;

    // user-configurable parameters:
    AP_Int32 time_until_maintenance;
    AP_Int32 total_runtime;
    AP_Float high_idle_throttle;
    AP_Float idle_throttle;
    AP_Float temp_required_for_run;
    AP_Float temp_required_for_idle;
    AP_Float temp_for_overtemp_warning;

    // update runtime and maintenance-required time
    void update_stats();
    uint32_t run_start_ms;
    bool was_running;
    uint32_t last_stats_saved_ms;
    uint32_t runtime_delta_ms;

    // update_common_backend_variables:  These are used by the base class
    // (AP_Generator_Backend) to provide data to the battery monitor
    // library.
    void update_common_backend_variables();

    float accumulated_consumed_fuel_litres;

    // should_emergency_stop - returns true if the generator must stop
    // immediately
    bool should_emergency_stop();

    // periodically checks parameters related to rc input and updates
    // some data sources appropriately:
    void check_rc_input_channels();

};

#endif  // AP_GENERATOR_LOWEHEISER_ENABLED
