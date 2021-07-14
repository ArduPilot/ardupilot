//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

static const float MAX_POS_ERROR_CM = 35.0f;  // Maximum possition errors horizontally and vertically for retry locations
static const uint32_t FAILSAFE_INIT_TIMEOUT_MS = 7000;   // Timeout in ms before failsafe measures are started. During this period vehicle is completely stopped to give user the time to take over
static const float RETRY_OFFSET_ALT_CM = 70.0f;  // This gets added to the altitude of the retry location

Mode::PrecLand_StateMachine Mode::precland_statemachine;

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;

    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm_glitch_protected;
    }

    precland.update(height_above_ground_cm, rangefinder_alt_ok());
}


void Mode::PrecLand_StateMachine::init()
{
    // init is only called ONCE per mode change. So in a particuar mode we can retry only a finite times.
    // The counter will be reset if the statemachine is called from a different mode
    _retry_count = 0;
    // reset every other statemachine
    reset_failed_landing_statemachines();
}

// Reset the landing statemachines. This needs to be called everytime the landing target is back in sight.
// So that if the landing target goes out of sight again, we can start the failed landing procedure back from the beginning stage
void Mode::PrecLand_StateMachine::reset_failed_landing_statemachines()
{
    landing_target_lost_action = TargetLostAction::INIT;
    retry_state = RetryLanding::INIT;
    failsafe_initialized = false;
}

// Run Prec Land State Machine. During Prec Landing, we might encounter four scenarios:
// 1. We had the target in sight, but have lost it now. 2. We never had the target in sight and user wants to land.
// 3. We have the target in sight and can continue landing. 4. The sensor is out of range
// This method deals with all of these scenarios
void Mode::PrecLand_StateMachine::update_precland_state_machine()
{
    // grab the current status of Landing Target
    AC_PrecLand::PldState current_state =  copter.precland.get_plnd_target_status();

    switch (current_state) {
    case AC_PrecLand::PldState::TARGET_RECENTLY_LOST:
        // we have lost the target but had it in sight at least once recently
        // action will depend on what user wants
        do_target_lost_actions();
        break;

    case AC_PrecLand::PldState::TARGET_NEVER_SEEN:
        // we have no clue where we are supposed to be landing
        // let user decide how strict our failsafe actions need to be
        do_failsafe_actions();
        break;

    case AC_PrecLand::PldState::TARGET_OUT_OF_RANGE:
        // The target isn't in sight, but we can't run any fail safe measures or do landing retry
        // Therefore just descend for now, and check again later if retry is allowed
    case AC_PrecLand::PldState::TARGET_FOUND:
        // no action required, target is in sight
        copter.flightmode->run_land_controllers();
        reset_failed_landing_statemachines();
        break;
    }
}


// Target is lost (i.e we had it in sight some time back), this method helps decide on what needs to be done next
// The chosen action depends on user set landing strictness
void Mode::PrecLand_StateMachine::do_target_lost_actions()
{
    switch (landing_target_lost_action) {
    case TargetLostAction::INIT:
        {
        // figure out how strict the user is with the landing
        PldRetryStrictness strictness = static_cast<PldRetryStrictness>(copter.precland.get_plnd_retry_strictness());
        switch (strictness) {
            case PldRetryStrictness::NORMAL:
            case PldRetryStrictness::VERY_STRICT:
                // We eventually want to retry landing, but lets descend for some time and hope the target gets in sight
                // If not, we will retry landing
                landing_target_lost_action = TargetLostAction::DESCEND;
                break;
            case PldRetryStrictness::NOT_STRICT:
                // User just wants to land, prec land isn't a priority
                landing_target_lost_action = TargetLostAction::LAND_VERTICALLY;
                break;
        }
        }
        break;

    case TargetLostAction::DESCEND:
        copter.flightmode->run_land_controllers();
        if (AP_HAL::millis() - copter.precland.get_last_precland_output_ms() >= copter.precland.get_min_retry_time_sec() * 1000) {
            // we have descended for some time and the target still isn't in sight
            // lets retry
            landing_target_lost_action = TargetLostAction::RETRY_LANDING;
            retry_state = RetryLanding::INIT;
        }
        break;

    case TargetLostAction::RETRY_LANDING:
        // retry the landing by going to the last known horizontal position of the target
        retry_landing();
        break;

    case TargetLostAction::LAND_VERTICALLY:
        // Just land vertically
        copter.flightmode->run_land_controllers();
        break;
    }
}

// Retry landing based on a previously known location of the landing target
void Mode::PrecLand_StateMachine::retry_landing()
{
    if (_retry_count > copter.precland.get_max_retry_allowed() || copter.precland.get_max_retry_allowed() == 0) {
        // we have exhausted the amount of times vehicle was allowed to retry landing
        // do failsafe measure so the vehicle isn't stuck in a constant loop
        do_failsafe_actions();
        return;
    }

    if (copter.flightmode->land_repo_active()) {
        // user has tried to take control
        // we will not allow any retries at this moment, do a failsafe instead
        do_failsafe_actions();
        return;
    }

    // get the retry location. This depends on what retry behavior has been set by user
    Vector3f go_to_location;
    RetryAction retry_action = static_cast<RetryAction>(copter.precland.get_plnd_retry_type());
    if (retry_action == RetryAction::GO_TO_TARGET_LOC) {
        copter.precland.get_last_detected_landing_location(go_to_location);
    } else if (retry_action == RetryAction::GO_TO_LAST_LOC) {
        copter.precland.get_last_pos_when_target_detected(go_to_location);
    }

    // add a little bit offset so the vehicle climbs slightly higher than where it was
    // remember this is "D" frame and in cm's
    go_to_location.z -= RETRY_OFFSET_ALT_CM;

    switch (retry_state) {
    case RetryLanding::INIT:
        // Init the Retry
        _retry_count ++;
        // initialise the position controller
        copter.pos_control->init_z_controller_no_descent();
        copter.pos_control->init_xy_controller();
        copter.flightmode->land_retry_position(go_to_location);
        retry_state = RetryLanding::IN_PROGRESS;
        // inform the user what we are doing
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Retrying Precision Landing");
        break;

    case RetryLanding::IN_PROGRESS:
    {
        // continue converging towards the target till we are close by
        copter.flightmode->land_retry_position(go_to_location);
        const float dist_to_target_xy = copter.pos_control->get_pos_error_xy_cm();
        const float dist_to_target_z = copter.pos_control->get_pos_error_z_cm();
        if ((dist_to_target_xy < MAX_POS_ERROR_CM) && (fabsf(dist_to_target_z) < MAX_POS_ERROR_CM)) {
            // we have approx reached landing location previously detected
            retry_state = RetryLanding::COMPLETE;
            copter.gcs().send_text(MAV_SEVERITY_INFO, "Landing Retry Completed");
        }
        break;
    }

    case RetryLanding::COMPLETE:
        // Vehicle has completed a retry, and most likely the landing location still isn't sight
        // we have no choice but to force a failsafe action
        do_failsafe_actions();

        break;
    }
}

// Stop landing (hover)
void Mode::PrecLand_StateMachine::stop_landing()
{
    // make use of the same position controller as the "landing controllers", sending the argument "true" will pause the decend
    copter.flightmode->run_land_controllers(true);
}

// Decide what the next action is going to be if we have hit a failsafe
// Failsafe will only trigger as a last resort
void Mode::PrecLand_StateMachine::do_failsafe_actions()
{
    if (!failsafe_initialized) {
        // start the timer
        failsafe_start_ms = AP_HAL::millis();
        // initialise the position controller
        copter.pos_control->init_z_controller_no_descent();
        copter.pos_control->init_xy_controller_stopping_point();
        failsafe_initialized = true;
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Starting Precision Landing Failsafe Measures");
    }

    // Depending on the strictness we will either land vertically, wait for some time and then land vertically, not land at all
    PldRetryStrictness strictness = static_cast<PldRetryStrictness>(copter.precland.get_plnd_retry_strictness());
    switch (strictness) {
    case PldRetryStrictness::VERY_STRICT:
        // user does not want to land on anything but the target
        // stop landing (hover)
        stop_landing();
        break;

    case PldRetryStrictness::NORMAL:
        if (AP_HAL::millis() - failsafe_start_ms < FAILSAFE_INIT_TIMEOUT_MS) {
            // stop the vehicle for at least a few seconds before descending
            // this might give user the chance to take over
            // we do not want to be too linent in landing vertically because of the strictness set by the user
            stop_landing();
            break;
        }
        copter.flightmode->run_land_controllers();
        break;

    case PldRetryStrictness::NOT_STRICT:
        // User wants to prioritize landing over staying in the air
        copter.flightmode->run_land_controllers();
        break;
    }
}

#endif
