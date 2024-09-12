#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_ENABLED

#include <AC_PrecLand/AC_PrecLand.h>
#include <AP_Math/AP_Math.h>

// This class constantly monitors what the status of the landing target is
// If it is not in sight, depending on user parameters, it decides what measures can be taken to bring the target back in sight
// If the target has been lost recently, the vehicle might try to retry the landing by going to the last known location of the target/going to the location where the target was last detected
// If we have no clue where the target might be, then failsafe measures are activated
// Failsafe measures can include stopping completely (hover), or landing vertically
class AC_PrecLand_StateMachine {
public:

    // Constructor
    AC_PrecLand_StateMachine() {
        init();
    };

    // Do not allow copies
    CLASS_NO_COPY(AC_PrecLand_StateMachine);

    // Initialize the state machine. This is called every time vehicle switches mode
    void init();

    // Current status of the precland state machine
    enum class Status: uint8_t {
        ERROR = 0,               // Unknown error
        DESCEND,                 // No action is required, just descend vertically
        RETRYING,                // Vehicle is attempting to retry landing
        FAILSAFE                 // Switch to prec landing failsafe
    };

    // FailSafe action needed
    enum class FailSafeAction: uint8_t {
        HOLD_POS = 0,            // Hold the current position of the vehicle
        DESCEND                  // Descend vertically
    };

    // Run Prec Land State Machine. During Prec Landing, we might encounter four scenarios:
    // 1. We had the target in sight, but have lost it now. 2. We never had the target in sight and user wants to land.
    // 3. We have the target in sight and can continue landing. 4. The sensor is out of range
    // This method deals with all of these scenarios
    // Returns the action needed to be done by the vehicle.
    // Parameters: Vector3f "retry_pos_m" is filled with the required location if we need to retry landing.
    Status update(Vector3f &retry_pos_m);

    // This is only called when the current status of the state machine returns "failsafe" and will return the action that the vehicle should do
    // At the moment this method only allows you to stop in air permanently, or land vertically
    // Failsafe will only trigger as a last resort
    FailSafeAction get_failsafe_actions();

    // Strictness that the user wants for Prec Landing
    enum class RetryStrictness: uint8_t {
        NOT_STRICT = 0,         // This is the behaviour on Copter 4.1 and below. The vehicle will land ASAP irrespective of target in sight or not
        NORMAL,                 // Vehicle will retry a failed prec landing; if the target isn't found, it will land vertically
        VERY_STRICT             // Same as above, except vehicle will never land if the target isn't found
    };

    // which retry action should be done
    enum class RetryAction: uint8_t {
        GO_TO_LAST_LOC = 0,     // Go to the last location where landing target was detected
        GO_TO_TARGET_LOC        // Go towards the location of the detected landing target
    };

private:

    // Target is lost (i.e we had it in sight some time back), this method helps decide on what needs to be done next
    // The chosen action depends on user set landing strictness and will be returned by this function
    // Parameters: Vector3f "retry_pos_m" is filled with the required location if we need to retry landing.
    Status get_target_lost_actions(Vector3f &retry_pos_m);

    // Retry landing based on a previously known location of the landing target
    // Returns the action that should be taken by the vehicle
    // Vector3f "retry_pos_m" is filled with the required location.
    Status retry_landing(Vector3f &retry_pos_m);

    // Reset the landing statemachine. This needs to be called every time the landing target is back in sight.
    // So that if the landing target goes out of sight again, we can start the failed landing procedure back from the beginning stage
    void reset_failed_landing_statemachine();

    // State machine for action to do when Landing target is lost (after it was in sight a while back)
    enum class TargetLostAction: uint8_t {
        INIT = 0,               // Decide on what action needs to be taken
        DESCEND,                // Descend for sometime (happens if we have just lost the target)
        LAND_VERTICALLY,        // Land vertically
        RETRY_LANDING,          // Retry landing (only possible if we had the landing target in sight sometime during the flight)
    };

    TargetLostAction landing_target_lost_action;  // Current action being done in the Lost Landing target state machine

    // State Machine for landing retry
    enum class RetryLanding : uint8_t {
        INIT = 0,               // Init the retry statemachine. This would involve increasing the retry counter (so we how many times we have already retried)
        IN_PROGRESS,            // Retry in progress, we wait for the vehicle to get close to the target location
        DESCEND,                // Descend to the original height from where we had started the retry
        COMPLETE                // Retry completed. We try failsafe measures after this
    };
    RetryLanding _retry_state;   // Current action being done in the Landing retry state machine
    uint8_t _retry_count;       // Total number of retires done in this mode

    bool failsafe_initialized;  // True if failsafe has been initalized
    uint32_t failsafe_start_ms; // timestamp of when failsafe was triggered

};

#endif // AC_PRECLAND_ENABLED
