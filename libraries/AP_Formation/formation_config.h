/*
   FlightDock Formation Controller Configuration

   Centralized configuration constants for the 400Hz C++ formation controller.
   Organized into logical namespaces for maintainability.

   Author: FlightDock Research Team
   Date: January 2026
   Refactored from formation_controller.h
*/

#pragma once

#include <AP_Math/AP_Math.h>

namespace FormationConfig {

// =========================================================================
// Build Identity
// =========================================================================
// Build version for FORM_E_BUILD beacon - set by build system
#ifndef FORM_E_BUILD_VERSION
#define FORM_E_BUILD_VERSION "7D.63"
#endif

// Feature flags for build beacon
constexpr bool FEATURE_CPP_STATION_KEEP = true;
constexpr bool FEATURE_OPTION_E = true;
constexpr bool FEATURE_TURN_FOLLOW = true;
constexpr bool FEATURE_AUTH_FAILFAST = true;

// =========================================================================
// Formation Geometry
// =========================================================================
namespace Geometry {
    constexpr float OFFSET_M = 50.0f;           // Trail distance (m)
    constexpr float TOLERANCE_M = 5.0f;         // Acceptable error (m)
    constexpr float PREDICT_TIME_S = 0.8f;      // Unified prediction horizon (s)
    constexpr float BAD_GEOM_THRESH_M = 100000.0f;  // Corrupt geometry threshold (100 km)
}

// =========================================================================
// Speed Control
// =========================================================================
namespace SpeedControl {
    constexpr float LEAD_SPEED_DEFAULT_MPS = 24.7f;   // Default leader TAS (m/s)
    constexpr float MAX_APPROACH_SPEED_MPS = 26.0f;   // Max approach speed
    constexpr float MIN_CLOSURE_RATE_MPS = 0.5f;      // Minimum closure rate
    constexpr float MAX_CLOSURE_RATE_MPS = 8.0f;      // Maximum closure rate
    constexpr float INTERCEPT_THRESHOLD_M = 70.0f;    // Range for intercept phase
    constexpr float TRANSITION_THRESHOLD_M = 50.0f;   // Range for transition phase
}

// =========================================================================
// Fine Control Zones (Test 359)
// =========================================================================
// Four-stage approach for contact docking:
//   STAGE 0: PRE-DECEL (2-30m)  - Graduated deceleration
//   STAGE 1: STANDOFF (0.5-2m)  - Velocity-gated hold
//   STAGE 2: CONTACT (<0.5m)    - Precision position hold
namespace FineControl {
    constexpr float ENABLE_RANGE_M = 30.0f;           // Python->C++ handoff
    constexpr float DISABLE_RANGE_M = 40.0f;          // C++->Python handoff (hysteresis)
    constexpr float PREDECEL_ZONE_OUTER_M = 30.0f;    // Pre-decel outer boundary
    constexpr float PREDECEL_ZONE_INNER_M = 2.0f;     // Pre-decel inner boundary
    constexpr float STANDOFF_DISTANCE_M = 2.0f;       // Velocity-gated standoff distance
    constexpr float STANDOFF_INNER_M = 0.5f;          // Contact zone boundary
    constexpr float CONTACT_ZONE_M = 0.5f;            // Contact success zone
    constexpr float STANDOFF_VC_THRESHOLD_MPS = 0.1f; // Max Vc to clear standoff
    constexpr float STANDOFF_DWELL_REQUIRED_S = 2.0f; // Time at low Vc to clear
    constexpr float PREDECEL_MAX_ADJ_MPS = 1.0f;      // Pre-decel authority
    constexpr float CONTACT_MAX_ADJ_MPS = 0.1f;       // Contact zone authority
    constexpr float PREDECEL_TARGET_VC_MPS = 0.05f;   // Target Vc at standoff entry
    constexpr float OVERTAKE_VC_THRESH_MPS = 0.5f;    // Emergency brake threshold
    constexpr float OVERTAKE_BRAKE_ADJ_MPS = 0.5f;    // Emergency brake adjustment
    constexpr uint32_t LOG_INTERVAL_MS = 500;         // Diagnostic log interval
}

// =========================================================================
// PD Controller Gains
// =========================================================================
// Range-dependent gain schedule for cross-track control
namespace PDGains {
    // Cross-track proportional gains (deg per meter)
    constexpr float K_CROSS_FAR_DEG_PER_M = 0.40f;    // range > 400m
    constexpr float K_CROSS_MID_DEG_PER_M = 0.50f;    // 200-400m
    constexpr float K_CROSS_NEAR_DEG_PER_M = 0.25f;   // range <= 200m

    // Derivative gain for cross-track damping
    constexpr float KD_CROSS_DEG_PER_M_S = 0.10f;     // deg*s/m

    // Cross-rate filter
    constexpr float CROSS_RATE_FILTER_TAU_S = 0.5f;   // Low-pass filter tau

    // Cross-rate spike rejection
    constexpr float MAX_CROSS_RATE_M_S = 80.0f;       // Max plausible cross rate
    constexpr float DT_MIN_S = 0.001f;                // Min valid dt (1ms)
    constexpr float DT_MAX_S = 0.10f;                 // Max valid dt (100ms)
    constexpr float MAX_D_TERM_DEG = 10.0f;           // Max D-term contribution
}

// =========================================================================
// Bank and Roll Limits
// =========================================================================
namespace BankLimits {
    constexpr float MAX_BANK_NORMAL_DEG = 35.0f;      // Normal max bank
    constexpr float MAX_BANK_TURN_DEG = 35.0f;        // Max bank during leader turns
    constexpr float MAX_ROLL_RATE_DEG_S = 30.0f;      // Legacy (unused)

    // Slew rate limits (Test 240)
    constexpr float ROLL_SLEW_RATE_NORMAL_DEG_S = 12.0f;   // Normal slew rate
    constexpr float ROLL_SLEW_RATE_BOOST_DEG_S = 15.0f;    // Boosted when saturating
    constexpr float ROLL_SLEW_BOOST_CROSS_THRESH_M = 80.0f; // Cross threshold for boost
    constexpr float ROLL_SLEW_BOOST_SAT_THRESH_PCT = 80.0f; // Saturation threshold
}

// =========================================================================
// Turn Following (Phase 7D.49)
// =========================================================================
namespace TurnFollow {
    constexpr float K_HDG_YR = 0.3f;                  // Heading error to yaw rate gain
    constexpr float YR_DES_MAX_DEG_S = 20.0f;         // Max desired yaw rate
    constexpr float HDG_ERR_TURN_ON_DEG = 15.0f;      // Heading error threshold
    constexpr float HDG_ERR_BLEND_FULL_DEG = 25.0f;   // Heading error for full blend
    constexpr float BANK_RATE_DEG_S = 20.0f;          // Bank rate limit in turn mode
    constexpr float SAFETY_FAR_RANGE_M = 300.0f;      // Block turn-follow beyond this
    constexpr float SAFETY_MAX_CROSS_M = 80.0f;       // Block turn-follow above this

    // Weight collapse fix (Phase 7D.59)
    constexpr float W_MIN = 0.30f;                    // Minimum weight when active
    constexpr float BANK_FF_MIN_DEG = 0.5f;           // Min |bank_turn| to enforce w_min
    constexpr float CROSS_MIN_NAV_ROLL_M = 20.0f;     // Cross threshold for min roll
    constexpr float CROSS_MIN_ROLL_DEG = 5.0f;        // Minimum nav_roll

    // Turn detection thresholds (Phase 7D.60)
    constexpr float YR_THRESH_ON_DEG_S = 0.5f;        // Yaw rate turn-on threshold
    constexpr float YR_THRESH_OFF_DEG_S = 0.3f;       // Yaw rate turn-off threshold
    constexpr float CROSS_RATE_THRESH_M_S = 3.0f;     // Cross rate for turn detection

    // Phase detection
    constexpr float TURN_PHASE_YAW_RATE_THRESH_DEG_S = 0.5f;
    constexpr float TURN_PHASE_FF_THRESH_DEG_S = 2.0f;
    constexpr float MAX_HEADING_RATE_NORMAL_DEG_S = 35.0f;
    constexpr float MAX_HEADING_RATE_TURN_DEG_S = 75.0f;

    // Turn feed-forward
    constexpr float TURN_FF_GAIN = 1.0f;
}

// =========================================================================
// Cross Divergence Protection (Phase 7D.60)
// =========================================================================
namespace CrossProtect {
    constexpr float ON_M = 50.0f;                     // Enable above this cross
    constexpr float OFF_M = 30.0f;                    // Disable below this cross
    constexpr float MIN_ROLL_DEG = 5.0f;              // Minimum corrective roll
    constexpr float MAX_ROLL_DEG = 25.0f;             // Maximum corrective roll
}

// =========================================================================
// Sign Divergence Brake + Panic (Test 233/234)
// =========================================================================
namespace SignDiverge {
    constexpr float CROSS_THRESH_M = 200.0f;          // Min |cross| to check
    constexpr float ROLL_THRESH_DEG = 5.0f;           // Min |roll| to check
    constexpr uint32_t PERSIST_MS = 2000;             // 2.0s continuous
    constexpr uint32_t BRAKE_DURATION_MS = 3000;      // 3.0s brake hold
    constexpr uint32_t PANIC_DURATION_MS = 10000;     // 10.0s panic hold
    constexpr uint32_t PANIC_WINDOW_MS = 30000;       // 30s rolling window
    constexpr uint8_t PANIC_TRIGGER_COUNT = 3;        // 3 triggers => panic
}

// =========================================================================
// Out of Engage Safety Guard (Test 238)
// =========================================================================
namespace OutOfEngage {
    constexpr float RANGE_M = 1500.0f;                // Max range for PD control
    constexpr float BEHIND_MIN_M = -500.0f;           // Min behind (allow small ahead)
    constexpr uint32_t LOG_INTERVAL_MS = 2000;        // Log every 2s
    constexpr uint32_t PERSIST_MS = 2000;             // 2s persistence
    constexpr float HYSTERESIS_M = 200.0f;            // 200m hysteresis to clear
}

// =========================================================================
// Direct Throttle Control (Test 360-367)
// =========================================================================
namespace DirectThrottle {
    constexpr float BASE_THROTTLE_PCT = 65.0f;        // Base throttle
    constexpr float KP_AIRSPEED_THROTTLE = 5.0f;      // P-gain for airspeed error
    constexpr float KP_VC_THROTTLE = 2.0f;            // P-gain for Vc error
    constexpr float MIN_THROTTLE_PCT = 20.0f;         // Safety floor
    constexpr float MAX_THROTTLE_PCT = 80.0f;         // Safety ceiling
    constexpr float ENABLE_BEHIND_M = 30.0f;          // Enable at 30m behind

    // Vc targets by zone
    constexpr float VC_TARGET_FAR_MPS = 0.5f;         // 30-10m: 0.5 m/s
    constexpr float VC_TARGET_MID_MPS = 0.2f;         // 10-2m: 0.2 m/s
    constexpr float VC_TARGET_NEAR_MPS = 0.0f;        // <2m: hold position
    constexpr float VC_ZONE_FAR_M = 10.0f;            // Far zone boundary
    constexpr float VC_ZONE_MID_M = 2.0f;             // Mid zone boundary

    // Altitude floor protection
    constexpr float ALT_FLOOR_DESCENT_RATE_MPS = 3.0f;
    constexpr float ALT_FLOOR_PITCH_DEG = 5.0f;

    // Diverge detection
    constexpr float DIVERGE_THRESHOLD_MPS = -0.3f;    // Abort if Vc < -0.3
    constexpr uint32_t DIVERGE_TIMEOUT_MS = 3000;     // For 3 seconds

    // Logging
    constexpr uint32_t LOG_INTERVAL_MS = 500;
}

// =========================================================================
// Pitch Control (Test 361)
// =========================================================================
namespace PitchControl {
    constexpr float CLOSING_DEG = -1.5f;              // Nose down for closing
    constexpr float FINE_DEG = 0.0f;                  // Level for speed match
    constexpr float CONTACT_DEG = 1.0f;               // Nose up to decelerate
    constexpr float ZONE_FAR_M = 10.0f;               // Closing zone boundary
    constexpr float ZONE_NEAR_M = 2.0f;               // Contact zone boundary
}

// =========================================================================
// Altitude Control
// =========================================================================
namespace AltitudeControl {
    constexpr float MAX_ALT_ERROR_M = 10.0f;          // +/-10m altitude error clamp
    constexpr float KP_ALT = 0.5f;                    // Conservative P gain
    constexpr float MAX_PITCH_DEG = 15.0f;            // +/-15 deg pitch limit
}

// =========================================================================
// Sensor Fusion
// =========================================================================
namespace SensorFusion {
    constexpr float UWB_TIMEOUT_MS = 500.0f;          // 500ms UWB timeout
    constexpr float UWB_MAX_RANGE_M = 200.0f;         // Max effective UWB range
    constexpr float SMOOTH_ALPHA = 0.3f;              // Smoothing filter coefficient
    constexpr uint32_t LEAD_STALE_THRESH_MS = 500;    // Lead telemetry freshness
}

// =========================================================================
// Timing and Loop Rate
// =========================================================================
namespace Timing {
    constexpr float LOOP_HZ = 400.0f;                 // 400 Hz update rate
    constexpr uint32_t BUILD_BEACON_INTERVAL_MS = 2000;
    constexpr uint32_t BUILD_BEACON_WINDOW_MS = 20000;
}

} // namespace FormationConfig
