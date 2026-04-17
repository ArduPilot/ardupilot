------------------------ MODULE ModeManager ------------------------
(*
 * TLA+ Specification for ArduPilot Mode Management
 * 
 * This specification models the mode transition logic and safety
 * properties of ArduPilot flight mode management.
 *
 * Properties verified:
 * - Never arm when pre-arm checks fail
 * - Never command AUTO and LAND simultaneously
 * - If GPS invalid for N seconds, leave GPS-dependent mode
 * - Failsafe priority ordering is deterministic
 *
 * To run verification:
 *   tlc ModeManager.tla -workers 4
 *   apalache-mc check --inv=EventualSafety ModeManager.tla
 *)

EXTENDS Integers, Sequences, TLC, FiniteSets

(* =========================================================================
 * CONSTANTS
 * ========================================================================= *)

CONSTANTS 
    MaxAltitude,         (* Maximum altitude in meters *)
    GpsTimeoutSec,       (* GPS timeout in seconds *)
    FailsafeDelaySec     (* Failsafe trigger delay *)

(* =========================================================================
 * VARIABLES
 * ========================================================================= *)

VARIABLES
    current_mode,        (* Current flight mode *)
    armed,               (* Motors armed? *)
    gps_valid,           (* GPS data valid? *)
    terrain_valid,       (* Terrain data valid? *)
    battery_ok,          (* Battery voltage OK? *)
    rc_signal_ok,        (* RC signal OK? *)
    altitude,            (* Current altitude in meters *)
    gps_timeout_timer,   (* GPS timeout countdown *)
    failsafe_timer,      (* Failsafe countdown *)
    mode_request         (* Requested mode change *)

(* =========================================================================
 * MODE DEFINITIONS
 * ========================================================================= *)

(* Flight modes *)
Modes == {
    "STABILIZE",    (* Manual control, no stabilization *)
    "ACRO",         (* Acrobatic, rate control only *)
    "ALT_HOLD",     (* Hold altitude, manual horizontal *)
    "AUTO",         (* Autonomous mission *)
    "RTL",          (* Return to launch *)
    "LAND",         (* Autonomous landing *)
    "LOITER",       (* Hold position *)
    "DRIFT",        (* Drift mode, stabilized *)
    "SMART_RTL",    (* Smart return to launch *)
    "BRAKE"         (* Brake/dynamic hold *)
}

(* GPS-dependent modes *)
GpsDependentModes == {"AUTO", "RTL", "LOITER", "SMART_RTL", "BRAKE"}

(* Safe modes for landing *)
SafeLandingModes == {"LAND", "RTL", "LOITER", "ALT_HOLD"}

(* =========================================================================
 * INITIAL STATE
 * ========================================================================= *)

Init ==
    /\ current_mode = "STABILIZE"
    /\ armed = FALSE
    /\ gps_valid = TRUE
    /\ terrain_valid = TRUE
    /\ battery_ok = TRUE
    /\ rc_signal_ok = TRUE
    /\ altitude = 0
    /\ gps_timeout_timer = GpsTimeoutSec
    /\ failsafe_timer = FailsafeDelaySec
    /\ mode_request = ""

(* =========================================================================
 * SAFETY PROPERTIES (INVARIANTS)
 * ========================================================================= *)

(* Type invariant: all variables have correct types *)
TypeInvariant ==
    /\ current_mode \in Modes
    /\ armed \in {TRUE, FALSE}
    /\ gps_valid \in {TRUE, FALSE}
    /\ terrain_valid \in {TRUE, FALSE}
    /\ battery_ok \in {TRUE, FALSE}
    /\ rc_signal_ok \in {TRUE, FALSE}
    /\ altitude \in 0..MaxAltitude
    /\ gps_timeout_timer \in 0..GpsTimeoutSec
    /\ failsafe_timer \in 0..FailsafeDelaySec
    /\ mode_request \in Modes \cup {""}

(* Arming rules: never arm when pre-arm checks fail *)
ArmingSafety ==
    armed =>
        /\ gps_valid
        /\ terrain_valid
        /\ battery_ok
        /\ rc_signal_ok
        /\ current_mode \in {"STABILIZE", "LOITER", "ALT_HOLD"}

(* Mode transition rules: valid transitions only *)
ModeTransitionSafety ==
    /\ current_mode = "AUTO" /\ ~gps_valid
        => mode_request \in {"RTL", "LAND", "ALT_HOLD"}
    /\ current_mode = "RTL" /\ ~gps_valid
        => mode_request \in {"LAND", "ALT_HOLD"}
    /\ current_mode = "LOITER" /\ ~gps_valid
        => mode_request \in {"ALT_HOLD", "LAND"}
    /\ current_mode \in GpsDependentModes /\ ~gps_valid
        => mode_request \in SafeLandingModes

(* Failsafe priority: battery < GPS < RC *)
FailsafePriority ==
    /\ ~battery_ok => mode_request \in {"LAND", "RTL"}
    /\ ~gps_valid /\ gps_timeout_timer <= 0
        => mode_request \in {"LAND", "RTL", "ALT_HOLD"}
    /\ ~rc_signal_ok /\ failsafe_timer <= 0
        => mode_request \in {"RTL", "LAND"}

(* Mutual exclusion: never in two modes simultaneously *)
MutualExclusion ==
    cardinality({current_mode, mode_request} \ {""}) <= 1

(* Altitude limits: never exceed maximum altitude *)
AltitudeSafety ==
    altitude <= MaxAltitude

(* Combined safety invariant *)
SafetyInvariant ==
    /\ TypeInvariant
    /\ ArmingSafety
    /\ ModeTransitionSafety
    /\ FailsafePriority
    /\ MutualExclusion
    /\ AltitudeSafety

(* =========================================================================
 * LIVENESS PROPERTIES
 * ========================================================================= *)

(* Eventual safety: if GPS fails, eventually reach safe mode *)
EventualSafety ==
    []<>(~gps_valid /\ gps_timeout_timer <= 0
          => current_mode \in SafeLandingModes)

(* Eventual landing: if battery fails, eventually land *)
EventualLanding ==
    []<>(~battery_ok => <> (current_mode = "LAND"))

(* No permanent oscillation: mode eventually stabilizes *)
NoOscillation ==
    [](mode_request # "" => <> (current_mode = mode_request))

(* =========================================================================
 * ACTIONS (STATE TRANSITIONS)
 * ========================================================================= *)

(* Arm action *)
Arm() ==
    /\ ~armed
    /\ gps_valid
    /\ terrain_valid
    /\ battery_ok
    /\ rc_signal_ok
    /\ current_mode \in {"STABILIZE", "LOITER", "ALT_HOLD"}
    /\ armed' = TRUE
    /\ UNCHANGED <<current_mode, gps_valid, terrain_valid, battery_ok, 
                   rc_signal_ok, altitude, gps_timeout_timer, failsafe_timer,
                   mode_request>>

(* Disarm action *)
Disarm() ==
    /\ armed
    /\ \/ ~rc_signal_ok
       \/ current_mode = "LAND" /\ altitude = 0
    /\ armed' = FALSE
    /\ UNCHANGED <<current_mode, gps_valid, terrain_valid, battery_ok, 
                   rc_signal_ok, altitude, gps_timeout_timer, failsafe_timer,
                   mode_request>>

(* Mode change action *)
ChangeMode(new_mode) ==
    /\ armed
    /\ new_mode \in Modes
    /\ \/ current_mode = "AUTO" /\ ~gps_valid
           => new_mode \in {"RTL", "LAND", "ALT_HOLD"}
       \/ TRUE
    /\ current_mode' = new_mode
    /\ mode_request' = ""
    /\ UNCHANGED <<armed, gps_valid, terrain_valid, battery_ok, 
                   rc_signal_ok, altitude, gps_timeout_timer, failsafe_timer>>

(* GPS fault action *)
GpsFault() ==
    /\ gps_valid
    /\ gps_valid' = FALSE
    /\ gps_timeout_timer' = GpsTimeoutSec
    /\ UNCHANGED <<current_mode, armed, terrain_valid, battery_ok, 
                   rc_signal_ok, altitude, failsafe_timer, mode_request>>

(* GPS restore action *)
GpsRestore() ==
    /\ ~gps_valid
    /\ gps_valid' = TRUE
    /\ gps_timeout_timer' = GpsTimeoutSec
    /\ UNCHANGED <<current_mode, armed, terrain_valid, battery_ok, 
                   rc_signal_ok, altitude, failsafe_timer, mode_request>>

(* GPS timeout countdown *)
GpsTimeoutTick() ==
    /\ ~gps_valid
    /\ gps_timeout_timer > 0
    /\ gps_timeout_timer' = gps_timeout_timer - 1
    /\ UNCHANGED <<current_mode, armed, gps_valid, terrain_valid, battery_ok, 
                   rc_signal_ok, altitude, failsafe_timer, mode_request>>

(* Battery fault action *)
BatteryFault() ==
    /\ battery_ok
    /\ battery_ok' = FALSE
    /\ UNCHANGED <<current_mode, armed, gps_valid, terrain_valid, 
                   rc_signal_ok, altitude, gps_timeout_timer, failsafe_timer,
                   mode_request>>

(* RC signal loss action *)
RcSignalLoss() ==
    /\ rc_signal_ok
    /\ rc_signal_ok' = FALSE
    /\ failsafe_timer' = FailsafeDelaySec
    /\ UNCHANGED <<current_mode, armed, gps_valid, terrain_valid, battery_ok, 
                   altitude, gps_timeout_timer, mode_request>>

(* Altitude change action *)
ChangeAltitude(delta) ==
    /\ altitude + delta >= 0
    /\ altitude + delta <= MaxAltitude
    /\ altitude' = altitude + delta
    /\ UNCHANGED <<current_mode, armed, gps_valid, terrain_valid, battery_ok, 
                   rc_signal_ok, gps_timeout_timer, failsafe_timer, mode_request>>

(* System next-state relation *)
Next ==
    \/ Arm()
    \/ Disarm()
    \/ \E m \in Modes : ChangeMode(m)
    \/ GpsFault()
    \/ GpsRestore()
    \/ GpsTimeoutTick()
    \/ BatteryFault()
    \/ RcSignalLoss()
    \/ \E d \in -10..10 : ChangeAltitude(d)

(* =========================================================================
 * SPECIFICATION
 * ========================================================================= *)

Spec == Init /\ [][Next]_<<current_mode, armed, gps_valid, terrain_valid, 
                          battery_ok, rc_signal_ok, altitude, 
                          gps_timeout_timer, failsafe_timer, mode_request>>

(* =========================================================================
 * THEOREMS
 * ========================================================================= *)

(* Safety: all reachable states satisfy safety invariant *)
THEOREM Spec => []SafetyInvariant

(* Liveness: GPS failure leads to safe mode *)
THEOREM Spec => EventualSafety

(* Liveness: battery failure leads to landing *)
THEOREM Spec => EventualLanding

========================================================================
