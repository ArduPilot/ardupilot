#!/bin/bash
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

# One autotest per Godot backend airframe; the list builds each vehicle
# before its tests. usage: test-all.sh [autotest.py options]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

# cores a build may use; with no limit it takes every core on the machine
BUILD_JOBS=${BUILD_JOBS:-6}

# Speedup a test asks SITL for: ten keeps a test watchable. The physics
# sets its own ceiling under this.
SPEEDUP=${SPEEDUP:-10}

# Seconds a test may run before timeout kills its process group, SITL
# and backend included. It is there to catch a hang, so tests which fly
# for longer get their own limit rather than raising it for all.
TEST_TIMEOUT=120
declare -A TEST_TIMEOUTS=(
    [test.QuadPlane.Mission]=600
    [test.Blimp.FourMotorHold]=300
    [test.Blimp.FlyManualFourMotor]=240
    [test.Plane.AerobaticsScripting]=240
    [test.BalanceBot.DriveRTL]=240
)

# "test" flies the model the frame chooses; "test|model" names one of
# the backend's models instead, which is how the converted RealFlight
# vehicles get flown.
TESTS=(
    "build.Copter"
    "test.Copter.TakeoffAlt"          # quadx, the quadcopter
    "build.Helicopter"                # its own binary, arducopter-heli
    "test.Helicopter.PosHoldTakeOff"  # heli, the single-rotor helicopter
    "build.Plane"
    "test.Plane.TakeoffTakeoff1"      # plane, thrown off a catapult sled
    "test.Plane.AerobaticsScripting"  # plane3d.
    "test.Plane.TakeoffBungee"        # plane, hauled up a bungee ramp
    "test.Plane.GliderDrop"           # glider, let go at 400 m with no motor
    "test.QuadPlane.Mission"          # quadplane: VTOL takeoff, cruise, VTOL
                                      # landing. It flies the plane binary, so
                                      # it has to run before the next build
                                      # step configures and cleans it away
    "build.Rover"
    "test.Rover.DriveRTL"             # buggy
    "test.Rover.DriveMotorBoat"       # speedboat, on the lake
    "test.BalanceBot.DriveRTL"        # balancebot: two wheels and nothing
                                      # else, so it falls over if the
                                      # physics is wrong
    "test.Sailboat.DriveRTL"          # sailboat: no engine, a run and then
                                      # a reach on the wind alone
    "test.Hexapod.Walk"               # hexapod: eighteen servos on jointed
                                      # legs, walked by a gait script
    "build.Sub"
    "test.Sub.DiveMission"            # vectored ROV: dives fifty metres in
                                      # AUTO, then flies two short legs at
                                      # that depth
    "build.Tracker"
    "test.Tracker.TrackPlane"         # antenna tracker: follows an aeroplane
                                      # circling overhead, round and round
    # This one passes too: it swings the dish to five fixed points.
    #"test.Tracker.Tracking"
    "build.Blimp"
    "test.Blimp.FourMotorHold"        # blimp, braking and holding station in
                                      # LOITER on its thrusters
    # Passes too, in MANUAL; the finned frame's flapping is not modelled.
    #"test.Blimp.FlyManualFourMotor"
    #todo add more tests here that work with godot:

)

KNOWN_MARGINAL=()

FAILED=()
MARGINAL=()
for entry in "${TESTS[@]}"; do
    t=${entry%%|*}
    model=${entry#*|}
    model=${model#"$t"}          # no "|" in the entry: no model named
    model_arg=()
    label="$t"
    if [ -n "$model" ]; then
        model_arg=(--godot-model "$model")
        label="$t on $model"
    fi
    echo "==================== $label"
    rc=0
    if [[ $t == build.* ]]; then
        # a build is not a test: it wants no physics backend, and a cold
        # one takes far longer than a test is allowed to run
        Tools/autotest/autotest.py -j"${BUILD_JOBS:-6}" "$t" || rc=$?
    else
        timeout --kill-after=10 "${TEST_TIMEOUTS[$t]:-$TEST_TIMEOUT}" \
            Tools/autotest/autotest.py --godot --speedup="$SPEEDUP" \
            ${model_arg[@]+"${model_arg[@]}"} \
            "$@" "$t" || rc=$?
    fi
    if [ "$rc" -ne 0 ]; then
        if [ "$rc" -eq 124 ] || [ "$rc" -eq 137 ]; then
            echo "$label: killed after ${TEST_TIMEOUTS[$t]:-$TEST_TIMEOUT}s"
        fi
        if [[ " ${KNOWN_MARGINAL[*]} " == *" $entry "* ]]; then
            MARGINAL+=("$label")
        else
            FAILED+=("$label")
        fi
    fi
done

echo "===================="
if [ ${#MARGINAL[@]} -gt 0 ]; then
    echo "known marginal, not counted:"
    printf '  %s\n' "${MARGINAL[@]}"
fi
if [ ${#FAILED[@]} -eq 0 ]; then
    echo "all counted tests passed (${#TESTS[@]} run)"
    exit 0
fi
echo "${#FAILED[@]} of ${#TESTS[@]} failed:"
printf '  %s\n' "${FAILED[@]}"
exit 1
