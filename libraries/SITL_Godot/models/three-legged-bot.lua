-- Copyright (C) 2026 David Buzz
-- SPDX-License-Identifier: GPL-3.0-or-later
-- Part of ArduPilot and released under its licence: see COPYING.txt.

-- Walk models/three-legged-bot.gltf. Written by tools/make_hexapod.py: change the
-- legs there and write both again, not here. It walks at the throttle
-- ArduPilot asks for, and steers by a longer stride on one side.
local LEGS = 3
local JOINTS = 3
-- which side each leg is on: 1 left, -1 right, 0 on the centreline
local SIDE = {1, -1, 0}
-- and for a leg on the centreline, which cannot swing fore and aft,
-- whether it is ahead of the body or behind it: 1 or -1
local AHEAD = {1, 1, -1}
-- when in the stride each leg's turn comes, as a fraction of it
local TURN = {0.000, 0.333, 0.667}
-- how much of the stride a foot spends on the ground
local STANCE = 0.90
local SWING_DEG = 32        -- reach fore and aft at full throttle
local REACH_DEG = 18        -- and in and out, for a leg on the centreline
local LIFT_DEG = 10         -- how far a foot is picked up
local RANGE_DEG = 40.0    -- servo travel either side of centre
local KNEE_FOLD = 1.2       -- knee bend for each degree of lift
local STEPS_HZ = 1.2        -- strides a second at full throttle
local PERIOD_MS = 20

local THROTTLE = 3
local STEERING = 4
local phase = 0.0

local function pwm(deg)
    local v = 1500 + deg / RANGE_DEG * 500
    return math.floor(math.max(1000, math.min(2000, v)))
end

function update()
    local thr = vehicle:get_control_output(THROTTLE) or 0
    local steer = vehicle:get_control_output(STEERING) or 0
    if not arming:is_armed() then
        thr = 0
        steer = 0
    end
    local drive = math.max(math.abs(thr), math.abs(steer))
    phase = (phase + STEPS_HZ * math.min(drive, 1.0) * PERIOD_MS * 0.001) % 1.0
    for leg = 0, LEGS - 1 do
        -- where this leg is in its own stride: on the ground and
        -- sweeping back from 1 to -1, then in the air and coming
        -- forward again
        local u = (phase + TURN[leg + 1]) % 1.0
        local reach
        local lift = 0
        if u < STANCE then
            reach = 1.0 - 2.0 * u / STANCE
        else
            local s = (u - STANCE) / (1.0 - STANCE)
            reach = 2.0 * s - 1.0
            if drive > 0.03 then
                lift = math.sin(math.pi * s) * LIFT_DEG
            end
        end
        -- steering to the right is a longer stride on the left
        local side = SIDE[leg + 1]
        local stride = thr + side * steer
        stride = math.max(-1.0, math.min(1.0, stride))
        local swing = 0
        local knee = lift * KNEE_FOLD
        if side ~= 0 then
            swing = reach * SWING_DEG * stride
        elseif JOINTS == 3 then
            -- a leg on the centreline pushes by reaching out and
            -- drawing in, which is the knee's work
            knee = knee + reach * REACH_DEG * thr * AHEAD[leg + 1]
        end
        local chan = leg * JOINTS
        SRV_Channels:set_output_pwm_chan_timeout(chan, pwm(swing), 1000)
        SRV_Channels:set_output_pwm_chan_timeout(chan + 1, pwm(lift), 1000)
        if JOINTS == 3 then
            SRV_Channels:set_output_pwm_chan_timeout(chan + 2, pwm(knee), 1000)
        end
    end
    return update, PERIOD_MS
end

return update()
