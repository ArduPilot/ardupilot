-- This script will preform a control surface doublet
-- The desired axis to perform a doublet on is selected and started with a momentary switch
-- The plane maintains trim conditions while the switch is activated using servo overrides
-- If the momentary switch is released before "DOUBLET FINISHED" is seen on the GCS,
-- the aircraft recovers to FBWA mode to assist the pilot.
-- Magnitude and duration of the doublet can also be controlled.
-- It is suggested to allow the aircraft to trim for straight, level, unaccelerated flight (SLUF) in FBWB mode before
-- starting a doublet
-- Charlie Johnson, Oklahoma State University 2020

local DOUBLET_ACTION_CHANNEL = 6 -- RCIN channel to start a doublet when high (>1700)
local DOUBLET_CHOICE_CHANNEL = 7 -- RCIN channel to choose elevator (low) or rudder (high)
local DOUBLET_FUCNTION = 19 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen
-- A (Servo 1, Function 4), E (Servo 2, Function 19), and R (Servo 4, Function 21)
local DOUBLET_MAGNITUDE = 6 -- defined out of 45 deg used for set_output_scaled
local DOUBLET_TIME = 500 -- period of doublet signal in ms

-- flight mode numbers for plane https://mavlink.io/en/messages/ardupilotmega.html
local MODE_MANUAL = 0
local MODE_FBWA = 5
local MODE_FBWB = 6
local MODE_RTL = 11
local K_AILERON = 4
local K_ELEVATOR = 19
local K_THROTTLE = 70
local K_RUDDER = 21

-- store timing information during doublet
local start_time = -1
local end_time = -1
local now = -1
local desired_mode = -1

-- store information about the doublet channel
local doublet_srv_chan = SRV_Channels:find_channel(DOUBLET_FUCNTION)
local doublet_srv_min = param:get("SERVO" .. doublet_srv_chan + 1 .. "_MIN")
local doublet_srv_max = param:get("SERVO" .. doublet_srv_chan + 1 .. "_MAX")
local doublet_srv_trim = param:get("SERVO" .. doublet_srv_chan + 1 .. "_TRIM")
local pre_doublet_mode = vehicle:get_mode()

function retry_set_mode(mode)
    if vehicle:set_mode(mode) then
        -- if the mode was set successfully, carry on as normal
        desired_mode = -1
        return doublet, 1
    else
        -- if the mode was not set successfully, try again ASAP
        desired_mode = mode
        return retry_set_mode, 1
    end
end

function doublet()
    local callback_time = 100
    if arming:is_armed() == true and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 and end_time ==-1 then
        -- try for the best timing possible while performing a doublet
        callback_time = DOUBLET_TIME / 10
        -- start a quick doublet based on some math/logic
        now = millis()
        if start_time == -1 then
            -- this is the time that we started
            -- stop time will be start_time + DOUBLET_TIME
            start_time = now
            pre_doublet_mode = vehicle:get_mode()
            -- are we doing a doublet on elevator or rudder? set the other controls to trim
            local doublet_choice_pwm = rc:get_pwm(DOUBLET_CHOICE_CHANNEL)
            local trim_funcs = {}
            local pre_doublet_elevator = SRV_Channels:get_output_pwm(K_ELEVATOR)
            if doublet_choice_pwm < 1500 then
                -- doublet on elevator
                DOUBLET_FUCNTION = K_ELEVATOR
                trim_funcs = {K_AILERON, K_RUDDER}
                DOUBLET_MAGNITUDE = 12
                doublet_srv_trim = pre_doublet_elevator
            else
                -- doublet on rudder
                DOUBLET_FUCNTION = K_RUDDER
                trim_funcs = {K_AILERON}
                DOUBLET_MAGNITUDE = 15
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVATOR), pre_doublet_elevator, DOUBLET_TIME * 4)
            end
            -- notify the gcs that we are starting a doublet
            gcs:send_text(6, "STARTING DOUBLET " .. DOUBLET_FUCNTION)
            -- get info about the doublet channel
            doublet_srv_chan = SRV_Channels:find_channel(DOUBLET_FUCNTION)
            doublet_srv_min = param:get("SERVO" .. doublet_srv_chan + 1 .. "_MIN")
            doublet_srv_max = param:get("SERVO" .. doublet_srv_chan + 1 .. "_MAX")
            -- set the channels that need to be pinned to trim until the doublet is done
            for i = 1, #trim_funcs do
                local trim_chan = SRV_Channels:find_channel(trim_funcs[i])
                local trim_pwm = param:get("SERVO" .. trim_chan + 1 .. "_TRIM")
                SRV_Channels:set_output_pwm_chan_timeout(trim_chan, trim_pwm, DOUBLET_TIME * 2)
            end
            -- get the current throttle PWM and pin it there until the doublet is done
            local pre_doublet_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
            SRV_Channels:set_output_pwm_chan_timeout(
                SRV_Channels:find_channel(K_THROTTLE),
                pre_doublet_throttle,
                DOUBLET_TIME * 3
            )
            -- enter manual mode
            retry_set_mode(MODE_MANUAL)
        end
        -- split time evenly between high and low signal
        if now < start_time + (DOUBLET_TIME / 2) then
            down = doublet_srv_trim - math.floor((doublet_srv_trim - doublet_srv_min) * (DOUBLET_MAGNITUDE / 45))
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, down, DOUBLET_TIME / 2 + 100)
        elseif now < start_time + DOUBLET_TIME then
            up = doublet_srv_trim + math.floor((doublet_srv_max - doublet_srv_trim) * (DOUBLET_MAGNITUDE / 45))
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, up, DOUBLET_TIME / 2 + 100)
        elseif now < start_time + (DOUBLET_TIME * 2) then
            -- stick fixed response
            -- hold at pre doublet trim position for any damping effects
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, doublet_srv_trim, DOUBLET_TIME * 2)
        elseif now > start_time + (DOUBLET_TIME * 2) then
            -- notify GCS
            end_time = now
            gcs:send_text(6, "DOUBLET FINISHED")
        else
            gcs:send_text(6, "this should not be reached")
        end
    elseif end_time ~= -1 and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 then
        -- wait for RC input channel to go low
        gcs:send_text(6, "RC" .. DOUBLET_ACTION_CHANNEL .. " still high")
        callback_time = 100 -- prevent spamming messages to the GCS
    elseif now ~= -1 and end_time ~= -1 then
        gcs:send_text(6, "RETURN TO PREVIOUS FLIGHT MODE")
        now = -1
        end_time = -1
        start_time = -1
        -- clear all of the timeouts
        control_functions = {K_AILERON, K_ELEVATOR, K_THROTTLE, K_RUDDER}
        for i = 1, 4 do
            local control_chan = SRV_Channels:find_channel(control_functions[i])
            SRV_Channels:set_output_pwm_chan_timeout(control_chan, param:get("SERVO" .. control_chan + 1 .. "_TRIM"), 0)
        end
        retry_set_mode(pre_doublet_mode)
        callback_time = 100 -- don't need to rerun for a little while
    elseif now ~= -1 then
        -- stopped before finishing. recover to level attitude
        gcs:send_text(6, "FBWA RECOVER")
        now = -1
        end_time = -1
        start_time = -1
        -- clear all of the timeouts
        control_functions = {K_AILERON, K_ELEVATOR, K_THROTTLE, K_RUDDER}
        for i = 1, 4 do
            local control_chan = SRV_Channels:find_channel(control_functions[i])
            SRV_Channels:set_output_pwm_chan_timeout(control_chan, param:get("SERVO" .. control_chan + 1 .. "_TRIM"), 0)
        end
        retry_set_mode(MODE_FBWA)
        callback_time = 100
    end
    return doublet, callback_time
end

gcs:send_text(6, "plane-doublets.lua is running")
return doublet(), 500
