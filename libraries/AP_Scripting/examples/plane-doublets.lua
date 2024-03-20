-- This script will preform a control surface doublet
-- The desired axis to perform a doublet on is selected and started with a momentary switch
-- The plane maintains trim conditions while the switch is activated using servo overrides
-- If the momentary switch is released before "DOUBLET FINISHED" is seen on the GCS,
-- the aircraft recovers to FBWA mode to assist the pilot.
-- Magnitude and duration of the doublet can also be controlled.
-- It is suggested to allow the aircraft to trim for straight, level, unaccelerated flight (SLUF) in FBWB mode before
-- starting a doublet
-- Charlie Johnson, Oklahoma State University 2020, modification by Astik Srivastava, Delhi Technological University 2024

local DOUBLET_ACTION_CHANNEL = 6 -- RCIN channel to start a doublet when high (>1700)
local DOUBLET_CHOICE_CHANNEL = 7 -- RCIN channel to choose elevator (low) or rudder (high)
local DOUBLET_FUCNTION = 1 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen
-- A (Servo 1, Function 4), E (Servo 2, Function 19), and R (Servo 4, Function 21)
local DOUBLET_MAGNITUDE = 6 -- defined out of 45 deg used for set_output_scaled
local DOUBLET_TIME = 2000 -- period of doublet signal in ms
local start_time = -1
-- flight mode numbers for plane https://mavlink.io/en/messages/ardupilotmega.html
local MODE_MANUAL = 0
local MODE_FBWA = 5
local K_AILERON = 4
local K_ELEVATOR = 19
local K_THROTTLE = 70
local K_RUDDER = 21

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
    local now = millis()
    local pre_doublet_elevator = SRV_Channels:get_output_pwm(K_ELEVATOR)
    local pre_doublet_aileron = SRV_Channels:get_output_pwm(K_AILERON)
    local pre_doublet_rudder = SRV_Channels:get_output_pwm(K_RUDDER)
    local pre_doublet_throttle = 1200
    local doublet_choice_pwm = rc:get_pwm(DOUBLET_CHOICE_CHANNEL)
    local callback_time = 100
    if arming:is_armed() == true and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 then
        gcs:send_text(6, "in Doublet function")
        -- choosing control actuator, 900 for elevator, 1100 for aileron, 1300 for rudder, 1500 for throttle 
        if doublet_choice_pwm < 1000 then
            DOUBLET_FUCNTION = K_ELEVATOR
            doublet_srv_trim = pre_doublet_elevator
            DOUBLET_MAGNITUDE = 12
        elseif doublet_choice_pwm > 1000 and doublet_choice_pwm < 1200 then
            DOUBLET_FUCNTION = K_AILERON
            doublet_srv_trim = pre_doublet_aileron
            DOUBLET_MAGNITUDE = 15
        elseif doublet_choice_pwm > 1200 and doublet_choice_pwm < 1400 then
            DOUBLET_FUCNTION = K_RUDDER
            doublet_srv_trim = pre_doublet_rudder
            DOUBLET_MAGNITUDE = 15
        elseif doublet_choice_pwm > 1400 and doublet_choice_pwm < 1600 then
            DOUBLET_FUCNTION = K_THROTTLE
            doublet_srv_trim = pre_doublet_throttle
            DOUBLET_MAGNITUDE = 12
        else
            DOUBLET_FUCNTION = K_ELEVATOR
            doublet_srv_trim = pre_doublet_elevator
            DOUBLET_MAGNITUDE = 12
        end
        local doublet_srv_chan = SRV_Channels:find_channel(DOUBLET_FUCNTION)
        local doublet_srv_min = param:get("SERVO" .. doublet_srv_chan + 1 .. "_MIN")
        local doublet_srv_max = param:get("SERVO" .. doublet_srv_chan + 1 .. "_MAX")
        local doublet_srv_trim = param:get("SERVO" .. doublet_srv_chan + 1 .. "_TRIM")
        if start_time == -1 then
            start_time = now
            gcs:send_text(6, "STARTING DOUBLET " .. doublet_srv_chan)
            retry_set_mode(MODE_MANUAL)
        end
        if now < (start_time + (DOUBLET_TIME / 2)) then
            gcs:send_text(6, "DOUBLET up" .. doublet_srv_chan)
            up = doublet_srv_trim + math.floor((doublet_srv_max - doublet_srv_trim) * (DOUBLET_MAGNITUDE / 45))
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, up, DOUBLET_TIME / 2 + 100)
            gcs:send_text(6, "pwm" .. tostring(SRV_Channels:get_output_pwm(DOUBLET_FUCNTION)))
        elseif now < (start_time + DOUBLET_TIME) and now > (start_time + (DOUBLET_TIME / 2))then
            gcs:send_text(6, "DOUBLET down" .. doublet_srv_chan)
            down = doublet_srv_trim - math.floor((doublet_srv_trim - doublet_srv_min) * (DOUBLET_MAGNITUDE / 45))
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, down, DOUBLET_TIME / 2 + 100)
            gcs:send_text(6, "pwm" .. tostring(SRV_Channels:get_output_pwm(DOUBLET_FUCNTION)))
        elseif now > start_time + (DOUBLET_TIME * 2) then
            -- stick fixed response
            -- hold at pre doublet trim position for any damping effects
            gcs:send_text(6, "DOUBLET finished")

            if vehicle:get_mode() == MODE_MANUAL then
                retry_set_mode(MODE_FBWA)
                gcs:send_text(6, "DOUBLET FINISHED, in mode" ..vehicle:get_mode())
                SRV_Channels:set_output_pwm_chan_timeout(DOUBLET_ACTION_CHANNEL, 1000, DOUBLET_TIME)
            end
        end
    end
    return doublet, callback_time
end

gcs:send_text(6, "plane-doublets.lua is running")
return doublet(), 500
