-- This script will preform a control surface SWEEP
-- The desired axis to perform a SWEEP on is selected and started with a momentary switch
-- The plane maintains trim conditions while the switch is activated using servo overrides
-- If the momentary switch is released before "SWEEP FINISHED" is seen on the GCS,
-- the aircraft recovers to FBWA mode to assist the pilot.
-- Magnitude and duration of the SWEEP can also be controlled.
-- It is suggested to allow the aircraft to trim for straight, level, unaccelerated flight (SLUF) in FBWB mode before
-- starting a SWEEP
-- Charlie Johnson, Oklahoma State University 2020, modification by Astik Srivastava, Delhi Technological University 2024

local SWEEP_ACTION_CHANNEL = 6 -- RCIN channel to start a SWEEP when high (>1700)
local SWEEP_CHOICE_CHANNEL = 7 -- RCIN channel to choose elevator (low) or rudder (high)
local SWEEP_FUCNTION = 1 -- which control surface (SERVOx_FUNCTION) number will have a SWEEP happen
-- A (Servo 1, Function 4), E (Servo 2, Function 19), and R (Servo 4, Function 21)
local SWEEP_MAGNITUDE = 50 -- defined out of 45 deg used for set_output_scaled
local SWEEP_TIME = 120000 -- period of SWEEP signal in ms
local start_freq = 0.05 -- start freqeuncy in hz
local end_freqeuncy = 5 -- end freqeuncy in Hz
local k = (end_freqeuncy-start_freq)/SWEEP_TIME -- rate of change of frequency in sweep
local t_i = 0
local amplitude_sent = 0
local start_time = -1
-- flight mode numbers for plane https://mavlink.io/en/messages/ardupilotmega.html
local MODE_MANUAL = 0
local MODE_FBWA = 5
local K_AILERON = 4
local K_ELEVATOR = 19
local K_THROTTLE = 70
local K_RUDDER = 21
local SWEEP_srv_trim = -100

function retry_set_mode(mode)
    if vehicle:set_mode(mode) then
        -- if the mode was set successfully, carry on as normal
        desired_mode = -1
        return SWEEP, 1
    else
        -- if the mode was not set successfully, try again ASAP
        desired_mode = mode
        return retry_set_mode, 1
    end
end

function SWEEP(pre_SWEEP_elevator, pre_SWEEP_aileron, pre_SWEEP_rudder, pre_SWEEP_throttle)
    local now = millis()
    local SWEEP_choice_pwm = rc:get_pwm(SWEEP_CHOICE_CHANNEL)
    local callback_time = 100
    if rc:get_pwm(SWEEP_ACTION_CHANNEL) < 1700 then
        pre_SWEEP_elevator = SRV_Channels:get_output_pwm(K_ELEVATOR)
        pre_SWEEP_aileron = SRV_Channels:get_output_pwm(K_AILERON)
        pre_SWEEP_rudder = SRV_Channels:get_output_pwm(K_RUDDER)
        pre_SWEEP_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
    end
    if arming:is_armed() == true and rc:get_pwm(SWEEP_ACTION_CHANNEL) > 1700 then
        gcs:send_text(6, "in SWEEP function")
        -- choosing control actuator, 900 for elevator, 1100 for aileron, 1300 for rudder, 1500 for throttle 
        if SWEEP_choice_pwm < 1000 then
            SWEEP_FUCNTION = K_ELEVATOR
            if SWEEP_srv_trim == -100 then
                SWEEP_srv_trim = pre_SWEEP_elevator 
            end
        elseif SWEEP_choice_pwm > 1000 and SWEEP_choice_pwm < 1200 then
            SWEEP_FUCNTION = K_AILERON
            if SWEEP_srv_trim == -100 then
                SWEEP_srv_trim = pre_SWEEP_aileron 
            end
        elseif SWEEP_choice_pwm > 1200 and SWEEP_choice_pwm < 1400 then
            SWEEP_FUCNTION = K_RUDDER
            if SWEEP_srv_trim == -100 then
                SWEEP_srv_trim = pre_SWEEP_rudder 
            end
        elseif SWEEP_choice_pwm > 1400 and SWEEP_choice_pwm < 1600 then
            SWEEP_FUCNTION = K_THROTTLE
            if SWEEP_srv_trim == -100 then
                SWEEP_srv_trim = pre_SWEEP_throttle
            end
        else
            SWEEP_FUCNTION = K_ELEVATOR
            if SWEEP_srv_trim == -100 then
                SWEEP_srv_trim = pre_SWEEP_elevator 
            end
        end
        local SWEEP_srv_chan = SRV_Channels:find_channel(SWEEP_FUCNTION)
        local SWEEP_srv_min = param:get("SERVO" .. SWEEP_srv_chan + 1 .. "_MIN")
        local SWEEP_srv_max = param:get("SERVO" .. SWEEP_srv_chan + 1 .. "_MAX")
        local SWEEP_srv_trim = param:get("SERVO" .. SWEEP_srv_chan + 1 .. "_TRIM")
        if start_time == -1 then
            start_time = now
            gcs:send_text(6, "STARTING SWEEP " .. SWEEP_srv_chan)
            retry_set_mode(MODE_MANUAL)
        end
        if now < (start_time + (SWEEP_TIME)) then
        --     gcs:send_text(6, "SWEEP up" .. SWEEP_srv_chan)
        --     up = SWEEP_srv_trim + math.floor((SWEEP_srv_max - SWEEP_srv_trim) * (SWEEP_MAGNITUDE / 45))
        --     SRV_Channels:set_output_pwm_chan_timeout(SWEEP_srv_chan, up, SWEEP_TIME / 2 + 100)
        --     gcs:send_text(6, "pwm" .. tostring(SRV_Channels:get_output_pwm(SWEEP_FUCNTION)))
        -- elseif now < (start_time + SWEEP_TIME) and now > (start_time + (SWEEP_TIME / 2))then
        --     gcs:send_text(6, "SWEEP down" .. SWEEP_srv_chan)
        --     down = SWEEP_srv_trim - math.floor((SWEEP_srv_trim - SWEEP_srv_min) * (SWEEP_MAGNITUDE / 45))
        --     SRV_Channels:set_output_pwm_chan_timeout(SWEEP_srv_chan, down, SWEEP_TIME / 2 + 100)
        --     gcs:send_text(6, "pwm" .. tostring(SRV_Channels:get_output_pwm(SWEEP_FUCNTION)))
            gcs:send_text(6, "in sweep loop" .. SWEEP_srv_chan)
            t_i = tonumber(tostring(now - start_time))
            amplitude_sent = SWEEP_MAGNITUDE*math.sin(tonumber(2*math.pi*(start_freq * t_i + (k / 2) * t_i*t_i)))
            amplitude_sent = math.floor(amplitude_sent)
            SRV_Channels:set_output_pwm_chan_timeout(SWEEP_srv_chan,SWEEP_srv_trim + amplitude_sent,40)
            gcs:send_text(6, "pwm" .. tostring(SRV_Channels:get_output_pwm(SWEEP_FUCNTION)))
            
        elseif now > start_time + (SWEEP_TIME) then
            -- stick fixed response
            -- hold at pre SWEEP trim position for any damping effects
            gcs:send_text(6, "SWEEP finished")

            if vehicle:get_mode() == MODE_MANUAL then
                retry_set_mode(MODE_FBWA)
                gcs:send_text(6, "SWEEP FINISHED, in mode" ..vehicle:get_mode())
                SRV_Channels:set_output_pwm(SWEEP_ACTION_CHANNEL, 1000)
            end
        end
    end
    return SWEEP, callback_time
end

gcs:send_text(6, "plane-sweep.lua is running")
local pre_SWEEP_elevator = SRV_Channels:get_output_pwm(K_ELEVATOR)
local pre_SWEEP_aileron = SRV_Channels:get_output_pwm(K_AILERON)
local pre_SWEEP_rudder = SRV_Channels:get_output_pwm(K_RUDDER)
local pre_SWEEP_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
return SWEEP(pre_SWEEP_elevator, pre_SWEEP_aileron, pre_SWEEP_rudder, pre_SWEEP_throttle), 500
