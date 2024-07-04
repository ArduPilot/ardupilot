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
local end_freqeuncy = 5.00 -- end freqeuncy in Hz
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

--- logging parameters
local roll_rate = 1
local pitch_rate = 2
local yaw_rate = 3
local acc_x = 4
local acc_y = 5
local acc_z = 6
local RcOut = 7
local t_log = 8
local input = 9
local start_freq_rads = 2 * math.pi * start_freq
local end_freq_rads = 2 * math.pi * end_freqeuncy
local B = math.log(end_freq_rads / start_freq_rads)
local k = (end_freqeuncy - start_freq)*1000 / SWEEP_TIME
local callback_time = 40


local interesting_data = {}

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
local function write_to_dataflash()

    -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
    -- format characters specify the type of variable to be logged, see AP_Logger/README.md
    -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
    -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
    -- lua automatically adds a timestamp in micro seconds
    -- logger:write('SCR1','Gx(deg),Gy(deg),Gz(deg)','fff',interesting_data[roll_rate],interesting_data[pitch_rate],interesting_data[yaw_rate]) 
    logger:write('SCR1', 'Gx,Gy,Gz,accx,accy,accz,RCOut,input,time', 'fffffffff', interesting_data[roll_rate], interesting_data[pitch_rate], interesting_data[yaw_rate],interesting_data[acc_x], interesting_data[acc_y], interesting_data[acc_z],interesting_data[RcOut],interesting_data[input],interesting_data[t_log])


    
  end
function SWEEP(pre_SWEEP_elevator, pre_SWEEP_aileron, pre_SWEEP_rudder, pre_SWEEP_throttle)
    local now = millis()
    local SWEEP_choice_pwm = rc:get_pwm(SWEEP_CHOICE_CHANNEL)
    if rc:get_pwm(SWEEP_ACTION_CHANNEL) < 1700 then
        pre_SWEEP_elevator = SRV_Channels:get_output_pwm(K_ELEVATOR)
        pre_SWEEP_aileron = SRV_Channels:get_output_pwm(K_AILERON)
        pre_SWEEP_rudder = SRV_Channels:get_output_pwm(K_RUDDER)
        pre_SWEEP_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
    end
    if arming:is_armed() == true and rc:get_pwm(SWEEP_ACTION_CHANNEL) > 1700 then
        gcs:send_text(6, "in SWEEP function")
            pre_SWEEP_elevator = 1465
            pre_SWEEP_aileron = SRV_Channels:get_output_pwm(K_AILERON)
            pre_SWEEP_rudder = SRV_Channels:get_output_pwm(K_RUDDER)
            pre_SWEEP_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
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
            retry_set_mode(MODE_FBWA)
        end
        if now < (start_time + (SWEEP_TIME)) then
            gcs:send_text(6, "in sweep loop" .. SWEEP_srv_chan)
            t_i = tonumber(tostring(now - start_time))/1000
            amplitude_sent = SWEEP_MAGNITUDE*math.sin(tonumber(2*math.pi*(start_freq * t_i + (k / 2) * t_i*t_i)))
            -- amplitude_sent = SWEEP_MAGNITUDE * math.sin(start_freq_rads * (math.exp(k * t_i) - 1) / k)
            amplitude_sent = math.floor(amplitude_sent)
            SRV_Channels:set_output_pwm_chan_timeout(SWEEP_srv_chan,SWEEP_srv_trim + amplitude_sent,40)
            local op = SRV_Channels:get_output_pwm(SWEEP_FUCNTION)
            gcs:send_text(6, "pwm" .. tostring(op))
            local rate = ahrs:get_gyro()
            gcs:send_text(6, "rate[1]" .. tostring(rate:x()))
            local acc = ahrs:get_accel()
            gcs:send_text(6, "acc[1]" .. tostring(acc:x()))
            if rate and acc then
                interesting_data[roll_rate] = rate:x()
                interesting_data[pitch_rate] = rate:y()
                interesting_data[yaw_rate] = rate:z()
                interesting_data[acc_x] = acc:x()
                interesting_data[acc_y] = acc:y()
                interesting_data[acc_z] = acc:z()
                interesting_data[RcOut] = op
                interesting_data[t_log] = t_i
                interesting_data[input] = amplitude_sent
                write_to_dataflash()
            else
                gcs:send_text(6, "Invalid rate or acc data")
            end
            

            
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
return SWEEP(pre_SWEEP_elevator, pre_SWEEP_aileron, pre_SWEEP_rudder, pre_SWEEP_throttle), callback_time
