-- Tailsitter Recovery Loop Script
-- Logic: Bad Pitch -> Save Mode -> QLoiter
local LOOP_MS = 50 
local para_ahrs_pitch_threshold_max = -10
local para_ahrs_pitch_threshold_min = -50

-- 1. SETUP PARAMETER TABLE
local KEY = 110
assert(param:add_table(KEY, "AUTOB_", 7), "AUTOB table failed")

-- 2. ADD PARAMETERS
assert(param:add_param(KEY, 1,  "PIT_LIM", 40),'could not add AUTOB_PIT_LIM')    -- Pitch limit
assert(param:add_param(KEY, 2, "ENABLE", 1),'could not add AUTOB_ENABLE')    -- 1 = Enabled
assert(param:add_param(KEY, 3, "BTRN_DLY", 2500), 'could not add AUTOB_BTRN_DLY') -- Delay (ms) before checking pitch
assert(param:add_param(KEY, 4,"PIT_TOUT", 100),'could not add AUTOB_PIT_TOUT')
assert(param:add_param(KEY, 5, "PARA_EN", 1),'could not add AUTOB_PARA_EN')    -- 1 = Enabled
assert(param:add_param(KEY, 6,"PARA_ANG", -15),'could not add AUTOB_PARA_ANG')
assert(param:add_param(KEY, 7,"PARA_TOUT", 100),'could not add AUTOB_PARA_TOUT')

-- 3. BIND PARAMETERS
local function bind_param(name)
    local p = Parameter(name)
    if not p then
        gcs:send_text(2, "AUTOB: Reboot Required for " .. name)
    end
    return p
end

local p_enable = bind_param("AUTOB_ENABLE")
local p_pit_lim = bind_param("AUTOB_PIT_LIM")
local p_btrn_dly = bind_param("AUTOB_BTRN_DLY")
local p_pitch_timeout = bind_param("AUTOB_PIT_TOUT")
local p_para_enable = bind_param("AUTOB_PARA_EN")
local p_para_ang = bind_param("AUTOB_PARA_ANG")
local p_para_timeout = bind_param("AUTOB_PARA_TOUT")

-- Read Parachute trigger channel number from FCU parameter list 


-- 4. MODE DEFINITIONS (ArduPlane)
local MODE_QLOITER = 19 -- Auto-Hover (Functionally same as QLoiter 50% Thr)
local AUTOBAILOUT_EXCLUDE_MODES = {
    [17] = true, --QSTABILIZE
    [18] = true,  -- QHOVER
    [19] = true,  -- QLOITER(cannot autobailout in QLOITER)
    [22] = true,  -- QAUTOTUNE
    [23] = true,  -- QACRO
}
-- 5. STATE VARIABLES
local active = false
local last_mode_idx = 0
local first_pitch_exceeded_t = nil

-- Parachute state variables
local trigger_para_script = false
local first_para_pitch_exceeded_t = nil
local PARA_CHAN_HIGH = 1850
local backtransition_complete_time_ms = nil

-- Helper: Radians to Degrees
local function rad2deg(r) return r * 57.2958 end

function is_vehicle_landing()
    if quadplane:in_vtol_land_descent() then
        return true
    else
        return false
    end
end

function in_vtol_flight()
    local mode = vehicle:get_mode()
    local vtol_active = not quadplane:tailsitter_in_vtol_transition() and quadplane:in_vtol_mode() and not AUTOBAILOUT_EXCLUDE_MODES[mode]
    if vtol_active then
        if backtransition_complete_time_ms == nil then
            backtransition_complete_time_ms = millis():tofloat()
        end
        return true
    end
    backtransition_complete_time_ms = nil
    return false
end

function is_parachute_angle_threshold_valid(threshold_angle)
-- check if the threshold angle set by user is valid
    if threshold_angle < para_ahrs_pitch_threshold_max and threshold_angle > para_ahrs_pitch_threshold_min then
        return true
    end
    return false
end

function para_deploy()
    if p_para_enable:get() ~= 1 then return end

    para_trigger_rc_chan = nil
    PARA_TRIG_CHAN = Parameter()
    PARA_TRIG_CHAN:init('PARA_TRIG_CH')
    if PARA_TRIG_CHAN then
        channel_num = PARA_TRIG_CHAN:get()
        -- gcs:send_text(2, "Channel num" .. tostring(channel_num))
        para_trigger_rc_chan = rc:get_channel(channel_num)
    end

    if para_trigger_rc_chan == nil then
        gcs:send_text(2, "AUTOB:Para rc channel is nil")
        return
    end
    
    para_threshold = p_para_ang:get() or -45
    if not is_parachute_angle_threshold_valid(para_threshold) then
        gcs:send_text(2, string.format("AUTOB:AUTB_PARA_ANG invalid. Range(%.1f, %.1f)",para_ahrs_pitch_threshold_min, para_ahrs_pitch_threshold_max))    
        return
    end

    now = millis():tofloat()
    ahrs_pitch = rad2deg(ahrs:get_pitch() or 0)
    para_ang_timeout = p_para_timeout:get() or 200
    check_pitch = ahrs_pitch < para_threshold and (trigger_para_script == false) and quadplane:in_vtol_mode() and arming:is_armed()
    if check_pitch then
        if first_para_pitch_exceeded_t == nil then
            first_para_pitch_exceeded_t = millis():tofloat()
        else
            para_time_diff = now - first_para_pitch_exceeded_t
            if para_time_diff > para_ang_timeout then
                first_para_pitch_exceeded_t = nil
                trigger_para_script = true
                gcs:send_text(2, "AUTOB:trigger parachute via rc override")
            end
        end
    else
        first_para_pitch_exceeded_t = nil
    end
    
    if trigger_para_script then
        para_trigger_rc_chan:set_override(PARA_CHAN_HIGH)
    end
end

function update()
    para_deploy()
    if trigger_para_script then return  update, LOOP_MS end

    -- Safety Check
    if p_enable:get() ~= 1 then return update, LOOP_MS end

    local current_mode = vehicle:get_mode()
    if not current_mode then return update, LOOP_MS end

    local now = millis():tofloat()

    -- Detect Mode Changes
    if current_mode ~= last_mode_idx then
        last_mode_idx = current_mode
        first_pitch_exceeded_t = nil
    end

    -- ==========================================================
    -- LOGIC: MONITORING (Checking Pitch)
    -- ==========================================================
    if not active then
        if in_vtol_flight() and arming:is_armed() then
            -- Wait for Delay (settle time)
            local delay_ms = p_btrn_dly:get() or 1000
            if backtransition_complete_time_ms and (now - backtransition_complete_time_ms) > delay_ms then
                local pitch_deg = rad2deg(ahrs:get_pitch() or 0)
                local threshold = p_pit_lim:get() or 40
                local pitch_timeout = p_pitch_timeout:get() or 500
                if pitch_deg < threshold then
                    if first_pitch_exceeded_t == nil then
                        first_pitch_exceeded_t = millis():tofloat()
                    else
                        time_diff = now - first_pitch_exceeded_t
                        if time_diff > pitch_timeout then
                            if vehicle:set_mode(MODE_QLOITER) then
                                first_pitch_exceeded_t = nil
                                active = true
                                gcs:send_text(2, "AUTOB: Switching to QLoiter" )
                            end
                        end
                    end
                else
                    first_pitch_exceeded_t = nil
                end
            end
        end
    elseif active then
        -- Cancel if pilot manually switched mode
        if current_mode ~= MODE_QLOITER then
            active = false
            gcs:send_text(6, "AUTOB: Manual Override Detected")
        end
    end
    return update, LOOP_MS

end

gcs:send_text(6, "Airbound Lua: Autobailout script loaded")
return update, 1000