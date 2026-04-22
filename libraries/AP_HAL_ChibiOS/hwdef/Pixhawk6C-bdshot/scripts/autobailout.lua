-- Tailsitter Recovery Loop Script
-- Logic: Bad Pitch -> Save Mode -> QLoiter
local LOOP_MS = 100 -- Run at 10Hz

-- 1. SETUP PARAMETER TABLE
local KEY = 110
assert(param:add_table(KEY, "AUTOB_", 4), "AUTOB table failed")

-- 2. ADD PARAMETERS
assert(param:add_param(KEY, 1,  "PIT_LIM", 40),'could not add AUTOB_PIT_LIM')    -- Pitch limit
assert(param:add_param(KEY, 2, "ENABLE", 1),'could not add AUTOB_ENABLE')    -- 1 = Enabled
assert(param:add_param(KEY, 3, "MODE_DLY", 1000), 'could not add AUTOB_MODE_DLY') -- Delay (ms) before checking pitch
assert(param:add_param(KEY, 4,"PIT_TOUT", 500),'could not add AUTOB_PIT_TOUT')

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
local p_mode_dly = bind_param("AUTOB_MODE_DLY")
local p_pitch_timeout = bind_param("AUTOB_PIT_TOUT")

-- 4. MODE DEFINITIONS (ArduPlane)
local MODE_QLOITER = 19 -- Auto-Hover (Functionally same as QLoiter 50% Thr)
local MODE_QLAND  = 20

-- 5. STATE VARIABLES
local active = false
local last_mode_idx = 0
local mode_entry_time = 0
local first_pitch_exceeded_t = nil

-- Helper: Radians to Degrees
local function rad2deg(r) return r * 57.2958 end

function is_vehicle_landing()
    local current_mode = vehicle:get_mode()
    if current_mode == MODE_QLAND then
        return true
    elseif quadplane:in_vtol_land_descent() then
        return true
    else
        return false
    end
end

function update()
    -- Safety Check
    if not p_enable  then return update, 1000 end
    if p_enable:get() ~= 1 then return update, LOOP_MS end

    local current_mode = vehicle:get_mode()
    if not current_mode then return update, LOOP_MS end

    local now = millis():tofloat()

    -- Detect Mode Changes
    if current_mode ~= last_mode_idx then
        last_mode_idx = current_mode
        mode_entry_time = now
        first_pitch_exceeded_t = nil
    end

    -- ==========================================================
    -- LOGIC: MONITORING (Checking Pitch)
    -- ==========================================================
    if not active then
        if is_vehicle_landing() then
            -- Wait for Delay (settle time)
            local delay_ms = p_mode_dly:get() or 1000
            if (now - mode_entry_time) > delay_ms then
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
return update, LOOP_MS