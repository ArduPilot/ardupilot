--[[
    Fly an outbound transition utilizing TECs for a quadplane.

    * VTOL takeoff to min(alt, Q_NAVALT_MIN)
    * Start a hold timer
    * Set TRIM_ARSPD_CM and ARSPD_FWB_MIN to the hold speed
    * Cycle waypoint to WP2 (this sets the altitude desired and thus profile via TECs, VTOL is
    protected by Q_ASSIST_SPEED which is unmodified here)
    * Hold timer expires
    * Reset TRIM_ARSPD_CM and ARSPD_FWB_MIN to values at script boot
    * Airspeed is now trying to reach cruise setpoint
    * Cruise airspeed is reached, cycle to WP3 which sets cruise pattern (this is to allow a
    different altitude during the hold timer)
--]]

local MAV_SEVERITY =
    {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_RATE_HZ = 100

local MODE_AUTO = 10
local NAV_WAYPOINT = 16
local NAV_VTOL_LAND = 85

local PARAM_TABLE_KEY = 44
local PARAM_TABLE_PREFIX = "TOT_"

local starting_altitude_cm = nil
local hold_timer_start_ms = nil

local is_transitioned = false
local need_reset = false


-- Bind a parameter to a variable.
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- Add a parameter and bind it to a variable.
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
       string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Setup specific parameters.
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

--[[
    // @Param: TOT_ENABLE
    // @DisplayName: TECS outbound transition enable
    // @Description: Enable the use of TECS outbound transitions.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
--]]
local TOT_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
    // @Param: TOT_HOLD_S
    // @DisplayName: Hold time, seconds
    // @Description: The amount of time to hold a reduced airspeed.
    // @Values: 0 30
    // @User: Advanced
--]]
local TOT_HOLD_TIME_S = bind_add_param('HOLD_S', 2, 5)

--[[
    // @Param: TOT_HOLD_SPD_M_S
    // @DisplayName: Hold speed setpoint, m/s
    // @Description: The speed to set during the reduced speed hold time, m/s.
    // @Values: 0 25
    // @User: Advanced
--]]
local TOT_HOLD_SPD_M_S = bind_add_param('HOLD_SPD_M_S', 3, 10)

--[[
    // @Param: TOT_NAVALT_MIN
    // @DisplayName: Minimum altitude to begin outbound transition
    // @Description: The minimum altitude to begin maneuver for outbound transition in meters.
    // @Values: 0 50
    // @User: Advanced
--]]
local TOT_NAVALT_MIN = bind_add_param('NAVALT_MIN', 4, 1)

-- Force TOT_NAVALT_MIN to be at least Q_NAVALT_MIN
local Q_NAVALT_MIN = bind_param("Q_NAVALT_MIN")
if TOT_NAVALT_MIN:get() < Q_NAVALT_MIN:get() then
    TOT_NAVALT_MIN:set(Q_NAVALT_MIN:get())
end

-- Bind parameters for adjustment during reduced speed hold time.
local ARSPD_FBW_MIN = bind_param("ARSPD_FBW_MIN")
local arspd_fbw_min_initial = ARSPD_FBW_MIN:get()

local TRIM_ARSPD_CM = bind_param("TRIM_ARSPD_CM")
local trim_arspd_cm_intial = TRIM_ARSPD_CM:get()

gcs:send_text(MAV_SEVERITY.INFO, "Loaded quadplane-tecs-outbound-transition.lua")

function next_waypoint()
    local index = mission:get_current_nav_index()
    mission:set_current_cmd(index + 1)
end

function maybe_reset_params()
    if need_reset then
        if ARSPD_FBW_MIN:get() ~= arspd_fbw_min_initial then
            ARSPD_FBW_MIN:set(arspd_fbw_min_initial)
        end
        if TRIM_ARSPD_CM:get() ~= trim_arspd_cm_intial then
            TRIM_ARSPD_CM:set(trim_arspd_cm_intial)
        end
        need_reset = false
    end
end

function update()
    if TOT_ENABLE:get() < 1 then
        return
    end

    local vehicle_mode = vehicle:get_mode()
    local nav_id = mission:get_current_nav_id()

    -- At VTOL Land reset everything to support multiple landing missions.
    if nav_id == NAV_VTOL_LAND then
        maybe_reset_params()
        starting_altitude_cm = nil
        hold_timer_start_ms = nil
        is_transitioned = false
        return
    end

    if not arming:is_armed() or vehicle_mode ~= MODE_AUTO or starting_altitude_cm == nil then
        maybe_reset_params()
        local current_location = ahrs:get_location()
        if current_location then
            starting_altitude_cm = current_location:alt()
        end
        return
    end

    -- Make sure we are above minimum altitude before doing anything.
    if (starting_altitude_cm ~= nil and
        ahrs:get_location():alt() - starting_altitude_cm < TOT_NAVALT_MIN:get()*100) then
        return
    end

    -- Only allow while in auto and for one transition per landing.
    if vehicle_mode == MODE_AUTO and not is_transitioned then
        if hold_timer_start_ms == nil then
            hold_timer_start_ms = millis():tofloat()
        end

        if millis():tofloat() - hold_timer_start_ms < TOT_HOLD_TIME_S:get()*1000 then
            ARSPD_FBW_MIN:set(TOT_HOLD_SPD_M_S:get())
            TRIM_ARSPD_CM:set(TOT_HOLD_SPD_M_S:get()*100)
            need_reset = true
        else
            maybe_reset_params()
        end

        -- Cycle waypoint to get TECs working.
        if (nav_id ~= NAV_WAYPOINT and nav_id ~= NAV_VTOL_LAND) then
            next_waypoint()
        end

        -- Check for transition.
        if ahrs:airspeed_estimate() > arspd_fbw_min_initial then
            if not is_transitioned then
                next_waypoint()
            end
            is_transitioned = true
            -- Handle the case where we transition unexpectedly.
            maybe_reset_params()
            return
        end
    end
end

-- Wrapper around update(). If update faults then an error is displayed, but the script is not
-- stopped.
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
        return
    end
    return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- Start running update loop.
return protected_wrapper()
