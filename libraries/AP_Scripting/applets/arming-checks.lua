-- This script runs custom arming checks for validations that are important
-- to some, but might need to be different depending on the vehicle or use case
-- so we don't want to bake them into the firmware. Requires SCR_ENABLE =1 so must 
-- be a higher end FC in order to be used. (minimum 2M flash and 1M RAM)
-- Thanks to @yuri_rage and Peter Barker for help with the Lua and Autotests

local REFRESH_RATE      = 200
local MAV_SEVERITY_ERROR = 3        --/* Indicates an error in secondary/redundant systems. | */
local MAV_SEVERITY_WARNING = 4      --/* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
local MAV_SEVERITY_INFO = 6         --/* No rmal operational messages. Useful for logging. No action is required for these messages. | */

-- These are the plane modes that autoenable the geofence
local PLANE_MODE_AUTO = 10
local PLANE_MODE_TAKEOFF = 13

local arm_auth_id = arming:get_aux_auth_id()

----  CLASS: Arming_Check  ----
local Arming_Check = {}
Arming_Check.__index = Arming_Check

setmetatable(Arming_Check, {
    __call = function(cls, func, pass_value, severity, text) -- constructor
        local self      = setmetatable({}, cls)
        self.func       = func
        self.pass_value = pass_value
        self.severity   = severity
        self.text       = text
        self.passed     = false
        self.changed    = false
        return self
    end
})

local function geofence_enabled_armingcheck()
    --We fail if there is a no fence but FENCE_ENABLE is set - so we pass if NOT that
    return not (not AC_Fence:present() and param:get('FENCE_ENABLE') == 1)
end
local function geofence_autoenabled_armingcheck()
    --We fail if there is a no fence but FENCE_AUTOENABLE is set - so we pass if NOT that
    --Plus we only fail this if in AUTO mode or TAKEOFF mode (on a plane)
    local vehicle_type = FWVersion:type()
    local mode = vehicle:get_mode()

    if (vehicle_type == 3 and (mode == PLANE_MODE_AUTO or mode == PLANE_MODE_TAKEOFF)) then
        return not (not AC_Fence:present() and param:get('FENCE_AUTOENABLE') == 1)
    end
    -- If this check is useful for other vehicles they will need to add them later
    return true
end

function Arming_Check:state()
    local passed = self.func() == self.pass_value
    self.changed = false
    if self.passed ~= passed then
        self.passed = passed
        self.changed = true
    end
    return self.passed
end

local arming_checks = {
    GeoFence_Enabled = Arming_Check(geofence_enabled_armingcheck, 
                            true, MAV_SEVERITY_ERROR,
                            "FENCE_ENABLE = 1 but no fence present"),
    GeoFence_AutoEnabled = Arming_Check(geofence_autoenabled_armingcheck, true, MAV_SEVERITY_ERROR,
                            "FENCE_AUTOENABLE > 0 but no fence present"
                            )
}

local function idle_while_armed()
    if not arming:is_armed() then return Validate, REFRESH_RATE end
    return idle_while_armed, REFRESH_RATE * 10
end

function Validate() -- this is the loop which periodically runs

    if arming:is_armed() then return idle_while_armed() end

    local validated = true

    for key, check in pairs(arming_checks) do
        validated = validated and check:state()
        if check.changed  and check.passed then
            gcs:send_text(MAV_SEVERITY_INFO, string.format('ARMING: %s passed', check.text))
        end
        if check.changed  and not check.passed then
            if check.severity == MAV_SEVERITY_ERROR then
                gcs:send_text(MAV_SEVERITY_ERROR, string.format('ARMING: %s failed', check.text))
                arming:set_aux_auth_failed(arm_auth_id, string.format('%s failed', check.text))
            elseif check.severity == MAV_SEVERITY_WARNING then
                gcs:send_text(MAV_SEVERITY_WARNING, check.text)
            end
        end
    end

    if validated then
        arming:set_aux_auth_passed(arm_auth_id)
    end

    return Validate, REFRESH_RATE
end

gcs:send_text(MAV_SEVERITY_INFO, "ARMING: scripted validation active")

return Validate() -- run immediately before starting to reschedule