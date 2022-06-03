-- This script runs custom arming checks for valudations that are importnat
-- but might need to be different depending on the vehicle or use case
-- so we don't want to bake them into the firmward. Requres SCR_ENABLE =1 so must 
-- be a higher end FC in order to be used. (minimum 2M flash and 1M RAM)

-- What does this do? I can't find any docs that explain this
-- local auth_id = arming:get_aux_auth_id()

local REFRESH_RATE = 2000
local SEVERITY_INFO = 1
local SEVERITY_WARNING = 2
local SEVERITY_ERROR = 3

-- I think registering for an arming "aux_auth_id" allows the script to block arming
local arm_auth_id = arming:get_aux_auth_id()

local MAV_CMD_NAV_TAKEOFF = 22
local MAV_CMD_NAV_VTOL_TAKEOFF = 84

-- It might confuse a pilot to enable GeoFence when there is no fence on the vehicle,
-- as they might think they they are "protected" when they are actually not
-- This throws two different errors. One if FENCE_ENABLE is true and one 
-- if FENCE_AUTOENABLE is set but only when triggering a AUTO takeoff
function validate_geofence(severity)
    local fence_enable = param:get('FENCE_ENABLE')
    if fence_enable then
        if fence.present then
            gcs:send_text(4, "WARNING: FENCE fence present")
        else
            if severity == SEVERITY_ERROR then
                arming:set_aux_auth_failed(arm_auth_id, "ERROR: FENCE_ENABLE = 1 but no fence present")
            elseif severity == SEVERITY_WARNING or severity == SEVERITY_INFO then
                gcs:send_text(4, "WARNING: FENCE_ENABLE = 1 but no fence present")
            end
        end
    end
end

local armed_old = arming:is_armed()
function validate() -- this is the loop which periodically runs 
    gcs:send_text(4, "ARMING: checks now executing")
    --[[
    armed_new = arming:is_armed()
    if armed_old == armed_new or armed_old then
        armed_old  = armed_new
        return update(), REFRESH_RATE
    end
    ]]--
    -- Run all the checks we want - you can edit this to decide if you want these checks or not
    -- echeck check takes a "Severity" parameter to decide if it's a warning or an error
    validate_geofence(SEVERITY_WARNING)
    
    return validate, REFRESH_RATE 
end

gcs:send_text(4, "ARMING: validation active")

return validate() -- run immediately before starting to reschedule


