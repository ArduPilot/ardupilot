-- This script runs custom arming checks for valudations that are importnat
-- but might need to be different depending on the vehicle or use case
-- so we don't want to bake them into the firmward. Requres SCR_ENABLE =1 so must 
-- be a higher end FC in order to be used. (minimum 2M flash and 1M RAM)

-- What does this do? I can't find any docs that explain this
-- local auth_id = arming:get_aux_auth_id()

-- defining variables seems to use a lot of memory, so commenting out the values not currently being used.

-- This seems like a pretty quick refresh rate, but it doesn't run if armed
local REFRESH_RATE = 2000
local SEVERITY_INFO = 1
local SEVERITY_WARNING = 2
local SEVERITY_ERROR = 3

--local MAV_SEVERITY_EMERGENCY = 0    --/* System is unusable. This is a "panic" condition. | */
--local MAV_SEVERITY_ALERT = 1        --/* Action should be taken immediately. Indicates error in non-critical systems. | */
--local MAV_SEVERITY_CRITICAL = 2     --/* Action must be taken immediately. Indicates failure in a primary system. | */
local MAV_SEVERITY_ERROR = 3        --/* Indicates an error in secondary/redundant systems. | */
local MAV_SEVERITY_WARNING = 4      --/* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
--local MAV_SEVERITY_NOTICE = 5       --/* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
local MAV_SEVERITY_INFO = 6         --/* No rmal operational messages. Useful for logging. No action is required for these messages. | */
local MAV_SEVERITY_DEBUG = 7        --/* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */

-- These are the plane modes that autoenable the geofence
local PLANE_MODE_AUTO = 10
local PLANE_MODE_TAKEOFF = 13

local RTL_DISABLE = 0
--local RTL_THEN_DO_LAND_START = 1
--local RTL_IMMEDIATE_DO_LAND_START = 2
--local NO_RTL_GO_AROUND = 3

local STATE_UNKNOWN = 0
local STATE_OK = 1
local STATE_WARNING = 2
local STATE_ERROR = 3

-- I think registering for an arming "aux_auth_id" allows the script to block arming
local arm_auth_id = arming:get_aux_auth_id()
local armed_old = arming:is_armed()
local vehicle_type = FWVersion:type()
local mode = vehicle:get_mode()

local RESOLVED = "Resolved:"

-- It might confuse a pilot to enable GeoFence when there is no fence on the vehicle,
-- as they might think they they are "protected" when they are actually not
-- This throws two different errors. One if FENCE_ENABLE is true and one 
-- if FENCE_AUTOENABLE is set but only when triggering a AUTO takeoff
local geofence_enable = -1
local geofence_enable_state = STATE_UNKNOWN
local geofence_enable_message = "FENCE_ENABLE = 1 but no fence present"
local geofence_enable_present = -1
local function validate_geofence_enable(severity)
    local present_new = AC_Fence:present()
    if(present_new ~= geofence_enable_present) then
        geofence_enable_present = present_new
        geofence_enable = -1
    end
    local geofence_enable_new = param:get('FENCE_ENABLE')
    if geofence_enable == geofence_enable_new then
        return geofence_enable_state ~= STATE_ERROR
    end
    geofence_enable = geofence_enable_new
    --gcs:send_text(MAV_SEVERITY_DEBUG, "FENCE_ENABLE: "..geofence_enable)
    if geofence_enable ~= 0 and not geofence_enable_present then
        --gcs:send_text(MAV_SEVERITY_DEBUG, "FENCE_ENABLE: no fence")
        if severity == SEVERITY_ERROR then
            if geofence_enable_state ~= STATE_ERROR then
                arming:set_aux_auth_failed(arm_auth_id, geofence_enable_message)
                gcs:send_text(MAV_SEVERITY_ERROR, geofence_enable_message)
                geofence_enable_state = STATE_ERROR
            end
            return false
        elseif severity == SEVERITY_WARNING or severity == SEVERITY_INFO then
            if geofence_enable_state ~= STATE_WARNING then
                gcs:send_text(MAV_SEVERITY_WARNING, geofence_enable_message)
                geofence_enable_state = STATE_WARNING
            end
        end
    elseif geofence_enable_state ~= STATE_OK then
        --gcs:send_text(MAV_SEVERITY_DEBUG, "FENCE_ENABLE: old state:" .. geofence_enable_state)

        if(geofence_enable_state ~= STATE_UNKNOWN) then
            gcs:send_text(MAV_SEVERITY_INFO,RESOLVED..geofence_enable_message)
        end
        geofence_enable_state = STATE_OK
    end
    return true
end 


local geofence_autoenable = -1
local geofence_autoenable_state = STATE_UNKNOWN
local geofence_autoenable_message = "FENCE_AUTOENABLE > 0 but no fence present"
local geofence_autoenable_present = -1
local function validate_geofence_autoenable(severity)
    local mode_new = vehicle:get_mode()
    if mode_new ~= mode then
        -- If the mode changed, the results might be different, so need to force recheck
        mode    = mode_new
        geofence_autoenable = -1
    end
    local present_new = AC_Fence:present()
    if(present_new ~= geofence_autoenable_present) then
        geofence_autoenable_present = present_new
        geofence_autoenable = -1
    end
    local geofence_autoenable_new = param:get('FENCE_AUTOENABLE')
    gcs:send_text(MAV_SEVERITY_DEBUG, "FENCE_autoENABLE: "..geofence_autoenable)
    if geofence_autoenable_new == geofence_autoenable then
        return geofence_autoenable_state ~= STATE_ERROR
    end
    geofence_autoenable = geofence_autoenable_new

        -- we only want to warn the user if the mode is currently AUTO or TAKEOFF
        -- otherwise we let the AP code warn the user when/if the fence is actually enabled
        -- These are the plane checks (type == 3) - the modes are DIFFERENT depending on the vehicle
    --gcs:send_text(MAV_SEVERITY_DEBUG, "AUTOENABLE vehicle:"..vehicle_type.." mode:"..mode )
    if geofence_autoenable > 0 and
        (vehicle_type == 3 and (mode == PLANE_MODE_AUTO or mode == PLANE_MODE_TAKEOFF)) and
        not geofence_autoenable_present then
        --gcs:send_text(MAV_SEVERITY_DEBUG, "not present ")
        if severity == SEVERITY_ERROR then
            if geofence_autoenable_state ~= STATE_ERROR then
                arming:set_aux_auth_failed(arm_auth_id, geofence_autoenable_message)
                gcs:send_text(MAV_SEVERITY_ERROR, geofence_autoenable_message)
                geofence_autoenable_state = STATE_ERROR
            end
            return false
        else 
            if geofence_autoenable_state ~= SEVERITY_WARNING then
                gcs:send_text(MAV_SEVERITY_WARNING, geofence_autoenable_message)
                geofence_autoenable_state = STATE_WARNING
            end
        end
    elseif geofence_autoenable_state ~= STATE_OK then
        if(geofence_autoenable_state ~= STATE_UNKNOWN) then
            gcs:send_text(MAV_SEVERITY_INFO,RESOLVED..geofence_autoenable_message)
        end
        geofence_autoenable_state = STATE_OK
    end

    return true
end

-- This was added to arduplane in PR#20345. Adding this here
-- makes it possible for this check to be optional
local rtlautoland_old = -1
local rtlautoland_state = STATE_UNKNOWN
local rtlautoland_message = "DO_LAND_START set and RTL_AUTOLAND disabled"
local function validate_rtlautoland(severity)
    local rtlautoland = param:get('RTL_AUTOLAND')
    if rtlautoland_old == rtlautoland then
        return rtlautoland_state ~= STATE_ERROR
    end
    rtlautoland_old = rtlautoland
    if rtlautoland == RTL_DISABLE then
        --gcs:send_text(MAV_SEVERITY_DEBUG,"RTL Disabled")
        local landing_start_count = mission:get_landing_sequence_start()
        if landing_start_count > 0 then
            --gcs:send_text(MAV_SEVERITY_DEBUG,"landing_start count > 0")
            if severity == SEVERITY_ERROR then
                if rtlautoland_state ~= STATE_ERROR then -- don't spam the GCS - only send the message once
                    arming:set_aux_auth_failed(arm_auth_id, rtlautoland_message)
                    gcs:send_text(MAV_SEVERITY_ERROR, rtlautoland_message)
                    rtlautoland_state = STATE_ERROR
                --else
                    --gcs:send_text(MAV_SEVERITY_DEBUG,rtlautoland_message)
                end
                return false
            elseif severity == SEVERITY_WARNING then
                if rtlautoland_state ~=STATE_WARNING then -- Don't spam the GCS - only send the message once
                    gcs:send_text(MAV_SEVERITY_WARNING, rtlautoland_message)
                    rtlautoland_state = STATE_WARNING
                end
            end
        end
    elseif rtlautoland_state ~= STATE_OK then
        if(rtlautoland_state ~= STATE_UNKNOWN) then
            gcs:send_text(MAV_SEVERITY_INFO,RESOLVED..rtlautoland_message)
        end
        rtlautoland_state = STATE_OK
    end
    return true
end

function validate() -- this is the loop which periodically runs 

    -- (I think?) We only want to run these checks if we are disarmed
    if  arming:is_armed() then
        return validate(), REFRESH_RATE
    end

    -- Run all the checks we want - you can edit this to decide if you want these checks or not
    -- echeck check takes a "Severity" parameter to decide if it's a warning or an error
    -- gcs:send_text(MAV_SEVERITY_DEBUG, "ARMING: checks now executing on:" .. FWVersion:type())
    --gcs:send_text(MAV_SEVERITY_DEBUG, "ARMING: checks now executing on:" .. vehicle_type)

    local validated = true
    validated = validated and validate_geofence_enable(SEVERITY_ERROR)
    validated = validated and validate_geofence_autoenable(SEVERITY_WARNING)
    validated = validated and validate_rtlautoland(SEVERITY_ERROR)

    if validated then
        arming:set_aux_auth_passed(arm_auth_id)  
    end

    -- do some garbage collection before returning
    -- local memory_count = collectgarbage("count")
    -- gcs:send_text(MAV_SEVERITY_DEBUG,"memory:"..memory_count)

    return validate, REFRESH_RATE 
end

gcs:send_text(MAV_SEVERITY_INFO, "ARMING: scripted validation active")

return validate() -- run immediately before starting to reschedule


