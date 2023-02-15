
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local MAV_CMD_NAV_LAND = 21


local MISSION_TAG_DETERMINE_LAND_DIRECTION  = 200
local MISSION_TAG_LAND1_DIRECTION_NORMAL    = 300
local MISSION_TAG_LAND1_DIRECTION_REVERSE   = 301

function get_bearing_of_first_land_after_tag(tag)

    local index_start = mission:get_index_of_jump_tag(tag)
    mitem_prev = mission:get_item(index_start)
    if (not mitem_prev) then
        return nil
    end

    for index = index_start+1, mission:num_commands()-1 do
        mitem = mission:get_item(index)
        if (not mitem) then
            return nil
        end
        if (mitem:command() == MAV_CMD_NAV_LAND) then
            local wp1 = Location()
            wp1:lat(mitem_prev:x())
            wp1:lng(mitem_prev:y())
        
            local wp2 = Location()
            wp2:lat(mitem:x())
            wp2:lng(mitem:y())
        
            return wp1:get_bearing(wp2)
        end
        mitem_prev = mitem;
    end
    return nil
end

function check_wind_and_jump_to_INTO_wind_landing()
    if ((mission:get_last_jump_tag() ~= MISSION_TAG_DETERMINE_LAND_DIRECTION) or (mission:get_last_jump_tag_age() > 3)) then
        -- we're not at the decision point yet or we saw it a while ago and we dont' care any more
        return
    end

    local reverse_land_bearing = get_bearing_of_first_land_after_tag(MISSION_TAG_LAND1_DIRECTION_REVERSE)
    if (not reverse_land_bearing) then
        return
    end

    local wind = ahrs:wind_estimate()
    local tail_wind = (math.sin(reverse_land_bearing) * wind:y()) + (math.cos(reverse_land_bearing) * wind:x())

    -- we need at least 10 cm/s of tailwind. With very little wind (or a noisy 0 value) we don't want to flip around.
    local tail_wind_threshold = 0.1

    local tag
    if (tail_wind > tail_wind_threshold) then
        gcs:send_text(MAV_SEVERITY.INFO, "LUA: continuing with normal landing direction")
        tag = MISSION_TAG_LAND1_DIRECTION_NORMAL
    else
        gcs:send_text(MAV_SEVERITY.INFO, "LUA: jump mission to other into-wind landing direction")
        tag = MISSION_TAG_LAND1_DIRECTION_REVERSE
    end

    if (not mission:jump_to_tag(tag)) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("LUA: jump_to_tag %u failed", tag))
    end
end

function update()
    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 5000
    end

    check_wind_and_jump_to_INTO_wind_landing()

    return update, 1000
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("LUA: SCRIPT START: Jump_Tag into wind landing"))
return update() -- run immediately before starting to reschedule

