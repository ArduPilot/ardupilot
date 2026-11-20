--[[----------------------------------------------------------------------------

mission-resume ArduPilot Lua script

Resume an auto mission from the point at which a battery failsafe
action was initiated.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Feb 2023

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local RUN_INTERVAL_MS = 200
local RESUME_FILENAME = 'scripts/resume.txt'

-- MAVLink 'constants'
local MAV_SEVERITY_WARNING     =   4
local MAV_SEVERITY_NOTICE      =   5
local MAV_SEVERITY_INFO        =   6
local MAV_CMD_NAV_WAYPOINT     =  16

local last_msn_state = mission.MISSION_COMPLETE
local last_nav_index = 0
local last_location = Location()
local resume_index = 0

local function validate_resume_wp(index, f)
    if not index then return 0 end
    if not f then return 0 end
    local wp = mission:get_item(index)
    if not wp then return 0 end
    local s = f:read()
    local val = tonumber(s) or 0
    if wp:command() ~= val then return 0 end
    s = f:read()
    val = tonumber(s) or 0
    if wp:x() ~= val then return 0 end
    s = f:read()
    val = tonumber(s) or 0
    if wp:y() ~= val then return 0 end
    s = f:read()
    val = tonumber(s) or 0
    if math.floor(wp:z()) ~= val then return 0 end
    return index
end

local function get_resume_index()
    local file = io.open(RESUME_FILENAME, 'r')
    if not file then return 0 end
    local line  = file:read()
    if line then
        local index = tonumber(line) or 0
        if index == 0 then return 0 end
        index = validate_resume_wp(index, file)
        file:close()
        if index == 0 then
            gcs:send_text(MAV_SEVERITY_WARNING, 'Mission changed...unable to resume!')
        end
        return index
    end
    return 0
end

local function create_waypoint(location)
    local item = mavlink_mission_item_int_t()
    item:command(MAV_CMD_NAV_WAYPOINT)
    item:x(location:lat())
    item:y(location:lng())
    item:z(location:alt() / 100)
    return item
end

local function read_mission()
    local items = {}
    for n = 1, mission:num_commands() do
        items[#items+1] = mission:get_item(n)
    end
    return items
end

local function write_mission(items)
    for index, item in ipairs(items) do
        mission:set_item(index, item)
    end
end

function update()
    if not arming:is_armed() then return update, RUN_INTERVAL_MS end

    local msn_state = mission:state()

    -- if we read a non-zero index value, then assume we should resume
    if resume_index > 0 and
       msn_state == mission.MISSION_RUNNING and  -- only resume when the mission is running
       mission:get_current_nav_id() == MAV_CMD_NAV_WAYPOINT then  -- should ensure takeoff is complete
            mission:set_current_cmd(resume_index)
            gcs:send_text(MAV_SEVERITY_NOTICE, ('Resuming mission at WP %d'):format(resume_index))
            local file = assert(io.open(RESUME_FILENAME, 'w'), ('Mission Resume: %s file error'):format(RESUME_FILENAME))
            file:write('0\n0\n0\n0\n0')
            file:close()
            resume_index = 0
    end

    -- on battery failsafe when the mission was just active, insert a waypoint
    --    at the last stored location
    if battery:has_failsafed() and last_msn_state == mission.MISSION_RUNNING then
        local cur_msn = read_mission()
        table.insert(cur_msn, last_nav_index, create_waypoint(last_location))
        write_mission(cur_msn)
        local file = assert(io.open(RESUME_FILENAME, 'w'), ('Mission Resume: %s file error'):format(RESUME_FILENAME))
        file:write(('%d\n'):format(last_nav_index))
        file:write(('%d\n'):format(cur_msn[last_nav_index]:command()))
        file:write(('%d\n'):format(cur_msn[last_nav_index]:x()))
        file:write(('%d\n'):format(cur_msn[last_nav_index]:y()))
        file:write(('%d\n'):format(math.floor(cur_msn[last_nav_index]:z())))
        file:close()
        gcs:send_text(MAV_SEVERITY_INFO, ('Mission resume location saved at WP %d'):format(last_nav_index))
    end

    if msn_state == mission.MISSION_RUNNING then
        last_location = assert(ahrs:get_location(), 'Mission Resume: AHRS location error')
        last_nav_index = mission:get_current_nav_index()
    end

    last_msn_state = msn_state

    return update, RUN_INTERVAL_MS
end

-- check for a non-zero resume index once at boot,
--    since failsafes are typically cleared by power cycling
resume_index = get_resume_index()
if resume_index > 0 then
    gcs:send_text(MAV_SEVERITY_NOTICE, ('Mission will resume at WP %d'):format(resume_index))
    return update, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY_INFO, ('Mission will begin at WP %d'):format(math.max(mission:get_current_nav_index(), 1)))

return update, RUN_INTERVAL_MS
