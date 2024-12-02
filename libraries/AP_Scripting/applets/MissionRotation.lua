-- Script to load one of up to 10 mission files based on AUX switch state
-- Always loads mission0.txt at boot, cycles missions on AUX high if held less than 3 seconds
-- Reset the mission0.txt if AUX is high for more than 3 seconds.

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local rc_switch = rc:find_channel_for_option(24)  -- AUX FUNC switch for mission loading
if not rc_switch then
    gcs:send_text(MAV_SEVERITY.ERROR, "Mission Reset switch not assigned.")
    return
end

local last_sw_pos = -1  -- Track the last switch position
local current_mission_index = 0  -- Start with mission0.txt
local max_missions = 9  -- Maximum mission index (mission0.txt to mission9.txt)
local high_timer = 0  -- Timer to track how long AUX stays high

local function read_mission(file_name)
    local file = io.open(file_name, "r")
    if not file then
        return false
    end

    local header = file:read('l')
    assert(string.find(header, 'QGC WPL 110') == 1, file_name .. ': incorrect format')

    assert(mission:clear(), 'Could not clear current mission')

    local item = mavlink_mission_item_int_t()
    local index = 0

    while true do
        local data = {}
        for i = 1, 12 do
            data[i] = file:read('n')
            if data[i] == nil then
                if i == 1 then
                    gcs:send_text(MAV_SEVERITY.WARNING, "Loaded mission: " .. file_name)
                    file:close()
                    return true
                else
                    mission:clear()
                    error('Failed to read file: premature end of data')
                end
            end
        end

        item:seq(data[1])
        item:frame(data[3])
        item:command(data[4])
        item:param1(data[5])
        item:param2(data[6])
        item:param3(data[7])
        item:param4(data[8])
        item:x(data[9] * 10^7)
        item:y(data[10] * 10^7)
        item:z(data[11])

        if not mission:set_item(index, item) then
            mission:clear()
            error(string.format('Failed to set mission item %i', index))
        end
        index = index + 1
    end
end

-- Ensure mission0.txt exists and load it; stop if not found
if not read_mission("mission0.txt") then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "Critical error: mission0.txt not found, script stopped.")
    return
end

local function load_next_mission()
    current_mission_index = (current_mission_index + 1) % (max_missions + 1)
    local file_name = string.format("mission%d.txt", current_mission_index)
    if read_mission(file_name) then
        gcs:send_text(MAV_SEVERITY.WARNING, "Loaded mission: " .. file_name)
    end
end

function update()
    local sw_pos = rc_switch:get_aux_switch_pos()

    if sw_pos == 2 then  -- AUX high position
        high_timer = high_timer + 1
        if high_timer >= 3 then  -- 3 seconds elapsed
            current_mission_index = 0
            read_mission("mission0.txt")
            gcs:send_text(MAV_SEVERITY.WARNING, "Reset to mission0.txt")
            high_timer = 0  -- Reset the timer after reset
        end
    else
        if high_timer > 0 and high_timer < 3 then
            load_next_mission()  -- Load next mission only if held high for less than 3 seconds
        end
        high_timer = 0  -- Reset timer when AUX is not high
    end

    last_sw_pos = sw_pos
    return update, 1000  -- Run every second
end

gcs:send_text(MAV_SEVERITY.NOTICE, "MissionSelector loaded")
return update, 1000
