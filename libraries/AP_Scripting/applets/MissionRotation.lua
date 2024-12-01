-- Script to load one of up to 10 mission files based on AUX switch state
-- Always loads mission0.txt at boot, then cycles through files on AUX high transitions

local rc_switch = rc:find_channel_for_option(24)  -- AUX FUNC switch for mission loading
if not rc_switch then
    gcs:send_text(6, "Mission Reset switch not assigned.")
    return
end

local last_sw_pos = -1  -- Track the last switch position
local current_mission_index = 0  -- Start with mission0.txt
local max_missions = 9  -- Maximum mission index (mission0.txt to mission9.txt)

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
                    gcs:send_text(6, "Loaded mission: " .. file_name)
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
    gcs:send_text(6, "Critical error: mission0.txt not found. Script stopped.")
    return
end

local function load_next_mission()
    while true do
        local file_name = string.format("mission%d.txt", current_mission_index)
        if read_mission(file_name) then
            break
        end
        current_mission_index = (current_mission_index + 1) % (max_missions + 1)
    end
end

function update()
    local sw_pos = rc_switch:get_aux_switch_pos()

    -- Check if AUX switch transitioned from low to high
    if sw_pos == 2 and last_sw_pos ~= 2 then
        current_mission_index = (current_mission_index + 1) % (max_missions + 1)
        load_next_mission()
    end

    last_sw_pos = sw_pos

    return update, 1000
end

gcs:send_text(5, "MissionRotation.lua loaded")
return update, 1000
