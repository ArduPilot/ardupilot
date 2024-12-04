-- Script to load one of up to 10 mission files based on AUX switch state
-- Always loads mission0.txt at boot, cycles missions on AUX high if held less than 3 seconds
-- Reset the mission0.txt if AUX is high for more than 3 seconds
-- Prevents mission change if the vehicle is in AUTO mode
-- Prevents the script from loading if the vehicle is in AUTO

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local rc_switch = rc:find_channel_for_option(24)

if not rc_switch then
    gcs:send_text(MAV_SEVERITY.ERROR, "Mission Reset switch not assigned")
    return
end

local vehicle_fw_type = FWVersion:type()

local vehicle_type
if vehicle_fw_type == 1 then
    vehicle_type = "Rover"
elseif vehicle_fw_type == 2 then
    vehicle_type = "Copter"
elseif vehicle_fw_type == 3 then
    vehicle_type = "Plane"
elseif vehicle_fw_type == 4 then
    vehicle_type = "Tracker"
elseif vehicle_fw_type == 5 then
    vehicle_type = "Submarine"
else
    gcs:send_text(MAV_SEVERITY.ERROR, "Unrecognized vehicle type!")
    return
end

gcs:send_text(MAV_SEVERITY.INFO, "Vehicle Type: " .. vehicle_type)

local mode_auto
if vehicle_type == "Plane" then
    mode_auto = 10
elseif vehicle_type == "Copter" then
    mode_auto = 3
elseif vehicle_type == "Rover" then
    mode_auto = 10
elseif vehicle_type == "Submarine" then
    mode_auto = 10
elseif vehicle_type == "Tracker" then
    mode_auto = 10
else
    return
end

if vehicle:get_mode() == mode_auto then
    gcs:send_text(MAV_SEVERITY.ERROR, "The script cannot be loaded in AUTO mode")
    return
end

local current_mission_index = 0
local max_missions = 9
local high_timer = 0

local function read_mission(file_name)
    local file = io.open(file_name, "r")
    if not file then
        gcs:send_text(MAV_SEVERITY.ERROR, file_name .. " not found")
        return false
    end

    local header = file:read('l')
    assert(string.find(header, 'QGC WPL 110') == 1, file_name .. ': incorrect format')

    if not mission:clear() then
        gcs:send_text(MAV_SEVERITY.ERROR, "Could not clear current mission")
        file:close()
        return false
    end

    local item = mavlink_mission_item_int_t()
    local index = 0

    while true do
        local data = {}
        for i = 1, 12 do
            data[i] = file:read('n')
            if data[i] == nil then
                if i == 1 then
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

if not read_mission("mission0.txt") then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "mission0.txt not found, script stopped")
    return
end

local function load_next_mission()
    if vehicle:get_mode() == mode_auto then
        gcs:send_text(MAV_SEVERITY.WARNING, "Cannot switch missions in AUTO mode")
        return
    end

    local attempts = 0
    repeat
        current_mission_index = (current_mission_index + 1) % (max_missions + 1)
        local file_name = string.format("mission%d.txt", current_mission_index)
        local file = io.open(file_name, "r")
        if file then
            file:close()
            if read_mission(file_name) then
                gcs:send_text(MAV_SEVERITY.WARNING, "Loaded mission: " .. file_name)
                return
            end
        end
        attempts = attempts + 1
    until attempts > max_missions

    gcs:send_text(MAV_SEVERITY.ERROR, "No valid mission files found")
end

function update()
    local sw_pos = rc_switch:get_aux_switch_pos()

    if sw_pos == 2 then
        high_timer = high_timer + 1
        if high_timer >= 3 then
            if vehicle:get_mode() ~= mode_auto then
                current_mission_index = 0
                if read_mission("mission0.txt") then
                    gcs:send_text(MAV_SEVERITY.WARNING, "Reset to mission0.txt")
                end
            end
            high_timer = 0
        end
    else
        if high_timer > 0 and high_timer < 3 then
            load_next_mission()
        end
        high_timer = 0
    end

    return update, 1000
end

gcs:send_text(MAV_SEVERITY.NOTICE, "Mission Rotation loaded")
gcs:send_text(MAV_SEVERITY.NOTICE, "Loaded default mission: mission0.txt")
return update, 1000
