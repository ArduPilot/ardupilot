-- Example of loading a mission from the SD card using Scripting
-- Would be trivial to select a mission based on scripting params or RC switch
--Copy this "mission-load.lua" script to the "scripts" directory of the simulation or autopilot's SD card. 
--The "mission1.txt" file containing the mission items should be placed in the directory above the "scripts" directory. 
--In case of placing it on SD Card, mission1.txt file should be placed in the APM directory root.

---@diagnostic disable: param-type-mismatch

local function read_mission(file_name)

  -- Open file
  file = assert(io.open(file_name), 'Could not open :' .. file_name)

  -- check header
  assert(string.find(file:read('l'),'QGC WPL 110') == 1, file_name .. ': incorrect format')

  -- clear any existing mission
  assert(mission:clear(), 'Could not clear current mission')

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t()
  local index = 0
  local fail = false
  while true and not fail do
     local line = file:read()
     if not line then
        break
     end
     local ret, _, seq, _--[[ curr ]], frame, cmd, p1, p2, p3, p4, x, y, z, _--[[ autocont ]] = string.find(line, "^(%d+)%s+(%d+)%s+(%d+)%s+(%d+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+(%d+)")
     if not ret then
        fail = true
        break
     end
     if tonumber(seq) ~= index then
        fail = true
        break
     end
     item:seq(tonumber(seq))
     item:frame(tonumber(frame))
     item:command(tonumber(cmd))
     item:param1(tonumber(p1))
     item:param2(tonumber(p2))
     item:param3(tonumber(p3))
     item:param4(tonumber(p4))
     if mission:cmd_has_location(tonumber(cmd)) then
        item:x(math.floor(tonumber(x)*10^7))
        item:y(math.floor(tonumber(y)*10^7))
     else
        item:x(math.floor(tonumber(x)))
        item:y(math.floor(tonumber(y)))
     end
     item:z(tonumber(z))
     if not mission:set_item(index,item) then
        mission:clear() -- clear part loaded mission
        fail = true
        break
     end
     index = index + 1
  end
  if fail then
     mission:clear()  --clear anything already loaded
     error(string.format('failed to load mission at seq num %u', index))
  end
  gcs:send_text(0, string.format("Loaded %u mission items", index))
end

function update()
  read_mission('mission1.txt')
  return
end

return update, 5000
