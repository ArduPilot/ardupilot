-- Loads one of three mission files to autopilot on each arm, depending on position of the Mission Reset AUX FUNC switch
-- Must have Mission Reset switch assigned, it will function normally when armed or disarmed
-- but also on the disarm to arm transition, it will load (if file exists) a file in the root named
-- missionH.txt, missionM.txt, or missionH.txt corresponding to the the Mission Reset switch position of High/Mid/Low
-- luacheck: only 0

local mission_loaded = false
local rc_switch = rc:find_channel_for_option(24)  --AUX FUNC sw for mission restart

if not rc_switch then  -- requires the switch to be assigned in order to run script
  return
end

local function read_mission(file_name)

   -- Open file try and read header
  local file = io.open(file_name,"r")
  local header = file:read('l')
  if not header then
    return update, 1000 --could not read, file probably does not exist
  end

  -- check header
  assert(string.find(header,'QGC WPL 110') == 1, file_name .. ': incorrect format')

  -- clear any existing mission
  assert(mission:clear(), 'Could not clear current mission')

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          gcs:send_text(6, 'loaded mission: ' .. file_name)
          file:close()
          return -- got to the end of the file
        else
          mission:clear() -- clear part loaded mission
          error('failed to read file')
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
    item:x(data[9]*10^7)
    item:y(data[10]*10^7)
    item:z(data[11])

    if not mission:set_item(index,item) then
      mission:clear() -- clear part loaded mission
      error(string.format('failed to set mission item %i',index))
    end
    index = index + 1
  end
  file:close()
end

function update()
  if not arming:is_armed() then --if disarmed, wait until armed
    mission_loaded = false
    return update,1000
  end
  if not mission_loaded then --if first time after arm and switch is valid then try to load based on switch position
    local filename
    local sw_pos = rc_switch:get_aux_switch_pos()
    if sw_pos == 0 then
        filename = 'missionL.txt'
    elseif sw_pos == 1 then
        filename = 'missionM.txt'
    else
        filename = 'missionH.txt'
    end
    mission_loaded = true
    read_mission(filename)
  end
  return update, 1000
end

return update, 5000
