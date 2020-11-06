-- Example of loading a mission from the SD card using Scripting
-- Would be trivial to select a mission based on scripting params or RC switch

local function read_mission(file_name)

  -- Open file
  file = assert(io.open(file_name), 'Could open :' .. file_name)

  -- check header
  assert(string.find(file:read('l'),'QGC WPL 110') == 1, file_name .. ': incorrect format')

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

end

function update()
  read_mission('Tools/autotest/Generic_Missions/CMAC-bigloop.txt')
  return
end

return update, 5000
