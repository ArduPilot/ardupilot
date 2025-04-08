-- Example of saving the current mission to a file on the SD card on arming

---@diagnostic disable: need-check-nil

local function save_to_SD()

  -- check if there is a mission to save
  local num_wp = mission:num_commands()
  if num_wp <= 1 then
    return
  end

  local index = 0
  local file_name
  -- search for a index without a file
  while true do
    file_name = string.format('%i.waypoints',index)
    local file = io.open(file_name)
    if file == nil then
      break
    end
    local first_line = file:read(1) -- try and read the first character
    io.close(file)
    if first_line == nil then
      break
    end
    index = index + 1
  end

  -- create new file
  file = assert(io.open(file_name, 'w'), 'Could not make file :' .. file_name)

  -- header
  file:write('QGC WPL 110\n')

  -- read each item and write to file
  for i = 0, num_wp - 1 do
    local item = mission:get_item(i)
    file:write(string.format('%i\t0\t%i\t%i\t%0.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n',item:seq(),item:frame(),item:command(),item:param1(),item:param2(),item:param3(),item:param4(),item:x()*10^-7,item:y()*10^-7,item:z()))
  end

  file:close()

  gcs:send_text(6,'saved mission to: ' .. file_name)

end

function idle_disarmed()
  if arming:is_armed() then
    save_to_SD()
    return idle_armed, 1000
  end
  return idle_disarmed, 1000
end

function idle_armed()
  if not arming:is_armed() then
    return idle_disarmed, 1000
  end
  return idle_armed, 1000
end

return idle_disarmed, 1000
