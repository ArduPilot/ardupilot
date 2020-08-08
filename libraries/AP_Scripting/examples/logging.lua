-- example of logging to a file on the SD card and to data flash
local file_name = "AHRS_DATA.csv"
local file

-- index for the data and table
local roll = 1
local pitch = 2
local yaw = 3
local interesting_data = {}

local function write_to_file()

  if not file then
    error("Could not open file")
  end

  -- write data
  -- separate with comas and add a carriage return
  file:write(tostring(millis()) .. ", " .. table.concat(interesting_data,", ") .. "\n")

  -- make sure file is upto date
  file:flush()

end

local function write_to_dataflash()

  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- lua automatically adds a timestamp in micro seconds
  logger.write('SCR','roll(deg),pitch(deg),yaw(deg)','fff',interesting_data[roll],interesting_data[pitch],interesting_data[yaw])

end

function update()

  -- get some interesting data
  interesting_data[roll] = math.deg(ahrs:get_roll())
  interesting_data[pitch] = math.deg(ahrs:get_pitch())
  interesting_data[yaw] = math.deg(ahrs:get_yaw())

  -- write to then new file the SD card
  write_to_file()

  -- write to a new log in the data flash log
  write_to_dataflash()

  return update, 1000 -- reschedules the loop
end

-- make a file
-- note that this appends to the same the file each time, you will end up with a very big file
-- you may want to make a new file each time using a unique name
file = io.open(file_name, "a")
if not file then
  error("Could not make file")
end

-- write the CSV header
file:write('Time Stamp(ms), roll(deg), pitch(deg), yaw(deg)\n')
file:flush()

return update, 10000
