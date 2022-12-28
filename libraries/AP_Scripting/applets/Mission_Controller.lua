--[[
   a script to select missions using an auxillary switch
   from subdirectories in scripts directory labeled /1,/2, or /3
--]]

local SEL_CH = 302
local MISSION_FILENAME = "mission.txt"

--[[
   check that directory exists
--]]
function check_subdir_exists(n)
  return dirlist(get_scripts_dir() .. "/" .. n )
end

--[[
   get the path to the scripts directory. This will be scripts/ on SITL
   and APM/scripts on a ChibiOS board
--]]
function get_scripts_dir()
   local dlist1 = dirlist("APM/scripts")
   if dlist1 and #dlist1 > 0 then
      return "APM/scripts"
   end
   -- otherwise assume scripts/
   return "scripts"
end

 --[[ 
   load a mission from a MISSION_FILENAME file in subdirectory n
 --]]
function mission_load(n)
  file_name = get_scripts_dir() .. "/" .. n .."/" .. MISSION_FILENAME
  -- Open file
  file = io.open(file_name)
  if not file then
     gcs:send_text(0,string.format("%s not present",file_name))
     return
  end
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
 
local sw_last = -1
function update()
   local sw_current = rc:get_aux_cached(SEL_CH)
   if sw_current == sw_last then
      return update, 500
   end
   if sw_current == 0 then 
        subdir = 1
      elseif sw_current == 2 then
        subdir = 3
      else
        subdir = 2
   end
   sw_last = sw_current
   if not check_subdir_exists(subdir) then
      gcs:send_text(0,string.format("Scripts subdirectory /%s does not exist!",subdir))
      return update, 500
   end
   mission_load(subdir)
   return update, 500
end

gcs:send_text(5,"Loaded Mission_Controller.lua")
return update, 500
