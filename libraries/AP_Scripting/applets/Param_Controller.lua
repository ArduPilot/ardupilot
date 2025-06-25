--[[
   a script to select other parameters using an auxillary switch
   from subdirectories in scripts directory labeled /1,/2, or /3
--]]

local SEL_CH = 302
local PARAM_FILENAME = "params.param"

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
load parameters from a file PARAM_FILENAME from directory n
--]]
function param_load(n)
   count = 0
   failed = false
   file_name = get_scripts_dir() .. "/" .. n .."/" .. PARAM_FILENAME
  -- Open file
  file = io.open(file_name)
  if not file then
     gcs:send_text(0,string.format("%s not present",file_name))
     return
  end
   while true do
      local line = file:read()
      if not line then
         break
      end
      -- trim trailing spaces
      line = string.gsub(line, '^(.-)%s*$', '%1')

      -- skip empty lines and comments
      if line ~= "" and not string.match(line, "^%s*#") then
         -- Try comma-separated format first (PARAM,value)
         local _, _, parm, value = string.find(line, "^([%w_]+),([%d.-]+)")
         if not parm then
            -- Fall back to space-separated format (PARAM value)
            _, _, parm, value = string.find(line, "^([%w_]+)%s+([%d.-]+)")
         end

         if parm and value then
            local num_value = tonumber(value)
            if num_value then
               if param:set(parm, num_value) then
                  count = count + 1
               else
                  failed = true
               end
            else
               failed = true
            end
         end
      end
   end 
   if not failed then
      gcs:send_text(6,string.format("Loaded %u parameters",count)) 
   else
      gcs:send_text(6,string.format("Loaded %u parameters but some params did not exist to set",count)) 
   end
end   
 
local sw_last = -1
local load_param = true

function update()
   local sw_current = rc:get_aux_cached(SEL_CH)
   if (sw_current == sw_last) or (sw_current == nil) then
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
   if load_param then
      param_load(subdir)
      load_param = false
   end
   
   return update, 500
end

gcs:send_text(5,"Loaded Parameter_Controller.lua")
return update, 500
