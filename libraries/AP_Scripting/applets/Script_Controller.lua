--[[
   a script to select other lua scripts using an auxillary switch from
    /1 /2 or /3 subdirectories of the scripts directory
--]]
---@diagnostic disable: param-type-mismatch


local THIS_SCRIPT = "Script_Controller.lua"
local sel_ch = Parameter("SCR_USER6")
SEL_CH = sel_ch:get()
if SEL_CH == 0 then
   SEL_CH = 302
end

MISSION_FILENAME = "mission.txt"
--[[
   check that directory exists
--]]
function check_subdir_exists(n)
  return dirlist(get_scripts_dir() .. "/" .. n )
end

--[[
   copy file src to dest, return true on success
--]]
function file_copy(src, dest)
   local block_size = 256
   local file1 = io.open(src, "rb")
   if not file1 then
      return false
   end
   local file2 = io.open(dest, "wb")
   if not file2 then
      file1:close()
      return false
   end
   while true do
      local block = file1:read(block_size)
      if not block then
         break
      end
      file2:write(block)
   end
   local ret = false
   if file1:seek("end") == file2:seek("end") then
      ret = true
   end
   file1:close()
   file2:close()
   return ret
end

--[[
   compare two files, return true if they are the same
--]]
function file_compare(filename1, filename2)
   local block_size = 256
   local file1 = io.open(filename1, "rb")
   if not file1 then
      return false
   end
   local file2 = io.open(filename2, "rb")
   if not file2 then
      file1:close()
      return false
   end
   local ret = true
   while true do
      local block1 = file1:read(block_size)
      local block2 = file2:read(block_size)
      if block1 ~= block2 then
         ret = false
         break
      end
      if not block1 then
         break
      end
   end
   file1:close()
   file2:close()
   return ret
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
    check if a file exists, returns true if it does' fname is complete path to file
--]]
function file_exists(fname)
   local f = io.open(fname,"rb")
   if not f then
      return false
   end
   f:close()
   return true
end

--[[ compare strings case insensitive and return true if match

--]]
function compare_strings_ci(a,b)
   return string.upper(a) == string.upper(b)
end

--[[
   remove any lua scripts in the scripts directory that are not in the given subdir besides ourselves
   returns true if any files were removed
--]]
function remove_scripts(subdir)
   local sdir = get_scripts_dir()
   local dlist = dirlist(sdir)
   if not dlist then
      return false
   end
   local ret = false
   for _,v in ipairs(dlist) do
      local suffix = v:sub(-4)
      if compare_strings_ci(suffix,".LUA") and not compare_strings_ci(v,THIS_SCRIPT) then
         if not file_exists(subdir .. "/" .. v) then
            ret = true
            remove(sdir .. "/" .. v)
         end
      end
   end
   return ret
end

--[[
   copy scripts from a subdir to the main scripts directory
   return true if any changes were made
--]]
function copy_scripts(subdir)
   local dlist = dirlist(subdir)
   if not dlist then
      return false
   end
   local ret = false
   local sdir = get_scripts_dir()
   for _, v in ipairs(dlist) do
      local suffix = v:sub(-4)
      if compare_strings_ci(suffix,".LUA") and not compare_strings_ci(v,THIS_SCRIPT) then
         local src = subdir .. "/" .. v
         local dest = sdir .. "/" .. v
         if not file_compare(src, dest) then
            ret = true
            file_copy(src, dest)
         end
      end
   end
   if ret then
      gcs:send_text(5,"Copied new files to scripts directory")
   end
   return ret
end

 --[[ 
   load a mission from a MISSION_FILENAME file in subdirectory n
 --]]
function mission_load(n)
  file_name = get_scripts_dir() .. "/" .. n .."/" .. MISSION_FILENAME
  -- Open file
  file = io.open(file_name)
  if not file then
     return
  end
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

--[[
   activate a scripting subdirectory
--]]
function activate_subdir(n)
   -- step1, remove lua files from scripts/ that are not in the given subdirectory
   local subdir = get_scripts_dir() .. "/" .. n
   local changes_made = remove_scripts(subdir)
   changes_made = changes_made or copy_scripts(subdir)  --copy files if different
   return changes_made
end

local sw_last = -1
function update()
   local sw_current = rc:get_aux_cached(SEL_CH)
   if sw_current == sw_last or sw_current == nil then
      return update, 500
   end
   if sw_current == 1  then 
        subdir = 2
      elseif sw_current == 2 then
        subdir = 3
      else
        subdir = 1  --default if RC not established yet
   end
   sw_last = sw_current
   if not check_subdir_exists(subdir) then
      gcs:send_text(0,string.format("Scripts subdirectory /%s does not exist!",subdir))
      return update, 500
   end

   changes_made = activate_subdir(subdir)
   if changes_made then
      scripting:restart_all()
   else
      mission_load(subdir)
      gcs:send_text(0, string.format("Scripts subbdirectory %s active", subdir))
   end
   return update, 500
end

gcs:send_text(6,"Loaded Script_Controller.lua")
return update, 500




