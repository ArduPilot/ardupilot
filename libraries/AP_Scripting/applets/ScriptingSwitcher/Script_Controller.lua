--[[
   a script to select other lua scripts using an auxillary switch from
    /1 /2 or /3 subdirectories of the scripts directory
--]]

local THIS_SCRIPT = "Script_Controller.lua"
local SEL_CH = 302

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
   for k,v in ipairs(dlist) do
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
   for k, v in ipairs(dlist) do
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
   changes_made = activate_subdir(subdir)
   if changes_made then
      scripting:restart_all()
   else
      gcs:send_text(0, string.format("Script subbdirectory %s active", subdir))
   end
   return update, 500
end

gcs:send_text(5,"Loaded Script_Controller.lua")
return update, 500




