--[[
   a script to select other lua scripts using an auxillary switch
--]]

local THIS_SCRIPT = "script_controller.lua"

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

function file_exists(fname)
   local f = io.open(fname,"rb")
   if not f then
      return false
   end
   f:close()
   return true
end

--[[
   remove any lua scripts in the scripts directory that are not in the given subdir
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
      if suffix == ".lua" and v ~= THIS_SCRIPT then
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
      gcs:send_text(0, string.format("checking %s", v))
      if suffix == ".lua" and v ~= THIS_SCRIPT then
         local src = subdir .. "/" .. v
         local dest = sdir .. "/" .. v
         if not file_compare(src, dest) then
            ret = true
            gcs:send_text(0, string.format("copying %s -> %s", src, dest))
            file_copy(src, dest)
         else
            gcs:send_text(0, string.format("same %s -> %s", src, dest))
         end
      end
   end
   return ret
end

--[[
   activate a scripting subdirectory
--]]
function activate_subdir(n)
   gcs:send_text(0, string.format("Activating %s", n))
   -- step1, remove lua files from scripts/ that are not in the givem subdirectory
   local subdir = get_scripts_dir() .. "/" .. n
   local changes_made = remove_scripts(subdir)
   changes_made = changes_made or copy_scripts(subdir)
end

activate_subdir(1)
