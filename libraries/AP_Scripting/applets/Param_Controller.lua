--[[
   a script to select other parameters using an auxillary switch
   from subdirectories in scripts directory labeled /1,/2, or /3
--]]

local SEL_CH = 302
local PARAM_FILENAME = "params.param"

-- Global variables for chunked processing
local param_lines = {}
local param_index = 0
local param_loading = false
local param_subdir = 0
local file_handle = nil
local file_reading = false
local file_name = ""

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
   read file in chunks to avoid timeout
--]]
function read_file_chunk()
   if not file_reading or not file_handle then
      return false
   end

   local chunk_size = 200
   for _ = 1, chunk_size do
      local line = file_handle:read()
      if not line then
         file_handle:close()
         file_handle = nil
         file_reading = false
         param_loading = true
         return false
      end
      table.insert(param_lines, line)
   end
   return true
end

--[[
load parameters from a file PARAM_FILENAME from directory n
--]]
function param_load(n)
   file_name = get_scripts_dir() .. "/" .. n .."/" .. PARAM_FILENAME
  -- Open file
  file_handle = io.open(file_name)
  if not file_handle then
     gcs:send_text(0,string.format("%s not present",file_name))
     return
  end

  gcs:send_text(6,string.format("Loading config %d",n))

  -- Initialize for chunked reading
  param_lines = {}
  param_index = 1
  param_loading = false
  file_reading = true
  param_subdir = n
end

--[[
   process parameters in batches to avoid timeout
--]]
function param_load_batch()
   if not param_loading then
      return false
   end

   local count = 0
   local failed = false
   local batch_size = 200

   for _ = 1, batch_size do
      if param_index > #param_lines then
         param_loading = false
         if not failed then
            gcs:send_text(6,string.format("Config %d loaded", param_subdir))
         else
            gcs:send_text(6,string.format("Config %d loaded (some failed)", param_subdir))
         end
         return false
      end

      local line = param_lines[param_index]
      param_index = param_index + 1

      -- trim trailing spaces
      line = string.gsub(line, '^(.-)%s*$', '%1')

      -- skip empty lines and comments
      if line ~= "" and not string.match(line, "^%s*#") then
         -- Parse parameter using unified pattern for both comma and space delimited formats
         local _, _, parm, value = string.find(line, "^([%w_]+)[, ] *([%d.-]+)")

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

   return true
end

local sw_last = -1

function update()
   -- Handle chunked file reading
   if file_reading then
      read_file_chunk()
      return update, 100
   end

   -- Handle chunked parameter loading
   if param_loading then
      param_load_batch()
      return update, 200
   end

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

   -- Load parameters for new switch position
   param_load(subdir)

   return update, 100
end

gcs:send_text(5,"Loaded Parameter_Controller.lua")
return update, 500
