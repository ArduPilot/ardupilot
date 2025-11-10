--[[
   demonstrate using the gcs:command_int() interface to send commands from MAVLink MAV_CMD_xxx set
--]]

local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

local MAV_CMD_DO_SET_MODE = 176
local MAV_CMD_DO_REPOSITION = 192

-- some plane flight modes for testing
local MODE_LOITER = 12
local MODE_GUIDED = 15

local MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

--[[
create a NaN value
--]]
local function NaN()
   return 0/0
end

--[[
   test API calls. When in LOITER mode change to GUIDED and force flying to a location NE of home
--]]
local function test_command_int()
   if vehicle:get_mode() ~= MODE_LOITER then
      return
   end
   local home = ahrs:get_home()
   if not home then
      return
   end

   -- force mode GUIDED
   gcs:run_command_int(MAV_CMD_DO_SET_MODE, { p1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2 = MODE_GUIDED })

   -- and fly to 200m NE of home and 100m above home
   local dest = home:copy()
   dest:offset_bearing(45, 200)

   gcs:run_command_int(MAV_CMD_DO_REPOSITION, { frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                p1 = -1,
                                                p4 = NaN(),
                                                x = dest:lat(),
                                                y = dest:lng(),
                                                z = 100 })
end

local function update()
   test_command_int()
   return update, 1000
end

return update, 1000

