--[[
script to allow users to more easily change camera settings
--]]

local PARAM_TABLE_KEY = 85
local PARAM_TABLE_PREFIX = "CAM1_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local CAMERA_SETTINGS = {THERMAL_PALETTE=0, THERMAL_GAIN=1, THERMAL_RAW_DATA=2}  -- see AP_Camera_shareddefs.h

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')

--[[
  // @Param: CAM1_THERM_PAL
  // @DisplayName: Camera1 Thermal Palette
  // @Description: thermal image colour palette
  // @Values: -1:Leave Unchanged, 0:WhiteHot, 2:Sepia, 3:IronBow, 4:Rainbow, 5:Night, 6:Aurora, 7:RedHot, 8:Jungle, 9:Medical, 10:BlackHot, 11:GloryHot
  // @User: Standard
--]]
local CAM1_THERM_PAL = bind_add_param('THERM_PAL', 1, -1)

--[[
  // @Param: CAM1_THERM_GAIN
  // @DisplayName: Camera1 Thermal Gain
  // @Description: thermal image temperature range
  // @Values: -1:Leave Unchanged, 0:LowGain (50C to 550C), 1:HighGain (-20C to 150C)
  // @User: Standard
--]]
local CAM1_THERM_GAIN = bind_add_param('THERM_GAIN', 2, -1)

--[[
  // @Param: CAM1_THERM_RAW
  // @DisplayName: Camera1 Thermal Raw Data
  // @Description: save images with raw temperatures
  // @Values: -1:Leave Unchanged, 0:Disabled (30fps), 1:Enabled (25 fps)
  // @Units: m
  // @User: Standard
--]]
local CAM1_THERM_RAW = bind_add_param('THERM_RAW', 3, -1)

-- local variables
local update_rate_ms = 3000            -- update every 3 seconds
local CAM1_THERM_PAL_saved_value = -1  -- true if thermal palette has been saved
local CAM1_THERM_GAIN_saved_value = -1 -- true if thermal gain has been saved
local CAM1_THERM_RAW_saved_value = -1  -- true if thermal raw data has been saved

--[[
   main update function, called at 1Hz
--]]
function update()
   
   -- check if we should update any settings
   if CAM1_THERM_PAL:get() >= 0 and CAM1_THERM_PAL:get() ~= CAM1_THERM_PAL_saved_value then
      if camera:change_setting(0, CAMERA_SETTINGS.THERMAL_PALETTE, CAM1_THERM_PAL:get()) then
         gcs:send_text(MAV_SEVERITY.INFO, string.format("Camera1 Thermal Palette to %d", CAM1_THERM_PAL:get()))
         CAM1_THERM_PAL_saved_value = CAM1_THERM_PAL:get()
      else
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("Failed to set Camera1 Thermal Palette to %d", CAM1_THERM_PAL:get()))
      end
   end

   if CAM1_THERM_GAIN:get() >= 0 and CAM1_THERM_GAIN:get() ~= CAM1_THERM_GAIN_saved_value then
      if camera:change_setting(0, CAMERA_SETTINGS.THERMAL_GAIN, CAM1_THERM_GAIN:get()) then
         gcs:send_text(MAV_SEVERITY.INFO, string.format("Camera1 Thermal Gain to %d", CAM1_THERM_GAIN:get()))
         CAM1_THERM_GAIN_saved_value = CAM1_THERM_GAIN:get()
      else
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("Failed to set Camera1 Thermal Gain to %d", CAM1_THERM_GAIN:get()))
      end
   end

   if CAM1_THERM_RAW:get() >= 0 and CAM1_THERM_RAW:get() ~= CAM1_THERM_RAW_saved_value then
      if camera:change_setting(0, CAMERA_SETTINGS.THERMAL_RAW_DATA, CAM1_THERM_RAW:get()) then
         gcs:send_text(MAV_SEVERITY.INFO, string.format("Camera1 Thermal Raw Data to %d", CAM1_THERM_RAW:get()))
         CAM1_THERM_RAW_saved_value = CAM1_THERM_RAW:get()
      else
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("Failed to set Camera1 Thermal Raw Data to %d", CAM1_THERM_RAW:get()))
      end
   end

   return update, update_rate_ms
end

-- print welcome message
gcs:send_text(MAV_SEVERITY.INFO, "Loaded camera-change-settings.lua")

-- start running update loop
return update, update_rate_ms
