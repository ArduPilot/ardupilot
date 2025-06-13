--[[
   demonstrate proving HAGL to plane code for landing
--]]

local MAV_CMD_SET_HAGL = 43005

local ROTATION_PITCH_90 = 24
local ROTATION_PITCH_270 = 25

-- for normal landing use PITCH_270, for inverted use PITCH_90
local RANGEFINDER_ORIENT = ROTATION_PITCH_270

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local RNG_STATUS = { NotConnected = 0, NoData = 1, OutOfRangeLow = 2, OutOfRangeHigh = 3, Good = 4 }


--[[
 create a NaN value
--]]
local function NaN()
   return 0/0
end

local last_active = false

--[[
   send HAGL data
--]]
local function send_HAGL()
   local status = rangefinder:status_orient(RANGEFINDER_ORIENT)
   if status ~= RNG_STATUS.Good then
      last_active = false
      return
   end
   local rangefinder_dist = rangefinder:distance_orient(RANGEFINDER_ORIENT)
   local correction = math.cos(ahrs:get_roll_rad())*math.cos(ahrs:get_pitch_rad())
   local rangefinder_corrected = rangefinder_dist * correction
   if RANGEFINDER_ORIENT == ROTATION_PITCH_90 then
      rangefinder_corrected = -rangefinder_corrected
   end
   if rangefinder_corrected < 0 then
      last_active = false
      return
   end

   -- tell plane the height above ground level
   local timeout_s = 0.2
   local accuracy = NaN()
   gcs:run_command_int(MAV_CMD_SET_HAGL, { p1 = rangefinder_corrected, p2 = accuracy, p3=timeout_s })

   if not last_active then
      last_active = true
      gcs:send_text(MAV_SEVERITY.INFO, string.format("HAGL Active %.1f", rangefinder_corrected))
   end

   -- log it
   logger:write("HAGL", "RDist,HAGL", "ff", rangefinder_dist, rangefinder_corrected)
end

local function update()
   send_HAGL()
   return update, 50
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded land_hagl")

return update, 1000

