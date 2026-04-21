--[[

   example script to show interrupting a mission and then 
   reseting the crosstracking to the correct line when
   returning to the mission after the interruption

   this functionality is only available in Plane
--]]

SCRIPT_NAME = "Crosstrack Restore"
SCRIPT_NAME_SHORT = "XTrack"
SCRIPT_VERSION = "4.6.0-001"


MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

local last_sw = -1
local AUX_FN = 300

-- Attempts to duplicate the code that updates the prev_WP_loc variable in the c++ code
local function LocationTracker()

   local self = {}

   -- to get this to work, need to keep 2 prior generations of "target_location"
   local previous_target_location            -- the target prior to the current one
   local previous_previous_target_location   -- the target prior to that - this is the one we want

   function self.same_loc_as(A, B)
      if A == nil or B == nil then
         return false
      end
      if (A:lat() ~= B:lat()) or (A:lng() ~= B:lng()) then
         return false
      end
      return (A:alt() == B:alt()) and (A:get_alt_frame() == B:get_alt_frame())
   end

   function self.save_previous_target_location()
      local target_location = vehicle:get_target_location()
      if target_location ~= nil then
         if not self.same_loc_as(previous_target_location, target_location) then
            -- maintain three generations of location
            previous_previous_target_location = previous_target_location
            previous_target_location = target_location
         end
      else
         previous_target_location = ahrs:get_location()
         previous_previous_target_location = previous_target_location
      end
   end

   function self.get_saved_location()
      return previous_previous_target_location
   end

   return self
end

local location_tracker = LocationTracker()

local function update()

   -- save the previous target location only if in auto mode, if restoring it in AUTO mode
   if vehicle:get_mode() == FLIGHT_MODE.AUTO and location_tracker ~= nil then
      location_tracker.save_previous_target_location()
   end

   local sw_current = rc:get_aux_cached(AUX_FN)
   if not sw_current then
      sw_current = 0
   end
   if sw_current ~= last_sw then
      last_sw = sw_current
      if sw_current == 0 then
        vehicle:set_mode(FLIGHT_MODE.AUTO)
        if location_tracker ~= nil then
            local previous_location = location_tracker.get_saved_location()
            if previous_location ~= nil then
               vehicle:set_crosstrack_start(previous_location)
            end
         end
         gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Switched to AUTO", SCRIPT_NAME_SHORT))
      else
         vehicle:set_mode(FLIGHT_MODE.LOITER)
         gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Switched to LOITER", SCRIPT_NAME_SHORT))
      end
   end

   return update,100
end

if FWVersion:type() == 3 then
   gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
   return update()
else
   gcs:send_text(MAV_SEVERITY.NOTICE,string.format("%s: Must run on Plane", SCRIPT_NAME_SHORT))
end
