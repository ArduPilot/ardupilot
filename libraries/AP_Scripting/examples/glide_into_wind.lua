-- Glide into wind, LUA script for glide into wind functionality

-- Background
-- When flying a fixed-wing drone on ad-hoc BVLOS missions, it might not be
-- suitable for the drone to return to home if the C2 link is lost, since that
-- might mean flying without control for an extended time and distance. One
-- option in ArduPlane is to set FS_Long to Glide, which makes the drone glide
-- and land in the direction it happened to have when the command was invoked,
-- without regard to the wind. This script offers a way to decrease the kinetic
-- energy in this blind landing by means of steering the drone towards the wind
-- as GLIDE is initiated, hence lowering the ground speed. The intention is to
-- minimize impact energy at landing - foremost for any third party, but also to
-- minimize damage to the drone.

-- Functionality and setup
-- 1. Set SCR_ENABLE = 1
-- 2. Put script in scripts folder, boot twice
-- 3. A new parameter has appeared:
--    - GLIDE_WIND_ENABL (0=disable, 1=enable)
-- 4. Set GLIDE_WIND_ENABL = 1
-- 5. Read the docs on FS:
--    https://ardupilot.org/plane/docs/apms-failsafe-function.html#failsafe-parameters-and-their-meanings
-- 6. Set FS_LONG_ACTN = 2
-- 7. Set FS_LONG_TIMEOUT as appropriate
-- 8. Set FS_GCS_ENABL = 1
-- 9. If in simulation, set SIM_WIND_SPD = 4 to get a reliable wind direction.
-- 10. Test in simulation: Fly a mission, disable heartbeats by typing 'set
--     heartbeat 0' into mavproxy/SITL, monitor what happens in the console. If
--     QGC or similar GCS is used, make sure it does not send heartbeats.
-- 11. Test in flight: Fly a mission, monitor estimated wind direction from GCS,
--     then fail GCS link and see what happens.
-- 12. Once heading is into wind script will stop steering and not steer again
--     until state machine is reset and failsafe is triggered again. Steering in low
--     airspeeds (thr=0) increases risks of stall and it is preferable touch
--     ground in level roll attitude. If the script parameter hdg_ok_lim is set
--     to tight or the wind estimate is not stable, the script will anyhow stop
--     steering after override_time_lim and enter FBWA - otherwise the script
--     would hinder the GLIDE fail safe.
-- 13. Script will stop interfering as soon as a new goto-point is received or
--     the flight mode is changed by the operator or the remote pilot.

-- During the fail safe maneuver a warning tune is played.

-- State machine
-- CAN_TRIGGER 
--   - Do: Nothing
--   - Change state: If the failsafe GLIDE is triggered: if FS_GCS_ENABL is set
--     and FS_LONG_ACTN is 2, change to TRIGGERED else change to CANCELED
--
-- TRIGGERED
--   - Do: First use GUIDED mode to steer into wind, then switch to FBWA to
--     Glide into wind. Play warning tune.
--   - Change state: If flight mode is changed by operator/remote pilot or
--     operator/remote pilot sends a new goto point, change state to CANCELED
--
-- CANCELED 
--   - Do: Nothing
--   - Change state: When new heart beat arrive, change state to CAN_TRIGGER

-- Credits
-- This script is developed by agising at UASolutions, commissioned by, and in
-- cooperation with Remote.aero, with funding from Swedish AeroEDIH, in response
-- to a need from the Swedish Sea Rescue Society (Sjöräddningssällskapet, SSRS). 

-- Disable diagnostics related to reading parameters to pass linter
---@diagnostic disable: need-check-nil
---@diagnostic disable: param-type-mismatch

-- Tuning parameters
local looptime = 250            -- Short looptime
local long_looptime = 2000      -- Long looptime, GLIDE_WIND is not enabled
local tune_repeat_t = 1000      -- How often to play tune in glide into wind, [ms]
local hdg_ok_lim = 15           -- Acceptable heading error in deg (when to stop steering)
local hdg_ok_t_lim = 5000       -- Stop steering towards wind after hdg_ok_t_lim ms with error less than hdg_ok_lim
local override_time_lim = 15000 -- Max time in GUIDED during GLIDE, after limit set FBWA independent of hdg

-- GCS text levels
local _INFO = 6
local _WARNING = 4

-- Plane flight modes mapping
local mode_FBWA = 5
local mode_GUIDED = 15

-- Tunes
local _tune_glide_warn = "MFT240 L16 cdefgfgfgfg"   --  The warning tone played during GLIDE_WIND

--State variable
local fs_state = nil

-- Flags
local override_enable = false     -- Flag to allow RC channel loverride

-- Variables
local wind_dir_rad = nil                -- param for wind dir in rad
local wind_dir_180 = nil                -- param for wind dir in deg
local hdg_error = nil                   -- Heading error, hdg vs wind_dir_180
local gw_enable = nil                   -- glide into wind enable flag
local hdg = nil                         -- vehicle heading
local wind = Vector3f()                 -- wind 3Dvector
local link_lost_for = nil               -- link loss time counter
local last_seen = nil                   -- timestamp last received heartbeat
local tune_time_since = 0               -- Timer for last played tune
local hdg_ok_t = 0                      -- Timer
local expected_flight_mode = nil        -- Flight mode set by this script
local location_here = nil               -- Current location
local location_upwind = nil             -- Location to hold the target location
local user_notified = false             -- Flag to keep track user being notified or not
local failed_location_counter = 0       -- Counter for failed location requests, possible GPS denied
local upwind_distance = 500             -- Distance to the upwind location, minimum 4x turn radius
local override_time = 0                 -- Time since override started in ms

-- Add param table
local PARAM_TABLE_KEY = 74
local PARAM_TABLE_PREFIX = "GLIDE_WIND_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'could not add param table')

-------
-- Init
-------

function _init()
  -- Add and init paramters
  GLIDE_WIND_ENABL = bind_add_param('ENABL', 1, 0)

  -- Init parameters
  FS_GCS_ENABL = bind_param('FS_GCS_ENABL')               -- Is set to 1 if GCS lol should trigger FS after FS_LONG_TIMEOUT
  FS_LONG_TIMEOUT = bind_param('FS_LONG_TIMEOUT')         -- FS long timeout in seconds
  FS_LONG_ACTN = bind_param('FS_LONG_ACTN')               -- Is set to 2 for Glide

  send_to_gcs(_INFO, 'LUA: FS_LONG_TIMEOUT timeout: ' .. FS_LONG_TIMEOUT:get() .. 's')

  -- Test paramter
  if GLIDE_WIND_ENABL:get() == nil then
    send_to_gcs(_INFO, 'LUA: Something went wrong, GLIDE_WIND_ENABL not created')
    return _init(), looptime
  else 
    gw_enable = GLIDE_WIND_ENABL:get()
    send_to_gcs(_INFO, 'LUA: GLIDE_WIND_ENABL: ' .. gw_enable)
  end

  -- Init last_seen.
  last_seen = gcs:last_seen()

  -- Init link_lost_for [ms] to FS_LONG_TIMEOUT [s] to prevent link to recover without
  -- new heartbeat. This is to properly init the state machine.
  link_lost_for = FS_LONG_TIMEOUT:get() * 1000

  -- Warn if GLIDE_WIND_ENABL is set and FS_LONG_ACTN is not GLIDE
  if gw_enable == 1 and FS_LONG_ACTN:get() ~= 2 then
    send_to_gcs(_WARNING, 'GLIDE_WIND_ENABL is set, but FS_LONG_ACTN is not GLIDE.')
  end

  -- Init fs_state machine to CANCELED. A heartbeat is required to set the state
  -- to CAN_TRIGGER from where Glide into wind can be triggered.
  fs_state = 'CANCELED'

  -- All set, go to update
  return update(), long_looptime
end


------------
-- Main loop
------------

function update()
  -- Check if state of GLIDE_WIND_ENABL parameter changed, print every change
  if gw_enable ~= GLIDE_WIND_ENABL:get() then
    gw_enable = GLIDE_WIND_ENABL:get()
    send_to_gcs(_INFO, 'LUA: GLIDE_WIND_ENABL: ' .. gw_enable)
    -- If GLIDE_WIND_ENABL was enabled, warn if not FS_LONG_ACTN is set accordingly
    if gw_enable == 1 then
      if FS_LONG_ACTN:get() ~=2 then
        send_to_gcs(_WARNING, 'GLIDE_WIND_ENABL is set, but FS_LONG_ACTN is not GLIDE.')
      end
    end
  end

  -- -- If feature is not enabled, loop slowly
  if gw_enable == 0 then
    return update, long_looptime
  end
  
  -- GLIDE_WIND_ENABL is enabled, look for triggers
  -- Monitor time since last gcs heartbeat
  if last_seen == gcs:last_seen() then
    link_lost_for = link_lost_for + looptime
  else
    -- There has been a new heartbeat, update last_seen and reset link_lost_for
    last_seen = gcs:last_seen()
    link_lost_for = 0
  end

  -- Run the state machine
  -- State CAN_TRIGGER
  if fs_state == 'CAN_TRIGGER' then
    if link_lost_for > FS_LONG_TIMEOUT:get() * 1000 then
      -- Double check that FS_GCS_ENABL is set
      if FS_GCS_ENABL:get() == 1 and FS_LONG_ACTN:get() == 2 then
        fs_state = "TRIGGERED"
        -- Reset some variables
        hdg_ok_t = 0
        user_notified = false
        override_enable = true
        override_time = 0
        failed_location_counter = 0
        -- Set mode to GUIDED before entering TRIGGERED state
        set_flight_mode(mode_GUIDED, 'LUA: Glide into wind state TRIGGERED')
      else
        -- Do not trigger glide into wind, require new heart beats to get here again
        fs_state = "CANCELED"
      end
    end
  -- State TRIGGERED
  elseif fs_state == "TRIGGERED" then
    -- Check for flight mode changes from outside script
    if vehicle:get_mode() ~= expected_flight_mode then
      fs_state = "CANCELED"
      send_to_gcs(_INFO, 'LUA: Glide into wind state CANCELED: flight mode change')
    end

    -- In GUIDED, check for target location changes from outside script (operator)
    if vehicle:get_mode() == mode_GUIDED then
      if not locations_are_equal(vehicle:get_target_location(), location_upwind) then
        fs_state = "CANCELED"
        send_to_gcs(_INFO, 'LUA: Glide into wind state CANCELED: new goto-point')
      end
    end

  -- State CANCELED
  elseif fs_state == "CANCELED" then
    -- Await link is not lost
    if link_lost_for < FS_LONG_TIMEOUT:get() * 1000 then
      fs_state = "CAN_TRIGGER"
      send_to_gcs(_INFO, 'LUA: Glide into wind state CAN_TRIGGER')
    end
  end

  -- State TRIGGERED actions
  if fs_state == "TRIGGERED" then
    -- Get the heading angle
    hdg = math.floor(math.deg(ahrs:get_yaw_rad()))

    -- Get wind direction. Function wind_estimate returns x and y for direction wind blows in, add pi to get true wind dir
    wind = ahrs:wind_estimate()
    wind_dir_rad = math.atan(wind:y(), wind:x())+math.pi
    wind_dir_180 = math.floor(wrap_180(math.deg(wind_dir_rad)))
    hdg_error = wrap_180(wind_dir_180 - hdg)

    -- Check if we are close to target heading
    if math.abs(hdg_error) < hdg_ok_lim then
      -- If we have been close to target heading for hdg_ok_t_lim, switch back to FBWA
      if hdg_ok_t > hdg_ok_t_lim then    
        if override_enable then
          set_flight_mode(mode_FBWA,'LUA: Glide into wind steering complete, GLIDE in FBWA')
        end
        -- Do not override again until state machine has triggered again
        override_enable = false
      else
        hdg_ok_t = hdg_ok_t + looptime
      end
    -- Heading error is big, reset timer hdg_ok_t 
    else
      hdg_ok_t = 0
    end

    -- Play tune every tune_repeat_t [ms]
    if tune_time_since > tune_repeat_t  then
      -- Play tune and reset timer
      send_to_gcs(_INFO, 'LUA: Play warning tune')
      play_tune(_tune_glide_warn)
      tune_time_since = 0
    else
      tune_time_since = tune_time_since + looptime
    end

    -- If not steered into wind yet, update goto point into wind
    if override_enable then
      -- Check override time, if above limit, switch back to FBWA
      override_time = override_time + looptime
      if override_time > override_time_lim then
        set_flight_mode(mode_FBWA, "LUA: Glide into wind override time out, GLIDE in current heading")
        override_enable = false
      end
      -- Get current position and handle if not valid
      location_here = ahrs:get_location()
      if location_here == nil then
        -- In case we cannot get location for some time we must give up and continue with GLIDE
        failed_location_counter = failed_location_counter + 1
        if failed_location_counter > 5 then
          set_flight_mode(mode_FBWA, "LUA: Glide failed to get location, GLIDE in current heading")
          override_enable = false
          return update, looptime
        end
        gcs:send_text(_WARNING, "LUA: Glide failed to get location")
        return update, looptime
      end
      -- Calc upwind position, copy and modify location_here
      location_upwind = location_here:copy()
      location_upwind:offset_bearing(wind_dir_180, upwind_distance)

      -- Set location_upwind as GUIDED target
      if vehicle:set_target_location(location_upwind) then
        if not user_notified then
          send_to_gcs(_INFO, "LUA: Guided target set " .. upwind_distance .. "m away at bearing " .. wind_dir_180)
          -- Just notify once
          user_notified = true
        end
      else
        -- Most likely we are not in GUIDED anymore (operator changed mode), state machine will handle this in next loop.
        gcs:send_text(_WARNING, "LUA: Glide failed to set upwind target")
      end
    end
  end
  return update, looptime
end


-------------------
-- Helper functions
-------------------

-- Set mode and wait for mode change
function set_flight_mode(mode, message)
  expected_flight_mode = mode
  vehicle:set_mode(expected_flight_mode)
  return wait_for_mode_change(mode, message, 0)
end

-- Wait for mode change
function wait_for_mode_change(mode, message, attempt)
  -- If mode change does not go through after 10 attempts, give up
  if attempt > 10 then
    send_to_gcs(_WARNING, 'LUA: Glide into wind mode change failed.')
    return update, looptime
  -- If mode change does not go through, wait and try again
  elseif vehicle:get_mode() ~= mode then
    return wait_for_mode_change(mode, message, attempt + 1), 5
  -- Mode change has gone through
  else
    send_to_gcs(_INFO, message)
    return update, looptime
  end
end

-- Function to compare two Location objects
function locations_are_equal(loc1, loc2)
  -- If either location is nil, they are not equal  
  if not loc1 or not loc2 then
      return false
  end
  -- Compare latitude and longitude, return bool
  return loc1:lat() == loc2:lat() and loc1:lng() == loc2:lng()
end

-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end

-- Add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
  return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Print to GCS
function send_to_gcs(level, mess)
  gcs:send_text(level, mess)
end

-- Play tune
function play_tune(tune)
  notify:play_tune(tune)
end

-- Returns the angle in range 0-360
function wrap_360(angle)
  local res = math.fmod(angle, 360.0)
   if res < 0 then
       res = res + 360.0
   end
   return res
end

-- Returns the angle in range -180-180
function wrap_180(angle)
  local res = wrap_360(angle)
  if res > 180 then
     res = res - 360
  end
  return res
end

-- Start up the script
return _init, 2000
