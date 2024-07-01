-- Glide into wind, LUA script for glide into wind functionality

-- Background
-- When flying a fixed-wing drone on ad-hoc BVLOS missions, it might not be
-- suitable for the drone to return to home if the C2 link is lost, since that
-- might mean flying without control for an extended time and distance. One
-- option in ArduPlane is to set FS_Long to Glide, which makes the drone glide
-- and land in the direction it happened to have when the command was invoked,
-- without regard to the wind. This script ensures that the drone turns into the
-- wind as GLIDE (FBWA) is initiated, thus minimizing the speed over ground and
-- impact energy at landing.

-- Functionality and setup
-- 1. Set SCR_ENABLE = 1
-- 2. Put script in scripts folder, boot twice
-- 3. Two new paramters has appeared:
--    - GLIDE_WIND_ENABL (0=disable, 1=enable),
--    - GLIDE_WIND_RKP (kP for RC-override roll, recommended setting is 4). 
-- 4. Set RC_OVERRIDE_TIME to 0.3 if this does not conflict with other scripts.
-- 5. Set GLIDE_WIND_ENABL = 1, GLIDE_WIND_RKP = 2 (Start easy, increase if needed)
-- 6. Read the docs on FS: 
--    https://ardupilot.org/plane/docs/apms-failsafe-function.html#failsafe-parameters-and-their-meanings
-- 7. Set FS_LONG_ACTN = 2
-- 8. Set FS_LONG_TIMEOUT as appropriate
-- 9. Set FS_GCS_ENABL = 1
-- 10. If in simulation, set SIM_WIND_SPD = 4 to get a reliable wind direction.
-- 11. Test in simulation: Fly a mission, disable heartbeats by typing 'set
--     heartbeat 0' into mavproxy/SITL, monitor what happens in the console
-- 12. Test in flight: Fly a mission, monitor estimated wind direction from GCS,
--     then fail GCS link and see what happens.
-- 13. Once heading is into wind script will stop steering and not steer again
--     until link is recovered and lost an other time.
-- 14. Script is aborted by changing flight mode, from RC or recovered GCS.

-- Credit
-- This script is developed by UASolutions, commissioned by, and in cooperation
-- with Remote.aero, with funding from Swedish AeroEDIH, in response to a need
-- from the Swedish Sea Rescue Society (Sjöräddningssällskapet, SSRS). 

-- Disable diagnostics related to reading paramters to pass linter
---@diagnostic disable: need-check-nil
---@diagnostic disable: param-type-mismatch

-- Tuning parameters
local looptime = 200            -- Short looptime
local long_looptime = 2000      -- Long looptime, GLIDE_WIND is not enabled
local rlim = 300                -- Absolute roll contribution limit [PWM]
local tune_repeat_t = 1000      -- How often to play tune in glide into wind, [ms]
local hdg_ok_lim = 15           -- Acceptable heading error in deg (when to stop steering)
local hdg_ok_t_lim = 5000       -- Stop steering towards wind after hdg_ok_t_lim ms with error less than hdg_ok_lim

-- GCS text levels
local _INFO = 6
local _WARNING = 4

-- Plane flight modes
local mode_MANUAL = 0
local mode_CIRCLE = 1
local mode_STABILIZE = 2
local mode_FBWA = 5
local mode_FBWB = 6 
local mode_AUTO = 10
local mode_RTL = 11
local mode_LOITER = 12
local mode_TAKEOFF = 13
local mode_GUIDED = 15

-- Tunes
local _tune_glide_warn = "MFT240 L16 cdefgfgfgfg"   --  The warning tone played during GLIDE_WIND

-- Flags
local override_enable = false     -- Flag to allow RC channel loverride

-- Variables
local wind_dir_rad = nil          -- param for wind dir in rad
local wind_dir_180 = nil          -- param for wind dir in deg
local error = nil                 -- Control error, hdg vs wind_dir_180
local roll = nil                  -- roll control signal contribution
local gw_enable = nil             -- glide into wind enable flag
local hdg = nil                   -- vehicle heading
local wind = Vector3f()           -- wind 3Dvector
local link_lost_for = 0           -- link loss time counter
local last_seen = nil             -- timestamp last received heartbeat
local tune_time_since = 0         -- Timer for last played tune
local hdg_ok_t = 0                -- Timer

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
  GLIDE_WIND_RKP = bind_add_param('RKP', 2, 2)
  
  -- Init parameters
  FS_GCS_ENABL = bind_param('FS_GCS_ENABL')               -- Is set to 1 if GCS lol should trigger FS after FS_LONG_TIMEOUT
  FS_LONG_TIMEOUT = bind_param('FS_LONG_TIMEOUT')         -- FS long timeout in seconds
  AFS_GCS_TIMEOUT = bind_param('AFS_GCS_TIMEOUT')         -- Doc: The time (in seconds) of persistent data link loss before GCS failsafe occurs. Not sure though
  RCMAP_ROLL = bind_param('RCMAP_ROLL')                   -- Shows the channel used for Roll input
  FS_LONG_ACTN = bind_param('FS_LONG_ACTN')                -- Is set to 2 for Glide

  
  send_to_gcs(_INFO, 'LUA: FS_LONG_TIMEOUT timeout: ' .. FS_LONG_TIMEOUT:get() .. 's')

  -- Test paramter
  if GLIDE_WIND_ENABL:get() == nil then
    send_to_gcs(_INFO, 'LUA: Something went wrong, GLIDE_WIND_WIND not created')
    return _init(), looptime
  else 
    gw_enable = GLIDE_WIND_ENABL:get()
    send_to_gcs(_INFO, 'LUA: GLIDE_WIND_ENABL: ' .. gw_enable)
  end

  -- Get the rc channel to override 
  RC_ROLL = rc:get_channel(RCMAP_ROLL:get())
  
  -- init last seen
  last_seen = gcs:last_seen()

  -- If GLIDE_WIND_ENABL, but other required setting missing, warning
  local fs_long_actn = FS_LONG_ACTN:get()
  
  -- Check if glide_wind is not enabled or if fs_long_actn is not glide
  if gw_enable == 1 and fs_long_actn ~= 2 then
    send_to_gcs(_WARNING, 'GLIDE_WIND_ENABL is set, but FS_LONG_ACTN is not 2.')
  end

  -- All set, go to update
  return update(), long_looptime
end

------------
-- Main loop
------------
function update()
  -- Check if state of GLIDE_WIND_WIND parameter changed, print every change
  if gw_enable ~= GLIDE_WIND_ENABL:get() then
    gw_enable = GLIDE_WIND_ENABL:get()
    send_to_gcs(_INFO, 'LUA: GLIDE_WIND_ENABL: ' .. gw_enable)
    -- If GLIDE_WIND_ENABL was enabled, warn if not FS_LONG_ACTN is set accordingly
    if gw_enable == 1 then
      local fs_long_actn = FS_LONG_ACTN:get()
      if fs_long_actn ~=2 then
        send_to_gcs(_WARNING, 'GLIDE_WIND_ENABL is set, but FS_LONG_ACTN is not 2.')
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

  -- If link has been lost for more than FS_LONG_TIMEOUT and we are in FBWA, turn into wind
  if link_lost_for > FS_LONG_TIMEOUT:get() * 1000 then
    if FS_LONG_ACTN:get() == 2 then
      if parse_flight_mode(vehicle:get_mode()) == "FBWA" then
        -- Get the heading angle
        hdg = math.floor(math.deg(ahrs:get_yaw()))
        -- Get wind direction. Function wind_estimate returns x and y for direction wind blows in, add pi to get true wind dir
        wind = ahrs:wind_estimate()
        wind_dir_rad = math.atan(wind:y(), wind:x())+math.pi
        wind_dir_180 = math.floor(wrap_180(math.deg(wind_dir_rad)))

        -- P-regulator, calc error choose closes way - right or left.
        error = wrap_180(wind_dir_180 - hdg)
        
        -- Multiply with kP and cast to int
        roll = math.floor(error*GLIDE_WIND_RKP:get())
        
        -- Limit output
        if roll > rlim then
          roll = rlim
        elseif roll < -rlim then
          roll = -rlim
        end

        -- Check if we are close to target heading
        if math.abs(error) < hdg_ok_lim then
          -- If we have been close to target heading for hdg_ok_t_lim, stop overriding
          if hdg_ok_t > hdg_ok_t_lim then
            -- Reset roll input, send text to gcs
            if override_enable then
              RC_ROLL:set_override(1500)
              send_to_gcs(_INFO, 'LUA: Gliding into wind, no more steering')
            end
            -- Do not override again until link has been recovered, set flag to false
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

        if override_enable then
          RC_ROLL:set_override(1500+roll)  -- Is active for RC_OVERRIDE_TIME (default 3s)
        end
      end
    end
  -- Link is not lost, wait for loosing link
  else
    roll = 0
    hdg_ok_t = 0
    override_enable = true
  end
  return update, looptime
end

-- Fail safe functions:
-- https://ardupilot.org/plane/docs/apms-failsafe-function.html

-------------------
-- Helper functions
-------------------

-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end

-- add a parameter and bind it to a variable
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

-- Parse flight mode
function parse_flight_mode(flight_mode_num)
  if flight_mode_num == mode_MANUAL then
    return "MANUAL"
  elseif flight_mode_num == mode_CIRCLE then
    return "CIRLCE"
  elseif flight_mode_num == mode_STABILIZE then
    return "STABILIZED"
  elseif flight_mode_num == mode_FBWA then
    return "FBWA"
  elseif flight_mode_num == mode_FBWB then
    return "FBWB"
  elseif flight_mode_num == mode_AUTO then
    return "AUTO"
  elseif flight_mode_num == mode_RTL then
    return "RTL"
  elseif flight_mode_num == mode_LOITER then
    return "LOITER"
  elseif flight_mode_num == mode_TAKEOFF then
    return "TAKEOFF"
  elseif flight_mode_num == mode_GUIDED then
    return "GUIDED"
  else
    return string.format("%d",flight_mode_num)
  end
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
