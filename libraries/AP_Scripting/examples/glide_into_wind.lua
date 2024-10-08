-- Glide into wind, LUA script for glide into wind functionality

-- Background
-- When flying a fixed-wing drone on ad-hoc BVLOS missions, it might not be
-- suitable for the drone to return to home if the C2 link is lost, since that
-- might mean flying without control for an extended time and distance. One
-- option in ArduPlane is to set FS_Long to Glide, which makes the drone glide
-- and land in the direction it happened to have when the command was invoked,
-- without regard to the wind. This script offers a way to decrease the kinetic
-- energy in this blind landing by means of steering the drone towards the wind
-- as GLIDE is initiated, hence lowering the ground speed. The intentition is to
-- minimize impact energy at landing - foremost for any third party, but also to
-- minimize damages on the drone.

-- Functionality and setup
-- 1. Set SCR_ENABLE = 1
-- 2. Put script in scripts folder, boot twice
-- 3. Two new parameters have appeared:
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
--     until state machine is reset and failsafe is triggered again. The script
--     'stops steerig' mainly to not override input from RC-roll for longer than
--     needed since it can confuse the pilot. In addition, steering in low
--     airspeeds (thr=0) increases risks of stall and it is prefereable touch
--     ground in level roll attitude. If the script parameter hdg_ok_lim is set
--     to 0, the 'stop steering' is effectively disabled and the drone will try
--     to steer into the wind during the whole failsafe procedure (not
--     recommended).
-- 14. Script will stop interfering when mode is changed from FBWA to another
--     mode.

-- During the fail safe manouverouver a warning tune is played.

-- State machine
-- CAN_TRIGGER 
--   - Do: Nothing
--   - Change state: If the failsafe glide is triggered:
--     if FS_GCS_ENABL is set, chage to TRIGGERED else change to CANCELED
--
-- TRIGGERED
--   - Do: Steer into wind and play warning tune
--   - Change state: If flight mode is changed from FBWA, change state to CANCELED
--
-- CANCELED 
--   - Do: Nothing
--   - Change state: When new heart beat arrive, change state to CAN_TRIGGER

-- Credits
-- This script is developed by agising at UASolutions, commissioned by, and in
-- cooperation with Remote.aero, with funding from Swedish AeroEDIH, in response
-- to a need from the Swedish Sea Rescue Society (Sjöräddningssällskapet, SSRS). 

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

-- Plane flight modes mapping
local mode_FBWA = 5


-- Tunes
local _tune_glide_warn = "MFT240 L16 cdefgfgfgfg"   --  The warning tone played during GLIDE_WIND

--State variable
local fs_state = nil

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
local link_lost_for = nil         -- link loss time counter
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
  RCMAP_ROLL = bind_param('RCMAP_ROLL')                   -- Shows the channel used for Roll input
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

  -- Get the rc channel to override 
  RC_ROLL = rc:get_channel(RCMAP_ROLL:get())
  
  -- Init last_seen.
  last_seen = gcs:last_seen()

  -- Init link_lost_for [ms] to FS_LONG_TIMEOUT [s] to prevent link to recover without
  -- new heartbeat. This is to properly init the state machine.
  link_lost_for = FS_LONG_TIMEOUT:get() * 1000

  -- If GLIDE_WIND_ENABL, but other required setting missing, warning
  local fs_long_actn = FS_LONG_ACTN:get()
  
  -- Check if glide_wind is not enabled or if fs_long_actn is not glide
  if gw_enable == 1 and fs_long_actn ~= 2 then
    send_to_gcs(_WARNING, 'GLIDE_WIND_ENABL is set, but FS_LONG_ACTN is not 2.')
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

  -- Run the state machine
  -- State CAN_TRIGGER
  if fs_state == 'CAN_TRIGGER' then
    if link_lost_for > FS_LONG_TIMEOUT:get() * 1000 then
      -- Double check that FS_GCS_ENABL is set
      if FS_GCS_ENABL:get() == 1 then
        fs_state = "TRIGGERED"
        -- Reset some variables
        roll = 0
        hdg_ok_t = 0
        override_enable = true
        send_to_gcs(_INFO, 'LUA: Glide into wind TRIGGERED')
      else
        -- Do not trigger glide into wind, require new heart beats to get here again
        fs_state = "CANCELED"
      end
    end
  -- State TRIGGERED
  elseif fs_state == "TRIGGERED" then
    if vehicle:get_mode() ~= mode_FBWA then
      fs_state = "CANCELED"
      send_to_gcs(_INFO, 'LUA: Glide into wind CANCELED')
    end
  -- State CANCELED
  elseif fs_state == "CANCELED" then
    if link_lost_for < FS_LONG_TIMEOUT:get() * 1000 then
      fs_state = "CAN_TRIGGER"
      send_to_gcs(_INFO, 'LUA: Glide into wind CAN_TRIGGER')
    end
  end

  -- State TRIGGERED actions
  if fs_state == "TRIGGERED" then
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
          send_to_gcs(_INFO, 'LUA: Glide into wind steering complete, just GLIDE')
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

    if override_enable then
      RC_ROLL:set_override(1500+roll)  -- Is active for RC_OVERRIDE_TIME (default 3s)
    end
  end

  return update, looptime
end


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
