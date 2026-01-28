-- This script allows for best effort return for Copters on loss of Engine

-- When an engine failure is detected (RPM and Vibration values drop below a certain threshold), the aircraft will:
-- Send a warning ground station
-- Fly towards the home location or rally point, monitoring the Altitude.
-- If the aircraft drops below a predetermined minimum altitude, LAND mode is engaged and the aircraft lands at its current position.

-- setup param block for RPM failsafe params

gcs:send_text(0, "RPM FS Script Started")

local PARAM_TABLE_KEY = 140
local PARAM_TABLE_PREFIX = "RFS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- consider engine stopped when vibe is low and RPM low for more than 4s
ENGINE_STOPPED_MS = bind_add_param('ENG_STOP_MS', 1, 4000)

-- RPM threshold below which engine may be stopped
RPM_LOW_THRESH = bind_add_param('RPM_THRESH',     2, 500)

-- vibration threshold below which engine may be stopped
VIBE_LOW_THRESH = bind_add_param('VIBE_LOW',      3,  7)

-- altitude threshold for LAND. Set 0 to disable LAND, will always RTL
LOW_ALT_THRESH = bind_add_param('LOW_ALT',        4,  0)

-- Just test the script, don't switch modes. Set 0 to enable failsafe
TEST = bind_add_param('TEST',        5,  1)

-- time when engine stopped
local engine_stop_ms = -1
local engine_stopped = false

-- have we triggered failsafe? Only trigger once
local triggered_failsafe = false

-- flight mode numbers for Copter
local MODE_LAND = 9
local MODE_RTL = 6

-- update engine running status
function check_engine()
  if not arming:is_armed() then
     engine_stopped = false
     engine_stop_ms = -1
     return true
  end

  local rpm = RPM:get_rpm(0)
  local vibe = ahrs:get_vibration():length()

  -- if either RPM is high or vibe is high then assume engine is running
  if (rpm and (rpm > RPM_LOW_THRESH:get())) or (vibe > VIBE_LOW_THRESH:get()) then
     -- engine is definately running
     engine_stop_ms = -1
     if engine_stopped then
        -- notify user engine has started
        gcs:send_text(0, "Engine: STARTED")
        engine_stopped = false
     end
     return true
  end
  local now = millis()

  if engine_stop_ms == -1 then
     -- start timeout period
     engine_stop_ms = now
     return true
  end
  if now - engine_stop_ms < ENGINE_STOPPED_MS:get() then
     return false
  end
  -- engine has been stopped for ENGINE_STOPPED_MS milliseconds, notify user
  if not engine_stopped then
     engine_stopped = true
     gcs:send_text(0, "Engine: STOPPED")
  end
  return engine_stopped
end

-- trigger failsafe function
function trigger_failsafe()
  -- trigger an RTL to start bringing the vehicle home
  -- it will automatically go to the nearest rally point if set or go to home
  -- if no rally point available within the RALLY_LIMIT_KM
  vehicle:set_mode(MODE_RTL)
end

-- check if we should switch to LAND
function check_land()
  if LOW_ALT_THRESH:get() == 0 then
    -- we don't want to land
    return
  end

  local pos = ahrs:get_location()
  if not pos then
    -- we can't estimate distance
    return
  end
  local terrain_height = terrain:height_above_terrain(true)
  if terrain_height and terrain_height < LOW_ALT_THRESH:get() then
    gcs:send_text(0, "Failsafe: LANDING LAND ".. terrain_height)
    if TEST:get() == 0 then
      vehicle:set_mode(MODE_LAND)
    end
  end
end

function update()
  -- check engine status
  check_engine()

  -- if armed then consider triggering failsafe
  if engine_stopped and arming:is_armed() and not triggered_failsafe then
    triggered_failsafe = true
    if TEST:get() == 0 then
      trigger_failsafe()
    end
    gcs:send_text(0, "Failsafe: TRIGGERED")
  end

  if not engine_stopped and triggered_failsafe then
    triggered_failsafe = false
    gcs:send_text(0, "Failsafe: RECOVERED")
  end

  if triggered_failsafe and vehicle:get_mode() == MODE_RTL then
    check_land()
  end

  -- run at 5Hz
  return update, 200
end

-- start running update loop
return update()
