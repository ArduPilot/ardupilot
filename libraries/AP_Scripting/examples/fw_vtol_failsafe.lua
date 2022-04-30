-- This script allows for best effort return for quadplanes
-- on loss of forward motor

-- When an engine failure is detected (RPM and Vibration values drop below a certain threshold), the aircraft will:
-- Send a warning ground station
-- Start prioritizing airspeed completely
-- Glide back towards the home location or rally point, monitoring the distance away from the point as well as Altitude.
-- If the aircraft drops below a predetermined minimum altitude, QLAND mode is engaged and the aircraft lands at its current position.
-- If the aircraft arrives within Q_FW_LND_APR_RAD of the return point before dropping below the minimum altitude, it should loiter down to the minimum altitude before switching to QRTL and landing.

-- setup param block for VTOL failsafe params
local PARAM_TABLE_KEY = 77
local PARAM_TABLE_PREFIX = "VTFS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

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

-- consider engine stopped when vibe is low and RPM low for more than 4s
ENGINE_STOPPED_MS = bind_add_param('ENG_STOP_MS', 1, 4000)

-- RPM threshold below which engine may be stopped
RPM_LOW_THRESH = bind_add_param('RPM_THRESH',     2, 500)

-- vibration threshold below which engine may be stopped
VIBE_LOW_THRESH = bind_add_param('VIBE_LOW',      3,  2)

-- altitude threshold for QLAND
LOW_ALT_THRESH = bind_add_param('LOW_ALT',        4,  70)

-- time when engine stopped
local engine_stop_ms = -1
local engine_stopped = false

-- have we triggered failsafe? Only trigger once
local triggered_failsafe = false

-- flight mode numbers for plane
local MODE_AUTO = 10
local MODE_RTL = 11
local MODE_QLAND = 20
local MODE_QRTL = 21

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
  if param:get('RTL_AUTOLAND') == 2 then
     -- we don't want to do the mission based autoland, so disable
     -- RTL_AUTOLAND
     param:set('RTL_AUTOLAND', 0)
  end
  if param:get('TECS_SPDWEIGHT') < 2 then
     -- force airspeed priority
     param:set('TECS_SPDWEIGHT', 2)
  end
  -- trigger an RTL to start bringing the vehicle home
  -- it will automatically go to the nearest rally point if set or go to home
  -- if no rally point available within the RALLY_LIMIT_KM
  vehicle:set_mode(MODE_RTL)
end

-- check if we should switch to QLAND
function check_qland()
  local target = vehicle:get_target_location()
  local pos = ahrs:get_location()
  if not target or not pos then
    -- we can't estimate distance
    return
  end
  local dist = target:get_distance(pos)
  local terrain_height = terrain:height_above_terrain(true)
  local threshold = param:get('Q_FW_LND_APR_RAD')
  if dist < threshold and (terrain_height and (terrain_height < LOW_ALT_THRESH:get())) then
    gcs:send_text(0, "Failsafe: LANDING QRTL")
    vehicle:set_mode(MODE_QRTL)
  elseif terrain_height and terrain_height < LOW_ALT_THRESH:get() then
    gcs:send_text(0, "Failsafe: LANDING QLAND")
    vehicle:set_mode(MODE_QLAND)
  end
end

function update()
  -- check engine status
  check_engine()

  -- if armed and in AUTO mode then consider triggering failsafe
  if engine_stopped and vehicle:get_mode() == MODE_AUTO and arming:is_armed() and not triggered_failsafe then
    triggered_failsafe = true
    trigger_failsafe()
    gcs:send_text(0, "Failsafe: TRIGGERED")
  end

  if not engine_stopped and triggered_failsafe then
    triggered_failsafe = false
    gcs:send_text(0, "Failsafe: RECOVERED")
  end

  if triggered_failsafe and vehicle:get_mode() == MODE_RTL then
    check_qland()
  end

  -- run at 5Hz
  return update, 200
end

-- start running update loop
return update()
