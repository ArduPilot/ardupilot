-- This is a script overrides a forward flight mode at low altitude and within radius of home
-- configured with Q_LOW_ALT_* params

-- maker sure on a quadplane
assert(quadplane, "Quadplane not setup")

-- add new params
local PARAM_TABLE_KEY = 76
assert(param:add_table(PARAM_TABLE_KEY, "Q_LOW_ALT_", 3), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 0), "could not add param") -- enable low alt mode change, 1: Switch to QLAND, 2: Switch back to previous mode
assert(param:add_param(PARAM_TABLE_KEY, 2, "ALT", 15), "could not add param") -- threshold altitude
assert(param:add_param(PARAM_TABLE_KEY, 3, "RADIUS", 50), "could not add param") -- threshold radius

local enabled = Parameter()
local alt = Parameter()
local radius = Parameter()

assert(enabled:init("Q_LOW_ALT_ENABLE"), "could not find param")
assert(alt:init("Q_LOW_ALT_ALT"), "could not find param")
assert(radius:init("Q_LOW_ALT_RADIUS"), "could not find param")

-- all plane mode numbers
local MODE_MANUAL        = 0
local MODE_CIRCLE        = 1
local MODE_STABILIZE     = 2
local MODE_TRAINING      = 3
local MODE_ACRO          = 4
local MODE_FLY_BY_WIRE_A = 5
local MODE_FLY_BY_WIRE_B = 6
local MODE_CRUISE        = 7
local MODE_AUTOTUNE      = 8
local MODE_AUTO          = 10
local MODE_RTL           = 11
local MODE_LOITER        = 12
local MODE_TAKEOFF       = 13
local MODE_AVOID_ADSB    = 14
local MODE_GUIDED        = 15
local MODE_INITIALISING  = 16
local MODE_QSTABILIZE    = 17
local MODE_QHOVER        = 18
local MODE_QLOITER       = 19
local MODE_QLAND         = 20
local MODE_QRTL          = 21
local MODE_QAUTOTUNE     = 22
local MODE_QACRO         = 23
local MODE_THERMAL       = 24
local MODE_LOITER_ALT_QLAND = 25

local FS_enabled_in_mode = {}

-- enable in all FW modes except auto and guided
FS_enabled_in_mode[MODE_MANUAL] = true
FS_enabled_in_mode[MODE_CIRCLE] = true
FS_enabled_in_mode[MODE_STABILIZE] = true
FS_enabled_in_mode[MODE_TRAINING] = true
FS_enabled_in_mode[MODE_ACRO] = true
FS_enabled_in_mode[MODE_FLY_BY_WIRE_A] = true
FS_enabled_in_mode[MODE_FLY_BY_WIRE_B] = true
FS_enabled_in_mode[MODE_CRUISE] = true
FS_enabled_in_mode[MODE_AUTOTUNE] = true
FS_enabled_in_mode[MODE_AUTO] = false
FS_enabled_in_mode[MODE_RTL] = true
FS_enabled_in_mode[MODE_LOITER] = true
FS_enabled_in_mode[MODE_TAKEOFF] = true
FS_enabled_in_mode[MODE_AVOID_ADSB] = true
FS_enabled_in_mode[MODE_GUIDED] = false
FS_enabled_in_mode[MODE_INITIALISING] = true
FS_enabled_in_mode[MODE_QSTABILIZE] = false
FS_enabled_in_mode[MODE_QHOVER] = false
FS_enabled_in_mode[MODE_QLOITER] = false
FS_enabled_in_mode[MODE_QLAND] = false
FS_enabled_in_mode[MODE_QRTL] = false
FS_enabled_in_mode[MODE_QAUTOTUNE] = false
FS_enabled_in_mode[MODE_QACRO] = false
FS_enabled_in_mode[MODE_THERMAL] = true
FS_enabled_in_mode[MODE_LOITER_ALT_QLAND] = true

local last_mode = vehicle:get_mode()
function update()

  local mode = vehicle:get_mode()
  local previous_mode = last_mode
  if mode == last_mode then
    -- no mode change
    return update, 100
  end
  last_mode = mode

  if enabled:get() <= 0 or not arming:is_armed() then
    -- not enabled or not armed
    return update, 100
  end

  if not FS_enabled_in_mode[mode] then
    -- not enabled in this mode
    return update, 100
  end

  if FS_enabled_in_mode[previous_mode] then
    -- fail safe is enabled in this mode, but it was also enabled in the last mode
    -- Don't stop switch from one FW flight mode to another
    return update, 100
  end

  local pos = ahrs:get_relative_position_NED_home()
  if not pos then
    -- no position
    return update, 100
  end

  if pos:xy():length() > radius:get() or -pos:z() > alt:get() then
    -- outside threshold radius or above altitude
    return update, 100
  end

  -- Enabled, armed and in enabled mode, switching out of disabled mode
  -- Within radius and below altitude threshold
  -- switch to Qland or back to previous mode

  local new_mode = MODE_QLAND
  if enabled:get() == 2 then
    new_mode = previous_mode
    gcs:send_text(5, 'Too low for fixedwing flight')
  else
    gcs:send_text(5, 'Too low for fixedwing flight, mode set to QLand')
  end

  vehicle:set_mode(new_mode)

  -- update last mode to prevent trigger on this mode change
  last_mode = new_mode

  return update, 100
end

return update()
