-- mount-driver.lua: Example scripting gimbal driver
--
-- Template for writing a Lua gimbal driver using the scripting mount backend.
-- Populate send_target_angles and send_target_rates with your gimbal's
-- protocol (serial, CAN, etc). This example simulates a gimbal by tracking
-- targets internally and reporting them back as attitude.
--
-- Setup:
--   Set MNT1_TYPE = 9 (Scripting) and reboot
--   Copy this script to the APM/scripts directory and reboot
--
-- Advanced usage:
--   The gimbal can be used as the Nth mount by setting MNTn_TYPE = 9 and
--   modifying the MOUNT_INSTANCE below.

-- user definitions
local MOUNT_INSTANCE = 0                -- default to MNT1

-- global definitions
local INIT_INTERVAL_MS = 3000           -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 100          -- update at 10hz
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- local variables
local sim_state = {
  roll_ef_deg=0,  -- roll/pitch earth frame, yaw body frame
  pitch_ef_deg=0, -- (common for pwm-controlled brushless gimbals)
  yaw_bf_deg=0
}
local initialised = false
local last_update_ms = 0

-- wrap yaw angle in degrees to value between 0 and 360
local function wrap_360(angle)
  local res = math.fmod(angle, 360.0)
  if res < 0 then
    res = res + 360.0
  end
  return res
end

-- wrap yaw angle in degrees to value between -180 and +180
local function wrap_180(angle_deg)
  local res = wrap_360(angle_deg)
  if res > 180 then
    res = res - 360
  end
  return res
end

-- bind mount type parameter
local MNT_TYPE = Parameter("MNT" .. (MOUNT_INSTANCE + 1) .. "_TYPE")

-- perform any required initialisation
local function init()
  if MNT_TYPE:get() ~= 9 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "MountDriver: set MNT" .. (MOUNT_INSTANCE + 1) .. "_TYPE=9")
    return
  end

  initialised = true
  last_update_ms = millis():tofloat()
  gcs:send_text(MAV_SEVERITY.INFO, "MountDriver: started")
end

-- send target angles (in degrees) to gimbal
local function send_target_angles(roll_ef_deg, pitch_ef_deg, yaw_deg, yaw_is_ef)
  -- default argument values
  roll_ef_deg = roll_ef_deg or 0
  pitch_ef_deg = pitch_ef_deg or 0
  yaw_deg = yaw_deg or 0
  yaw_is_ef = yaw_is_ef or false

  if yaw_is_ef then
    -- convert to body-frame
    yaw_deg = wrap_180(yaw_deg - math.deg(ahrs:get_yaw_rad()))
  end

  sim_state.roll_ef_deg = roll_ef_deg
  sim_state.pitch_ef_deg = pitch_ef_deg
  sim_state.yaw_bf_deg = yaw_deg
end

-- send target rates (in deg/sec) to gimbal
local function send_target_rates(roll_degs, pitch_degs, yaw_degs, yaw_is_ef, dt_s)
  -- default argument values
  roll_degs = roll_degs or 0
  pitch_degs = pitch_degs or 0
  yaw_degs = yaw_degs or 0
  yaw_is_ef = yaw_is_ef or false

  if yaw_is_ef then
    yaw_degs = yaw_degs - math.deg(ahrs:get_gyro():z())
  end

  send_target_angles(
    sim_state.roll_ef_deg + roll_degs * dt_s,
    sim_state.pitch_ef_deg + pitch_degs * dt_s,
    sim_state.yaw_bf_deg + yaw_degs * dt_s,
    false
  )
end

-- the main update function
local function update()

  -- initialise connection to gimbal
  if not initialised then
    init()
    return
  end

  -- calculate dt
  local now_ms = millis():tofloat()
  local dt_s = (now_ms - last_update_ms) / 1000.0
  last_update_ms = now_ms

  -- report gimbal attitude. Must be called periodically or the backend reports
  -- unhealthy. Ideally, populate this from a gimbal attitude message. If your
  -- gimbal doesn't report attitude but you can detect it is alive, stop calling
  -- this when it stops responding so ArduPilot gets real health feedback. Here
  -- we just report our sim state directly since there is no real gimbal.
  mount:set_attitude_euler(MOUNT_INSTANCE, sim_state.roll_ef_deg, sim_state.pitch_ef_deg, sim_state.yaw_bf_deg)

  -- send angle target
  local roll_deg, pitch_deg, yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
  if roll_deg and pitch_deg and yaw_deg then
    send_target_angles(roll_deg, pitch_deg, yaw_deg, yaw_is_ef)
    return
  end

  -- send rate target
  local roll_degs, pitch_degs, yaw_degs
  roll_degs, pitch_degs, yaw_degs, yaw_is_ef = mount:get_rate_target(MOUNT_INSTANCE)
  if roll_degs and pitch_degs and yaw_degs then
    send_target_rates(roll_degs, pitch_degs, yaw_degs, yaw_is_ef, dt_s)
    return
  end
end

local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
    gcs:send_text(MAV_SEVERITY.ERROR, "MountDriver: " .. err)
    return protected_wrapper, 1000
  end
  if not initialised then
    return protected_wrapper, INIT_INTERVAL_MS
  end
  return protected_wrapper, UPDATE_INTERVAL_MS
end

return protected_wrapper()
