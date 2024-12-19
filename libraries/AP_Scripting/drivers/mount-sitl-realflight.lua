-- mount-sitl-realflight.lua: RealFlight simulated gimbal driver

--[[
  RealFlight simulated gimbal driver

  This script provides a driver for a simulated gimbal in RealFlight. This
  script sends three servo outputs to control a camera: pitch, yaw, and zoom.

  An invisible camera gimbal can be added to any model in RealFlight, even if the
  3D assets for that camera gimbal are not there. Check the Titan Cobra model for
  an example of how to add and configure a camera for use with this script.

  Note that there is a 12 channel limit for servo outputs to RealFlight, so this
  can only be done with a vehicle that uses 9 or fewer channels (or 10 if you
  don't need the zoom control).
--]]

-- parameters
local PARAM_TABLE_KEY = 24
assert(param:add_table(PARAM_TABLE_KEY, "RFG_", 6), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add RFG_DEBUG param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "FOV_MIN", 10), "could not add RFG_FOV_MIN param")
assert(param:add_param(PARAM_TABLE_KEY, 3, "FOV_MAX", 60), "could not add RFG_FOV_MAX param")
assert(param:add_param(PARAM_TABLE_KEY, 4, "ASPECT", 1.777778), "could not add RFG_ASPECT param")
assert(param:add_param(PARAM_TABLE_KEY, 5, "ZOOM_SPEED", 10), "could not add RFG_ZOOM_SPEED param")
assert(param:add_param(PARAM_TABLE_KEY, 6, "ZOOM_SERVO", 94), "could not add RFG_ZOOM_SERVO param")

-- Servo outputs
local PITCH_SERVO = nil
local YAW_SERVO = nil
local ZOOM_SERVO = nil

--[[
  // @Param: RFG_DEBUG
  // @DisplayName: RealFlight Gimbal debug
  // @Description: RealFlight Gimbal debug
  // @Values: 0:Disabled, 1:Enabled
  // @User: Advanced
--]]
local RFG_DEBUG = Parameter("RFG_DEBUG") -- debug level. 0:disabled 1:enabled

--[[
  // @Param: RFG_FOV_MIN
  // @DisplayName: RealFlight Gimbal FOV Min
  // @Description: RealFlight Gimbal FOV Min
  // @Units: deg
  // @Range: 1 179
  // @User: Standard
--]]
local RFG_FOV_MIN = Parameter("RFG_FOV_MIN") -- minimum field of view in degrees

--[[
  // @Param: RFG_FOV_MAX
  // @DisplayName: RealFlight Gimbal FOV Max
  // @Description: RealFlight Gimbal FOV Max
  // @Units: deg
  // @Range: 1 179
  // @User: Standard
--]]
local RFG_FOV_MAX = Parameter("RFG_FOV_MAX") -- maximum field of view in degrees

--[[
  // @Param: RFG_ASPECT
  // @DisplayName: RealFlight Gimbal Aspect Ratio
  // @Description: RealFlight Gimbal Aspect Ratio. HFOV/VFOV for the camera image. Used for relaying horizontal and vertical FOV of the camera
  // @Range: 0.1 10
  // @User: Standard
--]]
local RFG_ASPECT = Parameter("RFG_ASPECT") -- aspect ratio of the camera

--[[
  // @Param: RFG_ZOOM_SPEED
  // @DisplayName: RealFlight Gimbal Zoom Speed
  // @Description: RealFlight Gimbal Zoom Speed.  Higher numbers result in faster zooming
  // @Units: %/s
  // @Range: 1 100
  // @User: Standard
--]]
local RFG_ZOOM_SPEED = Parameter("RFG_ZOOM_SPEED") -- zoom speed in percent per second

--[[
  // @Param: RFG_ZOOM_SERVO
  // @DisplayName: RealFlight Gimbal Zoom Servo Function
  // @Description: RealFlight Gimbal Zoom Servo Function. The servo function number for the zoom control: Script1 (94) to Script16 (109)
  // @Range: 94 109
  // @User: Standard
--]]
local RFG_ZOOM_SERVO = Parameter("RFG_ZOOM_SERVO") -- servo function number for the zoom control

-- global definitions
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local INIT_INTERVAL_MS = 3000  -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 100 -- update at 10hz
local MOUNT_INSTANCE = 0       -- always control the first mount/gimbal
local CAMERA_INSTANCE = 0      -- always use the first camera
local PITCH_MIN_DEG = -89.9    -- minimum pitch angle in degrees, as set up in the RealFlight model
local PITCH_MAX_DEG = 89.9     -- maximum pitch angle in degrees, as set up in the RealFlight model
local YAW_MIN_DEG = -180       -- minimum yaw angle in degrees, as set up in the RealFlight model
local YAW_MAX_DEG = 180        -- maximum yaw angle in degrees, as set up in the RealFlight model

-- local variables and definitions
local initialised = false   -- true once connection to gimbal has been initialised
local pitch_deg = 0         -- current pitch angle in degrees (always earth frame)
local yaw_deg = 0           -- current yaw angle in degrees (always body frame)
local zoom_absolute = 0     -- current zoom level from 0 to 100
local cam_pic_count = 0     -- last picture count.  used to detect trigger pic
local cam_rec_video = false -- last record video state.  used to detect record video
local cam_zoom_type = 0     -- last zoom type 1:Rate 2:Pct
local cam_zoom_value = 0    -- last zoom value.  If rate, zoom out = -1, hold = 0, zoom in = 1.  If Pct then value from 0 to 100

-- bind parameters to variables
local MNT_TYPE = Parameter("MNT" .. (MOUNT_INSTANCE + 1) .. "_TYPE")  -- should be 9:Scripting
local CAM_TYPE = Parameter("CAM" .. (CAMERA_INSTANCE + 1) .. "_TYPE") -- should be 7:Scripting
local CAM_HFOV = Parameter("CAM" .. (CAMERA_INSTANCE + 1) .. "_HFOV") -- horizontal field of view in degrees
local CAM_VFOV = Parameter("CAM" .. (CAMERA_INSTANCE + 1) .. "_VFOV") -- vertical field of view in degrees

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

-- find and initialise serial port connected to gimbal
local function init()
  -- check mount parameter
  if MNT_TYPE:get() ~= 9 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "RealFlight Gimbal: set MNT" .. (MOUNT_INSTANCE + 1) .. "_TYPE=9")
    do return end
  end

  -- check cam type parameter
  if CAM_TYPE:get() ~= 7 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "RealFlight Gimbal: set CAM" .. (CAMERA_INSTANCE + 1) .. "_TYPE=7")
    do return end
  end

  if not CAM_HFOV or not CAM_VFOV then
    gcs:send_text(MAV_SEVERITY.CRITICAL,
      "RealFlight Gimbal: could not find camera HFOV and VFOV for camera instance " .. CAMERA_INSTANCE)
    do return end
  end

  -- find pitch and yaw servos
  PITCH_SERVO = SRV_Channels:find_channel(7) -- Mount1Pitch
  YAW_SERVO = SRV_Channels:find_channel(6)   -- Mount1Yaw
  if not PITCH_SERVO or not YAW_SERVO then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "RealFlight Gimbal: could not find pitch and yaw servos")
    do return end
  end

  -- find zoom servo
  ZOOM_SERVO = SRV_Channels:find_channel(RFG_ZOOM_SERVO:get())
  if not ZOOM_SERVO then
    gcs:send_text(MAV_SEVERITY.INFO, "RealFlight Gimbal: WARNING: could not find zoom servo")
  end

  initialised = true
  gcs:send_text(MAV_SEVERITY.INFO, "RealFlight Gimbal: started")
end

-- send target angles to the servos
local function send_target_angles()
  if not PITCH_SERVO or not YAW_SERVO then
    return
  end
  -- convert to int
  SRV_Channels:set_output_pwm_chan_timeout(
    PITCH_SERVO,
    math.floor(1000 + 1000 * (pitch_deg - PITCH_MIN_DEG) / (PITCH_MAX_DEG - PITCH_MIN_DEG)),
    UPDATE_INTERVAL_MS * 10
  )
  SRV_Channels:set_output_pwm_chan_timeout(
    YAW_SERVO,
    math.floor(1000 + 1000 * (yaw_deg - YAW_MIN_DEG) / (YAW_MAX_DEG - YAW_MIN_DEG)),
    UPDATE_INTERVAL_MS * 10
  )
end

-- set zoom servo
local function set_camera_zoom(zoom_pct)
  local hfov = RFG_FOV_MIN:get() + (RFG_FOV_MAX:get() - RFG_FOV_MIN:get()) * (100-zoom_pct) / 100
  local vfov = hfov / RFG_ASPECT:get()

  CAM_HFOV:set(hfov)
  CAM_VFOV:set(vfov)

  if not ZOOM_SERVO then
    return
  end
  SRV_Channels:set_output_pwm_chan_timeout(
    ZOOM_SERVO,
    1000 + 1000 * zoom_pct / 100,
    UPDATE_INTERVAL_MS * 10
  )
end

-- check for changes in camera state and update accordingly (primarily for zoom)
local function update_camera()
  -- get latest camera state from AP driver
  local cam_state = camera:get_state(CAMERA_INSTANCE)
  if not cam_state then
    return
  end

  -- check for take picture
  if cam_state:take_pic_incr() and cam_state:take_pic_incr() ~= cam_pic_count then
    cam_pic_count = cam_state:take_pic_incr()
    -- Do nothing (useful for testing ArduPilot, but does nothing in RealFlight)
    if RFG_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RealFlight Gimbal: took pic %u", cam_pic_count))
    end
  end

  -- check for start/stop recording video
  if cam_state:recording_video() ~= cam_rec_video then
    cam_rec_video = cam_state:recording_video()
    -- Do nothing (useful for testing ArduPilot, but does nothing in RealFlight)
    if cam_rec_video > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "RealFlight Gimbal: start recording")
    else
      -- Do nothing (useful for testing ArduPilot, but does nothing in RealFlight)
      gcs:send_text(MAV_SEVERITY.INFO, "RealFlight Gimbal: stop recording")
    end
    if RFG_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "RealFlight Gimbal: rec video:" .. tostring(cam_rec_video))
    end
  end

  -- check zoom
  -- zoom out = -1, hold = 0, zoom in = 1
  local zoom_type_changed = cam_state:zoom_type() and (cam_state:zoom_type() ~= cam_zoom_type)
  local zoom_value_changed = cam_state:zoom_value() and (cam_state:zoom_value() ~= cam_zoom_value)
  if (zoom_type_changed or zoom_value_changed) then
    cam_zoom_type = cam_state:zoom_type()
    cam_zoom_value = cam_state:zoom_value()

    -- zoom percent
    if cam_zoom_type == 2 then
      zoom_absolute = cam_zoom_value
    end
  end

  -- Slew zoom if in rate mode
  if cam_zoom_type == 1 then
    zoom_absolute = math.max(0, math.min(100, zoom_absolute + cam_zoom_value * RFG_ZOOM_SPEED:get() * UPDATE_INTERVAL_MS / 1000))
  end

  if RFG_DEBUG:get() > 0 and (zoom_type_changed or zoom_value_changed or (cam_zoom_type == 1 and cam_zoom_value ~= 0)) then
    gcs:send_text(MAV_SEVERITY.INFO,
      "RealFlight Gimbal: zoom type:" .. tostring(cam_zoom_type) .. " value:" .. tostring(cam_zoom_value))
    gcs:send_text(MAV_SEVERITY.INFO, "RealFlight Gimbal: zoom absolute:" .. tostring(zoom_absolute))
  end

  set_camera_zoom(zoom_absolute)
end

-- the main update function
local function update()
  -- initialise connection to gimbal
  if not initialised then
    init()
    return update, INIT_INTERVAL_MS
  end

  update_camera()

  -- update pitch and yaw targets if in angle-targeting mode
  local yaw_is_ef
  local next_pitch_deg, next_yaw_deg
  _, next_pitch_deg, next_yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
  if next_pitch_deg and next_yaw_deg then
    if yaw_is_ef then
      next_yaw_deg = wrap_180(next_yaw_deg - math.deg(ahrs:get_yaw()))
    end
    pitch_deg = next_pitch_deg
    yaw_deg = next_yaw_deg
  end

  -- update pitch and yaw targets if in rate-targeting mode
  local pitch_rate_dps, yaw_rate_dps
  _, pitch_rate_dps, yaw_rate_dps, yaw_is_ef = mount:get_rate_target(MOUNT_INSTANCE)
  if pitch_rate_dps and yaw_rate_dps then
    if yaw_is_ef then
      yaw_rate_dps = yaw_rate_dps - math.deg(ahrs:get_gyro():z())
    end
    pitch_deg = pitch_deg + pitch_rate_dps * UPDATE_INTERVAL_MS / 1000
    yaw_deg = yaw_deg + yaw_rate_dps * UPDATE_INTERVAL_MS / 1000
  end

  yaw_deg = wrap_180(yaw_deg)

  send_target_angles()

  -- update the backend so it's healthy
  mount:set_attitude_euler(MOUNT_INSTANCE, 0, pitch_deg, yaw_deg);

  return update, UPDATE_INTERVAL_MS
end

return update()
