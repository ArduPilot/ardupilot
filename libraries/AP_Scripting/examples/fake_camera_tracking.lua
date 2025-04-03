--[[
  Fake Camera Tracking

  This script simulates camera tracking functionality within the ArduPilot
  environment, useful for testing Ground Control Station (GCS) software. This
  script acts as a camera backend, so the user must set the CAM1_TYPE parameter
  to 7 for this to work correctly. Additionally, since ArduPilot does not yet
  support CAMERA_TRACKING_IMAGE_STATUS, this handles all the MAVLink
  communication for communicating the tracking data to the GCS.

  Key Features:
  - **Tracking Mode Support:** Includes support for both POINT and RECTANGLE
    tracking modes. While tracking is active, the script moves the tracked
    point/rectangle around randomly.
  - **MAVLink Communication:** The script sends CAMERA_TRACKING_IMAGE_STATUS,
    allowing the GCS to display the current tracking state.
  - **Stream Rate Control:** Handles requests to set the stream rate for
    CAMERA_TRACKING_IMAGE_STATUS messages, sending those messages at the
    requested rate.

  Required Parameters:
  - **CAM1_TYPE:** Set to 7 to enable the camera tracking functionality.
--]]

-- constant definitions
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local CAMERA_TRACKING_STATUS_FLAGS = { IDLE = 0, ACTIVE = 1, ERROR = 2 }
local CAMERA_TRACKING_MODE = { NONE = 0, POINT = 1, RECTANGLE = 2 }
local CAMERA_TRACKING_TARGET_DATA = { NONE = 0, EMBEDDED = 1, RENDERED = 2, IN_STATUS = 4 }
local UPDATE_INTERVAL_MS = 100 -- update at 10hz
local CAMERA_INSTANCE = 0      -- always use the first camera
local NAN = 0 / 0

-- Tracking state variables
local last_tracking_type = 0  -- last tracking type 0:off 1:point 2:rectangle
local last_tracking_p1x = 0   -- last tracking point 1 x (for detecting changes from GCS)
local last_tracking_p1y = 0   -- last tracking point 1 y (for detecting changes from GCS)
local last_tracking_p2x = 0   -- last tracking point 2 x (for detecting changes from GCS)
local last_tracking_p2y = 0   -- last tracking point 2 y (for detecting changes from GCS)
local tracking_p1x = 0        -- tracking point 1 x (for reporting to GCS)
local tracking_p1y = 0        -- tracking point 1 y (for reporting to GCS)
local tracking_active = false -- tracking is active

-- MAVLink handling
local mavlink_msgs = require("MAVLink/mavlink_msgs")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")
local CAMERA_TRACKING_IMAGE_STATUS_ID = mavlink_msgs.get_msgid("CAMERA_TRACKING_IMAGE_STATUS")
local msg_map = {}
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"
msg_map[CAMERA_TRACKING_IMAGE_STATUS_ID] = "CAMERA_TRACKING_IMAGE_STATUS"
mavlink:init(1, 10)
mavlink:register_rx_msgid(COMMAND_LONG_ID)
local camera_tracking_interval_ms  = 0 -- interval in milliseconds to send tracking status
local camera_tracking_last_ms      = 0 -- last time tracking status was sent
local camera_tracking_channel      = nil -- channel to send tracking status (we only support one channel at a time)
local MAV_CMD_SET_MESSAGE_INTERVAL = 511

local function update_camera()
  -- get latest camera state from AP driver
  local cam_state = camera:get_state(CAMERA_INSTANCE)
  if not cam_state then
    return
  end

  -- check for change in tracking state
  local tracking_type_changed = cam_state:tracking_type() ~= last_tracking_type
  local tracking_type_p1_changed = (cam_state:tracking_p1():x() ~= last_tracking_p1x) or (cam_state:tracking_p1():y() ~= last_tracking_p1y)
  local tracking_type_p2_changed = (cam_state:tracking_p2():x() ~= last_tracking_p2x) or (cam_state:tracking_p2():y() ~= last_tracking_p2y)
  last_tracking_type = cam_state:tracking_type()
  last_tracking_p1x = cam_state:tracking_p1():x()
  last_tracking_p1y = cam_state:tracking_p1():y()
  last_tracking_p2x = cam_state:tracking_p2():x()
  last_tracking_p2y = cam_state:tracking_p2():y()

  --  print("tracking_type: " .. cam_state:tracking_type())

  if tracking_type_p1_changed then
    tracking_p1x = last_tracking_p1x
    tracking_p1y = last_tracking_p1y
  end

  if (last_tracking_type == CAMERA_TRACKING_MODE.NONE) and tracking_type_changed then
    -- turn off tracking
    tracking_active = false
    gcs:send_text(MAV_SEVERITY.INFO, "Fake Tracking: OFF")
  end

  if (last_tracking_type == CAMERA_TRACKING_MODE.POINT) and (tracking_type_changed or tracking_type_p1_changed) then
    -- turn tracking point on
    tracking_active = true
    gcs:send_text(MAV_SEVERITY.INFO, "Fake Tracking: point ON")
  end

  if (last_tracking_type == CAMERA_TRACKING_MODE.RECTANGLE) and (tracking_type_changed or tracking_type_p1_changed or tracking_type_p2_changed) then
    -- turn tracking rectangle on
    tracking_active = true
    gcs:send_text(MAV_SEVERITY.INFO, "Fake Tracking: rectangle ON")
    print("tracking_p1x: " .. tracking_p1x)
    print("tracking_p1y: " .. tracking_p1y)
    print("tracking_p2x: " .. last_tracking_p2x)
    print("tracking_p2y: " .. last_tracking_p2y)
  end
end

-- Check a command long message for a relevant message interval command
local function handle_command_long(chan, msg)
  if msg.msgid ~= COMMAND_LONG_ID then
    return
  end
  if msg.command ~= MAV_CMD_SET_MESSAGE_INTERVAL then
    return
  end
  print("got message interval command " .. msg.param1 .. " " .. msg.param2 / 1000)
  if msg.param1 ~= CAMERA_TRACKING_IMAGE_STATUS_ID then
    return
  end
  camera_tracking_interval_ms = msg.param2 / 1000
  camera_tracking_channel = chan
end

-- Check for message interval commands
local function check_message_interval_commands()
  while true do
    local msg, chan = mavlink:receive_chan()
    if not msg then
      break
    end
    if not camera_tracking_channel or camera_tracking_channel == chan then
      local parsed_msg = mavlink_msgs.decode(msg, msg_map)
      if parsed_msg then
        handle_command_long(chan, parsed_msg)
      end
    end
  end
end

local function update_fake_tracking()
  -- "drunk walk" the tracking point around the screen, keeping it within the screen bounds
  if last_tracking_type ~= CAMERA_TRACKING_MODE.NONE then
    tracking_p1x = tracking_p1x + (math.random()-0.5)*0.01
    tracking_p1y = tracking_p1y + (math.random()-0.5)*0.01
    local width = 0
    local height = 0
    if last_tracking_type == CAMERA_TRACKING_MODE.RECTANGLE then
      width = math.abs(last_tracking_p2x - last_tracking_p1x)
      height = math.abs(last_tracking_p2y - last_tracking_p1y)
    end
    tracking_p1x = math.min(1 - width, math.max(0, tracking_p1x))
    tracking_p1y = math.min(1 - height, math.max(0, tracking_p1y))
  end
end

local function send_tracking_status()
  -- Check if we even have a channel to send messages on
  local now = millis():toint()
  if not camera_tracking_channel then
    return
  end
  -- Check if this message is disabled
  if camera_tracking_interval_ms <= 0 then
    return
  end
  -- Check if it is time to send the message
  if camera_tracking_last_ms + camera_tracking_interval_ms > now then
    return
  end
  camera_tracking_last_ms = now

  -- Construct tracking status message
  local tracking_image_status = {}
  if tracking_active then
    tracking_image_status.tracking_status = CAMERA_TRACKING_STATUS_FLAGS.ACTIVE
  else
    tracking_image_status.tracking_status = CAMERA_TRACKING_STATUS_FLAGS.IDLE
  end
  tracking_image_status.tracking_mode = last_tracking_type
  tracking_image_status.target_data = CAMERA_TRACKING_TARGET_DATA.IN_STATUS
  tracking_image_status.point_x = NAN
  tracking_image_status.point_y = NAN
  tracking_image_status.radius = NAN
  tracking_image_status.rec_top_x = NAN
  tracking_image_status.rec_top_y = NAN
  tracking_image_status.rec_bottom_x = NAN
  tracking_image_status.rec_bottom_y = NAN
  if last_tracking_type == CAMERA_TRACKING_MODE.POINT then
    tracking_image_status.point_x = tracking_p1x
    tracking_image_status.point_y = tracking_p1y
  end
  if last_tracking_type == CAMERA_TRACKING_MODE.RECTANGLE then
    tracking_image_status.rec_top_x = tracking_p1x
    tracking_image_status.rec_top_y = tracking_p1y
    tracking_image_status.rec_bottom_x = tracking_p1x + (last_tracking_p2x - last_tracking_p1x)
    tracking_image_status.rec_bottom_y = tracking_p1y + (last_tracking_p2y - last_tracking_p1y)
  end
  tracking_image_status.camera_device_id = 1

  -- Send tracking status message
  mavlink:send_chan(camera_tracking_channel, mavlink_msgs.encode("CAMERA_TRACKING_IMAGE_STATUS", tracking_image_status))
end

local function update()
  check_message_interval_commands()
  update_camera()
  update_fake_tracking()
  send_tracking_status()

  return update, UPDATE_INTERVAL_MS
end

return update, UPDATE_INTERVAL_MS
