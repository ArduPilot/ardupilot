-- mount-viewpro-driver.lua: Viewpro mount/gimbal driver
--
-- How to use
--   Connect gimbal UART to one of the autopilot's serial ports
--   Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the gimbal
--   Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
--   Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
--   Set CAM_TRIGG_TYPE = 3 (Mount) to enable the camera driver
--   Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
--
-- Packet format
--    byte      description     notes
--    0~3       header          FF 01 0F 10
--    4         roll mode       00=no_control, 01=speed_mode, 02=angle_mode, 04= RC_mode, 05=mode_angle_rel_frame
--    5         pitch mode      same as above
--    6         yaw mode        same as above. should normally be 05
--    7,8       roll speed      low byte, high byte.  units: 0.122 deg/sec
--    9,10      roll angle      low byte, high byte.  units: 0.02197 deg
--    11,12     pitch speed     low byte, high byte.  units: 0.122 deg/sec
--    13,14     pitch angle     low byte, high byte.  units: 0.02197 deg
--    15,16     yaw speed       low byte, high byte.  units: 0.122 deg/sec
--    17,18     yaw angle       low byte, high byte.  units: 0.02197 deg
--    19        checksum        body checksum, checksum is calculated as a sum of all data bytes (from "roll mode" to "yaw angle") modulo 256
--

-- global definitions
local INIT_INTERVAL_MS = 3000           -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 20           -- update at 50hz
local MOUNT_INSTANCE = 0                -- always control the first mount/gimbal
local VIEWPRO_HEADER1 = 0xFF            -- Viewpro protocol's 1st header byte
local VIEWPRO_HEADER2 = 0x01            -- Viewpro protocol's 2nd header byte
local VIEWPRO_HEADER3 = 0x0F            -- Viewpro protocol's 3rd header byte
local VIEWPRO_HEADER4 = 0x10            -- Viewpro protocol's 4th header byte
local VIEWPRO_NOCONTROL_MODE = 0        -- no control mode for an axis
local VIEWPRO_SPEED_MODE = 1            -- speed mode for an axis
local VIEWPRO_ANGLE_MODE = 2            -- earth-frame angle mode for any axis
local VIEWPRO_ANGLE_REL_FRAME_MODE = 5  -- body-frame angle mode for yaw axis
local ANGLE_DEG_TO_OUTPUT_SCALING = (8192.0 / 180.0)    -- scalar to convert angles (in deg) to output scaling

-- incoming message definitions
local HEADER1 = 0x3E
local HEADER2 = 0x3D
local HEADER3 = 0x36
local HEADER4 = 0x73

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER1   = 0
local PARSE_STATE_WAITING_FOR_HEADER2   = 1
local PARSE_STATE_WAITING_FOR_HEADER3   = 2
local PARSE_STATE_WAITING_FOR_HEADER4   = 3
local PARSE_STATE_WAITING_FOR_DATA      = 4

-- hardcoded outgoing messages
local REQUEST_ATTITUDE_MSG = {0x3E,0x3D,0x00,0x3D,0x00}
local ZOOM_OUT_MSG = {0xFF,0x01,0x00,0x40,0x00,0x00,0x41}
local ZOOM_IN_MSG = {0xFF,0x01,0x00,0x20,0x00,0x00,0x21}
local ZOOM_STOP_MSG = {0xFF,0x01,0x00,0x00,0x00,0x00,0x01}
local FOCUS_IN_MSG = {0xFF,0x01,0x01,0x00,0x00,0x00,0x02}
local FOCUS_OUT_MSG = {0xFF,0x01,0x00,0x80,0x00,0x00,0x81}
local FOCUS_STOP_MSG = {0xFF,0x01,0x00,0x00,0x00,0x00,0x01} -- same as ZOOM_STOP
local RECORD_START = {0xFF,0x01,0x00,0x07,0x00,0x65,0x6d}
local RECORD_STOP = {0xFF,0x01,0x00,0x07,0x00,0x64,0x6c}
local TAKE_PICTURE = {0xFF,0x01,0x00,0x07,0x00,0x55,0x5D}

-- local variables and definitions
local uart                              -- uart object connected to mount
local initialised = false               -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER1 -- parse state
local parse_expected_crc = 0            -- incoming messages expected crc.  this is checked against actual crc received
local parse_data_buff = {}              -- data buffer holding roll, pitch and yaw angles from gimbal
local parse_data_bytes_recv = 0         -- count of the number of bytes received in the message so far
local cam_pic_count = 0                 -- last picture count.  used to detect trigger pic
local cam_rec_video = false             -- last record video state.  used to detect record video
local cam_zoom_step = 0                 -- last zoom step state.  zoom out = -1, hold = 0, zoom in = 1
local cam_focus_step = 0                -- last focus step state.  focus in = -1, focus hold = 0, focus out = 1

-- find and initialise serial port connected to gimbal
function init()
  uart = serial:find_serial(0)    -- 1st instance of SERIALx_PROTOCOL = 28 (Scripting)
  if uart == nil then
    gcs:send_text(3, "ViewPro: no SERIALx_PROTOCOL = 28") -- MAV_SEVERITY_ERR
  else
    uart:begin(115200)
    uart:set_flow_control(0)
    initialised = true
  end
end

-- send hard coded message
function send_msg(msg)
  for i=1,#msg do
    uart:write(msg[i])
  end
end

-- reading incoming packets from gimbal
function read_incoming_packets()
  local n_bytes = uart:available()
  while n_bytes > 0 do
    n_bytes = n_bytes - 1
    b = uart:read()

    -- waiting for header1
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER1 and b == HEADER1 then
      parse_state = PARSE_STATE_WAITING_FOR_HEADER2
      parse_expected_crc = 0
      parse_data_bytes_recv = 0
      break
    end

    -- waiting for header2
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER2 then
      if b == HEADER2 then
        parse_state = PARSE_STATE_WAITING_FOR_HEADER3
      else
        -- unexpected byte so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
      end
      break
    end

    -- waiting for header3
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER3 then
      if b == HEADER3 then
        parse_state = PARSE_STATE_WAITING_FOR_HEADER4
      else
        -- unexpected byte so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
      end
      break
    end

    -- waiting for header4
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER4 then
      if b == HEADER4 then
        parse_state = PARSE_STATE_WAITING_FOR_DATA
      else
        -- unexpected byte so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
      end
      break
    end

    -- waiting for data
    if parse_state == PARSE_STATE_WAITING_FOR_DATA then

      -- check for crc
      if parse_data_bytes_recv >= 54 then
        if b == parse_expected_crc then
          -- complete parsing
          local roll_deg = bytearray_to_angle_deg(parse_data_buff[5], parse_data_buff[6], parse_data_buff[7], parse_data_buff[8])
          local pitch_deg = bytearray_to_angle_deg(parse_data_buff[23], parse_data_buff[24], parse_data_buff[25], parse_data_buff[26])
          local yaw_deg = bytearray_to_angle_deg(parse_data_buff[41], parse_data_buff[42], parse_data_buff[43], parse_data_buff[44])          
          mount:set_attitude_euler(MOUNT_INSTANCE, roll_deg, pitch_deg, yaw_deg)
       end
       parse_state = PARSE_STATE_WAITING_FOR_HEADER1
       break
      end

      -- add latest byte to crc and buffer
      parse_expected_crc = (parse_expected_crc + b) & 0xFF
      parse_data_bytes_recv = parse_data_bytes_recv + 1
      parse_data_buff[parse_data_bytes_recv] = b
    end
  end
end

-- write a byte to the uart and update the checksum
function write_byte(b, checksum)
  if b == nil or checksum == nil then
    gcs:send_text(3, "ViewPro: failed to write byte") -- MAV_SEVERITY_ERR
    return
  end
  local byte_to_write = b & 0xFF
  uart:write(byte_to_write)

  return (checksum + byte_to_write) & 0xFF
end

-- get lowbyte of a number
function lowbyte(num)
  return num & 0xFF
end

-- get highbyte of a number
function highbyte(num)
  return (num >> 8) & 0xFF
end

-- convert 4 byte array to angle in degrees1
function bytearray_to_angle_deg(b0, b1, b2, b3)
  return ((b3 << 24) + (b2 << 16) + (b1 << 8) + b0) / ANGLE_DEG_TO_OUTPUT_SCALING
end

-- write num to the uart and update the checksum
function write_number(num, checksum)
  if not num or not checksum then
    gcs:send_text(3, "ViewPro: failed to write number") -- MAV_SEVERITY_ERR
    return
  end
  checksum = write_byte(lowbyte(num), checksum)
  checksum = write_byte(highbyte(num), checksum)
  return checksum
end

-- convert angle (in degres) to output scaling
function angle_to_output_scaling(angle_deg)
  local scaled_angle = math.floor(angle_deg * ANGLE_DEG_TO_OUTPUT_SCALING + 0.5)
  if scaled_angle > 8192 then
    scaled_angle = 8192
  end
  if scaled_angle < -8192 then
    scaled_angle = -8192
  end
  return scaled_angle
end

-- convert rate (in deg/sec) to output scaling
function rate_to_output_scaling(rate_degs)
  local scaled_rate = math.floor(rate_degs / 0.122 + 0.5)
  if scaled_rate > 8192 then
    scaled_rate = 8192
  end
  if scaled_rate < -8192 then
    scaled_rate = -8192
  end
  return scaled_rate
end

-- send target angles (in degrees) to gimbal
-- yaw_angle_deg is always a body-frame angle
function send_target_angles(roll_angle_deg, pitch_angle_deg, yaw_angle_deg)
  local roll_angle_scaled = angle_to_output_scaling(roll_angle_deg)
  local pitch_angle_scaled = angle_to_output_scaling(pitch_angle_deg)
  local yaw_angle_scaled = angle_to_output_scaling(yaw_angle_deg)
  local checksum = 0
  write_byte(VIEWPRO_HEADER1, 0)
  write_byte(VIEWPRO_HEADER2, 0)
  write_byte(VIEWPRO_HEADER3, 0)
  write_byte(VIEWPRO_HEADER4, 0)
  checksum = write_byte(VIEWPRO_ANGLE_REL_FRAME_MODE, checksum) -- roll angle mode
  checksum = write_byte(VIEWPRO_ANGLE_REL_FRAME_MODE, checksum) -- pitch angle mode
  checksum = write_byte(VIEWPRO_ANGLE_REL_FRAME_MODE, checksum) -- yaw body-frame angle mode
  checksum = write_number(0, checksum)                  -- roll speed low byte, high byte
  checksum = write_number(roll_angle_scaled, checksum)  -- roll angle low byte, high byte
  checksum = write_number(0, checksum)                  -- pitch speed low byte, high byte
  checksum = write_number(pitch_angle_scaled, checksum) -- pitch angle low byte, high byte
  checksum = write_number(0, checksum)                  -- yaw speed low byte, high byte
  checksum = write_number(yaw_angle_scaled, checksum)   -- yaw angle low byte, high byte
  write_byte(checksum, 0)                               -- checksum
end

-- send target rates (in deg/sec) to gimbal
function send_target_rates(roll_rate_deg, pitch_rate_degs, yaw_rate_degs)
  local roll_rate_scaled = rate_to_output_scaling(roll_rate_deg)
  local pitch_rate_scaled = rate_to_output_scaling(pitch_rate_degs)
  local yaw_rate_scaled = rate_to_output_scaling(yaw_rate_degs)
  local checksum = 0
  write_byte(VIEWPRO_HEADER1, 0)
  write_byte(VIEWPRO_HEADER2, 0)
  write_byte(VIEWPRO_HEADER3, 0)
  write_byte(VIEWPRO_HEADER4, 0)
  checksum = write_byte(VIEWPRO_SPEED_MODE, checksum)   -- roll rate mode
  checksum = write_byte(VIEWPRO_SPEED_MODE, checksum)   -- pitch rate mode
  checksum = write_byte(VIEWPRO_SPEED_MODE, checksum)   -- yaw rate mode
  checksum = write_number(roll_rate_scaled, checksum)   -- roll speed low byte, high byte
  checksum = write_number(0, checksum)                  -- roll angle low byte, high byte
  checksum = write_number(pitch_rate_scaled, checksum)  -- pitch speed low byte, high byte
  checksum = write_number(0, checksum)                  -- pitch angle low byte, high byte
  checksum = write_number(yaw_rate_scaled, checksum)    -- yaw speed low byte, high byte
  checksum = write_number(0, checksum)                  -- yaw angle low byte, high byte
  write_byte(checksum, 0)                               -- checksum
end

-- check for changes in camera state and send messages to gimbal if required
function check_camera_state()

  -- get latest state from AP driver
  -- auto focus is not supported
  local pic_count, rec_video, zoom_step, focus_step, auto_focus = mount:get_camera_state(MOUNT_INSTANCE)

  -- check for take picture
  if pic_count and pic_count ~= cam_pic_count then
    cam_pic_count = pic_count
    send_msg(TAKE_PICTURE)
    gcs:send_text(6, string.format("ViewPro: took pic %u", pic_count)) -- MAV_SEVERITY_INFO
  end

  -- check for start/stop recording video
  if rec_video ~= cam_rec_video then
    cam_rec_video = rec_video
    if cam_rec_video == true then
      send_msg(RECORD_START)
    elseif cam_rec_video == false then
      send_msg(RECORD_STOP)
    end
    gcs:send_text(6, "ViewPro: rec video:" .. tostring(cam_rec_video)) -- MAV_SEVERITY_INFO
  end
    
  -- check manual zoom
  -- zoom out = -1, hold = 0, zoom in = 1
  if zoom_step ~= cam_zoom_step then
    cam_zoom_step = zoom_step
    if cam_zoom_step < 0 then
      send_msg(ZOOM_OUT_MSG)
    elseif cam_zoom_step == 0 then
      send_msg(ZOOM_STOP_MSG)
    elseif cam_zoom_step > 0 then
      send_msg(ZOOM_IN_MSG)
    end
    gcs:send_text(6, "ViewPro: zoom:" .. tostring(cam_zoom_step)) -- MAV_SEVERITY_INFO
  end

  -- check manual focus
  -- focus in = -1, focus hold = 0, focus out = 1
  if focus_step ~= cam_focus_step then
    cam_focus_step = focus_step
    if cam_focus_step < 0 then
      send_msg(FOCUS_IN_MSG)
    elseif cam_focus_step == 0 then
      send_msg(FOCUS_STOP_MSG)
    elseif cam_focus_step > 0 then
      send_msg(FOCUS_IN_MSG)
    end
    gcs:send_text(6, "ViewPro: focus:" .. tostring(cam_focus_step)) -- MAV_SEVERITY_INFO
  end
end

-- the main update function that performs a simplified version of RTL
function update()

  -- get current system time
  local now_ms = millis()

  -- initialise connection to gimbal
  if not initialised then
    init()
    return update, INIT_INTERVAL_MS
  end

  -- request gimbal attitude
  send_msg(REQUEST_ATTITUDE_MSG)

  -- check camera status
  check_camera_state()

  -- consume incoming bytes
  read_incoming_packets()

  -- get target location
  -- local target_loc = mount:get_location_target(MOUNT_INSTANCE)
  -- if target_loc then
  --   gcs:send_text(6, string.format("ViewPro: lat:%.1f lon:%.1f alt:%.1f", target_loc:lat(), target_loc:lng(), target_loc:alt())) -- MAV_SEVERITY_INFO
  -- end

  -- get target angle
  local roll_deg, pitch_deg, yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
  if roll_deg and pitch_deg and yaw_deg then
    send_target_angles(roll_deg, pitch_deg, yaw_deg)
    return update, UPDATE_INTERVAL_MS
  end

  -- get target rate
  local roll_degs, pitch_degs, yaw_degs, yaw_is_ef = mount:get_rate_target(MOUNT_INSTANCE)
  if roll_degs and pitch_degs and yaw_degs then
    send_target_rates(roll_degs, pitch_degs, yaw_degs)
    return update, UPDATE_INTERVAL_MS
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
