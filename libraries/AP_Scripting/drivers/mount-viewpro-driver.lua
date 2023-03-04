-- mount-viewpro-driver.lua: Viewpro mount/gimbal driver

--[[
  How to use
    Connect gimbal UART to one of the autopilot's serial ports
    Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the gimbal
    Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
    Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
    Set CAM_TRIGG_TYPE = 3 (Mount) to enable the camera driver
    Set RCx_OPTION = 300 (Scripting1) to allow real-time selection of the video feed and camera control
    Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
    Set VIEP_CAM_SWLOW, VIEP_CAM_SWMID, VIEP_CAM_SWHIGH to which cameras are controlled by the auxiliary switch
        0: No change in camera selection
        1: EO1
        2: IR thermal
        3: EO1 + IR Picture-in-picture
        4: IR + EO1 Picture-in-picture
        5: Fusion
        6: IR1 13mm
        7: IR2 52mm
    Set VIEP_ZOOM_SPEED to control speed of zoom (value between 0 and 7)
    Set VIEP_DEBUG = 1 or 2 to increase level of debug output to the GCS
 
  Packet format
     byte      description     notes
     0~2       header          0x55 0xAA 0xDC
     3         body length     bit0~5: body length, n=all bytes from byte3 to checksum, min=4, max=63.  bits6~7: frame counter
     4         frame id
     5~n+1     data            1st byte is command id?
     n+2       checksum        XOR of byte3 to n+1 (inclusive)
--]]

-- parameters
local PARAM_TABLE_KEY = 39
assert(param:add_table(PARAM_TABLE_KEY, "VIEP_", 5), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add VIEP_DEBUG param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "CAM_SWLOW", 1), "could not add VIEP_CAM_SWLOW param")
assert(param:add_param(PARAM_TABLE_KEY, 3, "CAM_SWMID", 2), "could not add VIEP_CAM_SWMID param")
assert(param:add_param(PARAM_TABLE_KEY, 4, "CAM_SWHIGH", 6), "could not add VIEP_CAM_CAM_SWHIGH param")
assert(param:add_param(PARAM_TABLE_KEY, 5, "ZOOM_SPEED", 7), "could not add VIEP_ZOOM_SPEED param")

-- bind parameters to variables
local MNT1_TYPE = Parameter("MNT1_TYPE")    -- should be 9:Scripting
local VIEP_DEBUG = Parameter("VIEP_DEBUG")  -- debug level. 0:disabled 1:enabled 2:enabled with attitude reporting
local VIEP_CAM_SWLOW = Parameter("VIEP_CAM_SWLOW")      -- RC swith low position's camera selection
local VIEP_CAM_SWMID = Parameter("VIEP_CAM_SWMID")      -- RC swith middle position's camera selection
local VIEP_CAM_SWHIGH = Parameter("VIEP_CAM_SWHIGH")    -- RC swith high position's camera selection
local VIEP_ZOOM_SPEED = Parameter("VIEP_ZOOM_SPEED")    -- zoom speed from 0 (slow) to 7 (fast)

-- global definitions
local CAM_SELECT_RC_OPTION = 300        -- rc channel option used to control which camera/video is used. RCx_OPTION = 300 (scripting1)
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local INIT_INTERVAL_MS = 3000           -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 100          -- update at 10hz
local MOUNT_INSTANCE = 0                -- always control the first mount/gimbal

-- packet parsing definitions
local HEADER1 = 0x55                    -- 1st header byte
local HEADER2 = 0xAA                    -- 2nd header byte
local HEADER3 = 0xDC                    -- 3rd header byte
local PACKET_LENGTH_MIN = 4             -- serial packet minimum length.  used for sanity checks
local PACKET_LENGTH_MAX = 63            -- serial packet maximum length.  used for sanity checks

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER1   = 0
local PARSE_STATE_WAITING_FOR_HEADER2   = 1
local PARSE_STATE_WAITING_FOR_HEADER3   = 2
local PARSE_STATE_WAITING_FOR_LENGTH    = 3
local PARSE_STATE_WAITING_FOR_FRAMEID   = 4
local PARSE_STATE_WAITING_FOR_DATA      = 5

-- received FrameId
local T1_F1_B1_D1_FRAMEID = 0x40        -- includes roll, pitch, yaw angles

-- camera operation commands
local CAM_COMMAND_NO_ACTION = 0x00
local CAM_COMMAND_STOP_FOCUS_AND_ZOOM = 0x01
local CAM_COMMAND_ZOOM_OUT = 0x08
local CAM_COMMAND_ZOOM_IN = 0x09
local CAM_COMMAND_FOCUS_PLUS = 0x0A
local CAM_COMMAND_FOCUS_MINUS = 0x0B
local CAM_COMMAND_TAKE_PICTURE = 0x13
local CAM_COMMAND_START_RECORD = 0x14
local CAM_COMMAND_STOP_RECORD = 0x15
local CAM_COMMAND_AUTO_FOCUS = 0x19
local CAM_COMMAND_MANUAL_FOCUS = 0x1A

-- hardcoded outgoing messages
local HEARTBEAT_MSG = {0x55,0xAA,0xDC,0x44,0x00,0x00,0x44}

-- local variables and definitions
local uart                              -- uart object connected to mount
local initialised = false               -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER1 -- parse state
local parse_expected_crc = 0            -- incoming messages expected crc.  this is checked against actual crc received
local parse_length = 0                  -- incoming message parsed length
local parse_frameid = 0                 -- incoming message command id
local parse_data_buff = {}              -- data buffer holding roll, pitch and yaw angles from gimbal
local parse_data_bytes_recv = 0         -- count of the number of bytes received in the message so far
local last_frame_counter = 0            -- last frame counter sent to gimbal.  always between 0 and 3
local cam_choice = 0                    -- last camera choice (see VIEP_CAM_SWLOW/MID/HIGH parameters)
local cam_pic_count = 0                 -- last picture count.  used to detect trigger pic
local cam_rec_video = false             -- last record video state.  used to detect record video
local cam_zoom_step = 0                 -- last zoom step state.  zoom out = -1, hold = 0, zoom in = 1
local cam_focus_step = 0                -- last focus step state.  focus in = -1, focus hold = 0, focus out = 1
local cam_autofocus = false             -- last auto focus state

-- parsing status reporting variables
local last_print_ms = 0                 -- system time that debug output was last printed
local bytes_read = 0                    -- number of bytes read from gimbal
local bytes_written = 0                 -- number of bytes written to gimbal
local bytes_error = 0                   -- number of bytes read that could not be parsed
local msg_ignored = 0                   -- number of ignored messages (because frame id does not match)

-- debug variables
local last_test_send_md = 0             -- system time that a test message was last sent
local debug_buff = {}                   -- debug buffer to display bytes from gimbal

-- get lowbyte of a number
function lowbyte(num)
  return num & 0xFF
end

-- get highbyte of a number
function highbyte(num)
  return (num >> 8) & 0xFF
end

-- get uint16 from two bytes
function uint16_value(hbyte, lbyte)
  return ((hbyte & 0xFF) << 8) | (lbyte & 0xFF)
end

-- get int16 from two bytes
function int16_value(hbyte, lbyte)
  local uret = uint16_value(hbyte, lbyte)
  if uret <= 0x8000 then
    return uret
  else
    return uret - 0x10000
  end
end

-- wrap yaw angle in degrees to value between 0 and 360
function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

-- wrap yaw angle in degrees to value between -180 and +180
function wrap_180(angle_deg)
  local res = wrap_360(angle_deg)
  if res > 180 then
    res = res - 360
  end
  return res
end

-- calculate crc from existing crc value and new byte
function calc_crc(orig_crc, b)
  local crc = (orig_crc ~ b) & 0xFF
  return crc
end

-- find and initialise serial port connected to gimbal
function init()
  -- check mount parameter
  if MNT1_TYPE:get() ~= 9 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "ViewPro: set MNT1_TYPE=9")
    do return end
  end

  -- find and init first instance of SERIALx_PROTOCOL = 28 (Scripting)
  uart = serial:find_serial(0)
  if uart == nil then
    gcs:send_text(3, "ViewPro: no SERIALx_PROTOCOL = 28") -- MAV_SEVERITY_ERR
  else
    uart:begin(115200)
    uart:set_flow_control(0)
    initialised = true
    gcs:send_text(MAV_SEVERITY.INFO, "ViewPro: started")
  end
end

-- send hard coded message
function send_msg(msg)
  for i=1,#msg do
    uart:write(msg[i])
    -- debug
    bytes_written = bytes_written + 1
  end
end

-- parse test message
function parse_test_msg(msg)
  for i=1,#msg do
    parse_byte(msg[i])
  end
end

-- reading incoming packets from gimbal
function read_incoming_packets()
  local n_bytes = uart:available()
  while n_bytes > 0 do
    n_bytes = n_bytes - 1
    parse_byte(uart:read())
  end
end

-- parse a single byte from gimbal
function parse_byte(b)
    -- record num bytes for reporting
    bytes_read = bytes_read + 1

    -- debug
    if VIEP_DEBUG:get() > 1 then
      debug_buff[#debug_buff+1] = b
      if #debug_buff >= 10 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: %x %x %x %x %x %x %x %x %x %x", debug_buff[1], debug_buff[2], debug_buff[3], debug_buff[4], debug_buff[5], debug_buff[6], debug_buff[7], debug_buff[8], debug_buff[9], debug_buff[10]))
        debug_buff = {}
      end
    end

    -- waiting for header1
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER1 then
      if b == HEADER1 then
        parse_state = PARSE_STATE_WAITING_FOR_HEADER2
        parse_expected_crc = 0
        parse_data_bytes_recv = 0
        do return end
      end
      bytes_error = bytes_error + 1
    end

    -- waiting for header2
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER2 then
      if b == HEADER2 then
        parse_state = PARSE_STATE_WAITING_FOR_HEADER3
      else
        -- unexpected byte so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
        bytes_error = bytes_error + 1
      end
      do return end
    end

    -- waiting for header3
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER3 then
      if b == HEADER3 then
        parse_state = PARSE_STATE_WAITING_FOR_LENGTH
      else
        -- unexpected byte so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
        bytes_error = bytes_error + 1
      end
      do return end
    end

    -- waiting for length
    if parse_state == PARSE_STATE_WAITING_FOR_LENGTH then
      parse_expected_crc = calc_crc(parse_expected_crc, b)
      parse_length = b & 0x3F
      if parse_length >= PACKET_LENGTH_MIN and parse_length <= PACKET_LENGTH_MAX then
        parse_state = PARSE_STATE_WAITING_FOR_FRAMEID
      else
        -- unexpected length
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
        bytes_error = bytes_error + 1
        if VIEP_DEBUG:get() > 0 then
          gcs:send_text(MAV_SEVERITY.ERROR, string.format("ViewPro: invalid len:%d", parse_length))
        end
      end
      do return end
    end

    -- waiting for command id
    if parse_state == PARSE_STATE_WAITING_FOR_FRAMEID then
      parse_expected_crc = calc_crc(parse_expected_crc, b)
      parse_frameid = b
      parse_state = PARSE_STATE_WAITING_FOR_DATA
      do return end
    end

    -- waiting for data
    if parse_state == PARSE_STATE_WAITING_FOR_DATA then

      -- check for crc
      if parse_data_bytes_recv >= parse_length - 3 then
        if b == parse_expected_crc then
          -- crc matched, process packet
          local processed = false

          -- T1_F1_B1_D1
          if parse_frameid == T1_F1_B1_D1_FRAMEID then
            processed = true
            -- T1 holds target info including target lean angles
            -- F1 holds tracker sensor status (which camera, tracking vs lost)
            -- B1 section holds actual lean angles
            -- D1 section holds camera status including zoom level
            local servo_status = (parse_data_buff[24] & 0xF0 >> 4)
            local roll_deg = int16_value(parse_data_buff[24] & 0x0F, parse_data_buff[25]) * (180.0/4095.0) - 90.0
            local yaw_deg = int16_value(parse_data_buff[26], parse_data_buff[27]) * (360.0 / 65536.0)
            local pitch_deg = int16_value(parse_data_buff[28], parse_data_buff[29]) * (360.0 / 65536.0)
            mount:set_attitude_euler(MOUNT_INSTANCE, roll_deg, pitch_deg, yaw_deg)

            if VIEP_DEBUG:get() > 0 then
              gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: r:%f p:%f y:%f ss:%x", roll_deg, pitch_deg, yaw_deg, servo_status))
            end
          end

          if not processed then
            msg_ignored = msg_ignored + 1
            if VIEP_DEBUG:get() > 0 then
              gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: ignored frameid:%x", parse_frameid))
            end
          end
        else
          -- crc mismatch
          bytes_error = bytes_error + 1
          if VIEP_DEBUG:get() > 0 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: crc exp:%x got:%x", parse_expected_crc, b))
          end
        end
        parse_state = PARSE_STATE_WAITING_FOR_HEADER1
        do return end
      end

      -- add latest byte to crc and buffer
      parse_expected_crc = calc_crc(parse_expected_crc, b)
      parse_data_bytes_recv = parse_data_bytes_recv + 1
      parse_data_buff[parse_data_bytes_recv] = b
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
  bytes_written = bytes_written + 1
  local checksum_ret = (checksum ~ byte_to_write) & 0xFF
  return checksum_ret
end

-- calculate length and frame count byte
-- length is all bytes after the header including CRC
function calc_length_and_frame_byte(length)
  -- increment frame counter
  last_frame_counter = last_frame_counter + 1 & 0x03
  return  (last_frame_counter << 6 | length & 0x3F) & 0xFF
end

-- send target angles (in degrees) to gimbal
-- yaw_angle_deg is always a body-frame angle
function send_target_angles(pitch_angle_deg, yaw_angle_deg)

  -- prepare A1 message, FrameId 0x1A, CmdId 0x0B
  local length_and_frame_counter = calc_length_and_frame_byte(0x0C)
  local pitch_angle_output = math.floor((-pitch_angle_deg / 360.0 * 65536.0) + 0.5)
  local yaw_angle_output = math.floor((yaw_angle_deg / 360.0 * 65536.0) + 0.5)

  write_byte(HEADER1, 0)
  write_byte(HEADER2, 0)
  write_byte(HEADER3, 0)
  local checksum = write_byte(length_and_frame_counter, 0)      -- length and frame count
  checksum = write_byte(0x1A, checksum)                         -- 0x1A: A1 FrameId
  checksum = write_byte(0x0B, checksum)                         -- 0x0B: absolute angle
  checksum = write_byte(highbyte(yaw_angle_output), checksum)   -- yaw angle MSB
  checksum = write_byte(lowbyte(yaw_angle_output), checksum)    -- yaw angle LSB
  checksum = write_byte(highbyte(pitch_angle_output), checksum) -- pitch angle MSB
  checksum = write_byte(lowbyte(pitch_angle_output), checksum)  -- pitch angle LSB
  checksum = write_byte(0, checksum)                            -- unused
  checksum = write_byte(0, checksum)                            -- unused
  checksum = write_byte(0, checksum)                            -- unused
  checksum = write_byte(0, checksum)                            -- unused
  write_byte(checksum, 0)                                       -- checksum
end

-- send target rates (in deg/sec) to gimbal
function send_target_rates(pitch_rate_degs, yaw_rate_degs)

  -- prepare A1 message, FrameId 0x1A, CmdId 0x01
  local length_and_frame_counter = calc_length_and_frame_byte(0x0C)
  local pitch_rate_output = math.floor((-pitch_rate_degs * 100.0) + 0.5)
  local yaw_rate_output = math.floor((yaw_rate_degs * 100.0) + 0.5)

  write_byte(HEADER1, 0)
  write_byte(HEADER2, 0)
  write_byte(HEADER3, 0)
  local checksum = write_byte(length_and_frame_counter, 0)      -- length and frame count
  checksum = write_byte(0x1A, checksum)                         -- 0x1A: A1 FrameId
  checksum = write_byte(0x01, checksum)                         -- 0x01: manual rate angle
  checksum = write_byte(highbyte(yaw_rate_output), checksum)    -- yaw rate MSB
  checksum = write_byte(lowbyte(yaw_rate_output), checksum)     -- yaw rate LSB
  checksum = write_byte(highbyte(pitch_rate_output), checksum)  -- pitch angle MSB
  checksum = write_byte(lowbyte(pitch_rate_output), checksum)   -- pitch angle LSB
  checksum = write_byte(0, checksum)                            -- unused
  checksum = write_byte(0, checksum)                            -- unused
  checksum = write_byte(0, checksum)                            -- unused
  checksum = write_byte(0, checksum)                            -- unused
  write_byte(checksum, 0)                                       -- checksum
end

-- send camera commands
function send_camera_control(camera_choice, cam_command)

  -- prepare C1 message, FrameId 0x1C
  -- bits 0~2 : video choose
  -- bits 3~5 : zoom speed
  -- bits 6~12 : operation command
  -- bits 13~15 : LRF (rangefinder)
  local length_and_frame_counter = calc_length_and_frame_byte(0x05)

  local video_choose = camera_choice & 0x07

  local zoom_speed = 0
  if cam_command == CAM_COMMAND_ZOOM_OUT or cam_command == CAM_COMMAND_ZOOM_IN then
    zoom_speed = (VIEP_ZOOM_SPEED:get() & 0x07) << 3
  end

  local operation_cmd = ((cam_command & 0xFFFF) & 0x7F) << 6
  local data_bytes = video_choose | zoom_speed | operation_cmd

  -- debug
  --gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: camcmd:%x msb:%x lsb:%x", cam_command, highbyte(cmd_shifted), lowbyte(cmd_shifted)))
  gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: cam choice:%x", video_choose))

  write_byte(HEADER1, 0)
  write_byte(HEADER2, 0)
  write_byte(HEADER3, 0)
  local checksum = write_byte(length_and_frame_counter, 0)  -- length and frame count
  checksum = write_byte(0x1C, checksum)                 -- 0x1C: C1 FrameId
  checksum = write_byte(highbyte(data_bytes), checksum) -- msb
  checksum = write_byte(lowbyte(data_bytes), checksum)  -- lsb
  write_byte(checksum, 0)                               -- checksum
end

-- return camera selection according to RC switch position and VIEW_CAM_SWxxx parameter
-- used in C1 message's "video choose" to specify which cameras should be controlled
function get_camera_choice()
  local cam_switch_pos = rc:get_aux_cached(CAM_SELECT_RC_OPTION)
  if cam_switch_pos == 0 then
    return VIEP_CAM_SWLOW:get()
  end
  if cam_switch_pos == 1 then
    return VIEP_CAM_SWMID:get()
  end
  return VIEP_CAM_SWHIGH:get()
end

-- check for changes in camera state and send messages to gimbal if required
function check_camera_state()

  -- check for change in camera
  local curr_cam_choice = get_camera_choice()
  if cam_choice ~= curr_cam_choice then
    cam_choice = curr_cam_choice
    send_camera_control(cam_choice, CAM_COMMAND_NO_ACTION)
  end

  -- get latest state from AP driver
  local pic_count, rec_video, zoom_step, focus_step, auto_focus = mount:get_camera_state(MOUNT_INSTANCE)

  -- check for take picture
  if pic_count and pic_count ~= cam_pic_count then
    cam_pic_count = pic_count
    send_camera_control(cam_choice, CAM_COMMAND_TAKE_PICTURE)
    if VIEP_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: took pic %u", pic_count))
    end
  end

  -- check for start/stop recording video
  if rec_video ~= cam_rec_video then
    cam_rec_video = rec_video
    if cam_rec_video == true then
      send_camera_control(cam_choice, CAM_COMMAND_START_RECORD)
    elseif cam_rec_video == false then
      send_camera_control(cam_choice, CAM_COMMAND_STOP_RECORD)
    end
    if VIEP_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "ViewPro: rec video:" .. tostring(cam_rec_video))
    end
  end
    
  -- check manual zoom
  -- zoom out = -1, hold = 0, zoom in = 1
  if zoom_step ~= cam_zoom_step then
    cam_zoom_step = zoom_step
    if cam_zoom_step < 0 then
      send_camera_control(cam_choice, CAM_COMMAND_ZOOM_OUT)
    elseif cam_zoom_step == 0 then
      send_camera_control(cam_choice, CAM_COMMAND_STOP_FOCUS_AND_ZOOM)
    elseif cam_zoom_step > 0 then
      send_camera_control(cam_choice, CAM_COMMAND_ZOOM_IN)
    end
    if VIEP_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "ViewPro: zoom:" .. tostring(cam_zoom_step))
    end
  end

  -- check manual focus
  -- focus in = -1, focus hold = 0, focus out = 1
  if focus_step ~= cam_focus_step then
    cam_focus_step = focus_step
    if cam_focus_step < 0 then
      send_camera_control(cam_choice, CAM_COMMAND_MANUAL_FOCUS)
      send_camera_control(cam_choice, CAM_COMMAND_FOCUS_MINUS)
    elseif cam_focus_step == 0 then
      send_camera_control(cam_choice, CAM_COMMAND_STOP_FOCUS_AND_ZOOM)
    elseif cam_focus_step > 0 then
      send_camera_control(cam_choice, CAM_COMMAND_MANUAL_FOCUS)
      send_camera_control(cam_choice, CAM_COMMAND_FOCUS_PLUS)
    end
    if VIEP_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "ViewPro: focus:" .. tostring(cam_focus_step))
    end
  end

  -- check auto focus
  if auto_focus ~= cam_autofocus then
    cam_autofocus = auto_focus
    if cam_autofocus then
      send_camera_control(cam_choice, CAM_COMMAND_AUTO_FOCUS)
    end
    if VIEP_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "ViewPro: auto focus:" .. tostring(cam_autofocus))
    end
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

  -- status reporting
  if (VIEP_DEBUG:get() > 0) and (now_ms - last_print_ms > 5000) then
    last_print_ms = now_ms
    gcs:send_text(MAV_SEVERITY.INFO, string.format("ViewPro: r:%u w:%u err:%u ign:%u", bytes_read, bytes_written, bytes_error, msg_ignored))
  end

  -- consume incoming bytes
  read_incoming_packets()

  -- check camera state and send control messages
  check_camera_state()

  -- send heartbeat, gimbal should respond with T1+F1+B1+D1
  send_msg(HEARTBEAT_MSG)

  -- request gimbal attitude by sending heartbeat
  if now_ms - last_test_send_md > 1000 then
    last_test_send_md = now_ms
    send_msg(HEARTBEAT_MSG)
  end

  -- send target angle to gimbal
  local roll_deg, pitch_deg, yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
  if roll_deg and pitch_deg and yaw_deg then
    if yaw_is_ef then
      yaw_deg = wrap_180(yaw_deg - math.deg(ahrs:get_yaw()))
    end
    send_target_angles(pitch_deg, yaw_deg)
    return update, UPDATE_INTERVAL_MS
  end

  -- send target rate to gimbal
  local roll_degs, pitch_degs, yaw_degs, _ = mount:get_rate_target(MOUNT_INSTANCE)
  if roll_degs and pitch_degs and yaw_degs then
    send_target_rates(pitch_degs, yaw_degs)
    return update, UPDATE_INTERVAL_MS
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
