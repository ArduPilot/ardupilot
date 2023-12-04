-- mount-djirs2-driver.lua: DJIRS2 mount/gimbal driver
-- luacheck: only 0

--[[
  How to use
    Connect gimbal to autopilot's CAN1 port or CAN2 port
    If connected to CAN1, set CAN_D1_PROTOCOL = 10 (Scripting), CAN_P1_DRIVER = 1 (First driver)
    If connected to CAN2, set CAN_D2_PROTOCOL = 10 (Scripting), CAN_P2_DRIVER = 2 (Second driver)
    Set SCR_ENABLE = 1 to enable scripting
    Set SCR_HEAP_SIZE = 120000 (or higher)
    Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
    Reboot the autopilot
    Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
    set DJIR_DEBUG to 1 to display parsing and errors stats at 5sec.  Set to 2 to display gimbal angles
 
  The following sources were used as a reference during the development of this script
    Constant Robotics DJIR SDK: https://github.com/ConstantRobotics/DJIR_SDK
    Ceinem's ROS node for DJI RS2: https://github.com/ceinem/dji_rs2_ros_controller

  CAN format
    packet's data segment holds up to 8 serial bytes.  These should be extracted and sent to the serial packet parser
    the CAN packets will never hold the contents of more than one serial packet

    the external caller (e.g. this script) must send CAN frames with frameId = 0x223
    gimbal will reply with frameId = 0x222

  Serial Packet format
     byte      description     notes
     0         header          AA
     1~2       Ver/Length      bits 0~9 are length of entire frame, LSB first. bits 10~15 are version number
     3         CmdType         bits 0~4 are Reply type, 0=No Reply required, 1=Can reply or not after data sent, 2~31=Reply is required after data sent
                               bit 5 is Frame type, 0=Command frame, 1=Reply Frame
                               bits 6~7 reserved (0 by default)
     4         ENC             bits 0~4 length of supplementary bytes when encrypting
                               bits 5~7 Encryption type, 0=Unencrypted, 1=AES256 encyrption
     5~7       RES             reserved
     8~9       SEQ             sequence number.  increments with each packet
     10~11     CRC-16          frame header check
     12~n      Data            Data segment Start
       12        CmdSet        Data segment Command set
       13        CmdID         Data segment Command code
       14~n      Data content  Data content
     n+1       CRC-32          frame check (the entire frame)

  Used CmdSet and CmdId
     0x0E, 0x00: Handheld Gimbal Position Control
       Command frame bytes
        0~1: yaw angle * 10, int16, -1800 to +1800
        2~3: roll angle * 10, int16, -300 to +300
        4~5: pitch angle * 10, int16, -560 to +1460
        6: ctrl_byte, uint8
              bit0 = 0:relative control, 1:absolute control
              bit1 = 0:yaw axis valid, 1:invalid
              bit2 = 0:roll axis valid, 1:invalid
              bit3 = 0:pitch axis valid, 1:invalid
              bit4~7 = reserved, must be zero
        7: time for action, uint8_t, unit: 0.1s.  e.g. if 20, gimbal will rotate to the position desired within 2sec
      Reply frame bytes
        0: return code, uint8_t

     0x0E, 0x02: Obtain the angle information of handheld gimbal, including joint angle and attitude angle
       Command frame bytes
        0: ctrl_byte, uint8_t, 0x00:No operation, 0x01:angle of handlheld gimbal, 0x02:joint angle of handheld gimbal
      Reply frame bytes
        0: return code, uint8_t
        1: data_type, uint8_t, 0x00:Data is not ready, 0x01:attitude angle, 0x02:joint angle
        2~3: yaw angle * 10, int16, -1800 to +1800
        4~5: roll angle * 10, int16, -300 to +300
        6~7: pitch angle * 10, int16, -560 to +1460


--]]

-- global definitions
local INIT_INTERVAL_MS = 3000           -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 1            -- update interval in millis
local REPLY_TIMEOUT_MS = 100            -- timeout waiting for reply after 0.1 sec
local REQUEST_ATTITUDE_INTERVAL_MS = 100-- request attitude at 10hz
local SET_ATTITUDE_INTERVAL_MS = 100    -- set attitude at 10hz
local MOUNT_INSTANCE = 0                -- always control the first mount/gimbal
local SEND_FRAMEID = 0x223              -- send CAN messages with this frame id
local RECEIVE_FRAMEID = 0x222           -- receive CAN messages with this frame id
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- parameters
local PARAM_TABLE_KEY = 38
assert(param:add_table(PARAM_TABLE_KEY, "DJIR_", 2), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add DJIR_DEBUG param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "UPSIDEDOWN", 0), "could not add DJIR_UPSIDEDOWN param")

--[[
  // @Param: DJIR_DEBUG
  // @DisplayName: DJIRS2 debug
  // @Description: Enable DJIRS2 debug
  // @Values: 0:Disabled,1:Enabled,2:Enabled with attitude reporting
  // @User: Advanced
--]]
local DJIR_DEBUG = Parameter("DJIR_DEBUG")              -- debug level. 0:disabled 1:enabled 2:enabled with attitude reporting

--[[
  // @Param: DJIR_UPSIDEDOWN
  // @DisplayName: DJIRS2 upside down
  // @Description: DJIRS2 upside down
  // @Values: 0:Right side up,1:Upside down
  // @User: Standard
--]]
local DJIR_UPSIDEDOWN = Parameter("DJIR_UPSIDEDOWN")    -- 0:rightsideup, 1:upsidedown

-- bind parameters to variables
local CAN_P1_DRIVER = Parameter("CAN_P1_DRIVER")        -- If using CAN1, should be 1:First driver
local CAN_P1_BITRATE = Parameter("CAN_P1_BITRATE")      -- If using CAN1, should be 1000000
local CAN_D1_PROTOCOL = Parameter("CAN_D1_PROTOCOL")    -- If using CAN1, should be 10:Scripting
local CAN_P2_DRIVER = Parameter("CAN_P2_DRIVER")        -- If using CAN2, should be 2:Second driver
local CAN_P2_BITRATE = Parameter("CAN_P2_BITRATE")      -- If using CAN2, should be 1000000
local CAN_D2_PROTOCOL = Parameter("CAN_D2_PROTOCOL")    -- If using CAN2, should be 10:Scripting
local MNT1_TYPE = Parameter("MNT1_TYPE")                -- should be 9:Scripting

-- message definitions
local HEADER = 0xAA
local RETURN_CODE = {SUCCESS=0x00, PARSE_ERROR=0x01, EXECUTION_FAILED=0x02, UNDEFINED=0xFF}
local ATTITUDE_PACKET_LEN = {LEGACY=24, LATEST=26}      -- attitude packet expected length.  Legacy must be less than latest
local POSITION_CONTROL_PACKET_LEN = {LEGACY=17, LATEST=19}  -- position control packet expected length.  Legacy must be less than latest
local SPEED_CONTROL_PACKET_LEN = {LEGACY=17, LATEST=19} -- speed control packet expected length.  Legacy must be less than latest

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER        = 0
local PARSE_STATE_WAITING_FOR_VERLENGTH     = 1
local PARSE_STATE_WAITING_FOR_DATA          = 2

-- other parsing definitions
local CAN_PACKET_LENGTH_MAX = 8         -- CAN packet maximum length
local SERIAL_PACKET_LENGTH_MAX = 32     -- serial packet maximum length.  used to sanity check length of incoming messages
local SERIAL_PACKET_LENGTH_MIN = 16     -- serial packet minimum length.  used to sanity check sends

-- local variables and definitions
local driver                            -- CAN bus
local initialised = false               -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER  -- parse state
local parse_length = 0                  -- incoming message's packet length
local parse_buff = {}                   -- message buffer holding roll, pitch and yaw angles from gimbal
local parse_bytes_recv = 0              -- message buffer length.  count of the number of bytes received in the message so far
local last_send_seq = 0                 -- last sequence number sent
local last_req_attitude_ms = 0          -- system time of last request for attitude
local last_set_attitude_ms = 0          -- system time of last set attitude call
local REPLY_TYPE = {NONE=0, ATTITUDE=1, POSITION_CONTROL=2, SPEED_CONTROL=3} -- enum of expected reply types
local expected_reply = REPLY_TYPE.NONE  -- currently expected reply type
local expected_reply_ms = 0             -- system time that reply is first expected.  used for timeouts

-- parsing status reporting variables
local last_print_ms = 0                 -- system time that debug output was last printed
local bytes_read = 0                    -- number of bytes read from gimbal
local bytes_written = 0                 -- number of bytes written to gimbal
local bytes_error = 0                   -- number of bytes read that could not be parsed
local msg_ignored = 0                   -- number of ignored messages (because frame id does not match)
local write_fails = 0                   -- number of times write failed
local execute_fails = 0                 -- number of times that gimbal was unable to execute the command
local reply_timeouts = 0                -- number of timeouts waiting for replies

local crc16_lookup = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
}

local crc32_lookup = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
}

-- calculate crc16 for a series of bytes
-- byte_array should be a table of uint8 values
-- start_byte should be the first byte to start from (using 1 indexing) or left as nil to default to the first byte
-- num_bytes should be the number of bytes to process or left as nil to use the entire message
function calc_crc16(byte_array, start_byte, num_bytes)
  start_byte = start_byte or 1
  num_bytes = num_bytes or #byte_array - start_byte + 1
  local crc = 0x3AA3
  for i = start_byte, num_bytes do
    local b = byte_array[i] & 0xFF
    local crc16_lookup_index = ((crc ~ b) % 256) + 1
    local lookup_val = crc16_lookup[crc16_lookup_index]
    crc = ((crc >> 8) & 0xFF) ~ lookup_val
  end
  return crc
end

-- calculate crc32 for a series of bytes
-- byte_array should be a table of uint8 values
-- start_byte should be the first byte to start from (using 1 indexing) or left as nil to default to the first byte
-- num_bytes should be the number of bytes to process or left as nil to use the entire message
function calc_crc32(byte_array, start_byte, num_bytes)
  start_byte = start_byte or 1
  num_bytes = num_bytes or #byte_array - start_byte + 1
  local crc = 0x3AA3
  for i = start_byte, num_bytes do
    local b = byte_array[i] & 0xFF
    local crc32_lookup_index = (((crc ~ b) & 0xff) + 1)
    local lookup_val = crc32_lookup[crc32_lookup_index]
    crc = ((crc >> 8) & 0x00FFFFFF) ~ lookup_val
  end
  return crc
end

-- get lowbyte of a number
function lowbyte(num)
  return num & 0xFF
end

-- get highbyte of a number
function highbyte(num)
  return (num >> 8) & 0xFF
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

-- get uint16 from two bytes
function uint16_value(hbyte, lbyte)
  return ((hbyte & 0xFF) << 8) | (lbyte & 0xFF)
end

-- get uint32 from four bytes
function uint32_value(byte3, byte2, byte1, byte0)
  return (((byte3 & 0xFF) << 24) | ((byte2 & 0xFF) << 16) | ((byte1 & 0xFF) << 8) | (byte0 & 0xFF))
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

-- perform any require initialisation
function init()
  -- check parameter settings
  if CAN_D1_PROTOCOL:get() ~= 10 and CAN_D2_PROTOCOL:get() ~= 10 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set CAN_D1_PROTOCOL or CAN_D2_PROTOCOL=10")
    do return end
  end
  if CAN_D1_PROTOCOL:get() == 10 and CAN_D2_PROTOCOL:get() == 10 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set CAN_D1_PROTOCOL or CAN_D2_PROTOCOL=0")
    do return end
  end
  if CAN_D1_PROTOCOL:get() == 10 then
    if CAN_P1_DRIVER:get() <= 0 then
      gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set CAN_P1_DRIVER=1")
      do return end
    end
    if CAN_P1_BITRATE:get() ~= 1000000 then
      gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set CAN_P1_BITRATE=1000000")
      do return end
    end
  end
  if CAN_D2_PROTOCOL:get() == 10 then
    if CAN_P2_DRIVER:get() <= 0 then
      gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set CAN_P2_DRIVER=2")
      do return end
    end
    if CAN_P2_BITRATE:get() ~= 1000000 then
      gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set CAN_P2_BITRATE=1000000")
      do return end
    end
  end
  if MNT1_TYPE:get() ~= 9 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: set MNT1_TYPE=9")   
    do return end
  end

  -- get CAN device
  driver = CAN:get_device(25)
  if driver then
    initialised = true
    gcs:send_text(MAV_SEVERITY.INFO, "DJIR: mount driver started")   
  else
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: failed to connect to CAN bus")   
  end
end

-- send serial message over CAN bus
-- returns true on success, false on failure
function send_msg(serial_msg)

  if not serial_msg then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: cannot send invalid message")
    do return false end
  end

  -- calculate number of CAN frames required to send mesage
  local num_frames = math.floor(#serial_msg / CAN_PACKET_LENGTH_MAX)
  if #serial_msg % CAN_PACKET_LENGTH_MAX > 0 then
    num_frames = num_frames + 1
  end

  -- create and send CAN messages
  for i=0, num_frames-1 do
    local start_byte = i * CAN_PACKET_LENGTH_MAX + 1
    local finish_byte = math.min(start_byte + CAN_PACKET_LENGTH_MAX - 1 , #serial_msg)
    local num_bytes = finish_byte - start_byte + 1

    local canframe = CANFrame()
    canframe:id(SEND_FRAMEID)
    canframe:dlc(num_bytes)
    for j = 0, num_bytes-1 do
      canframe:data(j, serial_msg[start_byte+j])
    end
    if driver:write_frame(canframe, 10000) then
      bytes_written = bytes_written + num_bytes
    else
      write_fails = write_fails + 1
      -- on failure do not send rest of message
      do return false end
    end
  end

  return true
end

-- get next sequence number that should be used for send commands
function get_next_sequence_number()
  last_send_seq = last_send_seq + 1
  if last_send_seq > 0xFFFF then
    last_send_seq = 0
  end
  return last_send_seq
end

-- update serial message's sequence number, crc-16 and crc-32 fields
function update_msg_seq_and_crc(serial_msg)
  -- sanity checks
  if not serial_msg then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: update_msg_seq_and_crc null arg")
    do return end
  end
  if #serial_msg < SERIAL_PACKET_LENGTH_MIN then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DJIR: update_msg_seq_and_crc message too short")
    do return end
  end

  -- update sequence
  local seq = get_next_sequence_number()
  serial_msg[9] = lowbyte(seq)
  serial_msg[10] = highbyte(seq)

  -- update header crc16
  local crc16 = calc_crc16(serial_msg, 1, 10)
  serial_msg[11] = lowbyte(crc16)
  serial_msg[12] = highbyte(crc16)

  -- update entire frame's crc32
  local crc32 = calc_crc32(serial_msg, 1, #serial_msg-4)
  serial_msg[#serial_msg-3] = lowbyte(crc32)
  serial_msg[#serial_msg-2] = lowbyte(crc32 >> 8)
  serial_msg[#serial_msg-1] = lowbyte(crc32 >> 16)
  serial_msg[#serial_msg] = lowbyte(crc32 >> 24)
end

-- request attitude from gimbal
function request_attitude()
  -- Field number                  1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19
  --                             SOF  LenL  LenH CmdTyp  Enc   RES   RES   RES  SeqL  SeqH  CrcL  CrcH CmdSet CmdId Data1 CRC32 CRC32 CRC32 CRC32
  local request_attitude_msg = {0xAA, 0x13, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0E, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00}

  -- update_msg_seq_and_crc
  update_msg_seq_and_crc(request_attitude_msg)

  -- send bytes
  if send_msg(request_attitude_msg) then
    expected_reply = REPLY_TYPE.ATTITUDE
    expected_reply_ms = millis()
  else
    expected_reply = REPLY_TYPE.NONE
  end
end

-- send target angles (in degrees) to gimbal
-- yaw_angle_deg is always a body-frame angle
function send_target_angles(roll_angle_deg, pitch_angle_deg, yaw_angle_deg, time_sec)
  -- default argument values
  roll_angle_deg = roll_angle_deg or 0
  pitch_angle_deg = pitch_angle_deg or 0
  yaw_angle_deg = yaw_angle_deg or 0
  time_sec = time_sec or 2

  -- if upsidedown, add 180deg to yaw
  if DJIR_UPSIDEDOWN:get() > 0 then
    yaw_angle_deg = wrap_180(yaw_angle_deg + 180)
  end

  -- ensure angles are integers
  roll_angle_deg = math.floor(roll_angle_deg + 0.5)
  pitch_angle_deg = math.floor(pitch_angle_deg + 0.5)
  yaw_angle_deg = math.floor(yaw_angle_deg + 0.5)
  time_sec = math.floor(time_sec + 0.5)

  --    0x0E, 0x00: Handheld Gimbal Position Control
  --      Command frame bytes
  --       0~1: yaw angle * 10, int16, -1800 to +1800
  --       2~3: roll angle * 10, int16, -300 to +300
  --       4~5: pitch angle * 10, int16, -560 to +1460
  --       6: ctrl_byte, uint8
  --             bit0 = 0:relative control, 1:absolute control
  --             bit1 = 0:yaw axis valid, 1:invalid
  --             bit2 = 0:roll axis valid, 1:invalid
  --             bit3 = 0:pitch axis valid, 1:invalid
  --             bit4~7 = reserved, must be zero
  --       7: time for action, uint8_t, unit: 0.1s.  e.g. if 20, gimbal will rotate to the position desired within 2sec
  --
  -- Field number                1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26
  -- Field name                SOF  LenL  LenH CmdTyp  Enc   RES   RES   RES  SeqL  SeqH  CrcL  CrcH CmdSet CmdId YawL  YawH  RollL RollH PitL  PitH  Ctrl  Time CRC32 CRC32 CRC32 CRC32
  local set_target_att_msg = {0xAA, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x20, 0x00, 0x30, 0x00, 0x40, 0x00, 0x01, 0x14, 0x00, 0x00, 0x00, 0x00}

  -- set angles
  set_target_att_msg[15] = lowbyte(yaw_angle_deg * 10)
  set_target_att_msg[16] = highbyte(yaw_angle_deg * 10)
  set_target_att_msg[17] = lowbyte(roll_angle_deg * 10)
  set_target_att_msg[18] = highbyte(roll_angle_deg * 10)
  set_target_att_msg[19] = lowbyte(pitch_angle_deg * 10)
  set_target_att_msg[20] = highbyte(pitch_angle_deg * 10)

  -- set time
  set_target_att_msg[22] = lowbyte(time_sec * 10)

  -- update_msg_seq_and_crc
  update_msg_seq_and_crc(set_target_att_msg)

  -- send bytes
  if send_msg(set_target_att_msg) then
    expected_reply = REPLY_TYPE.POSITION_CONTROL
    expected_reply_ms = millis()
  else
    expected_reply = REPLY_TYPE.NONE
  end
end

-- send target rates (in deg/sec) to gimbal
function send_target_rates(roll_rate_degs, pitch_rate_degs, yaw_rate_degs)
  -- default argument values
  roll_rate_degs = roll_rate_degs or 0
  pitch_rate_degs = pitch_rate_degs or 0
  yaw_rate_degs = yaw_rate_degs or 0
  time_sec = time_sec or 2

  -- ensure rates are integers. invert roll direction
  roll_rate_degs = -math.floor(roll_rate_degs + 0.5)
  pitch_rate_degs = math.floor(pitch_rate_degs + 0.5)
  yaw_rate_degs = math.floor(yaw_rate_degs + 0.5)

  --    0x0E, 0x01: Handheld Gimbal Speed Control
  --      Command frame bytes
  --       0~1: yaw speed * 10, int16, -3600 to +3600
  --       2~3: roll speed * 10, int16, -3600 to +3600
  --       4~5: pitch speed * 10, int16, -3600 to +3600
  --       6: ctrl_byte, uint8, always use 0x88
  --             bit0~2 = reserved, must be zero
  --             bit3 = 0:consider focal length, 1:do not consider focal length
  --             bit4~6 = reserved, must be zero
  --             bit7 = 0:release speed control, 1:take over speed control
  --
  -- Field number                  1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25
  -- Field name                  SOF  LenL  LenH CmdTyp  Enc   RES   RES   RES  SeqL  SeqH  CrcL  CrcH CmdSet CmdId YawL  YawH  RollL RollH PitL  PitH  Ctrl CRC32 CRC32 CRC32 CRC32
  local set_target_speed_msg = {0xAA, 0x19, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x88, 0x00, 0x00, 0x00, 0x00}

  -- set rates
  set_target_speed_msg[15] = lowbyte(yaw_rate_degs * 10)
  set_target_speed_msg[16] = highbyte(yaw_rate_degs * 10)
  set_target_speed_msg[17] = lowbyte(roll_rate_degs * 10)
  set_target_speed_msg[18] = highbyte(roll_rate_degs * 10)
  set_target_speed_msg[19] = lowbyte(pitch_rate_degs * 10)
  set_target_speed_msg[20] = highbyte(pitch_rate_degs * 10)

  -- update_msg_seq_and_crc
  update_msg_seq_and_crc(set_target_speed_msg)

  -- send bytes
  if send_msg(set_target_speed_msg) then
    expected_reply = REPLY_TYPE.SPEED_CONTROL
    expected_reply_ms = millis()
  else
    expected_reply = REPLY_TYPE.NONE
  end
end

-- consume incoming CAN packets
function read_incoming_packets()
  local canframe
  repeat
    canframe = driver:read_frame()
    if canframe then
      if uint32_t(canframe:id()) == uint32_t(RECEIVE_FRAMEID) then
        for i = 0, canframe:dlc()-1 do
          parse_byte(canframe:data(i))
        end
      else
        msg_ignored = msg_ignored + 1
      end
    end
  until not canframe
end

-- parse an byte from the gimbal
function parse_byte(b)

    -- update num bytes read (for reporting only)
    bytes_read = bytes_read + 1

    -- clear buffer while waiting for header
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER then
      parse_expected_crc = 0
      parse_bytes_recv = 0
    end

    -- add byte to buffer
    parse_bytes_recv = parse_bytes_recv + 1
    parse_buff[parse_bytes_recv] = b

    -- waiting for header
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER then
      if b == HEADER then
        parse_state = PARSE_STATE_WAITING_FOR_VERLENGTH
        do return end
      else
        -- unexpected byte
        bytes_error = bytes_error + 1
      end
    end

    -- waiting for version/length LSB
    if parse_state == PARSE_STATE_WAITING_FOR_VERLENGTH then
      if parse_bytes_recv == 2 then
        parse_length = b
      else
        parse_length = uint16_value(b & 0x03, parse_length)
        if (parse_length < SERIAL_PACKET_LENGTH_MIN) or (parse_length > SERIAL_PACKET_LENGTH_MAX) then
          -- invalid length
          parse_state = PARSE_STATE_WAITING_FOR_HEADER
          bytes_error = bytes_error + 1
        else
          parse_state = PARSE_STATE_WAITING_FOR_DATA
        end
      end
      do return end
    end

    -- waiting for data
    if (parse_state == PARSE_STATE_WAITING_FOR_DATA) and (parse_bytes_recv >= parse_length) then
        -- check crc16
        local expected_crc16 = calc_crc16(parse_buff, 1, 10)
        local received_crc16 = uint16_value(parse_buff[12], parse_buff[11])
        if (expected_crc16 ~= received_crc16) then
          if DJIR_DEBUG:get() > 0 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("DJIR: crc16 exp:%x got:%x", expected_crc16, received_crc16))
          end
          bytes_error = bytes_error + 1
          parse_state = PARSE_STATE_WAITING_FOR_HEADER
          do return end
        end

        -- check crc32
        local expected_crc32 = calc_crc32(parse_buff, 1, parse_length-4)
        local received_crc32 = uint32_value(parse_buff[parse_length], parse_buff[parse_length-1], parse_buff[parse_length-2], parse_buff[parse_length-3])
        if (expected_crc32 ~= received_crc32) then
          if DJIR_DEBUG:get() > 0 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("DJIR: crc32 exp:%x got:%x", expected_crc32, received_crc32))
          end
          bytes_error = bytes_error + 1
          parse_state = PARSE_STATE_WAITING_FOR_HEADER
          do return end
        end

        -- check if reply
        local cmd_type_reply = (parse_buff[4] & 0x20) > 0

        -- process reply messages
        if cmd_type_reply then

          if expected_reply == REPLY_TYPE.NONE then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("DJIR: unexpected reply len:%d", parse_length))
          end

          -- parse attitude reply message
          if (expected_reply == REPLY_TYPE.ATTITUDE) and (parse_length >= ATTITUDE_PACKET_LEN.LEGACY) then
            -- default to legacy format but also handle latest format
            local ret_code_field = 13
            local yaw_field = 15
            local pitch_field = 17
            local roll_field = 19
            if (parse_length >= ATTITUDE_PACKET_LEN.LATEST) then
              ret_code_field = 15
              yaw_field = 17
              pitch_field = 21
              roll_field = 19
            end
            local ret_code = parse_buff[ret_code_field]
            if ret_code == RETURN_CODE.SUCCESS then
              local yaw_deg = int16_value(parse_buff[yaw_field+1],parse_buff[yaw_field]) * 0.1
              local pitch_deg = int16_value(parse_buff[pitch_field+1],parse_buff[pitch_field]) * 0.1
              local roll_deg = int16_value(parse_buff[roll_field+1],parse_buff[roll_field]) * 0.1
              -- if upsidedown, subtract 180deg from yaw to undo addition of target
              if DJIR_UPSIDEDOWN:get() > 0 then
                yaw_deg = wrap_180(yaw_deg - 180)
              end
              mount:set_attitude_euler(MOUNT_INSTANCE, roll_deg, pitch_deg, yaw_deg)
              if DJIR_DEBUG:get() > 1 then
                gcs:send_text(MAV_SEVERITY.INFO, string.format("DJIR: roll:%4.1f pitch:%4.1f yaw:%4.1f", roll_deg, pitch_deg, yaw_deg))
              end
            else
              execute_fails = execute_fails + 1
            end
          end

          -- parse position control reply message
          if (expected_reply == REPLY_TYPE.POSITION_CONTROL) and (parse_length >= POSITION_CONTROL_PACKET_LEN.LEGACY) then
            -- default to legacy format but also handle latest format
            local ret_code_field = 13
            if (parse_length >= POSITION_CONTROL_PACKET_LEN.LATEST) then
              ret_code_field = 15
            end
            local ret_code = parse_buff[ret_code_field]
            if ret_code ~= RETURN_CODE.SUCCESS then
              execute_fails = execute_fails + 1
            end
          end

          -- parse speed control reply message
          if (expected_reply == REPLY_TYPE.SPEED_CONTROL) and (parse_length >= SPEED_CONTROL_PACKET_LEN.LEGACY) then
            -- default to legacy format but also handle latest format
            local ret_code_field = 13
            if (parse_length >= SPEED_CONTROL_PACKET_LEN.LATEST) then
              ret_code_field = 15
            end
            local ret_code = parse_buff[ret_code_field]
            if ret_code ~= RETURN_CODE.SUCCESS then
              execute_fails = execute_fails + 1
            end
          end

          -- clear expected reply flag
          expected_reply = REPLY_TYPE.NONE
        else
          -- not attempting to parse
          gcs:send_text(MAV_SEVERITY.INFO, "DJIR: skipped reply:" .. tostring(cmd_type_reply) .. "len:" .. tostring(parse_length))
        end

       parse_state = PARSE_STATE_WAITING_FOR_HEADER
       do return end
    end

end

-- the main update function that performs a simplified version of RTL
function update()

  -- initialise connection to gimbal
  if not initialised then
    init()
    return update, INIT_INTERVAL_MS
  end

  -- consume incoming bytes
  read_incoming_packets()

  -- get system time
  local now_ms = millis()

  -- report parsing status
  if (DJIR_DEBUG:get() > 0) and ((now_ms - last_print_ms) > 5000) then
    last_print_ms = now_ms
    gcs:send_text(MAV_SEVERITY.INFO, string.format("DJIR: r:%u w:%u fail:%u,%u perr:%u to:%u ign:%u", bytes_read, bytes_written, write_fails, execute_fails, bytes_error, reply_timeouts, msg_ignored))
  end

  -- do not send any messages until CAN traffic has been seen
  if msg_ignored == 0 and bytes_read == 0 then
    return update, UPDATE_INTERVAL_MS
  end

  -- handle expected reply timeouts
  if (expected_reply ~= REPLY_TYPE.NONE) then
    if ((now_ms - expected_reply_ms) > REPLY_TIMEOUT_MS) then
      if DJIR_DEBUG:get() > 0 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("DJIR: timeout expecting %d", expected_reply))
      end
      expected_reply = REPLY_TYPE.NONE
      reply_timeouts = reply_timeouts + 1
    else
      -- do not process any messages
      return update, UPDATE_INTERVAL_MS
    end
  end

  -- request gimbal attitude
  if now_ms - last_req_attitude_ms > REQUEST_ATTITUDE_INTERVAL_MS then
    last_req_attitude_ms = now_ms
    request_attitude()
    return update, UPDATE_INTERVAL_MS
  end

  -- set gimbal attitude or rate
  if now_ms - last_set_attitude_ms > SET_ATTITUDE_INTERVAL_MS then
    last_set_attitude_ms = now_ms

    -- send angle target
    local roll_deg, pitch_deg, yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
    if roll_deg and pitch_deg and yaw_deg then
      if yaw_is_ef then
        -- convert to body-frame
        yaw_deg = wrap_180(yaw_deg - math.deg(ahrs:get_yaw()))
      end
      send_target_angles(roll_deg, pitch_deg, yaw_deg, 1)
      return update, UPDATE_INTERVAL_MS
    end

    -- send rate target
    local roll_degs, pitch_degs, yaw_degs, yaw_is_ef = mount:get_rate_target(MOUNT_INSTANCE)
    if roll_degs and pitch_degs and yaw_degs then
      send_target_rates(roll_degs, pitch_degs, yaw_degs)
      return update, UPDATE_INTERVAL_MS
    end
    return update, UPDATE_INTERVAL_MS
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
