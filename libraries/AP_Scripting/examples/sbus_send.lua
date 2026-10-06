--[[
EXPERIMENTAL EXAMPLE ONLY – NOT FOR FLIGHT CONTROL USE

This Lua script demonstrates generating an S.Bus-compatible serial
output stream from ArduPilot servo outputs using a scripting task.

WARNING:
- This implementation has NO timing guarantees and may produce corrupted
  frames under high CPU load.
- S.Bus provides only weak error detection (no CRC, parity only).
- Frame corruption or misalignment may result in severe control errors,
  including unintended full-throttle outputs.

This script MUST NOT be used for safety-critical functions such as:
- Motor or rotor control
- ESC input
- Primary flight control paths

It is provided solely as an example of serial protocol generation from Lua.
]]

local pwm_vals     = {}
local servo_min    = assert(param:get("SERVO1_MIN"), "Lua: Could not read SERVO1_MIN")
local servo_max    = assert(param:get("SERVO1_MAX"), "Lua: Could not read SERVO1_MAX")
local servo_trim   = assert(param:get("SERVO1_TRIM"), "Lua: Could not read SERVO1_TRIM")
local rc_min       = assert(param:get("RC3_MIN"), "Lua: Could not read RC3_MIN")
local rc_max       = assert(param:get("RC3_MAX"), "Lua: Could not read RC3_MAX")
local rc_trim      = assert(param:get("RC3_TRIM"), "Lua: Could not read RC3_TRIM")
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local SCR_INS      = 1

local function init()
  for ch = 1, 16 do
    pwm_vals[ch] = 1500
  end
end

-- debugging
local function notify_pwm_vals(label)
  -- Channels 1–8
  gcs:send_text(0, string.format(
    "CH1-8 %s: %d, %d, %d, %d, %d, %d, %d, %d",
    label,
    pwm_vals[1], pwm_vals[2], pwm_vals[3], pwm_vals[4],
    pwm_vals[5], pwm_vals[6], pwm_vals[7], pwm_vals[8]
  ))
  -- Channels 9–16
  gcs:send_text(0, string.format(
    "CH9-16 %s: %d, %d, %d, %d, %d, %d, %d, %d",
    label,
    pwm_vals[9], pwm_vals[10], pwm_vals[11], pwm_vals[12],
    pwm_vals[13], pwm_vals[14], pwm_vals[15], pwm_vals[16]
  ))
end

local function read_rcin()
  for ch = 1, 16 do
    pwm_vals[ch] = rc:get_pwm(ch) or 0
  end
end

local function pack_sbus()
  local frame = { 0x0F }
  for _ = 1, 22 do
    frame[#frame + 1] = 0x00
  end
  frame[#frame + 1] = 0x00
  frame[#frame + 1] = 0x00

  for ch = 1, 16 do
    local v = pwm_vals[ch]
    if v < 0 then
      v = 0
    elseif v > 2047 then
      v = 2047
    end

    local bit_index = (ch - 1) * 11
    local val       = v
    for _ = 0, 10 do
      if (val % 2) == 1 then
        local byte_idx    = math.floor(bit_index / 8) + 2
        local bit_in_byte = bit_index % 8
        frame[byte_idx]   = frame[byte_idx] + (2 ^ bit_in_byte)
      end
      val       = math.floor(val / 2)
      bit_index = bit_index + 1
    end
  end

  -- set failsafe flag
  if not rc:has_valid_input() then
    frame[24] = 8 -- FAILSAFE
  else
    frame[24] = 0 -- NORMAL
  end

  -- send
  local buf = {}
  for i = 1, #frame do
    buf[i] = string.char(frame[i])
  end
  return table.concat(buf)
end

local in_mid     = rc_trim
local in_half    = (rc_max - rc_min) / 2
local servo_mid  = servo_trim
local servo_half = (servo_max - servo_min) / 2
local SBUS_OFF   = 875 -- 1000 − (2000−1000)*200/1600
local SBUS_SCALE = 1.6 -- 1/(1000/1600)

--[[
servo_val: RCIN or PWM
reverse:  1 or -1
]]
local function convert_servo_to_sbus(servo_val, reverse)
  local norm = (servo_val - servo_mid) / servo_half * reverse
  local pwm  = in_mid + in_half * norm
  return math.floor((pwm - SBUS_OFF) * SBUS_SCALE + 0.5)
end

local function convert_rcin_to_sbus()
  -- gcs:send_text(0, string.format("### convert_rcin_to_sbus ENTER"))
  for ch = 1, 16 do
    local v = pwm_vals[ch]
    if v >= SBUS_OFF then
      pwm_vals[ch] = math.floor((v - SBUS_OFF) * SBUS_SCALE + 0.5)
    else
      pwm_vals[ch] = 0
    end
  end
end

-- debugging
local function notify_sbus_frame(frame)
  for i = 1, #frame, 10 do
    local bytes = {}
    for j = i, math.min(i + 9, #frame) do
      bytes[#bytes + 1] = string.format("%02X", string.byte(frame, j))
    end
    gcs:send_text(0, string.format(
      "SBUS %02d-%02d: %s",
      i,
      math.min(i + 9, #frame),
      table.concat(bytes, " ")
    ))
  end
end

-- find_serial (0: TELEM1 1: TELEM2)
port = serial:find_serial(SCR_INS)
if not port then
  gcs:send_text(0, "ERROR: TELEM2 port not found")
  return
else
  port:begin(100000)
  --  parity: 0=none, 1=odd, 2=even / stop bits: 1 or 2
  port:configure_parity(2)
  port:set_stop_bits(2)
  port:set_flow_control(0)
  gcs:send_text(MAV_SEVERITY.NOTICE, "TELEM2 port opened for writestring()")
end

-- main loop：70Hz
local function loop()
  read_rcin()
  -- debugging
  notify_pwm_vals("RCIN PWM")

  convert_rcin_to_sbus()
  -- debugging
  notify_pwm_vals("SBUS CNT")

  local servo1 = SRV_Channels:get_output_pwm_chan(0)
  local servo3 = SRV_Channels:get_output_pwm_chan(2)

  local reverse_ch3 = 1
  local reverse_ch2 = -1

  pwm_vals[3] = convert_servo_to_sbus(servo1, reverse_ch3)
  pwm_vals[2] = convert_servo_to_sbus(servo3, reverse_ch2)

  local sbus_frame = pack_sbus()
  -- debugging
  notify_sbus_frame(sbus_frame)

  if port then
    -- ignore returns
    local _ = port:writestring(sbus_frame)
  end

  return loop, 14
end

init()

return loop, 0
