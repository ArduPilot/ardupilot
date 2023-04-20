-- Ardupilot CRSF Data Rate Analyzer
-- Place script on flight controller folder /APM/scripts and reboot
-- Lua 5.3

local name = "CRSF-Rate-A.lua"

local report_sec = 3.0 -- GCS reporting interval in seconds

---[[
--enable/disable CRSF Telemetry, call without enable parameter to get current value
local function csrfTelemEnable(enable)
  local opt = param:get("RC_OPTIONS")
  local mask = (1<<8)
  local was_enabled = (math.floor(opt) & mask) ~= 0
  if enable == false and was_enabled then
    param:set("RC_OPTIONS",opt - mask)
  end
  if enable == true and not was_enabled then
    param:set("RC_OPTIONS",opt + mask)
  end
  return was_enabled
end

local was_enabled = csrfTelemEnable(false)
if was_enabled then
  gcs:send_text(0,"### "..name.." disabled RC_OPTIONS CRSF Telem")
else
  gcs:send_text(0,"### "..name)
end
--]]

--gcs:send_text(0,"### "..name)
--crsf.telemetry_enabled(false)

local hz = 3 --requested transmit packet frequency
local len = 6 --requested packet length (crsf size = len+4)
local minlen = 5
--transmit/receive stats
local rx_cnt = 0
local tx_cnt = 0
local loop_cnt = 0
local actual_tx_hz = 0
local actual_rx_hz = 0
local actual_loop_hz = 0
local actual_radio_tx_hz = 0
local actual_radio_rx_hz = 0

local filler = 0
local ts = millis():toint()

local function update()
  --statistics once per second
  loop_cnt = loop_cnt + 1
  local now = millis():toint() -- unit is 1ms
  local dt = (now - ts) / 1000.0 -- in seconds
  if dt<0 then
    --handle millis 32bit overflow
    rx_cnt = 0
    tx_cnt = 0
    loop_cnt = 0
    ts = now
    dt = 0
  end
  if dt >= report_sec then
    actual_rx_hz = rx_cnt / dt
    rx_cnt = 0
    actual_tx_hz = tx_cnt / dt
    tx_cnt = 0
    actual_loop_hz = loop_cnt / dt
    loop_cnt = 0
    gcs:send_text(0,
      "### l:"..tostring(len+4)..
      " rt:"..tostring(math.floor(actual_radio_tx_hz+0.5)).. --radio tx Hz
      " fr:"..tostring(math.floor(actual_rx_hz+0.5))..       --fc rx Hz
      " ft:"..tostring(math.floor(actual_tx_hz+0.5))..       --fc tx Hz
      " rr:"..tostring(math.floor(actual_radio_rx_hz+0.5)).. --radio rx Hz
      " lp:"..tostring(math.floor(actual_loop_hz+0.5))
    )
    ts = millis():toint()
    dt = 0
  end

  --receive requested hz and len
  local cmd, data
  repeat
    cmd, data = crsf.pop()
    if cmd == 0x80 and data and #data >= minlen and data:byte(1) == 0xC8 then
      if data:byte(2) > 0 then
        hz = data:byte(2)
        actual_radio_tx_hz = data:byte(3)
        actual_radio_rx_hz = data:byte(4)
      end
      len = #data
      rx_cnt = rx_cnt + 1
    end
  until not data

  --transmit actual hz
  local tx_cnt_requested = hz * dt
  while tx_cnt < tx_cnt_requested do
    filler = filler + 1
    if filler > 0xff then filler = 0 end
    local p = string.char(0xEA)
      ..string.char(math.floor(hz))
      ..string.char(math.floor(actual_rx_hz+0.5))
      ..string.char(math.floor(actual_tx_hz+0.5))
      ..string.char(math.floor(actual_loop_hz+0.5))
    for i = #p+1, len do
      p = p .. string.char(filler)
    end
    if crsf.push(0x80, p) then
      tx_cnt = tx_cnt + 1
    end
  end

  return update, 5
end

return update()
