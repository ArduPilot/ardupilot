-- EdgeTX CRSF Data Rate Analyzer
-- Copy to /SCRIPTS/TOOLS on the receiver
-- Start from SYS menu, press ENTER to pause/unpause
-- Lua 5.2

local hline
local initdone
local edit_item = 1
local mode = 1
local modes = {"TX/RX", "RX only"}

local hz = 3 -- requested telemetry rate
local len = 6 -- requested packet size (crsf size = len+4)
local minlen = 5
local actual_fc_len = 0 -- the fc sends packages of this len

--transmit/receive stats
local rx_cnt = 0
local tx_cnt = 0
local loop_cnt = 0
local actual_tx_hz = 0
local actual_rx_hz = 0
local actual_loop_hz = 0
local actual_fc_tx_hz = 0
local actual_fc_rx_hz = 0
local actual_fc_loop_hz = 0

local ts = getTime()
local filler = 0

local function drawItem(line, title, value, itemno)
  local attr = 0
  if itemno == edit_item then attr = INVERS end
  lcd.drawText(0, line*hline, title, SMLSIZE)
  lcd.drawText(LCD_W-1, line*hline, value, SMLSIZE + RIGHT + attr)
end

local function transmit()
  filler = filler + 1
  if filler > 0xff then filler = 0 end
  local p = {0xc8, hz, math.floor(actual_tx_hz+0.5), math.floor(actual_rx_hz+0.5)}
  for i = #p+1, len do
    p[i] = filler
  end
  local retry = 20
  local result_ok = false
  while retry > 0 and not result_ok do
    result_ok = crossfireTelemetryPush(0x80, p)
    retry = retry - 1
  end
  if result_ok then tx_cnt = tx_cnt + 1 end
  return result_ok
end

local function rotaryIncDec(diff)
  if edit_item == 1 then
    hz = hz + diff
    if hz > 255 then hz = 255 end
    if hz < 1 then hz = 1 end
  elseif edit_item == 2 then
    len = len + diff
    if len > 60 then len = 60 end
    if len < minlen then len = minlen end
  elseif edit_item == 3 then
    mode = mode + diff
    if mode > #modes then mode = 1 end
    if mode < 1 then mode = #modes end
  end
end

local function run(event)
  --first run (lcd.clear() in init does not work)
  if not initdone then
    hline = 8 --default line spacing for BW displays
    initdone = true
    if lcd.sizeText then --BW does not have lcd.sizeText
      _, hline = lcd.sizeText("XXX", SMLSIZE)
    end
  end

  --handle events
  if event == EVT_EXIT_BREAK then
    --RTN button
    do end -- do nothing
  elseif event == EVT_ENTER_BREAK or event == 34 then
    --Rotary enter button
    edit_item = edit_item + 1
    if edit_item > 3 then edit_item = 1 end
  elseif event == EVT_PLUS_BREAK or event == EVT_ROT_LEFT or event == EVT_PLUS_REPT or event == 36 then
    --Rotary decrease
    rotaryIncDec(-1)
  elseif event == EVT_MINUS_BREAK or event == EVT_ROT_RIGHT or event == EVT_MINUS_REPT or event == 35 then
    --Rotary increase
    rotaryIncDec(1)
  end

  --receive
  local cmd, data
  repeat
    cmd, data = crossfireTelemetryPop()
    if cmd==0x80 and data and #data >= minlen and data[1] == 0xEA then
      rx_cnt = rx_cnt + 1
      actual_fc_len = #data
      actual_fc_hz = data[2]
      actual_fc_rx_hz = data[3]
      actual_fc_tx_hz = data[4]
      actual_fc_loop_hz = data[5]
    end
  until not data

  --transmit
  if mode == 1 then
    --tx/rx mode -> transmit according to hz
    local dt = (getTime() - ts) / 100.0 -- in seconds
    local tx_cnt_requested = hz * dt
    while tx_cnt < tx_cnt_requested do
      transmit()
    end
  elseif mode == 2 then
    --rx only mode -> transmit once per second
    if tx_cnt == 0 then
      transmit()
    end
  end

  --statistics once per second
  local now = getTime() -- unit is 10ms
  local dt = (now - ts) / 100.0 -- in seconds
  loop_cnt = loop_cnt + 1
  if dt >= 1.0 then
    actual_rx_hz = rx_cnt / dt
    rx_cnt = 0
    actual_tx_hz = tx_cnt / dt
    tx_cnt = 0
    actual_loop_hz = loop_cnt / dt
    loop_cnt = 0
    ts = now
  end

  --gui
  lcd.clear()
  drawItem(0,"Settings:  [ENT to tab, ROLL to change]","",0)
  drawItem(1,"Requested rate:",string.format("%d Hz", hz), 1)
  drawItem(2,"CRSF Packet size:",string.format("%d bytes", len + 4), 2) --total CRSF packet size = #data+4 (sync+len+cmd+data+crc)
  drawItem(3,"Mode:",modes[mode], 3)
  --4
  drawItem(5,"Results:","",0)
  drawItem(6,string.format("FC len:%d  loop:%d  push:%d", actual_fc_len+4, actual_fc_loop_hz, actual_fc_tx_hz), string.format("FC pop: %d Hz", actual_fc_rx_hz), 0)
  drawItem(7,string.format("Radio loop:%d  push:%d", actual_loop_hz, actual_tx_hz), string.format("Radio pop: %d Hz", actual_rx_hz), 0)
  return 0
end

return {run=run}
