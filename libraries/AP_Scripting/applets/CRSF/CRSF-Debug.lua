-- EdgeTX script to show received CSRF telemetry data
-- Copy to /SCRIPTS/TOOLS on the receiver
-- Start from SYS menu, press ENTER to pause/unpause\
-- Lua 5.2

--print table or string as hex bytes
local function hex(data, max_n)
  local s = ""
  if data and type(data)=="string" then
    local n = #data
    if max_n and n > max_n then n = max_n end
    for i = 1,n do
        s = s .. string.format("%02X ", string.byte(data,i))
    end
  elseif data and type(data)=="table" then
    local n = #data
    if max_n and n > max_n then n = max_n end
    for i = 1,n do
        s = s .. string.format("%02X ", data[i])
    end
  end
  return s
end

local y
local hline
local pause
local initdone

local function init()

end

local function run(event)
    --first run (lcd.clear() in init does not work)
  if not initdone then
    y = 0
    hline = 8 --default line spacing for BW displays
    pause = false
    initdone = true
    lcd.clear()
    if lcd.sizeText then --BW does not have lcd.sizeText
      _, hline = lcd.sizeText("XXX", SMLSIZE)
    end
    lcd.drawText(0, y, "Waiting for data...", SMLSIZE)
    y = y + hline
    lcd.drawText(0, y, "ENTER toggles pause/run", SMLSIZE)
    y = y + hline
  end
  --handle events
  if event == EVT_ENTER_BREAK or event == 34 then
    pause = not pause
    if pause then
      lcd.drawText(0, y, "PAUSED     ", SMLSIZE)
    else
      lcd.drawText(0, y, "RUNNING     ", SMLSIZE)
    end
  end

  --draw
  if not pause then
    local cmd, data
    repeat
      cmd, data = crossfireTelemetryPop()
      if data then
        if(y==0) then lcd.clear() end
        lcd.drawText(0, y, string.format("[%d] %02X %s", #data, cmd, hex(data)), SMLSIZE)
        y = y + hline
        if y >= LCD_H then y = 0 end
      end
    until not data
  end

  return 0
end

return {run=run, init=init}
