-- EdgeTX Parameter Configurator Client
-- Lua 5.2

--=========================================================
-- EdgeTX config
--=========================================================
local rx_timeout  = 40 --receive timeout before retransmit, unit: 10 ms
local tx_interval = 20 --minimum time between transmits, unit: 10 ms
local tx_attempts = 5 --number of transmit attempts

local m = {}
m.load = {
-- value: 1:text 2:tag 3:min 4:max 5:format+unit 6:step 7:factor
-- combo: 1:text 2:tag 3:comboText 4:comboValue
-- tag is the parameter name, append ":bitno" to set/clear specific bits (0 based)
  {"Angle Max",          "ANGLE_MAX",       0, 8000, "%.0f deg", 100, 0.01},
  {"Waypoint Hor Speed", "WPNAV_SPEED",    20, 2000, "%.0f m/s", 100, 0.01},
  {"Max Speed Up",       "PILOT_SPEED_UP", 50,  500, "%.0f m/s",  50, 0.01},
  {"Max Speed Down",     "PILOT_SPEED_DN", 50,  500, "%.0f m/s",  50, 0.01},
  {"CRSF Telem",         "RC_OPTIONS:8",   {"Disabled", "Enabled"}, {0, 1} },
}


--=========================================================
-- Protocol config (shared EdgeTx & Ardupilot)
--=========================================================
local CRSF_DEST_FC = 0xC8 -- flight controller destination
local CRSF_DEST_RADIO = 0xEA -- radio destination
local CRSF_TYPE = 0x80 -- type

local prot = {}
prot.tokenRadio = "GCS-E"
prot.tokenFC = "GCS-A"
prot.def = {
-- [cmd_byte] = {tag, {arguments}, pack_format}
  [0x00] = {"hello", {"token"}, ">z"},
  [0x01] = {"helloR", {"token"}, ">z"}, --helloR = helloReply
  [0x02] = {"paGet", {"tag"}, ">z"},
  [0x03] = {"paGetR", {"val","tag"}, ">fz"},
  [0x04] = {"paSet", {"val","tag"}, ">fz"},
  [0x05] = {"paSetR", {"val","tag"}, ">fz"},
}


--=========================================================
-- EdgeTX command encode/decode
--=========================================================
--Note: some code from François Perrad's lua-MessagePack
--This function packs a lua number into a 4 byte big-endian single precision IEEE 754 floating point representation
function prot.pack_float(n)
    if n == 0.0 then return 0x00000000 end

    local sign = 0
    if n < 0.0 then
        sign = 0x80
        n = -n
    end

    local mant, expo = math.frexp(n)
    local dword = 0x00000000

    if mant ~= mant then
        dword = 0xFF880000
    elseif mant == math.huge or expo > 0x80 then
        if sign == 0 then
            dword = 0x7F800000
        else
            dword = 0xFF800000
        end
    elseif (mant == 0.0 and expo == 0) or expo < -0x7E then
        dword = bit32.replace(dword,sign,24,8)
    else
        expo = expo + 0x7E
        mant = (mant * 2.0 - 1.0) * math.ldexp(0.5, 24)
        -- match STM32 endianess
        dword = bit32.replace(dword,sign + math.floor(expo / 0x2),0,8)
        dword = bit32.replace(dword,(expo % 0x2) * 0x80 + math.floor(mant / 0x10000),8,8)
        dword = bit32.replace(dword,math.floor(mant / 0x100) % 0x100,16,8)
        dword = bit32.replace(dword,mant % 0x100,24,8)
    end

    return dword
end

--Note: some code from François Perrad's lua-MessagePack
--This function unpacks a 4 byte big-endian single precision IEEE 754 floating point representation into a lua double
function prot.unpack_float(dword)
    if dword == 0 then return 0.0 end
     -- match STM32 endianess
    local b1 = bit32.extract(dword,0,8)
    local b2 = bit32.extract(dword,8,8)
    local b3 = bit32.extract(dword,16,8)
    local b4 = bit32.extract(dword,24,8)

    local sign = b1 > 0x7F
    local expo = (b1 % 0x80) * 0x2 + math.floor(b2 / 0x80)
    local mant = ((b2 % 0x80) * 0x100 + b3) * 0x100 + b4

    if sign then
        sign = -1
    else
        sign = 1
    end

    local n

    if mant == 0 and expo == 0 then
        n = sign * 0.0
    elseif expo == 0xFF then
        if mant == 0 then
            n = sign * math.huge
        else
            n = 0.0/0.0
        end
    else
        n = sign * math.ldexp(1.0 + mant / 0x800000, expo - 0x7F)
    end

    return n
end

function prot.push_string(pkt, s)
  for i = 1, #s do
    pkt[#pkt+1] = string.byte(s,i)
  end
  pkt[#pkt+1] = 0
  return pkt
end

function prot.pop_string(pkt, pos)
  local s = ""
  while pos <= #pkt do
    if pkt[pos] == 0 then break end
    s = s .. string.char(pkt[pos])
    pos = pos + 1
  end
  return s, pos + 1
end

function prot.push_float(pkt, flt)
  local dword = prot.pack_float(flt)
  pkt[#pkt+1] = bit32.extract(dword,0,8)
  pkt[#pkt+1] = bit32.extract(dword,8,8)
  pkt[#pkt+1] = bit32.extract(dword,16,8)
  pkt[#pkt+1] = bit32.extract(dword,24,8)
  return pkt
end

function prot.pop_float(pkt, pos)
  if pos+4 > #pkt then
    return nil, pos+4
  else
    local dword = 0x00000000
    dword = bit32.replace(dword,pkt[pos],0,8)
    dword = bit32.replace(dword,pkt[pos+1],8,8)
    dword = bit32.replace(dword,pkt[pos+2],16,8)
    dword = bit32.replace(dword,pkt[pos+3],24,8)
    local f = prot.unpack_float(dword)
    return f, pos + 4
  end
end

--encode command + arg table into crsf byte table, nil on fail
function prot.encode(cmd, arg)
  local cmd_code
  for k,v in pairs(prot.def) do
    if v[1] == cmd then
      cmd_code = k
      break
    end
  end
  if cmd_code then
    local def = prot.def[cmd_code]
    local fmt = def[3]
    local parg = {}
    for k,v in pairs(def[2]) do
      parg[k] = arg[v]
    end
    local pkt = {prot.dest, cmd_code}
    local parg_i = 1
    for i = 1, #fmt do
      local c = string.sub(fmt,i,i)
      if c == "z" then
        pkt = prot.push_string(pkt, parg[parg_i])
        parg_i = parg_i + 1
      elseif c == "f" then
        pkt = prot.push_float(pkt, parg[parg_i])
        parg_i = parg_i + 1
      end
    end
    if string.sub(fmt,-1) == "z" and pkt[#pkt] == 0 then
      pkt[#pkt] = nil --remove trailing 0
    end
    return pkt
  end
end

function prot.decode(pkt)
  local cmd = pkt[2]
  local def = prot.def[cmd]
  local fmt = def[3]
  local u = {} --u = unpack
  local ui = 1
  local pkt_pos = 3
  for i = 1, #fmt do
    local c = string.sub(fmt,i,i)
    if c == "z" then
      u[ui], pkt_pos = prot.pop_string(pkt, pkt_pos)
      ui = ui + 1
    elseif c == "f" then
      u[ui], pkt_pos = prot.pop_float(pkt, pkt_pos)
      ui = ui + 1
    end
    if pkt_pos > #pkt then break end
  end
  if #u == #def[2] then
    local d = {} --d = decoded
    for k,v in pairs(def[2]) do
      d[v] = u[k]
    end
    d.cmd = def[1]
    d.dest = pkt[1]
    return d
  end
end


--=========================================================
-- Menu Class
--=========================================================
--[[ Menu Class Example:
local m = {
  load = {
    -- value: 1:text 2:tag 3:min 4:max 5:format+unit 6:step 7:factor
    {"Angle Max",          "ANGLE_MAX",       0, 8000, "%.0f deg", 100, 0.01},
    -- combo: 1:text 2:tag 3:comboText 4:comboValue
    {"CRSF Telem",         "RC_OPTIONS:8",   {"disable", "enable"}, {0, 1} },
  },
  title = "my menu",
  x = 0,
  y = 0,
  w = LCD_W/2,
  h = LCD_H,
}

--menu value change handler, return the value to use in menu
local function onValueChange(tag,val,oldval)
  --do something with the value
  return val
end

--menu value get handler
local function onValueGet(tag,oldval)
  --get the value from somewhere and return it
  return 1.0
end

--attach handlers
m.onValueChange = onValueChange
m.onValueGet = onValueGet

--create new Menu instance
m = Menu:new(m)

--call event handler in update function
local function run(event)
  m:update(event)
end

return {run=run, init=init}
--]]

local Menu = {
  --public
  load,
  item = {},
  title,
  status,
  x = 0,
  y = 0,
  w = LCD_W,
  h = LCD_H,
  hline,
  textXpad = 0,
  textYpad = 0,
  onValueGet,
  onValueChange,
  --private
  selIdx = 1,
  edit = false,
  editVal,
  offset = 0,
}

function Menu:new(obj)
  obj = obj or {}   -- create object if user does not provide one
  setmetatable(obj, self)
  self.__index = self
  obj:init()
  return obj
end

function Menu:init()
  --set padding and line distance for color/BW displays
  if LCD_W >= 320 then
    self.textXpad = 3
    self.textYpad = 3
    self.hline = 22
  else
    self.textXpad = 0
    self.textYpad = 0
    self.hline = 8
  end

  --load items and set values
  if self.load then
    for k,v in pairs(self.load) do
      self:addItem(v)
      self:setValueByIdx(#self.item, self.onValueGet(self.item[#self.item].tag, self.item[#self.item].val) )
    end
  end
  self.load = nil

  --calc last line index
  self.maxLineIndex = math.min(#self.item, math.floor(self.h/self.hline) - 2)
end

function Menu:addItem(a)
  if type(a[3]) == "table" then
    self.item[#self.item + 1] = {
      val = nil,
      text = a[1],
      tag = a[2],
      combo = a[3],
      comboVal = a[4] or a[3],
    }
  else
    self.item[#self.item + 1] = {
      val = nil,
      text = a[1],
      tag = a[2],
      min = a[3] or 0,
      max = a[4] or 100,
      format = a[5] or "%.0f",
      step = a[6] or 1,
      factor = a[7] or 1,
    }
  end
end

function Menu:draw()
  lcd.clear()
  --items
  for i = 1+self.offset, math.min(#self.item,self.maxLineIndex+self.offset) do
    self:drawItem(i)
  end
  --title
  lcd.drawFilledRectangle(self.x, self.y, self.w+1, self.hline-1, SOLID)
  if self.title then
    lcd.drawText(self.x+self.textXpad, self.y+self.textYpad, self.title, SMLSIZE+INVERS)
  end
  lcd.drawText(self.x+self.w-self.textXpad, self.y+self.textYpad, string.format("%d/%d",self.selIdx,#self.item), SMLSIZE+INVERS+RIGHT)
  --status
  lcd.drawFilledRectangle(self.x, self.y+self.h-self.hline-1, self.w+1, self.hline+1, SOLID)
  if self.status then
    lcd.drawText(self.x+self.textXpad, self.y+self.h+self.textYpad-self.hline, self.status, SMLSIZE+INVERS)
  end
end

function Menu:drawItem(i)
  local itm = self.item[i]
  local flags = 0
  local val = itm.val
  if i == self.selIdx then
    if self.edit then
      val = self.editVal
      flags = INVERS+BLINK
    else
      flags = INVERS
    end
  end
  local y = self.y + (i - self.offset) * self.hline + self.textYpad

  lcd.drawText(self.x+self.textXpad, y, itm.text, SMLSIZE)

  local s
  if val == nil then
    s = "---"
  elseif itm.combo then
    s = itm.combo[val]
  else
    s = string.format(itm.format, val * itm.factor)
  end
  s = s or "---"
  lcd.drawText(self.x+self.w-self.textXpad, y, s, SMLSIZE+flags+RIGHT)
end

function Menu:itemInc(i)
  local itm = self.item[i]
  if itm.combo then
    self.editVal = self.editVal + 1
    if self.editVal > #itm.combo then
      self.editVal = 1
    end
  else
    self.editVal = self.editVal + itm.step
    if self.editVal > itm.max then
      self.editVal = itm.max
    end
  end
end

function Menu:itemDec(i)
  local itm = self.item[i]
  if itm.combo then
    self.editVal = self.editVal - 1
    if self.editVal < 1 then
      self.editVal = #itm.combo
    end
  else
    self.editVal = self.editVal - itm.step
    if self.editVal < itm.min then
      self.editVal = itm.min
    end
  end
end

function Menu:update(event)
  local i = self.selIdx
  local itm = self.item[i]
  --cancel edit
  if event == EVT_EXIT_BREAK then
    self.edit = false
  --edit item
  elseif event == EVT_ENTER_BREAK or event == 34 then
    self.edit = not self.edit
    if self.edit then
      local val = itm.val
      if val then
        self.editVal = val
      else
        --attempt to get value vor this item
        local oldval = self:getValueByIdx(i)
        self:setValueByIdx(i, self.onValueGet(itm.tag, oldval) )
        if not self.item[i].val then
          self.edit = false --do not edit nil
        end
      end
    else
      local newval = self:getValueByIdx(i, self.editVal)
      local oldval = self:getValueByIdx(i)
      self:setValueByIdx(i, self.onValueChange(itm.tag, newval, oldval) )
    end
  --inc/dec item
  elseif self.edit and (event == EVT_PLUS_BREAK or event == EVT_ROT_LEFT or event == EVT_PLUS_REPT or event == 36) then
    self:itemDec(i)
  elseif self.edit and (event == EVT_MINUS_BREAK or event == EVT_ROT_RIGHT or event == EVT_MINUS_REPT or event == 35) then
    self:itemInc(i)
  --select next/prev item
  elseif not self.edit and (event == EVT_PLUS_BREAK or event == EVT_ROT_LEFT or event == 36) then
    self.selIdx = self.selIdx - 1
    if self.offset >= self.selIdx then
      self.offset = self.offset - 1
    end
  elseif not self.edit and (event == EVT_MINUS_BREAK or event == EVT_ROT_RIGHT or event == 35) then
    self.selIdx = self.selIdx + 1
    if self.selIdx - self.maxLineIndex > self.offset then
      self.offset = self.offset + 1
    end
  end
  --wrap
  if self.selIdx > #self.item then
    self.selIdx = 1
    self.offset = 0
  elseif self.selIdx < 1 then
    self.selIdx = #self.item
    self.offset = #self.item - self.maxLineIndex
  end
  --draw
  self:draw()
end

function Menu:findItem(tag)
  for i=1,#self.item do
    if self.item[i].tag == tag then
      return i
    end
  end
end

function Menu:setValueByTag(tag, value)
  self:setValueByIdx(self:findItem(tag), value)
end

function Menu:setValueByIdx(i, value)
  if i then
    if self.item[i].combo then
      self.item[i].val = nil
      for i2=1,#self.item[i].comboVal do
        if self.item[i].comboVal[i2] == value then
          self.item[i].val = i2
          break
        end
      end
    else
      self.item[i].val = value
    end
  end
end

function Menu:getValueByIdx(i, editval)
  if i then
    local v = editval or self.item[i].val
    if v and self.item[i].combo then
      v = self.item[i].comboVal[v]
    end
    return v
  end
end


--=========================================================
-- MAIN
--=========================================================
local crsfQ = {} --crsf queue
local tx_ts = getTime() -- last tx timestamp
local connected = false -- connected to FC?
local send_helloR = false -- received correct "hello", need to send "helloR"

--attach menu value change handlers
local function onValueChange(tag,val,oldval)
  crsfQ[tag] = {
    tx_pkt = prot.encode("paSet", {tag=tag,val=val}),
    oldval = oldval,
  }
  return nil --set value to nil to prevent user changes until crsfQ[tag] is processed/timed out
end
m.onValueChange = onValueChange

--attach menu value get handler
local function onValueGet(tag,oldval)
  crsfQ[tag] = {
    tx_pkt = prot.encode("paGet", {tag=tag}),
    oldval = oldval,
  }
  return oldval
end
m.onValueGet = onValueGet

local function init()
  --set csrf destination
  prot.dest = CRSF_DEST_FC

  --create new Menu instance
  m = Menu:new(m)
end

local tx_cnt = 0
local rx_cnt = 0

local function transmit(pkt)
  if getTime() - tx_ts > tx_interval then
    if crossfireTelemetryPush(CRSF_TYPE, pkt) then
      tx_cnt = tx_cnt + 1
      tx_ts = getTime()
      return true
    end
  end
  return false
end

local function run(event)
  --handel connect handshake
  if not connected then
    --try to connect
    m.status = "Not connected"
    transmit( prot.encode("hello", {token=prot.tokenRadio}) )
  elseif send_helloR then
    --reply to received hello
    send_helloR = not transmit( prot.encode("helloR", {token=prot.tokenRadio}) )
  end

  --send one item from crsfQ
  if connected then
    for k,v in pairs(crsfQ) do
      if not v.tx_ts then
        if transmit(v.tx_pkt) then
          crsfQ[k].tx_ts = tx_ts
          break
        end
      end
    end
  end

  --receive crsf data
  local cmd, data
  repeat
    cmd, data = crossfireTelemetryPop()
    if cmd then
      rx_cnt = rx_cnt + 1
    end
    if cmd == CRSF_TYPE and data and data[1] == CRSF_DEST_RADIO then
      local d = prot.decode(data)
      if d.cmd == "helloR" and d.token == prot.tokenFC then
        connected = true
        m.status = "Connected"
      elseif d.cmd == "hello" and d.token == prot.tokenFC then
        connected = true
        send_helloR = true
      elseif d.cmd == "paGetR" or d.cmd == "paSetR" then
        m:setValueByTag(d.tag, d.val) --set value in menu
        crsfQ[d.tag] = nil --remove queue item
      end
    end
  until not data

  --handle expired queue items
  for tag,v in pairs(crsfQ) do
    if v.tx_ts and getTime() - v.tx_ts > rx_timeout then
      if v.tx_attempt == nil then
        v.tx_attempt = 1
      end
      if v.tx_attempt < tx_attempts then
        crsfQ[tag].tx_attempt = v.tx_attempt + 1
        crsfQ[tag].tx_ts = nil
      else
        if v.oldval then
          m:setValueByTag(tag, nil) --remove value from menu (it is not clear if FC param was set or not)
        end
        m.status = "Timeout "..tag
        crsfQ[tag] = nil --remove queue item
      end
    end
  end

  --draw menu
  m.title = "GCS".." tx:"..tostring(tx_cnt).." rx:"..tostring(rx_cnt).." mem="..tostring(getAvailableMemory())
  m:update(event)

  return 0
end

return {run=run, init=init}
