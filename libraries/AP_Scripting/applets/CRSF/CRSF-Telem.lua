-- Ardupilot CRSF Telemetry Demo
-- Place script on flight controller folder /APM/scripts and reboot
-- Lua 5.3

local interval = 200 -- telemetry interval in milliseconds

--"declare" libraries
local sch = {}
local telem = {}

local function init()
  gcs:send_text(0,"### CRSF-Telem.lua")
  --set items to be scheduled, the number is how often the item is executed
  --for example: {{a,1},{b,2},{c,3}} runs every 6 intervals: 1*a, 2*b, and 3*c
  sch.items = {
    --CRSF native telemetry
    {telem.Attitude,1},
    {telem.Gps,1},
    {telem.Battery,1},
    --AP CRSF passthrough telemetry
    {telem.apAttitude,1},
    {telem.apAirspeedYaw,1},
    {telem.apVelYaw,1},
    {telem.apLat,1},
    {telem.apLng,1},
  }

  --disable Ardupilot CRSF telemetry (if enabled)
  local was_enabled = telem.AP_enable(false)
  gcs:send_text(0,"### AP CRSF Telem " .. (was_enabled and "was enabled" or "kept disabled") )

  sch.init()
end

local function update()
  local item = sch.run()
  item[1]()

  return update, 200
end

--enable/disable Ardupilot CRSF Telemetry, call without enable parameter to get current value
function telem.AP_enable(enable)
  local opt = param:get("RC_OPTIONS")
  local mask = (1<<8)
  local was_enabled = ((math.floor(opt) & mask) ~= 0)
  if enable == false and was_enabled then
    param:set("RC_OPTIONS",opt - mask)
  end
  if enable == true and not was_enabled then
    param:set("RC_OPTIONS",opt + mask)
  end
  return was_enabled
end

--=========================================================
-- native crsf telemetry messages (Big-Endian)
--=========================================================
--string.pack cheat sheet: <:le >:be Ix:uintx_t ix:intx_t f:float z:zero-term-str
function telem.Attitude()
  return crsf.push(0x1E, string.pack(">i2i2i2",
    math.floor((ahrs:get_pitch() or 0) * 10000 + 0.5), --rad / 10`000
    math.floor((ahrs:get_roll() or 0) * 10000 + 0.5), --rad / 10`000
    math.floor((ahrs:get_yaw() or 0) * 10000 + 0.5) --rad / 10`000
  ))
end

function telem.Gps()
  local s = gps:primary_sensor()
  return crsf.push(0x02, string.pack(">i4i4I2I2I2I1",
    math.floor(gps:location(s):lat() or 0), --degree / 10`000`000
    math.floor(gps:location(s):lng() or 0), --degree / 10`000`000
    math.floor((gps:ground_speed(s) or 0) * 360), --km/h / 100
    math.floor((gps:ground_course(s) or 0) * 100), --degree / 100
    math.floor((gps:location(s):alt() or 0) / 100 + 1000), --meter - 1000m offset
    math.floor(gps:num_sats(s) or 0)
  ))
end

function telem.Battery()
  return crsf.push(0x08, string.pack(">I2I2I3I1",
    math.floor((battery:voltage(1) or 0) * 10), --mV * 100
    math.floor((battery:current_amps(1) or 0) * 10), --mA * 100
    math.floor(battery:consumed_mah(1) or 0), --mAh
    math.floor(battery:capacity_remaining_pct(1) or 0) --percent
  ))
end

--=========================================================
-- ardupilot passthru telemetry messages (Little-Endian)
--=========================================================
local function bit32_replace(n,v,field,width)
  return n | ((math.floor(v+0.5) & ((1<<width)-1)) << field)
end

function telem.Fightmode(text)
    --16 byte Null-terminated string
    return crsf.push(0x08, string.pack("<z", string.sub(text,1,15)))
end

function telem.PassthroughSingle(appid, data)
    return crsf.push(0x80, string.pack("<I1I2I4", 0xF0, appid, data))
end

function telem.StatusText(sev, text)
    return crsf.push(0x80, string.pack("<I1I1z", 0xF1, sev, string.sub(text,1,49)))
end


function telem.apAttitude()
  local n = 0
  n = bit32_replace(n, (ahrs:get_roll() / math.pi * 180 + 180) * 5, 0, 11)
  n = bit32_replace(n, (ahrs:get_pitch() / math.pi * 180 + 90) * 5, 11, 10)
  return telem.PassthroughSingle(0x5006, n)
end

function telem.SpAirspeedYaw()
  local v = ahrs:get_velocity_NED()
  local vv = 0
  if v then vv = -v:z() end
  local n = 0
  n = bit32_replace(n, vv * 10, 0, 9)
  n = bit32_replace(n, 0 or ahrs:airspeed_estimate() * 10, 9, 8)
  n = bit32_replace(n, (ahrs:get_yaw() / math.pi * 180 + 180) * 5, 17, 11)
  n = bit32_replace(n, 1, 28, 1)
  return telem.PassthroughSingle(0x5005, n)
end

function telem.apVelYaw()
  local v = ahrs:get_velocity_NED()
  local vv = 0
  local vh = 0
  if v then
    vv = -v:z()
    vh = v:length()
  end
  --local vne = ahrs:get_velocity_NE() --copter error: attempt to call a nil value (method 'get_velocity_NE')
  local n = 0
  n = bit32_replace(n, vv * 10, 0, 9)
  n = bit32_replace(n, vh * 10, 9, 8)
  n = bit32_replace(n, (ahrs:get_yaw() / math.pi * 180 + 180) * 5, 17, 11)
  n = bit32_replace(n, 0, 28, 1)
  return telem.PassthroughSingle(0x5005, n)
end

function telem.apWind()
  local v = ahrs:wind_estimate()
  local vh = 0
  local dir = 0
  if v then
    vh = v:length()
    dir = math.atan(-v:y(), -v:x())
    if dir<0 then dir = dir + 2 * math.pi end
  end
  local n = 0
  n = bit32_replace(n, dir / math.pi * 180 / 3, 0, 7)
  n = bit32_replace(n, vh * 10, 7, 8)
  return crsftelem.PassthroughSingle(0x500C, n)
end

function telem.apLat()
    local s = gps:primary_sensor()
    local val = math.floor(gps:location(s):lat()/100*6 or 0)  --degree / 10`000`000
    local n = 0
    if(val<0) then n = 0x40000000 end
    n = bit32_replace(n, math.abs(val), 0, 29)
    return crsftelem.PassthroughSingle(0x0800, n)
end

function telem.apLng()
    local s = gps:primary_sensor()
    local val = math.floor(gps:location(s):lng()/100*6 or 0) --degree / 10`000`000
    local n = 0x80000000
    if(val<0) then n = 0xC0000000 end
    n = bit32_replace(n, math.abs(val), 0, 30)
    return crsftelem.PassthroughSingle(0x0800, n)
end

--[[TODO
    enum PassthroughPacketType : uint8_t {
        TEXT =          0,  // 0x5000 status text (dynamic)
        AP_STATUS =     5,  // 0x5001 AP status
        GPS_STATUS =    6,  // 0x5002 GPS status
        HOME =          7,  // 0x5004 Home
        BATT_2 =        8,  // 0x5008 Battery 2 status
        BATT_1 =        9,  // 0x5008 Battery 1 status
--]]

--=========================================================
-- scheduler
--=========================================================
sch.sum = 0

function sch.init()
  sch.sum = 0
  for i=1,#sch.items do
    sch.sum = sch.sum + sch.items[i][2]
    sch.items[i][3] = 0
  end
end

function sch.run()
  local mi = 0
  local mv = 0
  for i=1,#sch.items do
    local v = sch.items[i][2] + sch.items[i][3]
    sch.items[i][3] = v
    if mv < v then
      mv = v
      mi = i
    end
  end
  sch.items[mi][3] = sch.items[mi][3] - sch.sum
  return sch.items[mi]
end

--=========================================================
-- main
--=========================================================
init()
return update()
