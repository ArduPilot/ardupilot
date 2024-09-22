--[[
   simulate a silvus radio providing time of flight range
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 53
PARAM_TABLE_PREFIX = "SSIM_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 63), 'could not add param table')

--[[
  // @Param: SSIM_ENABLE
  // @DisplayName: enable Silvus simulator
  // @Description: Enable Silvus simulator
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SSIM_ENABLE = bind_add_param('ENABLE',  1, 0)
if SSIM_ENABLE:get() == 0 then
   return
end

--[[
  // @Param: SSIM_DIST_OFS
  // @DisplayName: Silvus distance offset
  // @Description: Silvus distance offset
  // @Units: m
  // @User: Standard
--]]
local SSIM_DIST_OFS = bind_add_param('DIST_OFS', 2, 0)

--[[
  // @Param: SSIM_DIST_MUL
  // @DisplayName: Silvus distance multiplier
  // @Description: Silvus distance multiplier
  // @Units: m
  // @User: Standard
--]]
local SSIM_DIST_MUL = bind_add_param('DIST_MUL', 3, 30)

--[[
  // @Param: SSIM_LISTEN_PORT
  // @DisplayName: Silvus listen port
  // @Description: Silvus listen port
  // @User: Standard
--]]
local SSIM_LISTEN_PORT = bind_add_param('LISTEN_PORT', 4, 8003)

--[[
  // @Param: SSIM_AGE_MAX_MS
  // @DisplayName: Silvus age max MS
  // @Description: Silvus age max MS
  // @User: Standard
--]]
local SSIM_AGE_MAX_MS = bind_add_param('AGE_MAX_MS', 5, 5000)


--[[
  // @Param: SSIM_NUM_RADIOS
  // @DisplayName: Silvus number of ground radios
  // @Description: Silvus number of ground radios
  // @Range: 1 8
  // @User: Standard
--]]
local SSIM_NUM_RADIOS = bind_add_param('NUM_RADIOS', 13, 0)

--[[
  // @Param: SSIM_GND1_NODEID
  // @DisplayName: Silvus node ID for first ground radio
  // @Description: Silvus node ID for first ground radio
  // @User: Standard
--]]

--[[
  // @Param: SSIM_GND1_LAT
  // @DisplayName: Silvus ground radio 1 latitude
  // @Description: Silvus ground radio 1 latitude
  // @Units: deg
  // @User: Standard
--]]

--[[
  // @Param: SSIM_GND1_LON
  // @DisplayName: Silvus ground radio 1 longitude
  // @Description: Silvus ground radio 1 longitude
  // @Units: deg
  // @User: Standard
--]]

--[[
  // @Param: SSIM_GND1_ALT
  // @DisplayName: Silvus ground radio 1 height AMSL
  // @Description: Silvus ground radio 1 height AMSL
  // @Units: m
  // @User: Standard
--]]

local MAX_GROUND_RADIOS = 8

local SSIM_GND_NODEID = {}
local SSIM_GND_LAT = {}
local SSIM_GND_LON = {}
local SSIM_GND_ALT = {}

-- clamp number of radios
if SSIM_NUM_RADIOS:get() > MAX_GROUND_RADIOS then
   SSIM_NUM_RADIOS:set(MAX_GROUND_RADIOS)
end

--[[
   create the parameters per ground radio (beacon)
--]]
for r = 1, SSIM_NUM_RADIOS:get() do
   SSIM_GND_NODEID[r] = bind_add_param(string.format('GND%u_NODEID',r), 20+(r-1)*5, 0)
   SSIM_GND_LAT[r] = bind_add_param(string.format('GND%u_LAT',r),       21+(r-1)*5, 0)
   SSIM_GND_LON[r] = bind_add_param(string.format('GND%u_LON',r),       22+(r-1)*5, 0)
   SSIM_GND_ALT[r] = bind_add_param(string.format('GND%u_ALT',r),       23+(r-1)*5, 0)
end

local listen_sock = nil

gcs:send_text(MAV_SEVERITY.INFO, string.format("SilvusSim: starting with %u beacons", #SSIM_GND_ALT))

--[[
   get ground radio location
--]]
local function get_radio_location(radio_idx)
   local loc = Location()
   if not loc then
      return nil
   end
   if radio_idx < 1 or radio_idx > #SSIM_GND_LAT then
      return nil
   end
   local lat = SSIM_GND_LAT[radio_idx]:get()
   local lon = SSIM_GND_LON[radio_idx]:get()
   local alt = SSIM_GND_ALT[radio_idx]:get()
   if lat == 0 or lon == 0 then
      return nil
   end
   loc:lat(math.floor(lat*1.0e7))
   loc:lng(math.floor(lon*1.0e7))
   loc:alt(math.floor(alt*100))
   return loc
end

--[[
   get range in TOF units (100ns), given offset of beacon from home
--]]
local function get_range_ticks(radio_idx)
   local gps_loc = gps:location(0)
   if not gps_loc then
      return nil
   end
   local loc = get_radio_location(radio_idx)
   if not loc then
      return nil
   end
   local range_2D_m = gps_loc:get_distance(loc)
   if range_2D_m > 300000 then
      return nil
   end
   local alt_diff = math.abs(loc:alt()*0.01 - gps_loc:alt()*0.01)
   local range_3D_m = math.sqrt(range_2D_m^2 + alt_diff^2)
   if range_3D_m < SSIM_DIST_OFS:get() then
      return 0
   end
   local range_ticks = math.floor((range_3D_m - SSIM_DIST_OFS:get()) / SSIM_DIST_MUL:get())
   return range_ticks
end

--[[
   check for new connections
--]]
local function update()
   if SSIM_ENABLE:get() <= 0 then
      return
   end
   if not listen_sock then
      listen_sock = Socket(0)
      if not listen_sock then
         return
      end
      assert(listen_sock:bind("0.0.0.0", SSIM_LISTEN_PORT:get()))
      assert(listen_sock:listen(2))
      listen_sock:reuseaddress()
   end
   local sock = listen_sock:accept()
   if not sock then
      return
   end
   local range_str = ""
   for i = 1, #SSIM_GND_NODEID do
      local range = get_range_ticks(i)
      if range then
         if #range_str > 0 then
            range_str = range_str .. ","
         end
         range_str = range_str .. string.format([["%u","%u","10"]], SSIM_GND_NODEID[i]:get(), range)
      end
   end
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], range_str)
   local msg = string.format([[Content-Type: application/json
Cache-Control: no-cache
Content-Length: %u
Server: silvus sim

%s]], #json, json)
   msg = string.gsub(msg, "\n", "\r\n")
   sock:send(msg, #msg)
   sock:close()
end

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end

return protected_wrapper,100
