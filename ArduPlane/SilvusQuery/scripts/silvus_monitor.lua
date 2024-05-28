--[[
   monitor silvus radio TOF data and give to AHRS for range fusion
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 46
PARAM_TABLE_PREFIX = "SLV_"

local PORT_HEATBEAT = 8888
local MAX_GROUND_RADIOS = 8

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 63), 'could not add param table')

--[[
  // @Param: SLV_ENABLE
  // @DisplayName: enable Silvus monitor
  // @Description: Enable Silvus monitor
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLV_ENABLE = bind_add_param('ENABLE',  1, 1)
if SLV_ENABLE:get() == 0 then
   return
end

local SLV_IP = { bind_add_param('IP0', 2, 192),
                 bind_add_param('IP1', 3, 168),
                 bind_add_param('IP2', 4, 0),
                 bind_add_param('IP3', 5, 2) }

--[[
  // @Param: SLV_RATE
  // @DisplayName: request rate
  // @Description: request rate
  // @Units: Hz
  // @User: Standard
--]]
local SLV_RATE = bind_add_param('RATE', 6, 1)

--[[
  // @Param: SLV_DIST_OFS
  // @DisplayName: Silvus distance offset
  // @Description: Silvus distance offset. A value of 0 for tof_request is assumed to be less than or equal to this distance
  // @Units: m
  // @User: Standard
--]]
local SLV_DIST_OFS = bind_add_param('DIST_OFS', 7, 0)

--[[
  // @Param: SLV_DIST_ACC
  // @DisplayName: Silvus distance accuracy
  // @Description: Silvus distance accuracy
  // @Units: m
  // @User: Standard
--]]
local SLV_DIST_ACC = bind_add_param('DIST_ACC', 8, 30)

--[[
  // @Param: SLV_DIST_MUL
  // @DisplayName: Silvus distance multiplier
  // @Description: Silvus distance multiplier
  // @Units: m
  // @User: Standard
--]]
local SLV_DIST_MUL = bind_add_param('DIST_MUL', 9, 30)

--[[
  // @Param: SLV_HTTP_PORT
  // @DisplayName: Silvus HTTP port
  // @Description: Silvus HTTP port
  // @Units: m
  // @User: Standard
--]]
local SLV_HTTP_PORT = bind_add_param('HTTP_PORT', 10, 80)

--[[
  // @Param: SLV_MAX_AGE_MS
  // @DisplayName: Silvus max age
  // @Description: Silvus max age. If a TOF range is older than this then it is discarded
  // @Units: ms
  // @User: Standard
--]]
local SLV_MAX_AGE_MS = bind_add_param('MAX_AGE_MS', 11, 5000)

--[[
  // @Param: SLV_OPTIONS
  // @DisplayName: Silvus options
  // @Description: Silvus options
  // @Bitmask: 0:EnableDualRangePosition
  // @User: Standard
--]]
local SLV_OPTIONS = bind_add_param('OPTIONS', 12, 0)

local OPTION_ENABLE_DUAL_RANGE = (1<<0)

--[[
  // @Param: SLV_NUM_RADIOS
  // @DisplayName: Silvus number of ground radios
  // @Description: Silvus number of ground radios
  // @Range: 1 8
  // @User: Standard
--]]
local SLV_NUM_RADIOS = bind_add_param('NUM_RADIOS', 13, 0)

--[[
  // @Param: SLV_GND1_NODEID
  // @DisplayName: Silvus node ID for first ground radio
  // @Description: Silvus node ID for first ground radio
  // @User: Standard
--]]

--[[
  // @Param: SLV_GND1_LAT
  // @DisplayName: Silvus ground radio 1 latitude
  // @Description: Silvus ground radio 1 latitude
  // @Units: deg
  // @User: Standard
--]]

--[[
  // @Param: SLV_GND1_LON
  // @DisplayName: Silvus ground radio 1 longitude
  // @Description: Silvus ground radio 1 longitude
  // @Units: deg
  // @User: Standard
--]]

--[[
  // @Param: SLV_GND1_ALT
  // @DisplayName: Silvus ground radio 1 height AMSL
  // @Description: Silvus ground radio 1 height AMSL
  // @Units: m
  // @User: Standard
--]]

--[[
  // @Param: SLV_GND1_IP3
  // @DisplayName: Silvus ground radio 1 IP3
  // @Description: Silvus ground radio 1 last octet of IP address
  // @User: Standard
--]]

local SLV_GND_NODEID = {}
local SLV_GND_LAT = {}
local SLV_GND_LON = {}
local SLV_GND_ALT = {}
local SLV_GND_IP3 = {}

-- clamp number of radios
if SLV_NUM_RADIOS:get() > MAX_GROUND_RADIOS then
   SLV_NUM_RADIOS:set(MAX_GROUND_RADIOS)
end

--[[
   create the parameters per ground radio (beacon)
--]]
for r = 1, SLV_NUM_RADIOS:get() do
   SLV_GND_NODEID[r] = bind_add_param(string.format('GND%u_NODEID',r), 20+(r-1)*5, 0)
   SLV_GND_LAT[r] = bind_add_param(string.format('GND%u_LAT',r),       21+(r-1)*5, 0)
   SLV_GND_LON[r] = bind_add_param(string.format('GND%u_LON',r),       22+(r-1)*5, 0)
   SLV_GND_ALT[r] = bind_add_param(string.format('GND%u_ALT',r),       23+(r-1)*5, 0)
   SLV_GND_IP3[r] = bind_add_param(string.format('GND%u_IP3',r),       24+(r-1)*5, 0)
end


local radio_ranges = {nil, nil}
local radio_tstamp_ms = {nil, nil}

gcs:send_text(MAV_SEVERITY.INFO, string.format("Silvus: starting with %u ground radios", SLV_NUM_RADIOS:get()))

--[[
   get IP address of air radio
--]]
local function silvus_ip()
   return string.format("%u.%u.%u.%u", SLV_IP[1]:get(), SLV_IP[2]:get(), SLV_IP[3]:get(), SLV_IP[4]:get())
end

--[[
   get IP address of a ground radio
--]]
local function ground_radio_ip(radio_index)
   return string.format("%u.%u.%u.%u", SLV_IP[1]:get(), SLV_IP[2]:get(), SLV_IP[3]:get(), SLV_GND_IP3[radio_index]:get())
end

local function save_to_file(fname, data)
   local fh = io.open(fname,'wb')
   fh:write(data)
   fh:close()
end

local sock = nil
local http_reply = nil
local reply_start = nil
local REQUEST_TIMEOUT = 250
local last_request_ms = nil
local last_heartbeat_ms = nil
local json = require("json")
local json_log = nil

--[[
   make a silvus API request
--]]
local function http_request(api)
   if sock then
      sock:close()
      sock = nil
   end
   sock = Socket(0)
   local node_ip = silvus_ip()
   if not sock:connect(node_ip, SLV_HTTP_PORT:get()) then
      --gcs:send_text(MAV_SEVERITY.ERROR, string.format("Silvus: failed to connect", name))
      return nil
   end
   local json = string.format([[{"jsonrpc":"2.0","method":"%s","id":"sbkb5u0c"}]], api)
   local cmd = string.format([[POST /streamscape_api HTTP/1.1
Host: %s
User-Agent: lua
Connection: close
Content-Length: %u

]], node_ip, #json)
   cmd = string.gsub(cmd,"\n","\r\n")
   local full_cmd = cmd .. json
   --save_to_file("json_req.txt", full_cmd)
   sock:set_blocking(false)
   sock:send(cmd, #cmd)
   sock:send(json, #json)
   http_reply = ''
   reply_start = millis()
end

--[[
   get ground radio location
--]]
local function get_radio_location(radio_idx)
   local loc = Location()
   if not loc then
      return nil
   end
   if radio_idx < 1 or radio_idx > #SLV_GND_LAT then
      return nil
   end
   local lat = SLV_GND_LAT[radio_idx]:get()
   local lon = SLV_GND_LON[radio_idx]:get()
   local alt = SLV_GND_ALT[radio_idx]:get()
   if lat == 0 or lon == 0 then
      return nil
   end
   loc:lat(math.floor(lat*1.0e7))
   loc:lng(math.floor(lon*1.0e7))
   loc:alt(math.floor(alt*100))
   return loc
end

--[[
   work out the radio index given node ID
--]]
local function get_radio_index(node_id)
   for i = 1, #SLV_GND_NODEID do
      if SLV_GND_NODEID[i]:get() == node_id then
         return i
      end
   end
   return nil
end

--[[
   handle a distance measurement
--]]
local function handle_TOF(node_id, distance_ticks, age_ms)
   radio_idx = get_radio_index(node_id)
   if not radio_idx then
      -- not for us
      return
   end
   local distance_m = 0
   if distance_ticks > 0 then
      distance_m = SLV_DIST_OFS:get() + distance_ticks * SLV_DIST_MUL:get()
   end
   local now_ms = millis()

   radio_ranges[radio_idx] = distance_m
   radio_tstamp_ms[radio_idx] = now_ms - age_ms

   gcs:send_named_float("SlvRange", distance_m)
   if distance_m > 0 or SLV_DIST_OFS:get() == 0 then
      local accuracy = SLV_DIST_ACC:get()
      if accuracy < SLV_DIST_MUL:get() then
         accuracy = SLV_DIST_MUL:get()
      end
      local radio_loc = get_radio_location(radio_idx)
      if not radio_loc then
         return
      end
      ahrs:writeRangeToLocation(distance_m, accuracy, radio_loc, now_ms - age_ms, radio_idx-1)
   end
end

--[[
   return true if a number is NaN
--]]
local function isNaN(v)
   return v ~= v
end

--[[
   get 2 possible vehicle positions given beacon1 location, two
   ranges, the baseline and the elev angles

   loc1: location of beacon1
   R1: range from beacon1 to vehicle (3D)
   R2: range from beacon2 to vehicle (3D)
   D: range from beacon1 to beacon2 (3D)
   elev_angle1: elevation angle in radians of vehicle from beacon1
   elev_angle2: elevation angle in radians of beacon2 from beacon1
   baseline_angle_rad: angle of line between beacon1 and beacon2
--]]
local function get_vehicle_position(loc1, R1, R2, D, elev_angle1, elev_angle2, baseline_angle_rad)
   local loc1_angle = math.acos((R1^2 + D^2 - R2^2) / (2 * R1 * D))
   if isNaN(loc1_angle) then
      return nil, nil
   end
   local vehicle_bearing1 = math.deg(baseline_angle_rad+loc1_angle)
   local vehicle_bearing2 = math.deg(baseline_angle_rad-loc1_angle)

   --[[
      need to correct bearing and distance for the elevation angles
   --]]
   local R1_corrected = R1 * math.cos(math.abs(elev_angle1))
   local vehicle_loc1 = loc1:copy()
   vehicle_loc1:offset_bearing(vehicle_bearing1, R1_corrected)
   local vehicle_loc2 = loc1:copy()
   vehicle_loc2:offset_bearing(vehicle_bearing2, R1_corrected)
   return vehicle_loc1, vehicle_loc2
end

--[[
   handle positioning using dual range position
--]]
local function handle_dual_range_position()
   local now_ms = millis()
   if not radio_ranges[1] or not radio_ranges[2] then
      return
   end
   if not radio_tstamp_ms[1] or not radio_tstamp_ms[2] then
      return
   end
   local age1_ms = now_ms - radio_tstamp_ms[1]
   local age2_ms = now_ms - radio_tstamp_ms[2]
   if age1_ms > 500 or age2_ms > 500 then
      -- too old
      return
   end
   local loc1 = get_radio_location(1)
   local loc2 = get_radio_location(2)
   if not loc1 or not loc2 then
      return
   end
   local baseline_2D = loc1:get_distance_NE(loc2)
   local baseline_3D = loc1:get_distance_NED(loc2)
   local baseline_length = baseline_3D:length()
   local baseline_angle_rad = baseline_2D:angle()
   if baseline_length < 50 then
      -- baseline too small
      return
   end
   local R1 = radio_ranges[1]
   local R2 = radio_ranges[2]
   local D = baseline_length
   local accuracy = SLV_DIST_ACC:get()

   -- gcs:send_text(0,string.format("baseline: %.1f %.2f R1:%.1f R2:%.1f D:%.1f", baseline_length, math.deg(baseline_angle_rad), R1, R2, D))

   local vehicle_loc = nil
   if R1 < 1 then
      vehicle_loc = loc1:copy()
   elseif R2 < 1 then
      vehicle_loc = loc2:copy()
   else
      local ahrs_pos = ahrs:get_location()
      if not ahrs_pos then
         return
      end
      local alt_diff1 = (ahrs_pos:alt() - loc1:alt())*0.01
      local alt_diff2 = (loc2:alt() - loc1:alt())*0.01
      local elev_angle1 = math.asin(alt_diff1/R1)
      local elev_angle2 = math.asin(alt_diff2/D)
      local vehicle_loc1, vehicle_loc2 = get_vehicle_position(loc1, R1, R2, D, elev_angle1, elev_angle2, baseline_angle_rad)
      if not vehicle_loc1 or not vehicle_loc2 then
         return
      end
      logger:write("SPO1","Lat,Lng,Alt", "LLf", 'DU-', 'GG-',
                vehicle_loc1:lat(),
                vehicle_loc1:lng(),
                vehicle_loc1:alt()*0.01)
      logger:write("SPO2","Lat,Lng,Alt", "LLf", 'DU-', 'GG-',
                vehicle_loc2:lat(),
                vehicle_loc2:lng(),
                vehicle_loc2:alt()*0.01)

      local ahrs_pos = ahrs:get_location()
      if not ahrs_pos then
         vehicle_loc = vehicle_loc1
      else
         --[[
            disambiguate using current ahrs location

            NOTE! this can get the wrong answer as we cross over the beacon line
         --]]
         if ahrs_pos:get_distance(vehicle_loc1) <= ahrs_pos:get_distance(vehicle_loc2) then
            vehicle_loc = vehicle_loc1
         else
            vehicle_loc = vehicle_loc2
         end
      end

      --[[
         use perturbation to get accuracy estimate
      --]]
      local vehicle_loc1_2, vehicle_loc2_2 = get_vehicle_position(loc1, R1+accuracy, R2, D, elev_angle1, elev_angle2, baseline_angle_rad)
      local vehicle_loc1_3, vehicle_loc2_3 = get_vehicle_position(loc1, R1, R2+accuracy, D, elev_angle1, elev_angle2, baseline_angle_rad)
      if not vehicle_loc1_2 or not vehicle_loc2_2 or not vehicle_loc1_3 or not vehicle_loc2_3 then
         return
      end

      accuracy = math.max(accuracy, vehicle_loc1:get_distance(vehicle_loc1_2))
      accuracy = math.max(accuracy, vehicle_loc2:get_distance(vehicle_loc2_2))
      accuracy = math.max(accuracy, vehicle_loc1:get_distance(vehicle_loc1_3))
      accuracy = math.max(accuracy, vehicle_loc2:get_distance(vehicle_loc2_3))
   end
   logger:write("SPOS","Lat,Lng,Alt,Acc", "LLff", 'DU--', 'GG--',
                vehicle_loc:lat(),
                vehicle_loc:lng(),
                vehicle_loc:alt()*0.01,
                accuracy)
   gcs:send_named_float("SDST", ahrs:get_home():get_distance(vehicle_loc))
   gcs:send_named_float("SBRG", math.deg(ahrs:get_home():get_bearing(vehicle_loc)))
   gcs:send_named_float("SACC", accuracy)
   ahrs:handle_external_position_estimate(vehicle_loc, accuracy, millis())
end

--[[
   parse JSON reply from remote radio
--]]
local function parse_reply()
   sock:close()
   sock = nil
   lines = {}
   if not http_reply then
      return
   end
   if not json_log then
      json_log = io.open("json.log",'wb')
   end
   if json_log then
      json_log:write(http_reply)
   end
   --save_to_file("json_rep.txt", http_reply)
   for s in http_reply:gmatch("[^\r\n]+") do
      table.insert(lines, s)
   end
   local success, req = pcall(json.parse, lines[#lines])
   if not success then
      return
   end
   -- gcs:send_text(0, lines[#lines])
   local result = req['result']
   if not result then
      -- badly formatted
      return
   end
   local num_nodes = math.floor(#result / 3)
   for i = 1, num_nodes do
      local node_id = tonumber(result[1+(i-1)*3])
      local distance_ticks = tonumber(result[2+(i-1)*3])
      local age_ms = tonumber(result[3+(i-1)*3])
      --gcs:send_text(MAV_SEVERITY.INFO, string.format("node:%u dist:%u age_ms:%u", node_id, distance_ticks, age_ms))
      logger:write("STOF","Node,Dist,Age", "III", '#--', '---', node_id, distance_ticks, age_ms)
      if age_ms < SLV_MAX_AGE_MS:get() then
         handle_TOF(node_id, distance_ticks, age_ms)
         gcs:send_named_float(string.format("R%u", node_id), distance_ticks)
         gcs:send_named_float(string.format("A%u", node_id), age_ms)
      end
   end
   if num_nodes >= 2 and (SLV_OPTIONS:get() & OPTION_ENABLE_DUAL_RANGE) ~= 0 then
      handle_dual_range_position()
   end
end

--[[
   see if we have a API reply
--]]
local function check_reply()
   if not sock then
      return
   end
   local now = millis()
   if reply_start and now - reply_start > REQUEST_TIMEOUT then
      parse_reply()
      return
   end
   local r = sock:recv(1024)
   if r then
      http_reply = http_reply .. r
   else
      parse_reply()
   end
end


local heartbeat_counter = 0

--[[
   send UDP heartbeat messages to all ground radios to ensure we get up to date TOF data.
   The silvus TOF system is opprtunistic, if no data is flowing it won't update
--]]
local function send_heartbeats()
   heartbeat_counter = heartbeat_counter + 1
   for i = 1, #SLV_GND_IP3 do
      local ip3 = SLV_GND_IP3[i]:get()
      if ip3 > 0 and ip3 < 255 then
         local sock = Socket(1)
         if not sock then
            return
         end
         local ip = ground_radio_ip(i)
         if sock:connect(ip, PORT_HEATBEAT) then
            local msg = ip .. string.format(":HEARTBEAT:%u", heartbeat_counter)
            sock:send(msg, #msg)
            -- gcs:send_text(0, msg)
         end
         sock:close()
      end
   end
end

--[[
   update called at 20Hz
--]]
local function update()
   if SLV_ENABLE:get() <= 0 then
      return
   end
   if sock then
      check_reply()
      return
   end
   local now = millis()
   -- heartbeat at 10Hz
   if not last_heartbeat_ms or now - last_heartbeat_ms >= 100 then
      last_heartbeat_ms = now
      send_heartbeats()
   end
   local period_ms = 1000.0 / SLV_RATE:get()
   if not last_request_ms or now - last_request_ms >= period_ms then
      last_request_ms = now
      http_request("current_tof")
   end
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
