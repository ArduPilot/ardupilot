--[[
  This example adds a new passthrough packet type for waypoints.
  Waypoint data is packet into 32bits and sent down the frsky bus with a DIY appid of 0x5009.

  - 10 bits for current waypoint index, max 1023
  - 12 bits for the distance in meters encoded with AP_Frsky_SPort::prep_number(distance, 3, 2) max is 102.3Km
  - 6 bits for xtrack error (5 bits + sign) encoded with prep_5bits
  - 3 bits for bearing from COG with a 45° resolution

  We'll be responding to an unused sensor ID

  This is a list of IDs we can't use:
  - serial protocol 4 uses IDs 0,2,3 and 6
  - serial protocol 10 uses ID 7,13,20,27
  - serial protocol 23, no IDs used

  For this test we'll use sensor ID 17 (0x71),
  Note: 17 is the index, 0x71 is the actual ID
--]]

local loop_time = 1000 -- number of ms between runs

local WP_OFFSET_DISTANCE 	= 10
local WP_OFFSET_BEARING 	= 29
local WP_OFFSET_XTRACK 		= 22

local WP_LIMIT_COUNT    = 0x3FF   -- 1023
local WP_LIMIT_XTRACK   = 0x7F    -- 127m
local WP_LIMIT_DISTANCE = 0x18F9C -- 102.3Km
local WP_ARROW_COUNT    = 8       -- 8 possible directions

local wp_bearing = 0
local wp_index = 0
local wp_distance = 0
local wp_xtrack = 0

function wrap_360(angle)
    local res = angle % 360
    if res < 0 then
      res = res + 360
    end
    return res
end

function prep_5bits(num)
    local res
    local abs_num = math.floor(math.abs(num) + 0.5)
    if abs_num < 10 then
      res = abs_num << 1
    elseif abs_num < 150 then
      res = ( math.floor((abs_num * 0.1)+0.5) << 1) | 0x1
    else
      res = 0x1F
    end
    if num < 0 then
      res = res | 0x1 << 5
    end
    return res
end

function wp_pack(index, distance, bearing, xtrack)
    local wp_dword
    wp_dword = math.min(index,WP_LIMIT_COUNT)
    wp_dword = wp_dword | frsky_sport:prep_number(math.min(math.floor(distance+0.5),WP_LIMIT_DISTANCE),3,2) << WP_OFFSET_DISTANCE
    wp_dword = wp_dword | prep_5bits(math.min(xtrack,WP_LIMIT_XTRACK)) << WP_OFFSET_XTRACK
    if gps:status(0)  >= gps.GPS_OK_FIX_2D then
      local cog = gps:ground_course(0)      -- deg
      local angle = wrap_360(bearing - cog) -- deg
      local interval = 360 / WP_ARROW_COUNT -- 45 deg
      -- hint from OSD code to avoid unreliable bearing at small distances
      if distance < 2 then
        angle = 0
      end
      -- bearing expressed as offset from cog as multiple of 45° ( 8 sectors) encoded as 3bits
      wp_dword = wp_dword | ((math.floor(((angle + interval/2) / interval)) % WP_ARROW_COUNT) & 0x7) << WP_OFFSET_BEARING
    end

    return wp_dword & 0xFFFFFFFF
end

function update_wp_info()
  local index = mission:get_current_nav_index()
  local distance = vehicle:get_wp_distance_m()
  local bearing = vehicle:get_wp_bearing_deg()
  local xtrack = vehicle:get_wp_crosstrack_error_m()

  if index ~= nil and distance ~= nil and bearing ~= nil and xtrack ~= nil then
    wp_index = index
    wp_bearing = bearing
    wp_distance = distance
    wp_xtrack = xtrack
    return true
  end
  return false
end

function update()

    if not update_wp_info() then
      return update, loop_time
    end

    local sensor_id = 0x71
    local wp_dword = wp_pack(wp_index, wp_distance, wp_bearing, wp_xtrack)
    frsky_sport:sport_telemetry_push(sensor_id, 0x10, 0x5009, wp_dword)

    return update, loop_time
end

return update() , 1000
