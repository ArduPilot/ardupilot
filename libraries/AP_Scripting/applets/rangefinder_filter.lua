-- This is an example script for a custom Lua Script based RangeFinder driver.
-- If RF_FILT_TYPE = 1
-- This script checks all RangeFinders connected in a particular direction. They must all report the distances within a set margin.
-- If not, then all rangefinders are ignored until the values match again.
-- If RF_FILT_TYPE = 0
-- Then this driver reports the minimum of the rangefinders connected in the given direction

-- User-settable parameters
local max_allowed_diff_m = 0.5   -- Maximum difference allowed in sensors looking at the same direction
local interested_rotation = 25      -- Direction to look at (25 is down)
local update_rate_ms = 25         -- Update rate (in ms) of the driver
local update_rate_error_ms = 5000 -- Update rate in case of a fatal error (in ms)

-- Global constants (DO NOT CHANGE)
local param_num_lua_backend = 36         -- Parameter number for lua rangefinder
local rangefinder_status_good_num = 4    -- Number indicating "good" status for the rangefinder
local PARAM_TABLE_KEY = 121
local PARAM_TABLE_PREFIX = "RF_FILT_"
rangefinder_lua_backend = nil

gcs:send_text(0, string.format("Lua RangeFinder Script Started"))

-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
  return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup RF_FILT_TYPE parameter
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')

--[[
  // @Param: RF_FILT_TYPE
  // @DisplayName: RangeFinder Filter Type
  // @Description: RangeFinder Filter Type. Set 0 to use minimum distances from all the other RangeFinders in the same direction. Set 1 to use average.
  // @Values: 0:Minimum of all RangeFinders, 1:Average of all RangeFinders
  // @User: Standard
--]]
RF_FILT_TYPE = bind_add_param('TYPE', 1, 0) -- 0 = Send Min of All Sensors in the direction. 1 = Send Avg of all sensors in the direction

-- Function to find the first Lua backend device (assuming you have only configured one)
function find_lua_backend()
  local sensor_count = rangefinder:num_sensors()
  local lua_backend = nil

  for j = 0, sensor_count do
    local device = rangefinder:get_backend(j)
    if (not lua_backend) and device and (device:type() == param_num_lua_backend) then
      lua_backend = device
      break
    end
  end

  return lua_backend
end

-- Function to handle sensor updates
function handle_sensor_updates()
  local sensor_count = rangefinder:num_sensors()
  local same_direction_sensor_count = 0
  local distances_sum = 0
  local min_element = 0
  local max_element = 0

  for i = 0, sensor_count do
    local device = rangefinder:get_backend(i)
    if device and device:type() ~= param_num_lua_backend then
      if device:orientation() == interested_rotation then
        local dist_m = device:distance()
        if device:status() == rangefinder_status_good_num then
          -- this device is "healthy"
          same_direction_sensor_count = same_direction_sensor_count + 1
          distances_sum = distances_sum + dist_m

          if min_element == 0 and max_element == 0 then
            -- first rangefinder detected
            min_element = dist_m
            max_element = dist_m
          end

          if dist_m < min_element then
            min_element = dist_m
          end
          if dist_m > max_element then
            max_element = dist_m
          end
        end
      end
    end
  end
  return same_direction_sensor_count, distances_sum, min_element, max_element
end

-- Function to update the Lua script
function run_filter(lua_backend, same_direction_sensor_count, distances_sum, min_element, max_element)
  if same_direction_sensor_count == 0 then
    gcs:send_text(0, "No valid Range Finder Reading")
    return update, update_rate_error_ms
  end

  if RF_FILT_TYPE:get() == 0 then
    local sent_successfully = lua_backend:handle_script_msg(min_element)
    if not sent_successfully then
      gcs:send_text(0, "RangeFinder Lua Script Error")
      return update, update_rate_error_ms
    end
  end

  if RF_FILT_TYPE:get() == 1 then
    if math.abs(max_element - min_element) > max_allowed_diff_m then
      gcs:send_text(0, "Sensor values do not match")
    else
      local sent_successfully = lua_backend:handle_script_msg(distances_sum / same_direction_sensor_count)
      if not sent_successfully then
        gcs:send_text(0, "RangeFinder Lua Script Error")
        return update, update_rate_error_ms
      end
    end
  end

  return update, update_rate_ms
end

-- Main update function
function update()
  if not rangefinder_lua_backend then
    -- find a rangefinder backend configured with "scripting"
    rangefinder_lua_backend = find_lua_backend()
  end

  -- check again
  if not rangefinder_lua_backend then
    gcs:send_text(0, "Configure Lua RangeFinder")
    return update, update_rate_error_ms
  end

  local same_direction_sensor_count, distances_sum, min_element, max_element = handle_sensor_updates()
  return run_filter(rangefinder_lua_backend, same_direction_sensor_count, distances_sum, min_element, max_element)
end

return update(), 5000 -- First data may be checked 5 seconds after start-up
