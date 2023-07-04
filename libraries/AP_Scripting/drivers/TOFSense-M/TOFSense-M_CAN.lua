--[[
   Driver for NoopLoop TOFSense-M CAN Version. Can be used as a 1-D RangeFidner or 3-D proximity sensor. Upto 3 CAN devices supported in this script although its easy to extend.
--]]

local update_rate_ms    = 10  -- update rate (in ms) of the driver. 10ms was found to be appropriate

-- Global variables (DO NOT CHANGE)
local param_num_lua_driver_backend = 36         -- parameter number for lua rangefinder
local param_num_lua_prx_backend = 15            -- parameter number for lua proximity
local sensor_setup_done = false

-- Table contains the following info for 3 sensors. If more sensors are needed, this table will need to be increased
-- approportate scritping backend from rngfnd/prx library, true if backend exists, index parsed last from sensor, minimum distance found since index was 0, Param to decide which rngfnd/prx backednd will match to this sensor, param to decide CAN ID of this sensor 
local backend_driver = {
  {lua_driver_backend = nil, sensor_driver_found = false, last_index = 0, min_distance = 0, INSTANCE, CAN_ID},
  {lua_driver_backend = nil, sensor_driver_found = false, last_index = 0, min_distance = 0, INSTANCE, CAN_ID},
  {lua_driver_backend = nil, sensor_driver_found = false, last_index = 0, min_distance = 0, INSTANCE, CAN_ID}
}

local PARAM_TABLE_KEY = 104
local PARAM_TABLE_PREFIX = "TOFSENSE_"

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

-- setup parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: TOFSENSE_PRX
  // @DisplayName: TOFSENSE-M to be used as Proximity sensor
  // @Description: Set 0 if sensor is to be used as a 1-D rangefinder (minimum of all distances will be sent, typically used for height detection). Set 1 if it should be used as a 3-D proximity device (Eg. Obstacle Avoidance)
  // @Values: 0:Set as Rangefinder, 1:Set as Proximity sensor
  // @User: Standard
--]]
SET_PRX = bind_add_param('PRX', 1, 0)

--[[
  // @Param: TOFSENSE_NO
  // @DisplayName: TOFSENSE-M Connected
  // @Description: Number of TOFSENSE-M CAN sensors connected
  // @Range: 1 3
  // @User: Standard
--]]
MAX_SENSORS = bind_add_param('NO', 2, 1)

--[[
  // @Param: TOFSENSE_MODE
  // @DisplayName: TOFSENSE-M mode to be used
  // @Description: TOFSENSE-M mode to be used. 0 for 8x8 mode. 1 for 4x4 mode
  // @Values: 0: 8x8 mode, 1: 4x4 mode
  // @User: Standard
--]]
MODE = bind_add_param('MODE', 3, 0)

-- first sensor
--[[
  // @Param: TOFSENSE_INST1
  // @DisplayName: TOFSENSE-M First Instance
  // @Description: First TOFSENSE-M sensors backend Instance. Setting this to 1 will pick the first backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)
  // @Range: 1 3
  // @User: Standard
--]]
backend_driver[1].INSTANCE = bind_add_param('INST1', 4, 1)

--[[
  // @Param: TOFSENSE_ID1
  // @DisplayName: TOFSENSE-M First ID
  // @Description: First TOFSENSE-M sensor ID. Leave this at 0 to accept all IDs and if only one sensor is present. You can change ID of sensor from NAssistant Software
  // @Range: 1 255
  // @User: Standard
--]]
backend_driver[1].CAN_ID = bind_add_param('ID1', 5, 0)

-- second sensor
--[[
  // @Param: TOFSENSE_INST2
  // @DisplayName: TOFSENSE-M Second Instance
  // @Description: Second TOFSENSE-M sensors backend Instance. Setting this to 2 will pick the second backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)
  // @Range: 1 3
  // @User: Standard
--]]
backend_driver[2].INSTANCE = bind_add_param('INST2', 6, 2)

--[[
  // @Param: TOFSENSE_ID2
  // @DisplayName: TOFSENSE-M Second ID
  // @Description: Second TOFSENSE-M sensor ID. This cannot be 0. You can change ID of sensor from NAssistant Software
  // @Range: 1 255
  // @User: Standard
--]]
backend_driver[2].CAN_ID = bind_add_param('ID2', 7, 2)

--third sensor
--[[
  // @Param: TOFSENSE_INST3
  // @DisplayName: TOFSENSE-M Third Instance
  // @Description: Third TOFSENSE-M sensors backend Instance. Setting this to 3 will pick the second backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)
  // @Range: 1 3
  // @User: Standard
--]]
backend_driver[2].INSTANCE = bind_add_param('INST3', 8, 2)

--[[
  // @Param: TOFSENSE_ID3
  // @DisplayName: TOFSENSE-M Thir ID
  // @Description: Third TOFSENSE-M sensor ID. This cannot be 0. You can change ID of sensor from NAssistant Software
  // @Range: 1 255
  // @User: Standard
--]]
backend_driver[2].CAN_ID = bind_add_param('ID3', 9, 3)


-- check both CAN device for scripting backend. CAN Buffer length set to fixed 5
local driver = CAN:get_device(5)
if not driver then
  driver = CAN:get_device2(5)
end
if not driver then
  error("No scripting CAN interfaces found")
  return
end

function setup_sensor(sensor, param_num)
  local sensor_count = sensor:num_sensors() -- number of sensors connected
  if  MAX_SENSORS:get() > 3 then
    error("TOFSENSE: Only 3 devices supported")
  end

  for i = 1, MAX_SENSORS:get() do
    local backends_found = 0
    local sensor_driver_found = false
    local lua_driver_backend
    for j = 0, sensor_count -1 do
      local device = sensor:get_backend(j)
      if ((not sensor_driver_found) and  device and (device:type() == param_num)) then
        -- this is a lua driver
        backends_found = backends_found + 1
        if backends_found == backend_driver[i].INSTANCE:get() then
          -- get the correct instance as we may have multile scripting backends doing different things
          sensor_driver_found = true
          lua_driver_backend = device
        end
      end
    end
    if not sensor_driver_found then
      -- We can't use this script if user hasn't setup a lua backend
      error(string.format("TOFSENSE: Could not find SCR Backend ".. tostring(i)))
      return
    end
    backend_driver[i].sensor_driver_found = true
    backend_driver[i].lua_driver_backend = lua_driver_backend
  end

end

-- get yaw and pitch of the pixel based message index.
function convert_to_angle(index)
  -- The distances are sent in either a 4x4 or 8x8 grid. The horizontal and vertical FOV are 45 degrees so we can work out the angles
  local index_row_max = 8
  if (MODE:get() ~= 0) then
    index_row_max = 4
  end
  local angle_division = 45/index_row_max
  local horizontal_index = (index) % index_row_max
  local vertical_index = math.floor(index / index_row_max)
  local yaw = -22.5 + (horizontal_index*angle_division)
  local pitch = -22.5 + (vertical_index*angle_division)
  return yaw, pitch
end

-- send the message down to proximity library. This needs to be a 3D vector
function sent_prx_message(prx_backend, dist, yaw_deg, pitch_deg, push_to_boundary)
  if (dist > 0) then
    prx_backend:set_distance_min_max(0,4)
    prx_backend:handle_script_distance_msg(dist, yaw_deg, pitch_deg, push_to_boundary)
  end
end

-- send the message down to proximity library. This needs to be a single distance
function send_rfnd_message(rfnd_backend, dist)
    if dist > 0 and (SET_PRX:get() == 0) then
      local sent_successfully = rfnd_backend:handle_script_msg(dist)
      if not sent_successfully then
        -- This should never happen as we already checked for a valid configured lua backend above
        gcs:send_text(0, string.format("RFND Lua Script Error"))
      end
  end
end

-- get the correct instance from parameters according to the CAN ID received
function get_instance_from_CAN_ID(frame)
  for i = 1, MAX_SENSORS:get() do
    if ((uint32_t(frame:id() - 0x200)) ==  uint32_t(backend_driver[i].CAN_ID:get())) then
       return i
    end
  end
  return 0
end

-- this is the loop which periodically runs
function update()

  -- setup the sensor according to user preference of using proximity library or rangefinder
  if not sensor_setup_done then
    if SET_PRX:get() == 0 then
      setup_sensor(rangefinder, param_num_lua_driver_backend)
    else
      setup_sensor(proximity, param_num_lua_prx_backend)
    end
    sensor_setup_done = true
  end

  -- read frame if available
  local frame = driver:read_frame()
  if not frame then
    return
  end

  local instance
  if ((backend_driver[1].CAN_ID:get() ~= 0)) then
    instance = get_instance_from_CAN_ID(frame)
    if (instance == 0) then
      -- wrong ID
      return
    end
  else
    -- Simply accept any ID
    instance = 1
  end

  -- Correct ID, so parse the data
  local distance = ((frame:data(0)<<8 | frame:data(1)<<16 | frame:data(2)<<24)/256) / 1000
  local status = frame:data(3)
  local index = frame:data(6)
  local update_rfnd = false
  if (index < backend_driver[instance].last_index) then
    -- One cycle of data has come. Lets update all backends involved
    if SET_PRX:get() == 1 then
      backend_driver[instance].lua_driver_backend:update_virtual_boundary()
    else
      update_rfnd = true
    end
  end
  backend_driver[instance].last_index = index

  if status < 255 then
    -- Status is healthy
    if (SET_PRX:get() == 1) then
      -- Send 3D data to Proximity Library
      local yaw, pitch =  convert_to_angle(index)
      sent_prx_message(backend_driver[instance].lua_driver_backend, distance, yaw, pitch, false)
    end
    if (backend_driver[instance].min_distance == 0 or distance < backend_driver[instance].min_distance) then
      -- store min data incase user wants to use it as a 1-D RangeFinder
      backend_driver[instance].min_distance = distance
    end
  end

  if (update_rfnd) then
    send_rfnd_message(backend_driver[instance].lua_driver_backend, backend_driver[instance].min_distance)
    -- reset
    backend_driver[instance].min_distance = 0
  end
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
      gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
  end
  return protected_wrapper, update_rate_ms
end

-- start running update loop
return protected_wrapper()
