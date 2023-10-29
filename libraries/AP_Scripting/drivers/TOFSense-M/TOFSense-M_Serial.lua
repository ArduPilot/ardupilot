--[[
  Upto 3 CAN devices supported in this script although its easy to extend.
--]]

local sensor_no         = 1     -- Sensor ID. Upload a copy of this script to the flight controller with this variable changed if you would like to use multiple of these sensors as serial. Switching to CAN highly recommended in that case
local update_rate_ms    = 10    -- update rate (in ms) of the driver. 10ms was found to be appropriate
local bytes_to_parse    = 100   -- serial bytes to parse in one interation of lua script. Reduce this if script is not able to complete in time
local debug_enable      = false -- helpgul debug GCS prints

-- Global variables (DO NOT CHANGE)
local NOOPLOOP_FRAME_HEADER = 0x57
local NOOPLOOP_FRAME_HEADER_1 = 0x01
local param_num_lua_driver_backend = 36         -- parameter number for lua rangefinder
local param_num_lua_prx_backend = 15            -- parameter number for lua proximity
local lua_driver_backend                        -- store lua backend here
local sensor_driver_found = false               -- true if user has configured lua backend
local num_pixels = 16                           -- automatically updated if 64. User can select between 16/64 from NAssistant software
local total_bytes_to_expect = 112               -- total bytes in one complete packet
local linebuf = {}                              -- serial buffer
local linebuf_len = 0
local index_row_max = 8

local PARAM_TABLE_KEY = 109 + sensor_no
local PARAM_TABLE_PREFIX = string.format("TOFSENSE_S" .. sensor_no.. "_")

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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')

--[[
  // @Param: TOFSENSE_S1_PRX
  // @DisplayName: TOFSENSE-M to be used as Proximity sensor
  // @Description: Set 0 if sensor is to be used as a 1-D rangefinder (minimum of all distances will be sent, typically used for height detection). Set 1 if it should be used as a 3-D proximity device (Eg. Obstacle Avoidance)
  // @Values: 0:Set as Rangefinder, 1:Set as Proximity sensor
  // @User: Standard
--]]
SET_PRX    = bind_add_param('PRX', 1, 0)

--[[
  // @Param: TOFSENSE_S1_SP
  // @DisplayName: TOFSENSE-M serial port config
  // @Description: UART instance sensor is connected to. Set 1 if sensor is connected to the port with fist SERIALx_PROTOCOL = 28. 
  // @Range: 1 4
  // @User: Standard
--]]
SERIAL_PORT = bind_add_param('SP', 2, 1)

--[[
  // @Param: TOFSENSE_S1_BR
  // @DisplayName: TOFSENSE-M serial port baudrate
  // @Description: Serial Port baud rate. Sensor baud rate can be changed from Nassistant software
  // @User: Standard
--]]
SERIAL_BAUD = bind_add_param('BR', 3, 230400)

-- find the serial scripting serial port instance. 0 indexed.
-- SERIALx_PROTOCOL 28
local port = assert(serial:find_serial(SERIAL_PORT:get() - 1),"Could not find Scripting Serial Port")

-- begin the serial port
port:begin(SERIAL_BAUD:get())
-- port:set_flow_control(0)

function setup_sensor(sensor, param_num)
  local sensor_count = sensor:num_sensors() -- number of sensors connected
  for j = 0, sensor_count -1 do
    local device = sensor:get_backend(j)
    if ((not sensor_driver_found) and  device and (device:type() == param_num)) then
      -- this is a lua driver
      sensor_driver_found = true
      lua_driver_backend = device
    end
  end
  if not sensor_driver_found then
    -- We can't use this script if user hasn't setup a lua backend
    gcs:send_text(0, string.format("Configure Lua Sensor"))
    return
  end
end

-- get yaw and pitch of the pixel based message index.
function convert_to_angle(index)
  -- The distances are sent in either a 4x4 or 8x8 grid. The horizontal and vertical FOV are 45 degrees so we can work out the angles
  local angle_division = 45/index_row_max
  local horizontal_index = (index) % index_row_max
  local vertical_index = math.floor(index / index_row_max)
  local yaw = -22.5 + (horizontal_index*angle_division)
  local pitch = -22.5 + (vertical_index*angle_division)
  return yaw, pitch
end

-- send the message down to proximity library. This needs to be a 3D vector. User of this function needs to ensure prx backend exists
function sent_prx_message(dist, yaw_deg, pitch_deg, push_to_boundary)
  if dist > 0 then
    lua_driver_backend:set_distance_min_max(0,4)
    lua_driver_backend:handle_script_distance_msg(dist, yaw_deg, pitch_deg, push_to_boundary)
  end
end

-- send the message down to proximity library. This needs to be a single distance. User of this function needs to ensure rngfnd backend exists
function send_rfnd_message(dist)
  if dist > 0 then
    local sent_successfully = lua_driver_backend:handle_script_msg(dist)
    if not sent_successfully then
      -- This should never happen as we already checked for a valid configured lua backend above
      gcs:send_text(0, string.format("RFND Lua Script Error"))
    end
end
end

function update() -- this is the loop which periodically runs

  if not sensor_driver_found then
    if SET_PRX:get() == 0 then
      setup_sensor(rangefinder, param_num_lua_driver_backend)
    else
      setup_sensor(proximity, param_num_lua_prx_backend)
    end
  end

  if (not sensor_driver_found) then
    -- We can't use this script if user hasn't setup a lua backend
    return
  end

  local nbytes = port:available()
  nbytes = math.min(nbytes, bytes_to_parse)
  while nbytes > 0 do
    nbytes = nbytes - 1
    local r = port:read()
    if r < 0 then
      break
    end
    local c = r

    -- if buffer is empty and this byte is 0x57, add to buffer
    if linebuf_len == 0 then
        if c == NOOPLOOP_FRAME_HEADER then
            -- lua table indexing starts from 1
            linebuf[linebuf_len + 1] = c
            linebuf_len = linebuf_len + 1
        else
          linebuf_len = 0
        end
    elseif linebuf_len == 1 then
        -- if buffer has 1 element and this byte is 0x00, add it to buffer
        -- if not clear the buffer
        if c == NOOPLOOP_FRAME_HEADER_1 then
            linebuf[linebuf_len + 1] = c
            linebuf_len = linebuf_len + 1
        else
            linebuf_len = 0
        end

    elseif linebuf_len == 8 then
        -- store the next character as "number of pixels"
        num_pixels = tonumber(c)
        linebuf[linebuf_len + 1] = c
        linebuf_len = linebuf_len + 1
        -- Check if num_pixels is either 64 or 16
        if num_pixels ~= 64 and num_pixels ~= 16 then
            linebuf_len = 0
        end
        --update total bytes to expect
        total_bytes_to_expect = 16 + (num_pixels * 6)
        if num_pixels == 16 then
          index_row_max = 4
        end
    else
      -- add character to buffer
      linebuf[linebuf_len + 1] = c
      linebuf_len = linebuf_len + 1
      if linebuf_len == total_bytes_to_expect then
          -- calculate checksum
          local checksum = 0
          for i = 1, total_bytes_to_expect - 1 do
              checksum = (checksum + linebuf[i]) % 256
          end
          -- if checksum matches extract contents
          if checksum ~= linebuf[total_bytes_to_expect] then
            if debug_enable then
              gcs:send_text(0, "Checksum does not matches")
            end
          else
            local min_distance = 0
            local min_index = 0
            local index = -1
            for i = 10, total_bytes_to_expect - 7, 6 do
                local distance = ((linebuf[i]<<8 | linebuf[i+1]<<16 | linebuf[i+2]<<24)/256) / 1000000
                local status = linebuf[i+3]
                index = index + 1
                if status < 255 then
                  if (SET_PRX:get() == 1) then
                    local yaw, pitch =  convert_to_angle(index)
                    if debug_enable then
                      gcs:send_text(0, "Distance debug: " .. distance .. " mm, status: " .. status .. "Yaw " .. yaw .. "pitch " .. pitch)
                    end
                    sent_prx_message(distance, yaw, pitch, false)
                  end
                  if status == 0 and (min_distance == 0 or distance < min_distance) then
                      min_distance = distance
                      min_index = index
                  end
                end
              end

              if (SET_PRX:get() == 1) then
                -- update prx boundary now that we have parsed all data
                lua_driver_backend:update_virtual_boundary()
              else if min_distance > 0 then
                  if debug_enable then
                    local yaw,pitch =  convert_to_angle(min_index)
                    gcs:send_text(0, "Distance: " .. min_distance .. " m. Yaw " .. yaw .. "pitch " .. pitch)
                  end
                  send_rfnd_message(min_distance)
                end
              end
            end
          -- clear buffer
          linebuf_len = 0
      end
    end
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
