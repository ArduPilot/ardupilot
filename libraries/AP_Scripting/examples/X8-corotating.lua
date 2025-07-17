--[[
   a dynamic motor mixer for an X8 coax multicopter with co-rotating
   motors on each of the arms

   adds parameters for a multiplier for top and bottom layer motors
--]]

local PARAM_TABLE_KEY = 105
local PARAM_TABLE_PREFIX = "MOT_X8C_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    -- use the old style of param constructor to be compatible with very old ArduPilot versions (tested on 4.2.x)
    local p = Parameter()
    p:init(PARAM_TABLE_PREFIX .. name)
    return p;
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'X8-corotating: could not add param table')

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

--[[
  // @Param: X8C_TOPMUL
  // @DisplayName: X8C Top Layer Multiplier
  // @Description: Multiplier for the top layer motors
  // @Range: 0 1
  // @User: Standard
--]]
local X8C_TOPMUL = bind_add_param('TOPMUL',  1, 1.0)

--[[
  // @Param: X8C_BOTMUL
  // @DisplayName: X8C Bottom Layer Multiplier
  // @Description: Multiplier for the bottom layer motors
  // @Range: 0 1
  // @User: Standard
--]]
local X8C_BOTMUL = bind_add_param('BOTMUL',  2, 1.0)

--[[
  // @Param: X8C_ORDER
  // @DisplayName: X8C motor ordering
  // @Description: Selecting between old and clockwise X8 ordering
  // @Values: 0:OldOrdering, 1:ClockwiseOrdering
  // @User: Standard
--]]
local X8C_ORDER = bind_add_param('ORDER',  3, 0)

local CCW = 1
local CW = -1

local FRAME_NAME = "X8-corotating"

--[[
   motors array for X8 with co-rotating motors, using old X8 ordering
--]]
local Motors_old_order = {
   { name = "Motor 1", test_order = 1, Xpos =  1, Ypos =  1, rotation = CCW, layer=2 },
   { name = "Motor 2", test_order = 7, Xpos =  1, Ypos = -1, rotation =  CW, layer=2 },
   { name = "Motor 3", test_order = 5, Xpos = -1, Ypos = -1, rotation = CCW, layer=2 },
   { name = "Motor 4", test_order = 3, Xpos = -1, Ypos =  1, rotation =  CW, layer=2 },
   { name = "Motor 5", test_order = 8, Xpos =  1, Ypos = -1, rotation =  CW, layer=1 },
   { name = "Motor 6", test_order = 2, Xpos =  1, Ypos =  1, rotation = CCW, layer=1 },
   { name = "Motor 7", test_order = 4, Xpos = -1, Ypos =  1, rotation =  CW, layer=1 },
   { name = "Motor 8", test_order = 6, Xpos = -1, Ypos = -1, rotation = CCW, layer=1 }
}

--[[
   motors array for X8 with co-rotating motors, using clockwise X8 ordering
--]]
local Motors_cw_order = {
   { name = "Motor 1", test_order = 1, Xpos =  1, Ypos =  1, rotation = CCW, layer=2 },
   { name = "Motor 2", test_order = 2, Xpos =  1, Ypos =  1, rotation = CCW, layer=1 },
   { name = "Motor 3", test_order = 3, Xpos = -1, Ypos =  1, rotation =  CW, layer=2 },
   { name = "Motor 4", test_order = 4, Xpos = -1, Ypos =  1, rotation =  CW, layer=1 },
   { name = "Motor 5", test_order = 5, Xpos = -1, Ypos = -1, rotation = CCW, layer=2 },
   { name = "Motor 6", test_order = 6, Xpos = -1, Ypos = -1, rotation = CCW, layer=1 },
   { name = "Motor 7", test_order = 7, Xpos =  1, Ypos = -1, rotation =  CW, layer=2 },
   { name = "Motor 8", test_order = 8, Xpos =  1, Ypos = -1, rotation =  CW, layer=1 }
}

if X8C_ORDER:get() == 0 then
    Motors = Motors_old_order
else
    Motors = Motors_cw_order
    FRAME_NAME = "X8-corotating-cw"
end

local num_motors = #Motors

--[[
   add the motors to the Motors_dynamic object
--]]
for i = 1, num_motors do
   local test_order = Motors[i].test_order
   Motors_dynamic:add_motor(i-1, test_order)
end

-- create a motor factor table
factors = motor_factor_table()

--[[
   multiplier for a layer, to account for the parameters
--]]
local function layer_multiplier(motor_num)
   local layer = Motors[motor_num].layer
   if layer == 2 then
      return X8C_TOPMUL:get()
   elseif layer == 1 then
      return X8C_BOTMUL:get()
   else
      return 1.0
   end
end

--[[
   when we setup the roll, pitch and yaw factors we normalise them to a maximum
   of 0.5, which is what is expected by the motor mixer
--]]

--[[
   setup the roll factors
--]]
local function setup_roll_factors()
   -- find the highest absolute value of Ypos
   local max_y = 0
   for i = 1, num_motors do
       max_y = math.max(math.abs(Motors[i].Ypos), max_y)
   end
   if max_y == 0 then
       max_y = 1 -- prevent division by zero
   end
   -- set the roll factors based on the Xpos of each motor
   -- positive Ypos means roll left, negative Ypos means roll right
   for i = 1, num_motors do
      local Ypos = Motors[i].Ypos
      local roll_factor = -(Ypos / max_y) * 0.5 * layer_multiplier(i)
      factors:roll(i-1, roll_factor)
   end
end

--[[
   setup the pitch factors
--]]
local function setup_pitch_factors()
   -- find the highest absolute value of Ypos
   local max_x = 0
   for i = 1, num_motors do
       max_x = math.max(math.abs(Motors[i].Xpos), max_x)
   end
   if max_x == 0 then
      max_x = 1 -- prevent division by zero
   end
   -- set the roll factors based on the Ypos of each motor
   -- positive Xpos means pitch up, negative Xpos means pitch down
   for i = 1, num_motors do
      local Xpos = Motors[i].Xpos
      local pitch_factor = (Xpos / max_x) * 0.5 * layer_multiplier(i)
      factors:pitch(i-1, pitch_factor)
   end
end

--[[
   setup the yaw factors
--]]
local function setup_yaw_factors()
   -- set the yaw factors based on the rotation of each motor
   for i = 1, num_motors do
      local rotation = Motors[i].rotation
      factors:yaw(i-1, rotation * 0.5 * layer_multiplier(i))
   end
end

--[[
   setup the thrust factors
--]]
local function setup_thrust_factors()
   for i = 1, num_motors do
      factors:throttle(i-1, 1.0 * layer_multiplier(i))
   end
end

--[[
   setup throttle factors
--]]
local function update_factors()
   if X8C_ORDER:get() == 0 then
       Motors = Motors_old_order
   else
       Motors = Motors_cw_order
   end
   setup_roll_factors()
   setup_pitch_factors()
   setup_yaw_factors()
   setup_thrust_factors()
   Motors_dynamic:load_factors(factors)
end

update_factors()

assert(Motors_dynamic:init(num_motors), string.format("Failed to init %s", FRAME_NAME))

motors:set_frame_string(FRAME_NAME)

local last_top_layer_mul = X8C_TOPMUL:get()
local last_bot_layer_mul = X8C_BOTMUL:get()
local last_order = X8C_ORDER:get()

local function update()
   if last_top_layer_mul ~= X8C_TOPMUL:get() or
      last_bot_layer_mul ~= X8C_BOTMUL:get() or
      last_order ~= X8C_ORDER:get() then
      last_top_layer_mul = X8C_TOPMUL:get()
      last_bot_layer_mul = X8C_BOTMUL:get()
      last_order = X8C_ORDER:get()
      update_factors()
   end
   return update, 200
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded frame %s", FRAME_NAME))

return update,200
