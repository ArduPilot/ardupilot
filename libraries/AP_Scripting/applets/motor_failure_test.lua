-- This is a script that stops motors in flight, for use testing motor failure handling

-- add new param MOT_STOP_BITMASK
local PARAM_TABLE_KEY = 75
assert(param:add_table(PARAM_TABLE_KEY, "MOT_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "STOP_BITMASK", 0), "could not add param")

local stop_motor_bitmask = Parameter()
assert(stop_motor_bitmask:init("MOT_STOP_BITMASK"), "could not find param")

-- find rc switch with option 300
local switch = assert(rc:find_channel_for_option(300),"Lua: Could not find switch")

-- read spin min param, we set motors to this PWM to stop them
local pwm_min
if quadplane then
  pwm_min = assert(param:get("Q_M_PWM_MIN"),"Lua: Could not read Q_M_PWM_MIN")
else
  pwm_min = assert(param:get("MOT_PWM_MIN"),"Lua: Could not read MOT_PWM_MIN")
end

local stop_motor_chan
local last_motor_bitmask

-- find any motors enabled, populate channels numbers to stop
local function update_stop_motors(new_bitmask)
  if last_motor_bitmask == new_bitmask then
    return
  end
  stop_motor_chan = {}
  for i = 1, 12 do
    if ((1 << (i-1)) & new_bitmask) ~= 0 then
      -- convert motor number to output function number
      local output_function
      if i <= 8 then
        output_function = i+32
      else
        output_function = i+81-8
      end

      -- get channel number for output function
      local temp_chan = SRV_Channels:find_channel(output_function)
      if temp_chan then
        table.insert(stop_motor_chan, temp_chan)
      end
    end
  end
  last_motor_bitmask = new_bitmask
end

function update()

  update_stop_motors(stop_motor_bitmask:get())

  if switch:get_aux_switch_pos() == 2 then
    for i = 1, #stop_motor_chan do
      -- override for 15ms, called every 10ms
      -- using timeout means if the script dies the timeout will expire and all motors will come back
      -- we cant leave the vehicle in a un-flyable state
      SRV_Channels:set_output_pwm_chan_timeout(stop_motor_chan[i],pwm_min,15)
    end
  end

  return update, 10 -- reschedule at 100hz
end

return update() -- run immediately before starting to reschedule
