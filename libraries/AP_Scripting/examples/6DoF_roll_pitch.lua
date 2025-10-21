-- A script for controlling the roll and pitch of 6DoF vehicles
-- The script sets the target roll and pitch to the current value when arming or when E-stop is removed
-- this allows the vehicle to be placed upside-down on the ground and to take off as normal
-- its is also possible to setup a switch with option 300 to use either 4 or 6DoF control

-- make sure the vehicle is capable of 6DoF control
assert(param:get('FRAME_CLASS') == 16, "script requires a 6DoF vehicle")

local e_stop = rc:find_channel_for_option(31)
local sw = rc:find_channel_for_option(300)
local motors_spinning = false

function update() -- this is the loop which periodically runs

  -- motor state
  local current_motors_spinning = false

  if arming:is_armed() then
    -- if armed then motors are spinning
    current_motors_spinning = true
  end

  if e_stop then
    -- if E-stop switch is setup
    if e_stop:get_aux_switch_pos() == 2 then
      -- E-stop on, motors stopped
      current_motors_spinning = false
    end
  end

  if not motors_spinning and current_motors_spinning then
    -- Just armed or removed E-stop
    -- set offsets to current attitude
    local roll = math.deg(ahrs:get_roll())
    local pitch = math.deg(ahrs:get_pitch())
    attitude_control:set_offset_roll_pitch(roll,pitch)
    gcs:send_text(0, string.format("Set Offsets Roll: %0.1f, Pitch: %0.1f",roll,pitch))

    if sw then
      if sw:get_aux_switch_pos() == 2 then
        -- switch to 'normal' 4 DoF attitude control
        gcs:send_text(0, "4 DoF attitude control")
        attitude_control:set_lateral_enable(false)
        attitude_control:set_forward_enable(false)

      else
        -- 6DoF attutude control
        gcs:send_text(0, "6 DoF attitude control")
        attitude_control:set_lateral_enable(true)
        attitude_control:set_forward_enable(true)
      end
    end

  end
  motors_spinning = current_motors_spinning

  return update, 100 -- 10hz
end

return update()
