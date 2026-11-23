-- This script demonstrates configuring and running kinematic motions
-- Ensure your position targets are within the limits of your motor!
local ORCA = require("ORCAMotor")

local PORT_NUM = 0    
local INSTANCE_NUM = 1

local motor = ORCA.new(PORT_NUM, INSTANCE_NUM)

motor:clear_motor_errors()
motor:load_params()
motor:set_auto_zero_timeout_ms(10000)
motor:set_target_mode(ORCA.MODE.AUTO_ZERO)

-- Configure motions 0-31 here
-- Set the final motion's next ID to 0 and auto next to true for infinite loop
motor:set_kin_motion(0, 50000, 1100, 50, ORCA.KIN_SHAPE.MAX_SMOOTHING, true, 1)
motor:set_kin_motion(1, 120000, 1000, 73, ORCA.KIN_SHAPE.MIN_POWER, true, 2)
motor:set_kin_motion(2, 75000, 1300, 189, ORCA.KIN_SHAPE.MAX_SMOOTHING, true, 3)
motor:set_kin_motion(3, 250000, 1300, 250, ORCA.KIN_SHAPE.MIN_POWER, false, 0)
motor:set_kin_home(0)
motor:set_target_mode(ORCA.MODE.KINEMATIC)

function update()
    motor:update()

    if motor.target_mode == ORCA.MODE.KINEMATIC then
        local kin = motor:get_current_kinematic()
        if kin.status == ORCA.KIN_STATUS.DONE then
             motor:set_target_mode(ORCA.MODE.SLEEP)
        end
    end

    return update, 50
end

return update()