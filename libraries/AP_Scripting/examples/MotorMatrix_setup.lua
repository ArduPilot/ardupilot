-- This script is an example setting up a custom motor matrix mix

-- duplicate the standard + Quad mix
MotorsMatrix:add_motor_raw(0,-1, 0, 1, 2)
MotorsMatrix:add_motor_raw(1, 1, 0, 1, 4)
MotorsMatrix:add_motor_raw(2, 0, 1,-1, 1)
MotorsMatrix:add_motor_raw(3, 0,-1,-1, 3)

assert(MotorsMatrix:init(4), "Failed to init MotorsMatrix")

motors:set_frame_string("scripting plus example")
