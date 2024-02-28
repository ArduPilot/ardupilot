-- This script is an example setting up a custom motor matrix mix

-- this is the config for hexacopter with the motor rotations configured for improved fault tolerance
-- see: https://arxiv.org/pdf/1403.5986.pdf

-- duplicate the #defines from AP_Motors
local AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1
local AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1

local AP_MOTORS_MOT_1 = 0
local AP_MOTORS_MOT_2 = 1
local AP_MOTORS_MOT_3 = 2
local AP_MOTORS_MOT_4 = 3
local AP_MOTORS_MOT_5 = 4
local AP_MOTORS_MOT_6 = 5

-- helper function duplication of the one found in AP_MotorsMatrix
local function add_motor(motor_num, angle_degrees, yaw_factor, testing_order)

    MotorsMatrix:add_motor_raw(motor_num,math.cos(math.rad(angle_degrees + 90)),
                                         math.cos(math.rad(angle_degrees)),
                                         yaw_factor,
                                         testing_order)

end

-- this duplicates the add motor format used in AP_Motors for ease of modification of existing mixes
add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5)
add_motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6)
add_motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
add_motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1)
add_motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)

assert(MotorsMatrix:init(6), "Failed to init MotorsMatrix")

motors:set_frame_string("fault tolerant hex")
