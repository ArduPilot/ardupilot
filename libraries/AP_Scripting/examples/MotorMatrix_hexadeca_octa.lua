-- This script is an example setting up a custom motor matrix mix

-- duplicate the #defines from AP_Motors
local AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1
local AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1

local AP_MOTORS_MOT_1 = 0
local AP_MOTORS_MOT_2 = 1
local AP_MOTORS_MOT_3 = 2
local AP_MOTORS_MOT_4 = 3
local AP_MOTORS_MOT_5 = 4
local AP_MOTORS_MOT_6 = 5
local AP_MOTORS_MOT_7 = 6
local AP_MOTORS_MOT_8 = 7
local AP_MOTORS_MOT_9 = 8
local AP_MOTORS_MOT_10 = 9
local AP_MOTORS_MOT_11 = 10
local AP_MOTORS_MOT_12 = 11
local AP_MOTORS_MOT_13 = 12
local AP_MOTORS_MOT_14 = 13
local AP_MOTORS_MOT_15 = 14
local AP_MOTORS_MOT_16 = 15

-- helper function duplication of the one found in AP_MotorsMatrix
local function add_motor(motor_num, angle_degrees, yaw_factor, testing_order)

    MotorsMatrix:add_motor_raw(motor_num,math.cos(math.rad(angle_degrees + 90)),
                                         math.cos(math.rad(angle_degrees)),
                                         yaw_factor,
                                         testing_order)

end

-- this duplicates the add motor format used in AP_Motors for ease of modification of existing mixes
add_motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1)
add_motor(AP_MOTORS_MOT_2,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2)
add_motor(AP_MOTORS_MOT_3,    45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3)
add_motor(AP_MOTORS_MOT_4,    45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4)
add_motor(AP_MOTORS_MOT_5,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5)
add_motor(AP_MOTORS_MOT_6,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6)
add_motor(AP_MOTORS_MOT_7,   135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7)
add_motor(AP_MOTORS_MOT_8,   135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8)
add_motor(AP_MOTORS_MOT_9,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   9)
add_motor(AP_MOTORS_MOT_10,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 10)
add_motor(AP_MOTORS_MOT_11, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 11)
add_motor(AP_MOTORS_MOT_12, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  12)
add_motor(AP_MOTORS_MOT_13,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  13)
add_motor(AP_MOTORS_MOT_14,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 14)
add_motor(AP_MOTORS_MOT_15,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 15)
add_motor(AP_MOTORS_MOT_16,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  16)

assert(MotorsMatrix:init(16), "Failed to init MotorsMatrix")

motors:set_frame_string("Hexadeca-Octa PLUS")
