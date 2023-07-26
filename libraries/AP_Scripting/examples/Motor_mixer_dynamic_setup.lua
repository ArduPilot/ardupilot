-- This script is an example setting up a custom dynamic motor matrix
-- allowing a vehicle to change geometry in flight

-- this mixer is for a plus quad copter, the default SITL vehicle.
-- Setup motor number (zero indexed) and testing order (1 indexed)
Motors_dynamic:add_motor(0, 2)
Motors_dynamic:add_motor(1, 4)
Motors_dynamic:add_motor(2, 1)
Motors_dynamic:add_motor(3, 3)

factors = motor_factor_table()

-- Roll for motors 1 - 4
factors:roll(0, -0.5)
factors:roll(1,  0.5)
factors:roll(2,  0)
factors:roll(3,  0)

-- pitch for motors 1 -4
factors:pitch(0,  0)
factors:pitch(1,  0)
factors:pitch(2,  0.5)
factors:pitch(3, -0.5)

-- yaw for motors 1 -4
factors:yaw(0,  0.5)
factors:yaw(1,  0.5)
factors:yaw(2, -0.5)
factors:yaw(3, -0.5)

-- throttle for motors 1 -4
factors:throttle(0,  1)
factors:throttle(1,  1)
factors:throttle(2,  1)
factors:throttle(3,  1)

-- must load factors before init
Motors_dynamic:load_factors(factors)

-- were expecting 4 motors
assert(Motors_dynamic:init(4), "Failed to init Motors_dynamic")

-- at any time we can then re-load new factors
Motors_dynamic:load_factors(factors)

motors:set_frame_string("Dynamic example")

-- if doing changes in flight it is a good idea to us pcall to protect the script from crashing
-- see 'protected_call.lua' example
