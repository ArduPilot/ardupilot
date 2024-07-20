-- This script is an example setting up a custom motor matrix mix

local NUM_BANKS = 8

-- duplicate a quad 8x for 32 motors
for i = 1, NUM_BANKS do
   local base = (i-1)*4
   gcs:send_text(0, string.format("Setting up motor bank %u at %u", i, base))
   MotorsMatrix:add_motor_raw(base+0, -1,  1,  1, base+2)
   MotorsMatrix:add_motor_raw(base+1,  1, -1,  1, base+4)
   MotorsMatrix:add_motor_raw(base+2,  1,  1, -1, base+1)
   MotorsMatrix:add_motor_raw(base+3, -1, -1, -1, base+3)
end

assert(MotorsMatrix:init(NUM_BANKS*4), "Failed to init MotorsMatrix")

motors:set_frame_string(string.format("Motors%u", NUM_BANKS*4))
