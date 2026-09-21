-- Replicate the standard Quad X frame via 6DoF scripting motor matrix
-- Raw factors derived from angle-based Quad X definition:
--   cos(angle+90) for roll, cos(angle) for pitch
-- Motor #  Roll     Pitch    Yaw  Throttle Forward Lateral Reversible TestOrder

Motors_6DoF:add_motor(0, -0.7071,  0.7071,  1.0, 1.0, 0, 0, false, 1)
Motors_6DoF:add_motor(1,  0.7071, -0.7071,  1.0, 1.0, 0, 0, false, 3)
Motors_6DoF:add_motor(2,  0.7071,  0.7071, -1.0, 1.0, 0, 0, false, 4)
Motors_6DoF:add_motor(3, -0.7071, -0.7071, -1.0, 1.0, 0, 0, false, 2)

assert(Motors_6DoF:init(4), 'unable to setup 4 motors')

-- no dedicated forward/lateral thrusters, use tilt like a normal copter
attitude_control:set_forward_enable(false)
attitude_control:set_lateral_enable(false)

motors:set_frame_string("6DoF Copter quad scripting")
gcs:send_text(6, "6DoF Copter quad scripting")
