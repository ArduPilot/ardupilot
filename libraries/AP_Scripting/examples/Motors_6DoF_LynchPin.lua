-- This script loads the 6DoF mixer matrix for the LynchPin frame

Motors_6DoF:add_motor(0,  0.012558,  0.391064,  0.069463,  0.867789,  0.507225, -0.015613, true, 1)
Motors_6DoF:add_motor(1, -0.385797, -0.187969,  0.106479,  0.782773, -0.243803,  0.479664, true, 2)
Motors_6DoF:add_motor(2,  0.373240, -0.203095, -0.105312,  0.865451, -0.263422, -0.464051, true, 3)
Motors_6DoF:add_motor(3, -0.153922, -0.166480, -0.345955,  0.003411,  0.908383, -0.456939, true, 4)
Motors_6DoF:add_motor(4, -0.210017, -0.010839,  0.379773, -0.056142, -0.014059, -1.035508, true, 5)
Motors_6DoF:add_motor(5,  0.142076, -0.202427,  0.347056, -0.030879,  0.861758,  0.471667, true, 6)

assert(Motors_6DoF:init(6),'unable to setup 6 motors')
