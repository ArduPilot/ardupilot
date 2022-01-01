-- This script loads the 6DoF mixer matrix for a 6 motor frame
-- https://youtu.be/QUDhnYvH66k

Motors_6DoF:add_motor(0,  0.003285,  0.470445,  0.031489,  0.885866,  0.563927,  0.031026, true, 1)
Motors_6DoF:add_motor(1, -0.373686, -0.267196,  0.092861,  0.789066, -0.223271,  0.495176, true, 2)
Motors_6DoF:add_motor(2,  0.370400, -0.203249, -0.053720,  0.841080, -0.340656, -0.526202, true, 3)
Motors_6DoF:add_motor(3, -0.146980, -0.216342, -0.338296,  0.001577,  0.900822, -0.460986, true, 4)
Motors_6DoF:add_motor(4, -0.205533, -0.035715,  0.359267, -0.048371,  0.010753, -1.013924, true, 5)
Motors_6DoF:add_motor(5,  0.143881, -0.227449,  0.375220, -0.046098,  0.811593,  0.431718, true, 6)

assert(Motors_6DoF:init(6),'unable to setup 6 motors')

motors:set_frame_string("6DoF example")

