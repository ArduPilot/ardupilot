-- example of AP_MotorsMatrix motor setup
-- performs all functions of AP_MotorsMatrix::setup_motors except the initial removal of all motors
-- That is done by setup_motors() which exits unsuccessfully when frame_type CUSTOM is selected
-- For frame_class QUAD, this script adds the normal 4 motors for a quad X configuration plus 2 boost motors.
-- It then calls normalise_rpy_factors() and set_initialised_ok() which indicates successful completion.

function update()
  local frame_class = motors:get_frame_class()
  local frame_type = motors:get_frame_type()
  if frame_type == motors.MOTOR_FRAME_TYPE_CUSTOM then
    if frame_class == motors.MOTOR_FRAME_QUAD then
        gcs.send_text(0, 0, "frame_class: MOTOR_FRAME_QUAD, frame type: MOTOR_FRAME_TYPE_CUSTOM")

        -- remove all motors and set default SERVOn_FUNCTION parameter values 
        for i = 0, 11 do
          motorsMatrix:remove_motor(i)
        end
        -- assign first 6 default motor functions (4 for quad frame plus 2 boost motors)
        for i = 0, 5 do
          param:set(string.format('SERVO%i_FUNCTION',i+5), i+33)
        end


        -- This is the existing QuadX code
        --motorsMatrix:add_motor_raw(0, math.cos(math.rad(45+90)), math.cos(math.rad(45)), 1, 1)
        --motorsMatrix:add_motor_raw(1, math.cos(math.rad(-135+90)), math.cos(math.rad(-135)), 1, 3)
        --motorsMatrix:add_motor_raw(2, math.cos(math.rad(-45+90)), math.cos(math.rad(-45)), -1, 4)
        --motorsMatrix:add_motor_raw(3, math.cos(math.rad(135+90)), math.cos(math.rad(135)), -1, 2)

        -- symmetric X frame: roll and pitch factors are the same for all motors, differing only in sign
        motorsMatrix:add_motor_raw(0, -1,  1,  1, 1)
        motorsMatrix:add_motor_raw(1,  1, -1,  1, 3)
        motorsMatrix:add_motor_raw(2,  1,  1, -1, 4)
        motorsMatrix:add_motor_raw(3, -1, -1, -1, 2)

        -- dual counter-rotating boost motors: located either on CG or displaced symmetrically
        motorsMatrix:add_motor_raw(4, 0, 0, 1, 5)
        motorsMatrix:add_motor_raw(5, 0, 0, -1, 6)
        -- set Q_TAILSIT_MOTMX to 0x30 to enable motors 5 and 6 in FW modes
        param:set_and_save('Q_TAILSIT_MOTMX', 0x30)

        -- normalize and indicate success
        motorsMatrix:normalise_rpy_factors()
        motors:set_initialised_ok();
      else
        gcs.send_text(0, 0, "CUSTOM frame type not supported for frame_class: " .. tostring(frame_class))
      end
  end
end

return update()
