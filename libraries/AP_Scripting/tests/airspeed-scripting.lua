-- test for the scripting airspeed backend, set ARSPD2_TYPE to 20 (Scripting)

local backend

function update()
  if not backend then
    -- the airspeed backends may not have been allocated yet when
    -- scripting first runs us; keep trying rather than dying
    backend = airspeed:get_backend(1)
    if not backend then
      return update, 100
    end
  end
  backend:handle_script_airspeed(30.0)
  backend:handle_script_temperature(25.0)
  return update, 100
end

return update()
