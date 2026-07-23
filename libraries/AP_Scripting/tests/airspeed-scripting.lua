-- test for the scripting airspeed backend, set ARSPD2_TYPE to 20 (Scripting)

local backend = airspeed:get_backend(1)
assert(backend, "ARSPD2_TYPE must be set to 20 (Scripting)")

function update()
  backend:handle_script_airspeed(30.0)
  backend:handle_script_temperature(25.0)
  return update, 100
end

return update()
