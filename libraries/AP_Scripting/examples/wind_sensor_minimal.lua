-- This script is an example of saying hello

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local backend = air_sensor:get_backend(0)
if not backend then
   gcs:send_text(MAV_SEVERITY.ERROR, "WIND3D: unable to find AIRSNS backend")
   return
end

local wind_vec = Vector3f()
wind_vec:x(10.0)
wind_vec:y(1.0)
wind_vec:z(0.0)

backend:handle_script_3d_msg(wind_vec)
gcs:send_text(MAV_SEVERITY.INFO, "WIND3D mocked some wind")
