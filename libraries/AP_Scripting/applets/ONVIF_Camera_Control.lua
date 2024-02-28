
local counter = 0
local running = 0
local last_pan_cmd = 0.0
local last_tilt_cmd = 0.0

function update()
  if onvif and onvif.start and counter >= 100 and running == 0 then
    gcs:send_text(0, "onvif starting")
    if onvif:start("user","123456","http://10.211.55.3:10000") then
        gcs:send_text(0, "onvif started")
        running = 1
    end
    counter = 0
  end
  counter = counter + 1

  if running == 1 then
    pan_norm, tilt_norm = vehicle:get_pan_tilt_norm()
    pan_tilt_limit_max = onvif:get_pan_tilt_limit_max()
    pan_tilt_limit_min = onvif:get_pan_tilt_limit_min()
    pan_norm = ((pan_norm + 1) * (pan_tilt_limit_max:x() - pan_tilt_limit_min:x())/2.0) + pan_tilt_limit_min:x()
    tilt_norm = ((tilt_norm + 1) * (pan_tilt_limit_max:y() - pan_tilt_limit_min:y())/2.0) + pan_tilt_limit_min:y()
    gcs:send_text(0, string.format("PAN: %f TILT: %f", pan_norm, tilt_norm))
    if math.floor(pan_norm*100.0) ~= math.floor(last_pan_cmd*100.0) or
        math.floor(tilt_norm*100.0) ~= math.floor(last_tilt_cmd*100.0) then
      -- actually send the command
      if not onvif:set_absolutemove(pan_norm, tilt_norm, 0.0) then
        gcs:send_text(0, "onvif failed to send absolutemove command")
      end
      last_pan_cmd = pan_norm;
      last_tilt_cmd = tilt_norm;
    end
  end
  return update, 20
end
gcs:send_text(0, "Starting ONVIF Control")

return update() -- run immediately before starting to reschedule