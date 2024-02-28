-- example of getting the current notify LED colour

function update() -- this is the loop which periodically runs

  local r, g, b = LED:get_rgb()
  gcs:send_text(0, "Notify LED: r: " .. tostring(r) .. ", g: " .. tostring(g) ..", b:" .. tostring(b))

  return update, 1000 -- reschedules the loop
end

-- make sure Scripting LED is enabled
local led_parm = param:get('NTF_LED_TYPES')
if not led_parm then
  error('Could not find NTF_LED_TYPES param')
end

if (led_parm & (1 << 10)) == 0 then
  -- try and enable it
  if param:set_and_save('NTF_LED_TYPES',led_parm | (1 << 10)) then
    error('Enabled Notify Scripting LED, please reboot')
  else
    error('Could not set NTF_LED_TYPES param')
  end
end

return update()
