-- Test ArduSub script buttons

-- This will map the script buttons to the _shifted_ XBox controller buttons A, B, Z, Y:
-- param set BTN0_SFUNCTION  108
-- param set BTN1_SFUNCTION  109
-- param set BTN2_SFUNCTION  110
-- param set BTN3_SFUNCTION  111

function update()
  -- called every 5s

  -- show current status of the buttons
  local is_pressed = {}
  for i = 1, 4 do
    is_pressed[i] = sub:is_button_pressed(i)
  end

  gcs:send_text(6, string.format("is script button pressed? %s, %s, %s, %s",
      tostring(is_pressed[1]), tostring(is_pressed[2]), tostring(is_pressed[3]), tostring(is_pressed[4])))

  -- count how many times the buttons were pressed in the last five seconds
  local count = {}
  for i = 1, 4 do
    count[i] = sub:get_and_clear_button_count(i)
  end

  gcs:send_text(6, string.format("script button counts: %d, %d, %d, %d",
      count[1], count[2], count[3], count[4]))

  return update, 5000
end

return update(), 5000
