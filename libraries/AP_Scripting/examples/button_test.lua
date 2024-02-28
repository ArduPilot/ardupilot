-- This script is an example button functionality

local button_number = 1 -- the button numbber we want to read, as deffined in AP_Button

local button_active_state = true -- the 'pressed' state of the button

local last_button_state

function update() -- this is the loop which periodically runs

  local button_new_state = button:get_button_state(button_number) == button_active_state

  -- the button has changes since the last loop
  if button_new_state ~= last_button_state then
    last_button_state = button_new_state
    if button_new_state then
      gcs:send_text(0, "LUA: Button pressed")
    else
      gcs:send_text(0, "LUA: Button released")
    end
  end

  return update, 1000 -- reschedules the loop (1hz)
end

return update() -- run immediately before starting to reschedule
