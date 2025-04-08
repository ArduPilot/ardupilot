-- this shows how to protect against faults in your scripts
-- you can wrap your update() call (or any other call) in a pcall()
-- which catches errors, allowing you to take an appropriate action


-- example main loop function
function update()
   local t = 0.001 * millis():tofloat()
   gcs:send_text(0, string.format("TICK %.1fs", t))
   if math.floor(t) % 10 == 0 then
      -- deliberately make a bad call to cause a fault, asking for the 6th GPS status
      -- as this is done inside a pcall() the error will be caught instead of stopping the script
      local status = gps:status(5)
      gcs:send_text(0, "GPS status: " .. tostring(status))
   end
end

-- wrapper around update(). This calls update() at 5Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 200
end

-- start running update loop
return protected_wrapper()
