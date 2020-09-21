-- demo of waving paw of opendog
--
local flipflop = true

pwm = { 1500, 1500, 2000,
        1500, 1500, 1000,
        1500, 1500, 1500,
        1500, 1500, 1500 }

local angle = 0.0

function update()
   local t = 0.001 * millis():tofloat()
   local angle = math.sin(t) * 0.5
   pwm[6] = math.floor(1500.0 + angle*500.0)
   for i = 1, 12 do
       SRV_Channels:set_output_pwm_chan_timeout(i-1, pwm[i], 1000)
    end
    return update, 200
end

gcs:send_text(0, "opendog demo starting")
return update, 1000
