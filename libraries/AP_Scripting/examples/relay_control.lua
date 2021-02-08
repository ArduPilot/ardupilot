
-- toggle a relay at 50Hz

local RELAY_NUM = 0

function update() -- this is the loop which periodically runs
   relay:toggle(RELAY_NUM)
   return update, 20 -- reschedules the loop at 50Hz
end

return update() -- run immediately before starting to reschedule
