-- Load the coverage tracker and schedule periodic stats saves.
local cov = require('luacov')

local SAVE_INTERVAL_MS = uint32_t(5000)
local last_save_ms = millis()

function update()
    local now = millis()
    if now - last_save_ms > SAVE_INTERVAL_MS then
        cov.save_stats()
        last_save_ms = now
    end
    return update, 1000
end

return update, 1000
