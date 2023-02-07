--[[ 
   example for getting cached aux function value
--]]

local RATE_HZ = 10

local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_INFO = 6

local AUX_FUNCTION_NUM = 302

local last_func_val = nil
local last_aux_pos = nil

function update()
   local aux_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
   if aux_pos ~= last_aux_pos then
      last_aux_pos = aux_pos
      gcs:send_text(MAV_SEVERITY_INFO, string.format("Aux set to %u", aux_pos))
   end
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / RATE_HZ)
end

-- start running update loop
return protected_wrapper()
