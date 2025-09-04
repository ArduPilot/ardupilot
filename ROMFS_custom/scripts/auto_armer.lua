--script to force arm on startup
local battery_instance = 0
local voltage_threshold = 7
local low_power = 25 -- mw value

function update()
    if not arming:is_armed() then
        arming:arm()
    end
    local voltage = battery:voltage(battery_instance)
    if voltage == nil then
        gcs:send_text(6, "No battery data!")
    else 
        gcs:send_text(6, string.format("Battery: %.2f V", voltage))
        if voltage < voltage_threshold then
            param:set("VTX_POWER", low_power)
            local power = param:get("VTX_POWER")
            gcs:send_text(6, "Battery low -> Current VTX_POWER: " .. tostring(power))
        else
            gcs:send_text(6, "Battery healthy -> VTX power normal")
            local power = param:get("VTX_POWER")
            gcs:send_text(6, "Current VTX_POWER: " .. tostring(power))
        end
    end
    return update, 1000
end

return update, 1000