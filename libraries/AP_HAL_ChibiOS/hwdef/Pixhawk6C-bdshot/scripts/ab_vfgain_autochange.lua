-- Set of quadplane VTOL modes
local quadplane_modes = {
    [17] = true,  -- QSTABILIZE
    [18] = true,  -- QHOVER
    [19] = true,  -- QLOITER
    [20] = true,  -- QLAND
    [21] = true,  -- QRTL
    [22] = true   -- QAUTOTUNE
}

-- Cache current value to avoid redundant writes
local current_value = -1

function update()
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()

    if not armed then
        return update, 1000  -- do nothing if disarmed
    end

    if quadplane_modes[mode] then
        if current_value ~= 0.2 then
            param:set('Q_TAILSIT_VFGAIN', 0.2)
            gcs:send_text(6, "Set Q_TAILSIT_VFGAIN to 0.2 (VTOL mode)")
            current_value = 0.2
        end
    else
        if current_value ~= 0 then
            param:set('Q_TAILSIT_VFGAIN', 0)
            gcs:send_text(6, "Set Q_TAILSIT_VFGAIN to 0 (Non-VTOL mode)")
            current_value = 0
        end
    end

    return update, 1000  -- repeat every 1 second
end

gcs:send_text(6, "Airbound Lua: VF gain script loaded.")
return update()
