-- Copyright (C) 2026 David Buzz
-- SPDX-License-Identifier: GPL-3.0-or-later
-- Part of ArduPilot and released under its licence: see COPYING.txt.

-- Tuning for scripts flying models/plane3d.gltf, set once the script has
-- registered its parameters, which a SITL defaults file cannot reach.
-- Installed by --godot beside the test's own scripts; inert without them.

function update()
    if param:get('AEROM_ERR_COR_P') == nil then
        return update, 500
    end
    -- only the path error correction: a stronger time corrector can halt
    -- the moving target, which NaNs the script's path rate, and throttle or
    -- damping changes measurably upset its speed control on this airframe
    param:set('AEROM_ERR_COR_P', 4.0)
    gcs:send_text(6, "plane3d: aerobatics gains tuned")
    -- done; no reschedule
end

return update, 500
