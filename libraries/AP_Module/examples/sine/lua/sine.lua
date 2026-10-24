-- SPDX-License-Identifier: GPL-3.0-or-later
-- AI-assisted sine demo sharing SINE_FREQ with Rust, Ada and Fortran.

local PARAM_TABLE_KEY = 120
local frequency = Parameter()

local function update()
    local seconds = millis():tofloat() * 0.001
    local value = math.sin(2.0 * math.pi * frequency:get() * seconds)
    gcs:send_named_float("Sine_lua", value)
    return update, 100
end

local function initialise()
    if frequency:init("SINE_FREQ") then
        return update, 100
    end
    if param:add_table(PARAM_TABLE_KEY, "SINE_", 1) then
        -- @Param: FREQ
        -- @DisplayName: Sine wave frequency
        -- @Description: Shared wave frequency for the four-language sine demo, sampled at 10 Hz.
        -- @Units: Hz
        -- @Range: 0 5
        -- @Increment: 0.1
        -- @User: Standard
        assert(param:add_param(PARAM_TABLE_KEY, 1, "FREQ", 0.2), "Sine: frequency parameter")
        assert(frequency:init("SINE_FREQ"), "Sine: frequency binding")
        return update, 100
    end
    -- A native module may still be publishing its retained table entry.
    return initialise, 100
end

return initialise, 100
