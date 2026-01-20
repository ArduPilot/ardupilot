
-- https://luacheck.readthedocs.io/en/stable/warnings.html
ignore = {"111", -- Setting an undefined global variable.
          "113", -- Accessing an undefined global variable.
          "212/_.*", -- Unused argument. Ignore only when argument starts with a underscore
          "631", -- Line is too long.
          "611", -- A line consists of nothing but whitespace.
          "612", -- A line contains trailing whitespace.
          "614"} -- Trailing whitespace in a comment.

-- These lua scripts are not for running on AP
exclude_files = {"Tools/CHDK-Scripts/*", "modules/*", "libraries/AP_Scripting/tests/luacheck.lua", "lua-language-server/*"}

-- Grab AP globals from docs file
stds.ArduPilot = {}
stds.ArduPilot.read_globals = {}

local env = setmetatable({}, {__index = _G})
assert(pcall(setfenv(assert(loadfile("libraries/AP_Scripting/docs/docs.lua")), env)))

for key, value in pairs(env) do
    local singleton = { other_fields = false }

    -- add sub-functions
    if type(value) == 'table' then
        singleton['fields'] = {}
        for s_key, _ in pairs(value) do
            singleton['fields'][s_key] = { other_fields = false }
        end
    end

    stds.ArduPilot.read_globals[key] = singleton
end

-- We cannot add enums to the docs without giving a value, we don't know the value until compile time
-- There are only a few, so I have added them manually here.
local function add_enum(singleton, enum)
    stds.ArduPilot.read_globals[singleton].fields[enum] = { other_fields = false }
end

add_enum('mission', 'MISSION_COMPLETE')
add_enum('mission', 'MISSION_RUNNING')
add_enum('mission', 'MISSION_STOPPED')

add_enum('terrain', 'TerrainStatusOK')
add_enum('terrain', 'TerrainStatusUnhealthy')
add_enum('terrain', 'TerrainStatusDisabled')

add_enum('gps', 'GPS_OK_FIX_3D_RTK_FIXED')
add_enum('gps', 'GPS_OK_FIX_3D_RTK_FLOAT')
add_enum('gps', 'GPS_OK_FIX_3D_DGPS')
add_enum('gps', 'GPS_OK_FIX_3D')
add_enum('gps', 'GPS_OK_FIX_2D')
add_enum('gps', 'NO_FIX')
add_enum('gps', 'NO_GPS')

std = "lua53+ArduPilot"

