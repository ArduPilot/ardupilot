-- Line coverage tracker for ArduPilot SITL Lua scripts.
-- Requires AP_LUA_COVERAGE_ENABLED build flag (enables the debug library and
-- the C-level line hook dispatch in lua_scripts.cpp).
-- Stats are saved to luacov.stats.out periodically by luacov_start.lua.

if not debug then
    return {}
end

local M = {}
local stats = {}  -- stats[source] = {[line] = hit_count}

local function line_hook(_, line)
    local info = debug.getinfo(2, "S")
    if not info then return end
    local src = info.source
    local s = stats[src]
    if not s then
        s = {}
        stats[src] = s
    end
    s[line] = (s[line] or 0) + 1
end

function M.save_stats()
    local f = io.open("luacov.stats.out", "w")
    if not f then return end
    for src, lines in pairs(stats) do
        local max_line = 0
        for ln in pairs(lines) do
            if ln > max_line then max_line = ln end
        end
        f:write("# " .. src .. "\n")
        f:write(max_line .. "\n")
        for i = 1, max_line do
            f:write((lines[i] or 0) .. "\n")
        end
    end
    f:close()
end

-- Register the hook in the Lua registry so lua_scripts.cpp can call it for
-- every line event without being overwritten by reset_loop_overtime().
debug.getregistry()["AP_LUA_LINE_HOOK"] = line_hook

return M
