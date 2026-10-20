-- Saves Lua line coverage statistics for autotest once a second.
-- The counts are recorded by lua_scripts.cpp in SITL builds configured
-- with --lua-coverage, which provide them to scripts as luacov_stats:
-- luacov_stats[source][line] = hit count, zero for lines never run.

local STATS_FILE = "scripts/luacov.stats.out"

-- a save is spread over several runs so a large amount of coverage
-- data doesn't exceed the per-run instruction limit
local ENTRIES_PER_RUN = 200

---@diagnostic disable-next-line: undefined-global
local stats = luacov_stats

if not stats then
    gcs:send_text(4, "luacov: unavailable (build with --lua-coverage)")
    return
end

gcs:send_text(6, "luacov: active")

-- file format:
--   # start <millis() when the save started>
--   then per source:
--   # <source>
--   <line> <hit count>
--   ...
-- followed by "# end" so readers can tell the file is complete

local f              -- file being written; nil when no save is in progress
local sources        -- names of the sources being saved
local source_index   -- index into sources of the source being saved
local line           -- last line saved for that source; nil if none yet

-- continue the current save, starting one if none is in progress.
-- Returns true if the save is not yet complete
local function save_stats()
    if not f then
        f = io.open(STATS_FILE, "w")
        if not f then
            return false
        end
        f:write("# start " .. tostring(millis()) .. "\n")
        sources = {}
        for src in pairs(stats) do
            -- only sources loaded from files; others (e.g. strings
            -- passed to load()) are named by their code, which may
            -- contain newlines
            if string.sub(src, 1, 1) == "@" then
                sources[#sources+1] = src
            end
        end
        source_index = 1
        line = nil
    end
    local entries = ENTRIES_PER_RUN
    while source_index <= #sources do
        local lines = stats[sources[source_index]]
        if line == nil then
            f:write("# " .. sources[source_index] .. "\n")
        end
        -- the firmware adds every line of a source to its table when
        -- the source first runs, so the table gains no keys between
        -- runs and next() can carry on from where it stopped
        local count
        line, count = next(lines, line)
        while line ~= nil do
            f:write(line .. " " .. count .. "\n")
            entries = entries - 1
            if entries <= 0 then
                return true
            end
            line, count = next(lines, line)
        end
        source_index = source_index + 1
    end
    f:write("# end\n")
    f:close()
    f = nil
    return false
end

local function update()
    if save_stats() then
        return update, 10
    end
    return update, 1000
end

return update, 1000
