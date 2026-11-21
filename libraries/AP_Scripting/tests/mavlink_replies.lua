-- luacheck: globals mavlink gcs param Parameter arming i2c
-- Exercise the actual script reply paths with narrow and wide source IDs.
-- Run by test.Rover.ScriptingMAVLink using ArduPilot's embedded Lua.
-- Upper-half system IDs are returned as signed 32-bit integers by this Lua.
assert(string.packsize('j') == 4 and string.packsize('n') == 4)
local mav = require('MAVLink/mavlink_msgs')
local pending, reply
local component = 190
local function noop() end
mavlink = {
    init = noop, register_rx_msgid = noop, block_command = noop,
    receive_chan = function()
        local msg = pending
        pending = nil
        return msg, 0
    end,
    send_chan = function(_, chan, msgid, payload, target_system)
        assert(chan == 0 and reply == nil)
        reply = {id=msgid, payload=payload, target_system=target_system}
    end,
}
gcs = {
    send_text = function(_, _, text)
        assert(not text:find('Internal Error', 1, true), text)
    end,
    get_allow_param_set = function() return true end,
    set_allow_param_set = noop,
}
param = {add_table = function() return true end, add_param = function() return true end}
Parameter = function() return {get = function() return 1 end} end
arming = {is_armed = function() return true end}
i2c = {get_device = function() return {transfer = noop} end}

for _, source in ipairs({42, 255, 256, 70000, 0x7fffffff, 0x80000000, 0xffffffff}) do
    for _, script in ipairs({'MAVLink_Commands.lua', 'BQ40Z_bms_shutdown.lua', 'param-lockdown.lua'}) do
        local is_param = script:find('param-lockdown', 1, true)
        local command = script:find('BQ40Z', 1, true) and 246 or 31000
        local payload = is_param and string.pack('<fBBc16B', 78, 1, 1, 'DISARM_DELAY', 9)
            or string.pack('<fffffffHBBB', 2, 0, 0, 0, 0, 0, 0, command, 1, 1, 0)
        pending = string.pack('<I2BBBBBI4BI3', 0, 253, #payload, 2, 0, 0, source, component,
                              is_param and 23 or 76)
            .. payload .. string.rep('\0', 279 - #payload) .. string.pack('<I4', 0)
        local crc = mav.generateCRC(pending:sub(4, 15 + #payload) .. string.char(is_param and 168 or 152))
        pending = string.pack('<I2', crc) .. pending:sub(3)
        assert(#pending == 298)
        reply = nil
        -- loadfile is not exposed in the AP sandbox. Load into this script's
        -- environment so the applet uses the mock vehicle APIs above.
        local path = 'scripts/modules/' .. script
        local file = assert(io.open(path))
        local content = assert(file:read('a'))
        file:close()
        local update = assert(load(content, '@' .. path, 't', _ENV))()
        if pending ~= nil then update() end
        assert(reply, script .. ' did not reply')
        local target, target_component
        if is_param then
            assert(reply.id == 345)
            local index, name, result
            index, target, target_component, name, result = string.unpack('<hBBc16B', reply.payload)
            assert(index == -1 and name:gsub('%z', '') == 'DISARM_DELAY' and result == 3)
        else
            assert(reply.id == 77)
            local ack_command, result, progress, result_param2
            ack_command, result, progress, result_param2, target, target_component = string.unpack('<HBBi4BB', reply.payload)
            assert(progress == 0 and result_param2 == 0)
            assert(ack_command == command and result == (command == 246 and 4 or 2))
        end
        -- The binding replaces the payload placeholder and supplies the header.
        assert(target == 0 and target_component == component, script)
        assert((reply.target_system & 0xffffffff) == (source & 0xffffffff), script)
    end
end
print('MAVLink script reply tests passed')
