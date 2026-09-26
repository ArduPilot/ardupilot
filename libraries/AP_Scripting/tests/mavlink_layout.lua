-- Run from the repository root with Lua 5.3 or newer.
package.path = 'libraries/AP_Scripting/modules/?.lua;' .. package.path
local mav = require('MAVLink/mavlink_msgs')
-- Independent wire CRC fixtures for all signing/source-width/target flags.
local checksums = {0x94ce, 0x3662, 0xb1b5, 0x878e, 0xa192, 0x2ef4, 0x468d, 0x9a1d}
for flags=0,7 do
  local source = flags & 2 ~= 0 and 0xabcdef12 or 42
  local target = flags & 4 ~= 0 and 0xfedcba98 or 7
  local payload = string.pack('<fffffffHBBB', 1,2,3,4,5,6,7,300, flags & 4 ~= 0 and 255 or target, 250, 1)
  for _,storage in ipairs({40,256,264}) do
    local data = string.pack('<I2BBBBBI4B', checksums[flags+1], 253, #payload, flags, 0, 0, source, 11)
      .. string.pack('<I3', 76) .. payload .. string.rep('\0', storage - #payload + 15)
      .. string.pack('<I4', target)
    local msg = assert(mav.decode(data, {[76]='COMMAND_LONG'}, true))
    assert(msg.sysid == source and msg.command == 300)
    assert(msg.target_sysid == (flags & 4 ~= 0 and target or nil))
    assert(msg.target_component == 250)
    if storage == 264 then assert(mav.decode(data, {[76]='COMMAND_LONG'})) end
    local bad = data:sub(1,4)..string.char(flags | 128)..data:sub(6)
    assert(mav.decode(bad, {[76]='COMMAND_LONG'}, true) == nil)
    bad = string.char(data:byte(1) ~ 1)..data:sub(2)
    assert(mav.decode(bad, {[76]='COMMAND_LONG'}, true) == nil)
  end
end
-- Legacy C storage is 291 bytes, regardless of MAVLink wire version.
local payload = string.pack('<fffffffHBBB', 1,2,3,4,5,6,7,300,7,250,1)
for _, magic in ipairs({253, 254}) do
  local header = string.pack('<I2BBBBBBB', 0, magic, #payload, 0, 0, 0, 42, 11)
  local data = header .. string.pack('<I3', 76) .. payload .. string.rep('\0', 264 - #payload + 15)
  assert(#data == 291)
  local decoded, offset = mav.decode_header(data)
  assert(decoded.sysid == 42 and decoded.compid == 11 and decoded.msgid == 76)
  assert(offset == 13 and decoded.target_sysid == nil)
  assert(mav.sysid32_layout(data) == false)
end
-- PARAM_SET is consumed by both param-lockdown and config_profiles.
for _, wide in ipairs({false, true}) do
  local source = wide and 0xFFFFFFFF or 42
  local payload = string.pack('<fBBc16B', 123.5, 1, 1, 'CRUISE_SPEED', 9)
  local prefix = wide and string.pack('<I2BBBBBI4B', 0,253,#payload,2,0,0,source,11)
    or string.pack('<I2BBBBBBB', 0,253,#payload,0,0,0,source,11)
  local data = prefix .. string.pack('<I3', 23) .. payload .. string.rep('\0',264 - #payload + 15)
    .. (wide and string.pack('<I4',0) or '')
  assert(#data == (wide and 298 or 291))
  local header, offset = mav.decode_header(data)
  local value, _, _, name = string.unpack('<fBBc16', data, offset)
  assert(header.sysid == source and value == 123.5 and name:gsub('%z','') == 'CRUISE_SPEED')
end
print('MAVLink layout and target tests passed')
