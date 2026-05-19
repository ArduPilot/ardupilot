--[[
   example script to test lua socket API
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 46
PARAM_TABLE_PREFIX = "NT_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'net_test: could not add param table')

--[[
  // @Param: NT_ENABLE
  // @DisplayName: enable network tests
  // @Description: Enable network tests
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local NT_ENABLE = bind_add_param('ENABLE',  1, 0)
if NT_ENABLE:get() == 0 then
   return
end

local NT_TEST_IP = { bind_add_param('TEST_IP0', 2, 192),
                     bind_add_param('TEST_IP1', 3, 168),
                     bind_add_param('TEST_IP2', 4, 13),
                     bind_add_param('TEST_IP3', 5, 15) }

local NT_BIND_PORT = bind_add_param('BIND_PORT', 6, 15001)

local PORT_ECHO = 7

gcs:send_text(MAV_SEVERITY.INFO, "net_test: starting")

local function test_ip()
   return string.format("%u.%u.%u.%u", NT_TEST_IP[1]:get(), NT_TEST_IP[2]:get(), NT_TEST_IP[3]:get(), NT_TEST_IP[4]:get())
end

local counter = 0
local sock_tcp_echo = Socket(0)
local sock_udp_echo = Socket(1)
local sock_tcp_in = Socket(0)
local sock_tcp_in2 = nil
local sock_udp_in = Socket(1)

assert(sock_tcp_echo, "net_test: failed to create tcp echo socket")
assert(sock_udp_echo, "net_test: failed to create udp echo socket")
assert(sock_tcp_in:bind("0.0.0.0",
                        NT_BIND_PORT:get()), string.format("net_test: failed to bind to TCP %u", NT_BIND_PORT:get()))
assert(sock_tcp_in:listen(1), "net_test: failed to listen")
assert(sock_udp_in:bind("0.0.0.0",
                        NT_BIND_PORT:get()), string.format("net_test: failed to bind to UDP %u", NT_BIND_PORT:get()))

--[[
   test TCP or UDP echo
--]]
local function test_echo(name, sock)
   if not sock:is_connected() then
      if not sock:connect(test_ip(), PORT_ECHO) then
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("test_echo(%s): failed to connect", name))
         return
      end

      if not sock:set_blocking(true) then
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("test_echo(%s): failed to set blocking", name))
         return
      end
   end

   local s = string.format("testing %u", counter)
   local nsent = sock:send(s, #s)
   if nsent ~= #s then
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("test_echo(%s): failed to send", name))
      return
   end
   local r = sock:recv(#s)
   if r then
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("test_echo(%s): got reply '%s'", name, r))
   end
end

--[[
   test a simple server
--]]
local function test_server(name, sock)
   if name == "TCP" then
      if not sock_tcp_in2 then
         sock_tcp_in2 = sock:accept()
         if not sock_tcp_in2 then
            return
         end
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("test_server(%s): new connection", name))
      end
      sock = sock_tcp_in2
   end

   local r, ip, port = sock:recv(1024)
   if r and #r > 0 then
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("test_server(%s): got input '%s'", name, r))
   end
   if ip then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("packet from %s:%u", ipv4_addr_to_string(ip), port))
   end
end

local function update()
   test_echo("TCP", sock_tcp_echo)
   test_echo("UDP", sock_udp_echo)
   test_server("TCP", sock_tcp_in)
   test_server("UDP", sock_udp_in)
   counter = counter + 1
   return update,1000
end

return update,100
