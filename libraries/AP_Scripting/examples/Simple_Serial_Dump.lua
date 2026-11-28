-- this script reads data from a serial port and dumps it to a file

local file_name = 'raw serial dump.txt'
local baud_rate = 230400

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local port = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- make a file
local file = assert(io.open(file_name, "w"),"Could not create file " .. file_name)
file:close()

-- begin the serial port
port:begin(baud_rate)
port:set_flow_control(0)

function update() -- this is the loop which periodically runs

  local n_bytes = math.min(port:available(), 512)
  file = io.open(file_name, "a")
  while n_bytes > 0 do
      c = port:read()
      if c == nil then
          break
      end
      file:write(string.char())
      n_bytes = n_bytes - 1
  end
  file:close()

  return update, 10 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
