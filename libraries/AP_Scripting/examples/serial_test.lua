-- Lua script to write and read from a serial

---@diagnostic disable: need-check-nil

local port = serial:find_serial(0)

port:begin(115200)
port:set_flow_control(0)

local step = 65

function spit ()
  if port:available() > 0 then
    read = port:read()
    gcs:send_text(0, read .. " = " .. step)
  end
  if step > 122 then
    step = 65
  else
    step = step + 1
  end
  port:write(step)
  return spit, 1000
end

return spit, 1000
