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
  return spit, 1000
  port:write(step)
end

return spit, 1000
