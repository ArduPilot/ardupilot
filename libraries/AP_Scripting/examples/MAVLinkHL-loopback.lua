-- Lua script to simulate a HL connection over a UART

-- Setup:
-- This script requires 1 serial port:
-- A "Script" serial port to connect directly to the GCS

-- Usage:
-- Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control
-- whether to send or not

-- Caveats:
-- This will send HIGH_LATENCY2 packets in place of HEARTBEAT packets
-- A single HIGH_LATENCY2 packet will be send every 5 sec
-- MAVLink 1 will be used, as it's slightly more efficient (50 vs 52 bytes for a HL2 message)
-- Only MAVLink commands (COMMAND_LONG, COMMAND_INT) from the GCS will *not* be ignored

-- Written by Stephen Dade (stephen_dade@hotmail.com)

local port = serial:find_serial(0)

if not port or baud == 0 then
    gcs:send_text(0, "No Scripting Serial Port")
    return
end

port:begin(19200)
port:set_flow_control(0)

-- true to use MAVLink1, false to use MAVLink2
MAVLink:begin(true)

local time_last_tx = millis():tofloat() * 0.001

-- enable high latency mode from here, instead of having to enable from GCS
MAVLink:set_high_latency_enabled(true)

function HLSatcom ()
  -- read in any bytes from GCS and and send to MAVLink processor
  while port:available() > 0 do
    local byte = port:read()
    MAVLink:receive(byte)
  end
  
  -- send HL2 packet every 5 sec
  if MAVLink:is_high_latency_enabled() and (millis():tofloat() * 0.001) - time_last_tx > 5 then
    
    local pkt = MAVLink:create_high_latency_packet()
    for idx = 1, #pkt do
      port:write(pkt:byte(idx))
    end
    gcs:send_text(3, "HL2 packet sent, size: " .. tostring(#pkt))
    
    time_last_tx = millis():tofloat() * 0.001

  end
  
  return HLSatcom, 100
end

return HLSatcom, 100
