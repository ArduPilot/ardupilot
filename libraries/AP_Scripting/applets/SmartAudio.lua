--------------------------------------------------
--------------------------------------------------
--------- VTX LUA for SMARTAUDIO 2.0 -------------
------------based on work by----------------------
---------Craig Fitches 07/07/2020 ----------------
-------------Mods by H. Wurzburg -----------------
------------clean up by Peter Hall----------------

-----------------HARDWARE------------------
-- tested on CUAVv5Nano and TX8111 VTX

-- Prerequisites ----------------------------------
-- 1. Only works in Ardupilot 4.1dev or later
-- 2. FC with 2MB cache for LUA Scripting
-- 3. Currently only works with SmartAudio 2.0

------------ Instructions ------------------------
-- 1. Set an unused Serial port in Ardupilot to protocol 28 (scripting) and option 4 (half-duplex)
-- 2. Setup an rc channel's RXc_OPTION to 300 for changing power and SCR_USER1 parameter for initial power upon boot
---------and set to -1 for unchanged, 0 (PitMode),1,2,3, or 4 for power level (1 lowest,4 maximum)
-- 3. Attach the UART's TX for the Serial port chosen above to the VTX's SmartAudio input

-- init local variables
local startup_pwr = param:get('SCR_USER1') 
local scripting_rc = rc:find_channel_for_option(300)
local port = serial:find_serial(0)
local _current_power = -1

-- hexadecimal smart audio 2.0 commands
local power_commands = {}
power_commands[1] = { {0x00,0x00,0xAA,0x55,0x0B,0x01,0x01,0xF8,0x00}, "VTX Pit Mode" }
power_commands[2] = { {0x00,0x00,0xAA,0x55,0x05,0x01,0x00,0x6B,0x00}, "VTX PWR LOW" } -- SMARTAUDIO_V2_COMMAND_POWER_0
power_commands[3] = { {0x00,0x00,0xAA,0x55,0x05,0x01,0x01,0xBE,0x00}, "VTX PWR MED" } -- SMARTAUDIO_V2_COMMAND_POWER_1
power_commands[4] = { {0x00,0x00,0xAA,0x55,0x05,0x01,0x02,0x14,0x00}, "VTX PWR HIGH" } -- SMARTAUDIO_V2_COMMAND_POWER_2
power_commands[5] = { {0x00,0x00,0xAA,0x55,0x05,0x01,0x03,0xC1,0x00}, "VTX PWR MAX" } -- SMARTAUDIO_V2_COMMAND_POWER_3


-- return a power level from 1 to 5 as set with a switch
function get_power()
  input = scripting_rc:norm_input() -- - 1 to 1
  input = (input + 1) * 2 -- 0 to 4
  return math.floor(input+0.5) + 1 -- integer 1 to 5
end

-- set the power in the range 1 to 5
function setPower(power)
  if power == _current_power then
    return
  end
  updateSerial(power_commands[power][1])
  gcs:send_text(4, power_commands[power][2])
  _current_power = power
end

-- write output to the serial port
function updateSerial(value)
  for count = 1, #value do
    port:write(value[count])
  end
end

---- main update ---
function update()
  setPower(get_power())
  return update, 500
end

-- initialization
function init()
  -- check if setup properly
  if not port then
    gcs:send_text(0, "SmartAudio: No Scripting Serial Port")
    return
  end
  if not scripting_rc then
    gcs:send_text(0, "SmartAudio: No RC option for scripting")
    return
  end

  port:begin(4800)

  -- Set initial power after boot based on SCR_USER1
  if startup_pwr then -- make sure we found the param
    if startup_pwr >= 0  and startup_pwr < 5 then
      setPower(math.floor(startup_pwr) + 1)

      -- set the current power local to that requested by the rc in
      -- this prevents instantly changing the power from the startup value
      _current_power = get_power()
    end
  end
  return update, 500
end

return init, 2000 --Wait 2 sec before initializing, gives time for RC in to come good in SITL, also gives a better chance to see errors
