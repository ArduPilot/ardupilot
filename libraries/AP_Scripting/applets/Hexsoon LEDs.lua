-- Hexsoon EDU 450 LED script
-- LEDs will be fixed colour when disarmed, when armed LEDs will also strobe

--[[ INSTRUCTIONS:
LEDs have two servo headers, plug into AUX 5 and 6 on cube.
Note white is ground, black and brown are signal
black into AUX 5 and brown into AUX 6
Other aux pins could be used but 5 and 6 make best use of the available groups
    BRD_PWM_COUNT must be 6
Aux 5 and 6 are servo outs 13 and 14
    SERVO13_FUNCITON 132 Profi LED Clock
    SERVO14_FUNCTION 94 Script 1
setup scripting:
    SCR_ENABLE 1
    SCR_HEAP_SIZE 88032
Reboot
Use MP config tab -> MAVFtp to place this script in 'APM/scripts' folder
Reboot
Check messages tab should see:
LEDs strip left: chan=14
RCOut: PWM:1-12 ProfiLED:13-14
Note that 1-12 might not be PWM, all than matters is: ProfiLED:13-14
If not check for scripting error messages.
LEDs should now work!, if not try swapping AUX 5 and 6, either by physically swapping or by swapping the servo functions and rebooting
To get colours to match either change the ordering in "local led_map ="  below or swap headers round on the LED distribution board
If using 6 les add two extra colours to "local led_map =" e.g:  "local led_map = {red, red, red, green, green, green}"
--]]
-- luacheck: only 0

-- helper colours, red, green, blue values from 0 to 255
local red   = {255, 0,   0}
local green = {0,   255, 0}
local blue =  {0,   0,   255}

-- led map giving the colour for the LEDs plugged in
local led_map = {red, red, green, green}

-- number of ms to strobe white, can be 0 for no strobe
-- strobe only active when armed
local strobe_on_time = 100

-- number of ms to hold colour set in led_map
local colour_map_time = 900

local led_chan = assert(SRV_Channels:find_channel(94),"LED: channel not set") + 1
gcs:send_text(6, "LED strip: chan=" .. tostring(led_chan))
assert(serialLED:set_num_profiled(led_chan, 8),"Failed LED setup")

local timer = false
function update_LEDs()
  local armed = arming:is_armed()
  if armed and timer and strobe_on_time > 0 then
    timer = false
    -- all white
    serialLED:set_RGB(led_chan, -1, 255, 255, 255)
    serialLED:send(led_chan)
    return update_LEDs, strobe_on_time
  else
    timer = armed
    for led_num = 1,#led_map do
      -- each LED module has two LEDs, set both to colour map colour
      serialLED:set_RGB(led_chan, ((led_num-1)*2)+0, led_map[led_num][1], led_map[led_num][2], led_map[led_num][3])
      serialLED:set_RGB(led_chan, ((led_num-1)*2)+1, led_map[led_num][1], led_map[led_num][2], led_map[led_num][3])
    end
    serialLED:send(led_chan)
    return update_LEDs, colour_map_time
  end
end

return update_LEDs()
