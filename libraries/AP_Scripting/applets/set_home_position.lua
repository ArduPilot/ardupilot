-- Script to set new home position with the RC
-- Sets the preset or current home position after 2 seconds that the switch is in the chosen position
-- Author: Marco Robustini and Alessandro Apostoli

local rc_option = 300
local sw_channel = rc:find_channel_for_option(rc_option)
local sw_last_pos = -1

-- Define and initialize the location object with desired coordinates; enter the coordinates with the first 9 numbers and the height in centimetres
local my_home_location = Location()
my_home_location:lat(448060084)
my_home_location:lng(116100275)
my_home_location:alt(0000)

function update()
  local sw_pos = sw_channel:get_aux_switch_pos() -- expected 0,1,2
  if sw_pos ~= sw_last_pos then
    if sw_pos == 1 then
      -- MID set HOME to preset my_home_location
      if my_home_location:lat() ~= 0 and my_home_location:lng() ~= 0 then
          gcs:send_text(6, string.format("Home set to preset position: Lat:%.7f Long:%.7f Alt:%.1f", my_home_location:lat()/10000000, my_home_location:lng()/10000000, my_home_location:alt()/100))
          ahrs:set_home(my_home_location)
          gcs:send_text(1, string.format("Preset home activated"))
        else
          gcs:send_text(6, string.format("Preset Home position missing, unable to set HOME to preset location"))
      end
    elseif sw_pos == 2 then
      -- HIGH set HOME to current position keeping altitude of previous home
      local location = ahrs:get_location()
      if location then
        current_home = ahrs:get_home()
        if current_home then
          location:alt(current_home:alt()) -- new home has same altitude as previous home
          gcs:send_text(6, string.format("Home set to current position: Lat:%.7f Long:%.7f Alt:%.1f", location:lat()/10000000, location:lng()/10000000, location:alt()/100))
          ahrs:set_home(location)
          gcs:send_text(1, string.format("Dynamic home set"))
        else
          gcs:send_text(6, string.format("Home position not set, unable to set HOME to current position"))
        end
      else
        gcs:send_text(6, "Waiting for GPS lock")
      end
    end
    sw_last_pos = sw_pos -- debounce
  end
  return update, 2000
end

return update()
