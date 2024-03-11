-- Script to set new home position with the RC
-- Sets the preset or current home position after 2 seconds that the switch is in the chosen position
-- Author: Marco Robustini

local rc_option = 300
local channel = rc:find_channel_for_option(rc_option)
local sw_state = -1

-- Define and initialize the location object with desired coordinates; enter the coordinates with the first 9 numbers and the height in centimetres
my_home_location = Location()
my_home_location:lat(448060084)
my_home_location:lng(116100275)
my_home_location:alt(0000)

function update()
  local sw_pos = channel:get_aux_switch_pos()
  if sw_state ~= sw_pos or sw_pos == 2 then
    if sw_pos == 1 then
      ahrs:set_home(my_home_location)
      gcs:send_text(6, string.format("Set Home to preset position: Lat:%.7f Long:%.7f Alt:%.1f", my_home_location:lat()/10000000, my_home_location:lng()/10000000, my_home_location:alt()/100))
    elseif sw_pos == 2 then
      if ahrs:home_is_set() then
        local location = ahrs:get_location()
        if location then
          location:alt(0)
          ahrs:set_home(location)
          gcs:send_text(6, string.format("Set Home to current position: Lat:%.7f Long:%.7f Alt:%.1f", location:lat()/10000000, location:lng()/10000000, location:alt()/100))
        else
          gcs:send_text(6, "Waiting for GPS lock")
        end
      end
    end
  end
  return update, 2000
end

return update()
