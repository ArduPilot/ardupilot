-- switch between DCM and EKF3 on a switch

local scripting_rc1 = rc:find_channel_for_option(300)
local EKF_TYPE = Parameter()
if not EKF_TYPE:init('AHRS_EKF_TYPE') then
  gcs:send_text(6, 'init AHRS_EKF_TYPE failed')
end

function update()
   local sw_pos = scripting_rc1:get_aux_switch_pos()
   if sw_pos == 0 then
      EKF_TYPE:set(3)
   else
      EKF_TYPE:set(0)
   end
   return update, 100
end

return update()
