-- switch between EKF2 and EKF3 on a switch

local AUX_FUNCTION_NUM = 300
local EKF_TYPE = Parameter('AHRS_EKF_TYPE')

function update()
   local sw_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
   if not sw_pos then
      return update, 100
   end
   if sw_pos == 2 then
      EKF_TYPE:set(3)
   else
      EKF_TYPE:set(2)
   end
   return update, 100
end

gcs:send_text(0, "Loaded AHRS switch for EKF3/EKF2")

return update()
