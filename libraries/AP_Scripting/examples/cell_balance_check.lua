--[[
 script implementing pre-arm check that batteries are well balanced
--]]

local MAX_CELL_DEVIATION = 0.2

local auth_id = arming:get_aux_auth_id()

local NCELLS_MAX = 16

function check_cell_balance(bnum)
   local min_volt = -1
   local max_volt = -1
   for c=0,NCELLS_MAX do
      local voltage = battery:get_cell_voltage(bnum, c)
      if not voltage then
         break
      end
      if min_volt == -1 or min_volt > voltage then
         min_volt = voltage
      end
      if max_volt == -1 or max_volt < voltage then
         max_volt = voltage
      end
   end
   local vdiff = max_volt - min_volt
   if vdiff > MAX_CELL_DEVIATION then
      arming:set_aux_auth_failed(auth_id, string.format("Batt[%u] imbalance %.1fV", bnum+1, vdiff))
      return false
   end
   return true
end

function update()
   local num_batts = battery:num_instances()
   local ok = true
   for i=0,num_batts do
      if not check_cell_balance(i) then
         ok = false
      end
   end
   if ok then
      arming:set_aux_auth_passed(auth_id)
   end
   return update, 500
end

return update()
