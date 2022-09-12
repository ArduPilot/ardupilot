-- master control script for selecting aerobatics scripts
-- the two rc channels below must be assigned

local scripting_rc_activation = assert(rc:find_channel_for_option(300), 'no sw assigned for trick activation') -- trick script selection channel
local scripting_rc_selection = assert(rc:find_channel_for_option(301), 'no switch assigned for trick selection') -- trick script activation channel (low:off/stop, mid:report trick ID to GCS, high:do trick

local trick_id = 0 --current trick ID, 0 means no trick, placing any number into the AERO_TRICK_ID param below means execute a script that recognizes it as its trick ID

local sw_current = scripting_rc_activation:norm_input() --current position of scripting_rc_activation switch -1 to 1
local sw_last = sw_current -- last position of scripting scripting_rc_activation switch

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 72
assert(param:add_table(PARAM_TABLE_KEY, "AERO_", 30), 'could not add param table')

-- this control script uses AERO_TRICK_ID to report the selected trick number from the scripting_rc_selection rc channel
assert(param:add_param(PARAM_TABLE_KEY, 1,  'TRICK_ID', 0), 'could not add param1') -- trick IDs for switch activation
assert(param:add_param(PARAM_TABLE_KEY, 2,  'RPT_COUNT', 0), 'could not add param2') -- trick repeats 
assert(param:add_param(PARAM_TABLE_KEY, 3,  'RATE_MAX', 120), 'could not add param3') -- max rates applied to axes(like ACRO rate param)
assert(param:add_param(PARAM_TABLE_KEY, 4,  'HGT_P', 1), 'could not add param4') -- height P gain
assert(param:add_param(PARAM_TABLE_KEY, 5,  'HGT_I', 2), 'could not add param5') -- height I gain
assert(param:add_param(PARAM_TABLE_KEY, 6,  'HGT_KE_BIAS', 20), 'could not add param6') --height knife-edge addition for pitch
assert(param:add_param(PARAM_TABLE_KEY, 7,  'THR_PIT_FF', 80), 'could not add param67') --throttle FF from pitch
assert(param:add_param(PARAM_TABLE_KEY, 8,  'SPD_P', 5), 'could not add param8') -- speed P gain
assert(param:add_param(PARAM_TABLE_KEY, 9,  'SPD_I', 25), 'could not add param9')  -- speed I gain
assert(param:add_param(PARAM_TABLE_KEY, 10, 'TRICK_ANG', 90), 'could not add param10')  -- knife-edge angle,etc.
assert(param:add_param(PARAM_TABLE_KEY, 11, 'TRICK_RAT', 90), 'could not add param11')  -- used for trick loop rate, axial roll rates, etc


local trick_id = Parameter("AERO_TRICK_ID")
local sw_last = 0
local sw_current = 0
local last_selection = 0
function update()
   local trick_selection = math.floor(5 * (scripting_rc_selection:norm_input_ignore_trim() +1)) -- produces range of 0 to 10 for selecting trick ID
   if trick_selection == 0 then -- anytime trick selection is zero force trick ID to 0, another way to terminate tricks
      trick_id:set(trick_selection)
   end
   sw_current = scripting_rc_activation:norm_input_ignore_trim()
   if sw_current == sw_last then
      if sw_current > -0.25 and sw_current < 0.75 and last_selection ~= trick_selection then 
        gcs:send_text(0, string.format("Current trick ID = %.0f",trick_selection))
        last_selection = trick_selection
      end
      return update, 500
   else
      if sw_current < -0.25 then 
        trick_id:set(0)
      elseif sw_current > 0.75 and arming:is_armed() then  --setting trick ID to non-zero should be recognized by trick with that ID to init and execute
        trick_id:set(trick_selection)
      elseif sw_last < -0.25 and sw_current > -0.25 and sw_current < 0.75 then
        gcs:send_text(0, string.format("Current trick ID = %.0f",trick_selection))
      end
      sw_last = sw_current
      return update, 500
    end
end

return update, 50
