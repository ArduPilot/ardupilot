
local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

local function create_params()
    local PARAM_TABLE_KEY = 80
    assert(param:add_table(PARAM_TABLE_KEY, "NV_", 5), 'could not add param table')

    --[[
      // @Param: NV_CAN_SRV_RC_CH
      // @DisplayName: CAN/servo switch RC channel
      // @Description: RC input channel used to select CAN control while the vehicle is armed. The channel number follows the ArduPilot RC API numbering (1 is RC1 and 16 is RC16). CAN control is selected when the normalised channel input is above 0.5.
      // @Range: 1 16
      // @Increment: 1
      // @User: Advanced
--]]
    assert(param:add_param(PARAM_TABLE_KEY, 1,  'CAN_SRV_RC_CH', 9), 'could not add param1')

    --[[
      // @Param: NV_RUN_CAN_COMM
      // @DisplayName: Run CAN control switching
      // @Description: Enables the script logic that switches the configured DroneCAN ESC outputs between CAN control and servo control.
      // @Values: 0:Disabled,1:Enabled
      // @User: Advanced
--]]
    assert(param:add_param(PARAM_TABLE_KEY, 2,  'RUN_CAN_COMM', 1), 'could not add param2')

    --[[
      // @Param: NV_ESC_BITMASK
      // @DisplayName: DroneCAN ESC output mask
      // @Description: Output-channel mask written to CAN_D1_UC_ESC_BM when CAN control is selected. Clear a bit to keep the corresponding output out of DroneCAN ESC commands.
      // @Bitmask: 0:ESC 1,1:ESC 2,2:ESC 3,3:ESC 4,4:ESC 5,5:ESC 6,6:ESC 7,7:ESC 8,8:ESC 9,9:ESC 10,10:ESC 11,11:ESC 12,12:ESC 13,13:ESC 14,14:ESC 15,15:ESC 16,16:ESC 17,17:ESC 18,18:ESC 19,19:ESC 20,20:ESC 21,21:ESC 22,22:ESC 23,23:ESC 24,24:ESC 25,25:ESC 26,26:ESC 27,27:ESC 28,28:ESC 29,29:ESC 30,30:ESC 31,31:ESC 32
      // @User: Advanced
--]]
    assert(param:add_param(PARAM_TABLE_KEY, 3,  'ESC_BITMASK', 15), 'could not add param3')


    local can_servo_switch_rc_channel = Parameter()
    local run_can_comm = Parameter()
    local esc_bitmask_param = Parameter()
    can_servo_switch_rc_channel:init('NV_CAN_SRV_RC_CH')
    run_can_comm:init('NV_RUN_CAN_COMM')
    esc_bitmask_param:init('NV_ESC_BITMASK')
    return esc_bitmask_param, run_can_comm, can_servo_switch_rc_channel
end

local esc_bitmask_param, run_can_comm, can_servo_switch_rc_channel = create_params()
local can_uc_esc_bm = bind_param('CAN_D1_UC_ESC_BM')

local channel_num = can_servo_switch_rc_channel:get()
local function is_can_turned_on()
    local channel = rc:get_channel(channel_num)
    return run_can_comm:get() == 1 and channel and channel:norm_input() > 0.5
end

local timestamp = millis()

local function timeout_reached(time, timeout)
    return millis() - time > timeout
end

local function update()
    if rc:has_valid_input() then
        timestamp = millis()
    end

    local is_extra_armed = vehicle:arm_extra_controller(true, false)
    if not is_extra_armed then
        if arming:is_armed() and timeout_reached(timestamp, 5000) then
            is_extra_armed = vehicle:arm_extra_controller(true, true)
        end
    end
    can_uc_esc_bm:set(is_can_turned_on() and (arming:is_armed() or is_extra_armed) and esc_bitmask_param:get() or 0)

    return update, 1000
end

return update()
