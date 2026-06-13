local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

local function create_params()
    local PARAM_TABLE_KEY = 80
    assert(param:add_table(PARAM_TABLE_KEY, "NV_", 5), 'could not add param table')
    assert(param:add_param(PARAM_TABLE_KEY, 1,  'CAN_SRV_RC_CH', 9), 'could not add param1')
    assert(param:add_param(PARAM_TABLE_KEY, 2,  'RUN_CAN_COMM', 1), 'could not add param2')
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

local update = nil;
local channel_num = can_servo_switch_rc_channel:get()
local function is_can_turned_on()
    if arming:is_armed() and rc:has_valid_input() then
        local channel = rc:get_channel(channel_num)
        return channel and channel:norm_input() > 0.5
    end

    return false
end

local function update_run_can_comm()
    if run_can_comm:get() == 0 then
        return update_run_can_comm, 1000
    end
    return update, 100
end

g_can_turned_on_last_state = false

local pwm = PWMSource()
pwm:set_pin(54) -- S5

local hotRC_ARMING_LOW = 0
local hotRC_ARMING_HIGH = 1
local hotRC_ARMING_ARMED = 2
local hotRC_ARMING = hotRC_ARMING_LOW

local prev_pwm = pwm:get_pwm_us()

local function arm_hotRC()
  relay:off(5)
  arming:disarm()
end

local function update_hotRC_ARM()
  local curr_pwm = pwm:get_pwm_us()
  if prev_pwm and curr_pwm then
    if curr_pwm < 1200 then
        if hotRC_ARMING == hotRC_ARMING_HIGH and prev_pwm > 1800 then
            gcs:send_text(6, "hotRC_ARMING_ARMED "..prev_pwm.."->"..curr_pwm)
            hotRC_ARMING = hotRC_ARMING_ARMED
            arm_hotRC()
            curr_pwm = nil
        elseif hotRC_ARMING ~= hotRC_ARMING_LOW then
            gcs:send_text(6, "hotRC_ARMING_LOW "..prev_pwm.."->"..curr_pwm)
            hotRC_ARMING = hotRC_ARMING_LOW
        end
    elseif curr_pwm > 1800 then
        if hotRC_ARMING == hotRC_ARMING_LOW and prev_pwm < 1200 then
            gcs:send_text(6, "hotRC_ARMING_HIGH "..prev_pwm.."->"..curr_pwm)
            hotRC_ARMING = hotRC_ARMING_HIGH
        elseif prev_pwm < 1200 then
            gcs:send_text(6, "hotRC_ARMING_LOW "..prev_pwm.."->"..curr_pwm)
            hotRC_ARMING = hotRC_ARMING_LOW
        end
    end
  end
  prev_pwm = curr_pwm
end

update = function()
    if run_can_comm:get() == 0 then
        return update_run_can_comm, 1000
    end

    local can_turned_on = is_can_turned_on()
    if can_turned_on ~= g_can_turned_on_last_state then
        g_can_turned_on_last_state = can_turned_on
        gcs:send_text(6, "CAN communication " .. (can_turned_on and "enabled" or "disabled"))
    end
    can_uc_esc_bm:set(can_turned_on and esc_bitmask_param:get() or 0)

    if arming:is_armed() then
        if rc:has_valid_input() then
            relay:on(5)
        else
            update_hotRC_ARM()
        end
    else
        relay:off(5)
    end

    return update, 100
end

return update()
