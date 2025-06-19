-- This script is a useful tool when first setting up a heli.  You can use it to dial in the IM_STB_COL_2 and 
-- IM_STAB_COL_3 parameters in a more intuative way.  It is meant for use with a transmitter that has two pots
-- available.  1 Pot can be used to control the gradient of the line between the 40% and 60% curve points.  Use 
-- this pot to adjust the sensitivity of the collective about the collective midstick.  The 2nd Pot then controls
-- the value of the 50% point on the curve.  This can be used to set the collective position to aid with hovering 
-- at the midstick.

-- Tested and working as of 25th Aug 2020 (Copter Dev)

function update()
    local rcin_50 = col50_val_ch:norm_input_ignore_trim()
    local rcin_grad = col_grad_ch:norm_input_ignore_trim()
    rcin_save = save_switch_ch:get_aux_switch_pos()

    -- Offset starting midstick curve value to get new value
    -- Max = +30% , Min = -30%
    local im_50 = (value_im_3 + value_im_2) * 0.5 --(%)
    local delta_50 = rcin_50*30
    im_50 = im_50 + delta_50

    -- Scale rcin_grad to be between 0 and 1
    rcin_grad = (rcin_grad+1)*0.5

    -- Calculate delta due to gradient
    local grad_40 = rcin_grad * -10 --(%)
    local grad_60 = rcin_grad * 10 --(%)

    -- Calculate param values
    local im_40_pct = im_50+grad_40
    local im_60_pct = im_50+grad_60

    -- Ensure IM_STB_COL_2 < IM_STB_COL_3
    if im_40_pct >= im_60_pct then
        im_40_pct = im_60_pct - 1
    end

    -- Ensure IM_STB_COL_2 and IM_STB_COL_3 are withing appropriate ranges
    im_40_pct = min_max(math.floor(im_40_pct),1,98)
    im_60_pct = min_max(math.floor(im_60_pct),2,99)

    -- Enforce that IM_STB_COL_1 < IM_STB_COL_2
    local im_0_pct = get_im_val('IM_STB_COL_1',false)
    if im_0_pct >= im_40_pct then
        -- Correct within parameter limits
        im_0_pct = min_max(im_40_pct - 1,0,97)

        -- Set correct value to prevent pre-arm warnings
        set_save_param('IM_STB_COL_1',im_0_pct,false)
    end

    -- Enforce that IM_STB_COL_4 > IM_STB_COL_3
    im_100_pct = get_im_val('IM_STB_COL_4',false)
    if im_100_pct <= im_60_pct then
        -- Correct within parameter limits
        im_100_pct = min_max(im_60_pct + 1,3,100)

        -- Set correct value to prevent pre-arm warnings
        set_save_param('IM_STB_COL_4',im_100_pct,false)
    end

    -- Save params only if switch active
    if rcin_save < 1 then
         -- just set 40% and 60% parameter values
        set_save_param('IM_STB_COL_2',im_40_pct,false)
        set_save_param('IM_STB_COL_3',im_60_pct,false)
    else
        set_save_param('IM_STB_COL_1',im_0_pct,true)
        set_save_param('IM_STB_COL_2',im_40_pct,true)
        set_save_param('IM_STB_COL_3',im_60_pct,true)
        set_save_param('IM_STB_COL_4',im_100_pct,true)

        -- Set script exit condition
        script_status = COMPLETE
    end

    -- Reschedule
    foo, sched = script_cont_stop()
    return foo, sched
end


-- Get parameter value and perform checks to ensure successful
function get_im_val(param_name,disp)
    local value = param:get(param_name)

    if value >= 0 then
        if disp then
            gcs:send_text(3, string.format('LUA: %s = %i',param_name,value))
        end
    else
        gcs:send_text(3, string.format('LUA: Failed get %s',param_name))
        script_status = false
    end

    return value
end


-- Prevent parameters from being set out of range
function min_max(value,min,max)
    if value < min then
        value = min
    end

    if value > max then
        value = max
    end

    return value
end


-- Standardised function for stopping or continuing
function script_cont_stop()
    if script_status == HEALTHY then
        return update, 500

    elseif script_status == COMPLETE then
        gcs:send_text(2, 'LUA: IM_COL tune complete')

    elseif script_status == NORC then
        gcs:send_text(2, 'LUA: RC channels not set')

    else --FAULT
        gcs:send_text(2, 'LUA: IM_COL setter stopped')
    end
end


-- function for setting and saving parameter
function set_save_param(str,val,save)
    if save then
        -- Set and save
        if param:set_and_save(str,val) then
            gcs:send_text(2, 'LUA: params saved')
        else
            gcs:send_text(2, 'LUA: param set failed')
        end
    else
        -- Just set
        if not(param:set(str,val)) then
            gcs:send_text(2, 'LUA: param set failed')
        end
    end
end

--- -- -- Initial Setup --- --- ---
-- Script status levels
FAULT = 0
NORC = 1
HEALTHY = 2
COMPLETE = 3

script_status = HEALTHY

-- Set RC channels to use to control im_stb_col params
col50_val_ch = rc:find_channel_for_option(300) --scripting ch 1
col_grad_ch = rc:find_channel_for_option(301) --scripting ch 2
save_switch_ch = rc:find_channel_for_option(302) --scripting ch 3

if (col50_val_ch == nil or col_grad_ch == nil or save_switch_ch == nil) then
    script_status = NORC
end

-- Get im_stb_col parameter values and store
value_im_2 = get_im_val('IM_STB_COL_2',true)
value_im_3 = get_im_val('IM_STB_COL_3',true)

-- Only continue running script if healthy
foo, sched = script_cont_stop()
return foo, sched
