-- This script is a useful tool when using gimbal and camera. You can use it to switch between manual
-- and auto control for mission mode. It is meant for use with a transmitter that has a free switch
-- available.
-- The camera is setup for picture trigger during auto but can do other things e.g. video during
-- manual mode also. The script changes parameters but does not store them.
-- Setup Gimbal and Camera incl. manual control channels, then setting SCR_USER1 to 1 enables this script.

-- (known issue is that manual and auto mode do not have same travel as SERVOn_ settings are ignored
-- for "pass-through" RCINn channels - a scaled and bounded variant must be introduced and used here)

-- Tested and working as of 25th Aug 2020 (Copter Dev)
-- Derived from Heli_IM_COL_Tune.lua in https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets
-- Tested and working as of 9th and 24th of Oct 2021 (drtrigon)


function update()
    -- read switch input from designated scripting RCx_OPTION
    local rcin_select = select_switch_ch:get_aux_switch_pos()

    -- Save params only if switch changed
    if rcin_select ~= rcin_select_pre then
        if rcin_select == 0 then
             -- low: set gimbal and cam to manual mode (RCIN9, RCIN10, RCIN12)
            set_save_param('SERVO8_FUNCTION',59,false)
            set_save_param('SERVO9_FUNCTION',60,false)
            set_save_param('SERVO10_FUNCTION',62,false)

            gcs:send_text(6, 'LUA: Gimbal-Cam mode manual')  -- MAV_SEVERITY_INFO
        elseif rcin_select == 1 then
             -- middle: set gimbal to auto and cam to manual mode (mount_tilt, mount_roll, RCIN12)
            set_save_param('SERVO8_FUNCTION',7,false)
            set_save_param('SERVO9_FUNCTION',8,false)
            set_save_param('SERVO10_FUNCTION',62,false)

            gcs:send_text(6, 'LUA: Gimbal-Cam mode auto/manual')  -- MAV_SEVERITY_INFO
        else
             -- high: set gimbal and cam to auto mode (mount_tilt, mount_roll, camera_trigger)
            set_save_param('SERVO8_FUNCTION',7,false)
            set_save_param('SERVO9_FUNCTION',8,false)
            set_save_param('SERVO10_FUNCTION',10,false)

            gcs:send_text(6, 'LUA: Gimbal-Cam mode auto')  -- MAV_SEVERITY_INFO

            -- No script exit condition (runs always)
        end

        rcin_select_pre = rcin_select
    end

    -- Reschedule
    return script_cont_stop()
end


-- Standardised function for stopping or continuing
function script_cont_stop()
    -- Get SCR_USER1 parameter value and check whether to run or not
    if param:get('SCR_USER1') ~= 1 then  -- DISABLED (any number) or FAULT (nil)
--        gcs:send_text(6, string.format('LUA: %s = %i',param_name,value))  -- MAV_SEVERITY_INFO
--        gcs:send_text(6, 'LUA: Gimbal-Cam mode disabled')  -- MAV_SEVERITY_INFO

        rcin_select_pre = -1

--        return  -- stop running until reboot of FC
        return script_cont_stop, 10000  -- stop running until change of param +10s
    end

    -- Set RC channels to use to control gimbal and cam servo params
    -- SERVOn_FUNCTION = 7 (mount_tilt)      -> SERVO8_FUNCTION   and   MNT_RC_IN_TILT -> RC9  -> RCIN9  (59)
    -- SERVOn_FUNCTION = 8 (mount_roll)      -> SERVO9_FUNCTION   and   MNT_RC_IN_ROLL -> RC10 -> RCIN10 (60)
    -- SERVOn_FUNCTION = ? (mount_pan)       -> ...
    -- SERVOn_FUNCTION = 10 (camera_trigger) -> SERVO10_FUNCTION  and   RCn_OPTION = 9 (Camera Trigger) -> RC12_OPTION -> RCIN12 (62)

    -- Set RC channels to use to control gimbal and cam servo params
    select_switch_ch = rc:find_channel_for_option(300) --scripting ch 1
    if (select_switch_ch == nil) then  -- NORC
        gcs:send_text(3, 'LUA: RC channels not set')  -- MAV_SEVERITY_ERROR

        return script_cont_stop, 10000  -- stop running until valid param +10s
    end

    -- Script is HEALTHY, continue running update() (in fast loop)
--    return update, 500
    return update, 1000
end


-- function for setting and saving parameter
function set_save_param(str,val,save)
    if save then
        -- Set and save
        if param:set_and_save(str,val) then
            gcs:send_text(2, 'LUA: params saved')  -- MAV_SEVERITY_CRITICAL
        else
            gcs:send_text(2, 'LUA: param set failed')  -- MAV_SEVERITY_CRITICAL
        end
    else
        -- Just set
        if not(param:set(str,val)) then
            gcs:send_text(2, 'LUA: param set failed')  -- MAV_SEVERITY_CRITICAL
        end
    end
end


--- -- -- Initial Setup --- --- ---
-- Script status buffers
rcin_select_pre = -1

gcs:send_text(6, 'LUA: Gimbal-Cam mode started')  -- MAV_SEVERITY_INFO

-- Only continue running script if healthy
return script_cont_stop()
