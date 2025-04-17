-- This script runs custom arming checks for validations that are important
-- to some, but might need to be different depending on the vehicle or use case
-- so we don't want to bake them into the firmware. Requires SCR_ENABLE =1 so must 
-- be a higher end FC in order to be used. (minimum 2M flash and 1M RAM)
-- Thanks to @yuri_rage and Peter Barker for help with the Lua and Autotests

SCRIPT_VERSION = "4.7.0-006"
SCRIPT_NAME = "Arming Checks"
SCRIPT_NAME_SHORT = "ArmCk"

REFRESH_RATE = 1000
INITIAL_DELAY = 20000   -- wait 20 seconds for the AP to settle down before starting to show messages

ALTITUDE_LEGAL_MAX = 120 -- max legal altitude is 120m by default, change if different limits apply

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7, NONE=-1}

local PARAM_TABLE_KEY = 122
local PARAM_TABLE_PREFIX = "ZAR_"

local param_idx = 1

local validate      -- forward definition of the validate() function

-- create parameter table for ZAR_ parameters that will be created below
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 25), 'could not add param table')

-- These are the plane modes that autoenable the geofence
local PLANE_MODE_AUTO = 10
local PLANE_MODE_TAKEOFF = 13

local arm_auth_id

----  CLASS: Arming_Check  ----
local Arming_Check = {}
Arming_Check.__index = Arming_Check

setmetatable(Arming_Check, {
    __call = function(cls, func, param_name, text, severity, passed, changed) -- constructor
        local self      = setmetatable({}, cls)
        self.func       = func             -- func() is called to validate arming
        self.param_name = param_name       -- the name of a ZAR_ parameter overriding the severity
        self.text       = text             -- the message on failure
        self.severity   = severity         -- is failure a warning or hard stop?
        self.passed     = passed or nil
        self.changed    = changed or true
        return self
    end
})

function Arming_Check:state()
    local passed_new = self.func()

    if self.passed == passed_new then
        self.changed = false
    else
        self.passed = passed_new
        self.changed = true
    end
    return passed_new
end

FENCE_TYPE = Parameter("FENCE_TYPE")
FENCE_TOTAL = Parameter("FENCE_TOTAL")
FENCE_ENABLE = Parameter("FENCE_ENABLE")
FENCE_AUTOENABLE = Parameter("FENCE_AUTOENABLE")
FOLL_ENABLE = Parameter("FOLL_ENABLE")
FOLL_SYSID = Parameter("FOLL_SYSID")
RTL_ALTITUDE = Parameter("RTL_ALTITUDE")
RTL_CLIMB_MIN = Parameter("RTL_CLIMB_MIN")
Q_ENABLE = Parameter("Q_ENABLE")
local q_options = 0
local q_rtl_alt = 0
if Q_ENABLE:get() == 1 then
    q_options = Parameter("Q_OPTIONS"):get() or 0
    q_rtl_alt = Parameter("Q_RTL_ALT"):get() or 0
end

-- this is the pre 4.7 name for this parameter
local function try_sysid_thismav()
    MAV_SYSID = Parameter("SYSID_THISMAV")
end
if not pcall(try_sysid_thismav) then
    MAV_SYSID = Parameter("MAV_SYSID")
end

-- Logic provided by Peter Barker 
-- Fences present if 
-- a. there are basic fences (assume this includes circle fences?) or 
-- b. there are polygon_fences with at least one side 
local function fence_present()
    local enabled_fences = FENCE_TYPE:get()
    local basic_fence = (enabled_fences & 0xB) ~= 0
    local polygon_fence = ((enabled_fences & 4) ~= 0) and FENCE_TOTAL:get() > 0
    return basic_fence or polygon_fence
end

local function geofence_enabled_armingcheck()
    --We fail if there is a no fence but FENCE_ENABLE is set - so we pass if NOT that
    --this duplicates the code in AP_Fence.cpp:
    --return not (not AC_Fence:present() and param:get('FENCE_ENABLE') == 1)

    return not (not fence_present() and FENCE_ENABLE:get() == 1)
end
local function geofence_autoenabled_armingcheck()
    --We fail if there is a no fence but FENCE_AUTOENABLE is set - so we pass if NOT that
    --Plus we only fail this if in AUTO mode or TAKEOFF mode (on a plane)
    local vehicle_type = FWVersion:type()
    local mode = vehicle:get_mode()

    -- If this check is useful for other vehicles they will need to add them later
    if (vehicle_type == 3 and (mode == PLANE_MODE_AUTO or mode == PLANE_MODE_TAKEOFF)) then
        return not (not fence_present() and FENCE_AUTOENABLE:get() > 0)
    end
    return true
end
local function sysid_not_set()
    local mav_sysid = MAV_SYSID:get()

    if mav_sysid == nil or mav_sysid <= 1 then
        return false
    end
    return true
end
local function foll_sysid_not_set()
    local foll_enable = FOLL_ENABLE:get()
    if foll_enable == 1 then
        local foll_sysid = FOLL_SYSID:get()
        if foll_sysid == 0 then
            return false
        end
    end
    return true
end
local function foll_sysid_not_thismav()
    local foll_enable = FOLL_ENABLE:get()
    if foll_enable == 1 then
        local mav_sysid = MAV_SYSID:get()
        local foll_sysid = FOLL_SYSID:get() or 0
        if foll_sysid > 0 and mav_sysid == foll_sysid then
            return false
        end
    end
    return true
end

-- for many use cases if MNTx_SYSID_DEFLT is set is should match the FOLL_SYSID (remember set ZAR_MNTX_SYSID severity to NONE to disable this check)
local function mntx_sysid_match()
    local foll_enable = FOLL_ENABLE:get()
    local mnt1_enable = Parameter("MNT1_TYPE"):get() ~= 0
    local mnt2_enable = Parameter("MNT2_TYPE"):get() ~= 0
    if foll_enable == 1 and (mnt1_enable or mnt2_enable) then
        local foll_sysid = FOLL_SYSID:get()
        if mnt1_enable then
            local mnt1_sysid = Parameter("MNT1_SYSID_DFLT"):get() or 0
            if mnt1_sysid > 0 and foll_sysid ~= mnt1_sysid then
                return false
            end
        end
        if mnt2_enable then
            local mnt2_sysid = Parameter("MNT2_SYSID_DFLT"):get() or 0
            if mnt2_sysid > 0 and foll_sysid ~= mnt2_sysid then
                return false
            end
        end
    end
    return true
end

local function motors_emergency_stopped()
    return not SRV_Channels:get_emergency_stop()
end

local function rtl_altitude_legal()
    if (RTL_ALTITUDE ~= nil and RTL_ALTITUDE:get() > ALTITUDE_LEGAL_MAX) then
        return false
    end

    if Q_ENABLE:get() == 1 then
        if q_rtl_alt > ALTITUDE_LEGAL_MAX then
            return false
        end
    end
    return true
end

local function rtl_climb_legal()
    if (RTL_CLIMB_MIN ~= nil and RTL_CLIMB_MIN:get() > ALTITUDE_LEGAL_MAX) then
        return false
    end
    return true
end

local function qland_warning()
    if Q_ENABLE:get() == 1 then
        if q_options ~= nil then
            local q_land_option = ((q_options & (1 << 5)) > 0)
            local q_RTL_option = ((q_options & (1 << 20)) > 0)

            if (not q_land_option) and (not q_RTL_option) then
                return false
            end
        end
    end
    return true -- always ok if not a QuadPlane
end
local function qrtl_warning()
    if Q_ENABLE:get() > 0 then
        if q_options ~= nil then
            local q_land_option = ((q_options & (1 << 5)) > 0)
            local q_RTL_option = ((q_options & (1 << 20)) > 0)
            if q_land_option or q_RTL_option then
                return false
            end
        end
    end
    return true -- always ok if not a QuadPlane
end

-- Arming checks can be deleted if not required, or set the parameter to MAV_SEVERITY.NONE to avoid changing the script
-- or new arming checks added 
local arming_checks = {
    -- __call = function(cls, func, param_name, text, severity, passed, changed) -- constructor
    GeoFence_Enabled = Arming_Check(geofence_enabled_armingcheck, "FENCE_ENABLE", 
                            "FENCE_ENABLE = 1 but no fence", MAV_SEVERITY.ERROR, true, false ),                  
    GeoFence_AutoEnabled = Arming_Check(geofence_autoenabled_armingcheck, "FENCE_AUTO",
                            "FENCE_AUTOENABLE > 0 but no fence", MAV_SEVERITY.ERROR, true, false ),
    MotorsEstopped = Arming_Check(motors_emergency_stopped, "ESTOP",
                            "Motors Emergency Stopped", MAV_SEVERITY.ERROR, true, false ),
    SYSID_NotSet = Arming_Check(sysid_not_set, "SYSID",
                            "MAV_SYSID not set", MAV_SEVERITY.ERROR, false, false ),
    FOLL_SYSID_NotSet = Arming_Check(foll_sysid_not_set, "FOLL_SYSID",
                            "FOLL_SYSID not set", MAV_SEVERITY.ERROR, true, false ),
    FOLL_SYSID_not_THISMAV = Arming_Check(foll_sysid_not_thismav, "FOLL_SYSID_X",
                            "FOLL_SYSID == MAV_SYSID", MAV_SEVERITY.ERROR, true, false ),
    MNTx_SYSID_match_FOLL = Arming_Check(mntx_sysid_match, "MNTX_SYSID",
                            "MNTx_SYSID != FOLL", MAV_SEVERITY.NOTICE, true, false ),
    RTLAltitudeLegal = Arming_Check(rtl_altitude_legal, "RTL_ALT",
                            "RTL_ALTITUDE or Q_RTL_ALT too high", MAV_SEVERITY.ERROR, false, false ),
    RTLClimbLegal = Arming_Check(rtl_climb_legal, "RTL_CLIMB",
                            "RTL_CLIMB_MIN too high", MAV_SEVERITY.WARNING, true, false ),
    -- I like having warnings for either Q_OPTIONS bit 5 = QLand AND QRTL because I want to know which is set on every boot
    QLandWarning = Arming_Check(qland_warning, "Q_FS_LAND",
                            "Q will land on failsafe", MAV_SEVERITY.NOTICE, true, false ),
    QRTLWarning = Arming_Check(qrtl_warning, "Q_FS_RTL",
                            "Q will RTL on failsafe", MAV_SEVERITY.NOTICE, true, false )
}

local function idle_while_armed()
    if not arming:is_armed() then return validate, REFRESH_RATE end
    return idle_while_armed, REFRESH_RATE * 20
end

local function initialize()
    for _, check in pairs(arming_checks) do
        --[[
        // @Param: ZAR_xxx
        // @DisplayName: Arming Check Severity
        // @Description: MAV_SEVERITY of the arming check MAV_SEVERITY.NONE to disable
        // @User: Standard
        --]]
        local param_name = PARAM_TABLE_PREFIX..check.param_name
        assert(param:add_param(PARAM_TABLE_KEY , param_idx, check.param_name, check.severity),
                        string.format('could not add param %s', param_name))
        param_idx = param_idx + 1
    end
end

validate = function() -- this is the loop which periodically runs to do the validations

    if arming:is_armed() then return idle_while_armed() end

    if Q_ENABLE:get() == 1 then
        q_options = Parameter("Q_OPTIONS"):get() or 0
        q_rtl_alt = Parameter("Q_RTL_ALT"):get() or 0
    end

    local validated = true

    for _, check in pairs(arming_checks) do
        local param_name = PARAM_TABLE_PREFIX..check.param_name
        local parameter = Parameter(param_name)
        local param_severity  = parameter:get() or MAV_SEVERITY.NONE

        if param_severity ~= MAV_SEVERITY.NONE then -- ignore if NONE
            local check_passed = check:state() -- sets check.passed and check.changed

            if check.changed then
                if check_passed then
                    gcs:send_text(MAV_SEVERITY.INFO, string.format('%s: Cleared: %s', SCRIPT_NAME_SHORT, check.text))
                else
                    if param_severity <= MAV_SEVERITY.ERROR then
                        arming:set_aux_auth_failed(arm_auth_id, string.format('%s: fail: %s', SCRIPT_NAME_SHORT, check.text))
                    else
                        gcs:send_text(param_severity, string.format('%s: %s', SCRIPT_NAME_SHORT, check.text))
                        -- display the message, but this should not block arming so is actually "passed"
                        check_passed = true
                    end
                end
                validated = validated and check_passed
            else -- if nothing changed we still want to fail on errors
                if not check_passed then -- it seems like logically "and" will work but it doesn't
                    if param_severity ~= MAV_SEVERITY.NONE and param_severity <= MAV_SEVERITY.ERROR then
                        arming:set_aux_auth_failed(arm_auth_id, string.format('%s: fail: %s', SCRIPT_NAME_SHORT, check.text))
                        validated = false
                    end
                end
            end
        end
    end

    if validated then
        arming:set_aux_auth_passed(arm_auth_id)
    end

    return validate, REFRESH_RATE
end

local function delayed_startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return validate, REFRESH_RATE
end

-- start running update loop - waiting 20s for the AP to initialize so we don't spam the user with spurios notices
if FWVersion:type() == 3 then -- 3 is Plane, could be extended to other vehicles but some parameters are different
    arm_auth_id = arming:get_aux_auth_id()
    if arm_auth_id == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s %s Failed", SCRIPT_NAME, SCRIPT_VERSION) )
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: get_aux_auth_id failed", SCRIPT_NAME_SHORT))
    else
        initialize()
        return delayed_startup, INITIAL_DELAY
    end
else
    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on Plane", SCRIPT_NAME_SHORT))
end
