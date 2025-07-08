-- This script runs arming checks for validations that are important
-- to some, but might need to behave differently depending on the vehicle or use case
-- so we don't want to bake them into the firmware. 
-- The severity of each check can be set by simply setting the value of parameters.
-- from warnings, which still allow arming to to errors failsafe which prevent arming 
-- Set the value of the matching parameter to the MAV_SEVERITY of the error level you want.
--
-- New checks can optionally be added, by adding a true/false function (false = fail)
-- and adding a check to the arming_checks table. 
--
-- Thanks to @yuri_rage and Peter Barker for help with the Lua and Autotests

SCRIPT_VERSION = "4.7.0-015"
SCRIPT_NAME = "Arming Checks"
SCRIPT_NAME_SHORT = "ArmCk"

REFRESH_RATE = 500      -- wait this many milliseconds between each update loop (while not armed)
INITIAL_DELAY = 20000   -- wait 20 seconds for the AP to settle down before starting to show messages

GLOBAL_PARAM_TABLE_BASE = 160
VALUES_PARAM_TABLE_KEY = GLOBAL_PARAM_TABLE_BASE + 9
VALUES_PARAM_TABLE_PREFIX = "ARM_V_"

FIRMWARE = {GLOBAL = 0, ROVER = 1, COPTER = 2, PLANE = 3, ANTENNA = 4, SUB = 7, BLIMP = 12, HELI = 13 }
VEHICLE = {
    [FIRMWARE.GLOBAL]   = {prefix = "ARM_", key = GLOBAL_PARAM_TABLE_BASE, index = 1},
    [FIRMWARE.ROVER]    = {prefix = "ARM_R_", key = GLOBAL_PARAM_TABLE_BASE + 1, index = 1},
    [FIRMWARE.COPTER]   = {prefix = "ARM_C_", key = GLOBAL_PARAM_TABLE_BASE + 2, index = 1},
    [FIRMWARE.PLANE]    = {prefix = "ARM_P_", key = GLOBAL_PARAM_TABLE_BASE + 3, index = 1},
    [FIRMWARE.ANTENNA]  = {prefix = "ARM_A_", key = GLOBAL_PARAM_TABLE_BASE + 4, index = 1},
    [FIRMWARE.BLIMP]    = {prefix = "ARM_B_", key = GLOBAL_PARAM_TABLE_BASE + 5, index = 1},
    [FIRMWARE.HELI]     = {prefix = "ARM_H_", key = GLOBAL_PARAM_TABLE_BASE + 6, index = 1},
}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7, NONE=-1}

MAV_SEVERITY_TEXT = {[MAV_SEVERITY.EMERGENCY]   = "emrg",
                    [MAV_SEVERITY.ALERT]        = "alrt",
                    [MAV_SEVERITY.CRITICAL]     = "crit",
                    [MAV_SEVERITY.ERROR]        = "fail",
                    [MAV_SEVERITY.WARNING]      = "warn",
                    [MAV_SEVERITY.NOTICE]       = "note",
                    [MAV_SEVERITY.INFO]         = "info",
                    [MAV_SEVERITY.DEBUG]        = "debg",
                }

PLANE_MODE = { AUTO = 10, TAKEOFF = 13}

local param_idx = 1

-- create parameter table for general/global ARM_ parameters that will be created below. 
assert(param:add_table(VEHICLE[FIRMWARE.GLOBAL].key, VEHICLE[FIRMWARE.GLOBAL].prefix, 25), 'could not add param table: '..VEHICLE[FIRMWARE.GLOBAL].prefix)
-- create parameter table for ARM_ parameters for plane that will be created below. 
assert(param:add_table(VEHICLE[FIRMWARE.COPTER].key, VEHICLE[FIRMWARE.COPTER].prefix, 10), 'could not add param table:'..VEHICLE[FIRMWARE.COPTER].prefix)
-- create parameter table for ARM_ parameters for plane that will be created below. 
assert(param:add_table(VEHICLE[FIRMWARE.PLANE].key, VEHICLE[FIRMWARE.PLANE].prefix, 10), 'could not add param table:'..VEHICLE[FIRMWARE.PLANE].prefix)

-- create parameter table for VALUE parameters which are "reference values" used by the parameter methods
assert(param:add_table(VALUES_PARAM_TABLE_KEY, VALUES_PARAM_TABLE_PREFIX, 10), 'could not add param table:'..VALUES_PARAM_TABLE_PREFIX)

local arm_auth_id = arming:get_aux_auth_id()
if arm_auth_id == nil then
    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s %s Failed", SCRIPT_NAME, SCRIPT_VERSION) )
    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: get_aux_auth_id failed", SCRIPT_NAME_SHORT))
    return -- don't execute the script if we can't get an auth_id
end

----  CLASS: Arming_Check  ----
local Arming_Check = {}
Arming_Check.__index = Arming_Check

setmetatable(Arming_Check, {
    __call = function(cls, firmware, func, param_name, text, severity, passed, changed) -- constructor
        local self      = setmetatable({}, cls)
        self.firmware   = firmware         -- which firmware type is this check for (0 for general/global)
        self.func       = func             -- func() is called to validate arming
        self.param_name = param_name       -- the name of a ARM_ parameter overriding the severity
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

-- parameters that store values used by the validation methods, rather than severities of the individual arming checks
ARM_V_ALT_LEGAL = Parameter()
local alt_legal_max = 120.0
ARM_V_RALLY_MAX = Parameter()

FENCE_TYPE = Parameter("FENCE_TYPE")
FENCE_TOTAL = Parameter("FENCE_TOTAL")
FENCE_ENABLE = Parameter("FENCE_ENABLE")
FENCE_AUTOENABLE = Parameter("FENCE_AUTOENABLE")
FOLL_ENABLE = Parameter("FOLL_ENABLE")
FOLL_SYSID = Parameter("FOLL_SYSID")
if FWVersion:type() == FIRMWARE.COPTER then -- Copter specific paramters
    RTL_ALT = Parameter("RTL_ALT")
elseif FWVersion:type() == FIRMWARE.PLANE then -- Plane specific Parameters
    RTL_ALTITUDE = Parameter("RTL_ALTITUDE")
    RTL_CLIMB_MIN = Parameter("RTL_CLIMB_MIN")
    Q_ENABLE = Parameter("Q_ENABLE")
    AIRSPEED_STALL = Parameter("AIRSPEED_STALL")
    AIRSPEED_MIN = Parameter("AIRSPEED_MIN")
    AIRSPEED_CRUISE = Parameter("AIRSPEED_CRUISE")
    AIRSPEED_MAX = Parameter("AIRSPEED_MAX")
    SCALING_SPEED = Parameter("SCALING_SPEED")
end

-- this is the pre 4.7 name for this parameter
MAV_SYSID = Parameter()
if not MAV_SYSID:init("MAV_SYSID") then
    assert(MAV_SYSID:init("SYSID_THISMAV"), string.format("%s SYSID not found", SCRIPT_NAME_SHORT ))
end

local validate      -- forward definition of the validate() function

-- check that the MAVLink SYSID has been set for this vehicle
local function sysid_set()
    local mav_sysid = MAV_SYSID:get()

    if mav_sysid == nil or mav_sysid <= 1 then
        return false
    end
    return true
end

-- check that if Follow is enabled, the FOLL_SYSID is set
local function foll_sysid_set()
    local foll_enable = FOLL_ENABLE:get() or 0
    if foll_enable ~= 1 then
        return true
    end
    local foll_sysid = FOLL_SYSID:get() or 0
    if foll_sysid == 0 then
        return false
    end
    return true
end

-- check that if the FOLL_SYSID is set it's not pointing to itself (the test returns true if "good" - hence the _not_)
local function foll_sysid_not_thismav()
    local foll_enable = FOLL_ENABLE:get() or 0
    if foll_enable ~= 1 then
        return true
    end
    local mav_sysid = MAV_SYSID:get() or 1
    local foll_sysid = FOLL_SYSID:get() or 0
    if foll_sysid > 0 and mav_sysid == foll_sysid then
        return false
    end
    return true
end

-- if FOLLOW is enabled make sure the XYZ offsets have been set
local function foll_ofs_default()
    local foll_enable = FOLL_ENABLE:get()
    if foll_enable ~= 1 then
        return true
    end
    local foll_ofs_x = Parameter("FOLL_OFS_X"):get()
    local foll_ofs_y = Parameter("FOLL_OFS_Y"):get()
    local foll_ofs_z = Parameter("FOLL_OFS_Z"):get()
    if foll_ofs_x == 0 and foll_ofs_y == 0 and foll_ofs_z == 0 then
        return false
    end
    return true
end

-- for many use cases if MNTx_SYSID_DEFLT is set is should match the FOLL_SYSID (remember set ARM_MNTX_SYSID severity to NONE to disable this check)
local function mntx_sysid_match()
    local foll_enable = FOLL_ENABLE:get()
    local mnt1_enable = (Parameter("MNT1_TYPE"):get() ~= 0)
    local mnt2_enable = (Parameter("MNT2_TYPE"):get() ~= 0)

    if foll_enable ~= 1 then
        return true
    end
    local foll_sysid = FOLL_SYSID:get() or 0
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
    return true
end

-- is the RTL altitude legal? This defaults to 120m (400') as set in ARM_ALT_LEGAL
local function rtl_altitude_legal()
    -- plane uses RTL_ALTITUDE in m
    -- RTL_ALTITUDE will be correct for either plane or copter
    if (RTL_ALTITUDE ~= nil and (RTL_ALTITUDE:get() or 300) > alt_legal_max) then
        return false
    end
    return true
end

-- is the Q RTL altitude (quadplane) legal? This defaults to 120m (400') as set in ARM_ALT_LEGAL
local function q_rtl_alt_legal()
    -- Plane specific Parameters
    if (Q_ENABLE:get() or 0) ~= 1 then
        return true
    end
    -- QuadPlane specific Paraemters
    local q_rtl_alt = Parameter("Q_RTL_ALT"):get() or 0
    if q_rtl_alt > alt_legal_max then
        return false
    end
    return true
end

-- is the RTL climb minimum legal? This defaults to 120m (400')
local function rtl_climb_legal()
    if (RTL_CLIMB_MIN ~= nil and RTL_CLIMB_MIN:get() > alt_legal_max) then
        return false
    end
    return true
end

-- Display a message as to whether failsafe will QLand or QRTL
local function qland_warning()
    if (Q_ENABLE:get() or 0) ~= 1 then
        return true
    end
    -- Only applies to QuadPlane
    local q_options = Parameter("Q_OPTIONS"):get() or 0
    local q_land_option = ((q_options & (1 << 5)) > 0)
    local q_RTL_option = ((q_options & (1 << 20)) > 0)

    if (not q_land_option) and (not q_RTL_option) then
        return false
    end
    return true -- always ok if not a QuadPlane
end
local function qrtl_warning()
    if (Q_ENABLE:get() or 0) ~= 1 then
        return true
    end
    local q_options = Parameter("Q_OPTIONS"):get() or 0
    local q_land_option = ((q_options & (1 << 5)) > 0)
    local q_RTL_option = ((q_options & (1 << 20)) > 0)
    if q_land_option or q_RTL_option then
        return false
    end
    return true -- always ok if not a QuadPlane
end

-- A single check to ensure AIRSPEED_STALL < AIRSPEED_MIN < AIRSPEED_CRUISE < AIRSPEED_MAX
local function airspeed_check()
    local airspeed_stall = AIRSPEED_STALL:get() or -1
    local airspeed_min = AIRSPEED_MIN:get() or -1
    local airspeed_cruise = AIRSPEED_CRUISE:get() or -1
    local airspeed_max = AIRSPEED_MAX:get() or -1

    if airspeed_stall > 0 then
        if airspeed_min < airspeed_stall then
            return false
        end
    end
    if airspeed_cruise < airspeed_min then
        return false
    end
    if airspeed_max < airspeed_cruise then
        return false
    end
    return true
end

-- If AIRSPEED_STALL is set AIRSPEED_MIN should be at least 25% higher than AIRSPEED_STALL
local function stallspeed_check()
    local airspeed_stall = AIRSPEED_STALL:get() or -1
    local airspeed_min = AIRSPEED_MIN:get() or -1

    if airspeed_stall > 0 then
        if airspeed_min > (airspeed_stall * 1.25) then
            return false
        end
    end

    return true
end

-- From the wiki: This is the center of the speed scaling range and should be set close to the normal cruising speed of the vehicle.
local function scaling_speed_check()
    local scaling_speed = SCALING_SPEED:get() or -1
    local airspeed_cruise = AIRSPEED_CRUISE:get() or -1
    if scaling_speed <= 0 or airspeed_cruise <= 0 then
        return false
    end
    -- use a 20% range for "close"
    if scaling_speed < (airspeed_cruise * 0.8) or scaling_speed > (airspeed_cruise * 1.2) then
        return false
    end
    -- if either are not set then
    return true
end

-- want to be notified if motors are emergency stopped or not
local function motors_emergency_stopped()
    return not SRV_Channels:get_emergency_stop()
end

-- Logic provided by Peter Barker 
-- Fences present if 
-- a. there are basic fences (assume this includes min(8)/max(1) and home circle(2) fences) or 
-- b. there are polygon_fences (fence type bit 2 = 4) with at least one side 
local function fence_present()
    local enabled_fences = (FENCE_TYPE:get() or 0)
    local basic_fence = (enabled_fences & (8+2+1)) ~= 0
    local polygon_fence = ((enabled_fences & 4) ~= 0) and (FENCE_TOTAL:get() or 0) > 0
    return basic_fence or polygon_fence
end

-- check if any rally point is too far away
local function rally_ok()
    local home_location = ahrs:get_home()
    local rally_distance_max_m = ARM_V_RALLY_MAX:get() or 0
    local rally_points = rally:get_rally_total()
    if rally_points == 0 or rally_distance_max_m <= 0 then
--        gcs:send_text(MAV_SEVERITY.ERROR, string.format('%s: %s: %s', SCRIPT_NAME_SHORT,"rally",
--             "no points:"..rally_distance_max_m))
        return true
    end
    local rally_location
    local rally_distance_m = 0
--        gcs:send_text(MAV_SEVERITY.ERROR, string.format('%s: %s: %s', SCRIPT_NAME_SHORT,"rally",
--             "points:"..rally_points))
    for i = 0, rally_points-1 do
        rally_location = rally:get_rally_location_with_index(i)
        if rally_location ~= Nil then
            rally_distance_m = home_location:get_distance(rally_location)
            if rally_distance_m > rally_distance_max_m then
                return false
            end
        end
    end
    return true
end

-- fail if there is a fence on the board, it must be enabled
local function geofence_present_enabled()
    if not fence_present() then
        return true
    end
    if (FENCE_ENABLE:get() or 0) == 1 or (FENCE_AUTOENABLE:get() or 0) > 0 then
        return true
    end
    return false
end

-- Arming checks can be deleted if not required, or set the parameter to MAV_SEVERITY.NONE to avoid changing the script
-- or new arming checks added 
-- First add checks that can apply to all vehicles.
local arming_checks = {
--[[
    // @Param: ARM_SYSID
    // @DisplayName: MAV_SYSID must be set
    // @Description: Check that MAV_SYSID (or SYDID_THISMAV) has been set. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    SYSID_NotSet = Arming_Check(FIRMWARE.GLOBAL, sysid_set, "SYSID",
                            "MAV_SYSID not set", MAV_SEVERITY.WARNING, false, false ),
--[[
    // @Param: ARM_FOLL_SYSID
    // @DisplayName: FOLL_SYSID must be set 
    // @Description: If FOLL_ENABLE = 1, check that FOLL_SYSID has been set. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    FOLL_SYSID_NotSet = Arming_Check(FIRMWARE.GLOBAL, foll_sysid_set, "FOLL_SYSID",
                            "FOLL_SYSID not set", MAV_SEVERITY.ERROR, true, false ),
--[[
    // @Param: ARM_FOLL_SYSID_X
    // @DisplayName: Vehicle should not follow itself
    // @Description: If FOLL_ENABLE = 1, check that FOLL_SYSID is different to MAV_SYSID. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    FOLL_SYSID_not_THISMAV = Arming_Check(FIRMWARE.GLOBAL, foll_sysid_not_thismav, "FOLL_SYSID_X",
                            "FOLL_SYSID == MAV_SYSID", MAV_SEVERITY.ERROR, true, false ),
--[[
    // @Param: ARM_FOLL_OFS_DEF
    // @DisplayName: Follow Offsets defaulted
    // @Description: Follow offsets should not be left as default (zero) if FOLL_ENABLE = 1. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    FOLL_OFS_Default = Arming_Check(FIRMWARE.GLOBAL, foll_ofs_default, "FOLL_OFS_DEF",
                            "FOLL_OFS_[XYZ] = 0", MAV_SEVERITY.ERROR, true, false ),
--[[
    // @Param: ARM_MNTX_SYSID
    // @DisplayName: Follow and Mount should follow the same vehicle
    // @Description: If FOLL_ENABLE = 1 and MNTx_SYSID_DEFLT is set, check that FOLL_SYSID is equal MNTx. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    MNTx_SYSID_match_FOLL = Arming_Check(FIRMWARE.GLOBAL, mntx_sysid_match, "MNTX_SYSID",
                            "MNTx_SYSID != FOLL", MAV_SEVERITY.WARNING, true, false ),
--[[
    // @Param: ARM_RTL_CLIMB
    // @DisplayName: RTL_CLIMB_MIN should be a valid value
    // @Description: RTL_CLIMB_MIN should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    RTLClimbLegal = Arming_Check(FIRMWARE.GLOBAL, rtl_climb_legal, "RTL_CLIMB",
                            "RTL_CLIMB_MIN too high", MAV_SEVERITY.WARNING, true, false ),
--[[
// @Param: ARM_ESTOP
// @DisplayName: Motors EStopped
// @Description: Emergency Stop disables arming. 3 or less to prevent arming. -1 to disable.
// @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
// @User: Standard
--]]
    EmergencyStop = Arming_Check(FIRMWARE.GLOBAL, motors_emergency_stopped, "ESTOP",
                                "Motors EStopped", MAV_SEVERITY.ERROR, true, false ),
--[[
// @Param: ARM_FENCE
// @DisplayName: Fence not enabled
// @Description: Fences loaded but no fence enabled. 3 or less to prevent arming. -1 to disable.
// @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
// @User: Standard
--]]
    FenceNotEnabled = Arming_Check(FIRMWARE.GLOBAL, geofence_present_enabled, "FENCE",
                                "Fence not enabled", MAV_SEVERITY.WARNING, true, false ),
--[[
// @Param: ARM_RALLY
// @DisplayName: Rally close
// @Description: Rally Point more than ARV_RALLY meters away. 3 or less to prevent arming. -1 to disable.
// @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
// @User: Standard
--]]
    RallyTooFar = Arming_Check(FIRMWARE.GLOBAL, rally_ok, "RALLY",
                                "Rally too far", MAV_SEVERITY.WARNING, true, false ),
}

if FWVersion:type() == FIRMWARE.COPTER then -- add Copter specific Parameters
--[[
    // @Param: ARM_C_RTL_ALT
    // @DisplayName: RTL_ALT should be a valid value
    // @Description: RTL_ALT should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local RTLAltitudeLegal = Arming_Check(FIRMWARE.COPTER, rtl_altitude_legal, "RTL_ALT",
                        "RTL_AL too high", MAV_SEVERITY.ERROR, false, false )
    table.insert(arming_checks, RTLAltitudeLegal)
elseif FWVersion:type() == FIRMWARE.PLANE then -- add Plane specific Parameters
--[[
    // @Param: ARM_P_Q_FS_LAND
    // @DisplayName: Warn if Q failsafe will land
    // @Description: Notify the user that on failsafe a QuadPlan will land. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local QLandWarning = Arming_Check(FIRMWARE.PLANE, qland_warning, "Q_FS_LAND",
                        "Q will land on failsafe", MAV_SEVERITY.NOTICE, true, false )
    table.insert(arming_checks, QLandWarning)
--[[
    // @Param: ARM_P_Q_FS_RTL
    // @DisplayName: Warn if Q failsafe will QRTL
    // @Description: Notify the user that on failsafe a QuadPlan will QRTL. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local QRTLWarning = Arming_Check(FIRMWARE.PLANE, qrtl_warning, "Q_FS_RTL",
                        "Q will RTL on failsafe", MAV_SEVERITY.NOTICE, true, false )
    table.insert(arming_checks, QRTLWarning)
--[[
    // @Param: ARM_P_AIRSPEED
    // @DisplayName: Check AIRSPEED_ parameters
    // @Description: Validate that AIRSPEED_STALL(if set) < MIN < CRUISE < MAX d. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local AirSpeed = Arming_Check(FIRMWARE.PLANE, airspeed_check, "AIRSPEED",
                        "stall < min < crs < max", MAV_SEVERITY.ERROR, true, false )
    table.insert(arming_checks, AirSpeed)
--[[
    // @Param: ARM_P_STALL
    // @DisplayName: AIRSPEED_MIN should be 25% above STALL
    // @Description: Validate that AIRSPEED_MIN is at least 25% above AIRSPEED_STALL(if set). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local StallSpeed = Arming_Check(FIRMWARE.PLANE, stallspeed_check, "STALL",
                        "Min speed not 25% above stall", MAV_SEVERITY.INFO, true, false )
    table.insert(arming_checks, StallSpeed)
--[[
    // @Param: ARM_P_SCALING
    // @DisplayName: SCALING_SPEED valid
    // @Description: Validate that SCALING_SPEED is within 20% of AIRSPEED_CRUISE. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local ScalingSpeed = Arming_Check(FIRMWARE.PLANE, scaling_speed_check, "SCALING",
                        "Scaling spd >< 20% of CRUISE", MAV_SEVERITY.ERROR, true, false )
    table.insert(arming_checks, ScalingSpeed)
--[[
    // @Param: ARM_P_RTL_ALT
    // @DisplayName: RTL_ALTITUDE should be a valid value
    // @Description: RTL_ALTITITUDE should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local RTLAltitudeLegal = Arming_Check(FIRMWARE.PLANE, rtl_altitude_legal, "RTL_ALT",
                        "RTL_ALTITUDE too high", MAV_SEVERITY.ERROR, false, false )
    table.insert(arming_checks, RTLAltitudeLegal)
--[[
    // @Param: ARM_P_QRTL_ALT
    // @DisplayName: Q_RTL_ALT should be a valid value
    // @Description: Q_RTL_ALT should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
--]]
    local QRTLAltitudeLegal = Arming_Check(FIRMWARE.PLANE, q_rtl_alt_legal, "QRTL_ALT",
                            "Q_RTL_ALT too high", MAV_SEVERITY.ERROR, false, false )
    table.insert(arming_checks, QRTLAltitudeLegal)

end

-- No need to actively check while armed.
local function idle_while_armed()
    if not arming:is_armed() then return validate, REFRESH_RATE end
    return idle_while_armed, REFRESH_RATE * 20
end

-- initialize all the parameters based on the data in the arming_checks table
local function initialize()
-- special case for the ARV_ALT_LEGAL parameter which ideally should be a "regular" parameter
--[[
    // @Param: ARM_V_ALT_LEGAL
    // @DisplayName: Legal max altitude
    // @Description: Legal max altitude for UAV/RPAS/drones in your jurisdiction
    // @Units: m
    // @User: Standard
--]]
    assert(param:add_param(VALUES_PARAM_TABLE_KEY, param_idx, "ALT_LEGAL", 120.0),
                        string.format('could not add param %s', VALUES_PARAM_TABLE_PREFIX.."ALT_LEGAL"))
    param_idx = param_idx + 1

-- special case for the ARV_RALLY_MAX parameter which ideally should be a "regular" parameter
--[[
    // @Param: ARM_V_RALLY_MAX
    // @DisplayName: Max distanct to rally
    // @Description: The maximum distance to a rally point from home
    // @Units: m
    // @User: Standard
--]]
    assert(param:add_param(VALUES_PARAM_TABLE_KEY, param_idx, "RALLY_MAX", 1000.0),
                        string.format('could not add param %s', VALUES_PARAM_TABLE_PREFIX.."RALLY_MAX"))
    param_idx = param_idx + 1
    ARM_V_RALLY_MAX:init(VALUES_PARAM_TABLE_PREFIX.."RALLY_MAX")

    for _, check in pairs(arming_checks) do
        local param_name = VEHICLE[check.firmware].prefix..check.param_name
        local index = VEHICLE[check.firmware].index
        assert(param:add_param(VEHICLE[check.firmware].key , index, check.param_name, check.severity),
                        string.format('could not add param %s', param_name))
        VEHICLE[check.firmware].index = index + 1
    end
end

-- run an indivual check
local function run_check(check, severity, validated)
    local check_passed = check:state() -- sets check.passed and check.changed
    local raise_prearm = not check_passed

    if check.changed then
        if check_passed then
            -- it's passed now, but changed, so previously failed. Let the user know its all clear
            gcs:send_text(MAV_SEVERITY.INFO, string.format('%s: clear: %s', SCRIPT_NAME_SHORT, check.text))
        elseif severity > MAV_SEVERITY.ERROR then
            -- deal with warning messages (MAV_SEVERITY > ERROR), we display a message but no prearm
            gcs:send_text(severity, string.format('%s: %s: %s', SCRIPT_NAME_SHORT,
                            MAV_SEVERITY_TEXT[severity], check.text))
            -- display the message, but this should not block arming so is actually "passed"
            raise_prearm = false
        end
    elseif not check_passed and severity > MAV_SEVERITY.ERROR then
        -- if nothing changed we only want to fail on errors
        raise_prearm = false
    end
    if raise_prearm then
        validated = false
        arming:set_aux_auth_failed(arm_auth_id, string.format('%s: %s: %s', SCRIPT_NAME_SHORT,
                                    MAV_SEVERITY_TEXT[severity], check.text))
    end
    return validated
end

validate = function() -- this is the loop which periodically runs to do the validations

    if arming:is_armed() then return idle_while_armed() end

    alt_legal_max = ARM_V_ALT_LEGAL:get() or 120.0
    local validated = true

    for _, check in pairs(arming_checks) do
        local param_name = VEHICLE[check.firmware].prefix..check.param_name
        local parameter = Parameter(param_name)
        local param_severity  = parameter:get() or MAV_SEVERITY.NONE
        if param_severity ~= MAV_SEVERITY.NONE then -- ignore if NONE
            validated = run_check(check, param_severity, validated)
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

initialize()
-- start running update loop - waiting 20s for the AP to initialize so we don't spam the user with spurios notices
return delayed_startup, INITIAL_DELAY
