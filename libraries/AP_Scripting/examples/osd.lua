--[[
   OSD scripting example

   This example demonstrates:
   1. Taking control of the OSD display from a Lua script
   2. Using OSD special symbols/characters
   3. Calculating and displaying a direction arrow to the next waypoint
--]]

------------------------------------------------------------
-- OSD Symbol Definitions
-- Symbol codes differ between OSD backends:
--   OSD_TYPE 1 = MAX7456 (analog OSD chip)
--   OSD_TYPE 2 = SITL (uses MAX7456 font)
--   OSD_TYPE 5 = MSP DisplayPort (digital HDZero, Walksnail, DJI)
-- The script auto-detects the backend and uses appropriate symbols
------------------------------------------------------------

-- MAX7456 symbol codes (used by OSD_TYPE 1 and 2)
local SYM_MAX7456 = {
    M           = 0xB9, KM          = 0xBA, FT          = 0x0F, MI          = 0xBB,
    ALT_M       = 0xB1, ALT_FT      = 0xB3, BATT_FULL   = 0x90, RSSI        = 0x01,
    VOLT        = 0x06, AMP         = 0x9A, MAH         = 0x07, MS          = 0x9F,
    FS          = 0x99, KMH         = 0xA1, MPH         = 0xB0, DEGR        = 0xA8,
    PCNT        = 0x25, RPM         = 0xE0, ASPD        = 0xE1, GSPD        = 0xE2,
    WSPD        = 0xE3, VSPD        = 0xE4, WPNO        = 0xE5, WPDIR       = 0xE6,
    WPDST       = 0xE7, SAT_L       = 0x1E, SAT_R       = 0x1F, HDOP_L      = 0xBD,
    HDOP_R      = 0xBE, GPS_LAT     = 0xA6, GPS_LONG    = 0xA7, HOME        = 0xBF,
    WIND        = 0x16, DIST        = 0x22, NM          = 0xF1, ARROW_START = 0x60,
    ARROW_COUNT = 16,   ARROW_RIGHT = 0xFD, ARROW_LEFT  = 0xFE, AH_H_START  = 0x80,
    AH_H_COUNT  = 9,    AH_V_START  = 0xCA, AH_V_COUNT  = 6,    AH_CENTER_LINE_LEFT = 0x26,
    AH_CENTER_LINE_RIGHT = 0x27, AH_CENTER = 0x7E, AH = 0xF3, HEADING_N   = 0x18,
    HEADING_S   = 0x19, HEADING_E   = 0x1A, HEADING_W   = 0x1B, HEADING_DIVIDED_LINE = 0x1C,
    HEADING_LINE = 0x1D, HEADING    = 0x89, UP_UP       = 0xA2, UP          = 0xA3,
    DOWN        = 0xA4, DOWN_DOWN   = 0xA5, DEGREES_C   = 0x0E, DEGREES_F   = 0x0D,
    ARMED       = 0x00, DISARMED    = 0xE9, ROLL0       = 0x2D, ROLLR       = 0xEA,
    ROLLL       = 0xEB, PTCH0       = 0x7C, PTCHUP      = 0xEC, PTCHDWN     = 0xED,
    ROLL        = 0xA9, PITCH       = 0xAF, XERR        = 0xEE, FLY         = 0x9C,
    EFF         = 0xF2, MW          = 0xF4, CLK         = 0xBC, KILO        = 0x4B,
    TERALT      = 0xEF, FENCE_ENABLED = 0xF5, FENCE_DISABLED = 0xF6, RNGFD = 0xF7,
    LQ          = 0xF8, WATT        = 0xAE, WH          = 0xAB, BATT_UNKNOWN = 0x97,
    DPS         = 0xAA, G           = 0xDF, KN          = 0xF0, FTMIN       = 0xE8,
    FTSEC       = 0x99, RADIUS      = 0x7A, FLAP        = 0x23, SIDEBAR_R_ARROW = 0x09,
    SIDEBAR_L_ARROW = 0x0A, SIDEBAR_A = 0x13, SIDEBAR_B = 0x14, SIDEBAR_C = 0x15,
    SIDEBAR_D   = 0xDD, SIDEBAR_E   = 0xDB, SIDEBAR_F   = 0xDC, SIDEBAR_G   = 0xDA,
    SIDEBAR_H   = 0xDE, SIDEBAR_I   = 0x11, SIDEBAR_J   = 0x12, DB          = 0xF9,
    DBM         = 0xFA, SNR         = 0xFB, ANT         = 0xFC,
}

-- MSP DisplayPort symbol codes (used by OSD_TYPE 5)
local SYM_MSP = {
    M           = 0x0C, KM          = 0x7D, FT          = 0x0F, MI          = 0x7E,
    ALT_M       = 0x0C, ALT_FT      = 0x0F, BATT_FULL   = 0x90, RSSI        = 0x01,
    VOLT        = 0x06, AMP         = 0x9A, MAH         = 0x07, MS          = 0x9F,
    FS          = 0x99, KMH         = 0x9E, MPH         = 0x9D, DEGR        = 0x08,
    PCNT        = 0x25, RPM         = 0x12, ASPD        = 0x41, GSPD        = 0x47,
    WSPD        = 0x57, VSPD        = 0x5E, WPNO        = 0x23, WPDIR       = 0xE6,
    WPDST       = 0xE7, SAT_L       = 0x1E, SAT_R       = 0x1F, HDOP_L      = 0x48,
    HDOP_R      = 0x44, GPS_LAT     = 0x89, GPS_LONG    = 0x98, HOME        = 0x11,
    WIND        = 0x57, DIST        = 0x04, NM          = 0xF1, ARROW_START = 0x60,
    ARROW_COUNT = 16,   ARROW_RIGHT = 0xFD, ARROW_LEFT  = 0xFE, AH_H_START  = 0x80,
    AH_H_COUNT  = 9,    AH_V_START  = 0x82, AH_V_COUNT  = 6,    AH_CENTER_LINE_LEFT = 0x84,
    AH_CENTER_LINE_RIGHT = 0x84, AH_CENTER = 0x2B, AH = 0xF3, HEADING_N   = 0x18,
    HEADING_S   = 0x19, HEADING_E   = 0x1A, HEADING_W   = 0x1B, HEADING_DIVIDED_LINE = 0x1C,
    HEADING_LINE = 0x1D, HEADING    = 0x89, UP_UP       = 0x68, UP          = 0x68,
    DOWN        = 0x60, DOWN_DOWN   = 0x60, DEGREES_C   = 0x0E, DEGREES_F   = 0x0D,
    ARMED       = 0x00, DISARMED    = 0x2A, ROLL0       = 0x2D, ROLLR       = 0x64,
    ROLLL       = 0x6C, PTCH0       = 0x7C, PTCHUP      = 0x68, PTCHDWN     = 0x60,
    ROLL        = 0xA9, PITCH       = 0xAF, XERR        = 0x21, FLY         = 0x9C,
    EFF         = 0xF2, MW          = 0xF4, CLK         = 0x08, KILO        = 0x4B,
    TERALT      = 0x7F, FENCE_ENABLED = 0xF5, FENCE_DISABLED = 0xF6, RNGFD = 0x7F,
    LQ          = 0xF8, WATT        = 0xAE, WH          = 0xAB, BATT_UNKNOWN = 0x97,
    DPS         = 0xAA, G           = 0xDF, KN          = 0xF0, FTMIN       = 0xE8,
    FTSEC       = 0x99, RADIUS      = 0x7A, FLAP        = 0x23, SIDEBAR_R_ARROW = 0x03,
    SIDEBAR_L_ARROW = 0x02, SIDEBAR_A = 0x13, SIDEBAR_B = 0x13, SIDEBAR_C = 0x13,
    SIDEBAR_D   = 0x13, SIDEBAR_E   = 0x13, SIDEBAR_F   = 0x13, SIDEBAR_G   = 0x13,
    SIDEBAR_H   = 0x13, SIDEBAR_I   = 0x13, SIDEBAR_J   = 0x13, DB          = 0xF9,
    DBM         = 0xFA, SNR         = 0xFB, ANT         = 0xFC,
}

-- Select symbol table based on OSD_TYPE parameter
-- OSD_TYPE: 1=MAX7456, 2=SITL, 3=MSP, 4=TXOnly, 5=MSP_DisplayPort
local osd_type = param:get("OSD_TYPE")
local SYM
if osd_type == 5 then
    SYM = SYM_MSP
    gcs:send_text(6, "OSD script: using MSP DisplayPort symbols")
else
    SYM = SYM_MAX7456
    gcs:send_text(6, "OSD script: using MAX7456 symbols")
end

------------------------------------------------------------
-- Helper function to get arrow character for a given angle
-- angle_deg: direction in degrees (0 = North, 90 = East, etc)
-- Returns the appropriate arrow character from the 16-direction set
------------------------------------------------------------
local function get_arrow_char(angle_deg)
    -- Normalize angle to 0-360
    angle_deg = angle_deg % 360
    if angle_deg < 0 then
        angle_deg = angle_deg + 360
    end

    -- 16 arrows, each covers 22.5 degrees
    -- Add 11.25 (half interval) for proper rounding
    local arrow_index = math.floor((angle_deg + 11.25) / 22.5) % 16

    return string.char(SYM.ARROW_START + arrow_index)
end

------------------------------------------------------------
-- Helper to convert symbol index to character
------------------------------------------------------------
local function sym(index)
    return string.char(index)
end

------------------------------------------------------------
-- Calculate bearing between two locations
-- Returns bearing in degrees (0-360, 0 = North)
------------------------------------------------------------
local function get_bearing_deg(loc1, loc2)
    if not loc1 or not loc2 then
        return nil
    end
    -- get_bearing returns centidegrees
    return loc1:get_bearing(loc2) * 0.01
end

------------------------------------------------------------
-- Check OSD availability
------------------------------------------------------------
-- The osd binding exists but may have no backend available at runtime
-- (e.g., when SITL is not built with --enable-sfml --sitl-osd)
-- We test availability by attempting a method call with pcall
if not osd then
    gcs:send_text(6, "OSD example: osd not available")
    return
end

-- Test if the OSD backend is actually available by calling a method
local osd_available = pcall(function() return osd:get_screen() end)
if not osd_available then
    gcs:send_text(6, "OSD example: osd not available")
    return
end

------------------------------------------------------------
-- Main update function
------------------------------------------------------------
local function update()
    -- Clear the OSD
    osd:clear()

    -- Display the standard ArduPilot OSD screen as base
    osd:draw_screen()

    -- Get current vehicle location and heading
    local current_loc = ahrs:get_location()
    local heading_deg = ahrs:get_yaw_rad() * 57.2958  -- radians to degrees

    -- Only show waypoint info if we have a mission loaded and are in AUTO mode
    local dominated_by_vehicle_mode = function()
        local mode = vehicle:get_mode()
        -- AUTO mode is typically 3 for copter, check if navigating
        return mode == 3  -- AUTO
    end

    local has_mission = mission and mission:num_commands() > 1

    if has_mission and dominated_by_vehicle_mode() then
        -- Get next waypoint location
        local wp_loc = vehicle:get_target_location()

        -- Display waypoint direction arrow if we have valid positions
        if current_loc and wp_loc then
            local dist_m = current_loc:get_distance(wp_loc)

            -- Only show if distance is reasonable (< 100km)
            if dist_m < 100000 then
                -- Calculate absolute bearing to waypoint
                local bearing_to_wp = get_bearing_deg(current_loc, wp_loc)

                if bearing_to_wp then
                    -- Calculate relative bearing (direction to turn)
                    local relative_bearing = bearing_to_wp - heading_deg

                    -- Get the arrow character for this relative bearing
                    local arrow = get_arrow_char(relative_bearing)

                    -- Display waypoint info in center, below status text (row 8)
                    -- Format: [WP icon] [arrow] [distance]m
                    osd:write(12, 9, sym(SYM.WPDIR) .. arrow .. string.format(" %d", math.floor(dist_m)) .. sym(SYM.M))
                end
            end
        end
    else
        -- No active waypoint navigation, show home direction instead if available
        local home_loc = ahrs:get_home()
        if current_loc and home_loc then
            local dist_m = current_loc:get_distance(home_loc)

            -- Only show if distance is reasonable (< 100km) and we have a valid home
            if dist_m < 100000 and dist_m > 0 then
                local bearing_to_home = get_bearing_deg(current_loc, home_loc)
                if bearing_to_home then
                    local relative_bearing = bearing_to_home - heading_deg
                    local arrow = get_arrow_char(relative_bearing)

                    osd:write(12, 9, sym(SYM.HOME) .. arrow .. string.format(" %d", math.floor(dist_m)) .. sym(SYM.M))
                end
            end
        end
    end

    -- Show some symbol examples at bottom of screen
    local example_row = 13

    -- Armed/disarmed status
    if arming:is_armed() then
        osd:write(1, example_row, sym(SYM.ARMED) .. " ARMED")
    else
        osd:write(1, example_row, sym(SYM.DISARMED) .. " DISARMED")
    end

    -- GPS satellite count with icon
    local gps_sats = gps:num_sats(0)
    osd:write(15, example_row, sym(SYM.SAT_L) .. sym(SYM.SAT_R) .. string.format("%d", gps_sats))

    -- Battery voltage with icon
    local voltage = battery:voltage(0)
    if voltage then
        osd:write(22, example_row, sym(SYM.VOLT) .. string.format("%.1f", voltage))
    end

    -- Flush the display
    osd:flush()

    return update, 100
end

return update()
