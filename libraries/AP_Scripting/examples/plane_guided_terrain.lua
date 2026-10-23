--[[

   example script to show reseting guided mode to terrain height.
   It's intended to be used when selecting "fly to altitude" in a ground station 
   for example by right clicking on the map in QGC. 
   
   This is actually a workaround to QGC and Mission Planner not having a
   way to set guided altitude above terrain.

   Depending on ZGP_MODE
   When the GCS requests a guided altitude X above home 
    1: reset to current terrain height ignore X 
    2: reset to X above terrain 
    3: reset to current alt + X
    
   this functionality is only available in Plane and 
   requires TERRAIN_ENABLE = 1 and TERRAIN_FOLLOW =1
--]]

SCRIPT_NAME = "OverheadIntel Guided Terrain"
SCRIPT_NAME_SHORT = "TerrGuided"
SCRIPT_VERSION = "4.6.0-005"

REFRESH_RATE = 0.2   -- in seconds, so 5Hz
ALTITUDE_MIN = 50
ALTITUDE_MAX = 120

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = { GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3}
ALT_FRAME = { ABSOLUTE = 0, ABOVE_HOME = 1, ABOVE_ORIGIN = 2, ABOVE_TERRAIN = 3 }
FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

PARAM_TABLE_KEY = 101
PARAM_TABLE_PREFIX = "ZGT_"

local now = millis():tofloat() * 0.001

-- bind a parameter to a variable
function bind_param(name)
    return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), SCRIPT_NAME_SHORT .. 'could not add param table: ' .. PARAM_TABLE_PREFIX)

--[[
    // @Param: ZGT_MODE
    // @DisplayName: Guided Terrain Mode
    // @Description: When the GCS requests a guided altitude X above home 1: reset to current terrain height ignore X 2: reset to X above terrain 3: reset to current alt + X
    // @Range: 1,2,3
--]]
ZGT_MODE = bind_add_param("MODE", 1, 1)
TERRAIN_ENABLE = bind_param("TERRAIN_ENABLE")
TERRAIN_FOLLOW = bind_param("TERRAIN_FOLLOW")

local zgt_mode = ZGT_MODE:get()
local terrain_enable = TERRAIN_ENABLE:get()
local terrain_follow = TERRAIN_FOLLOW:get()

MAV_CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_FRAME = { GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3, GLOBAL_TERRAIN_ALT = 10}

-- constrain a value between limits
local function constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
end

local function follow_frame_to_mavlink(follow_frame)
    local mavlink_frame = MAV_FRAME.GLOBAL
    if (follow_frame == ALT_FRAME.ABOVE_TERRAIN) then
        mavlink_frame = MAV_FRAME.GLOBAL_TERRAIN_ALT
    elseif (follow_frame == ALT_FRAME.ABOVE_HOME) then
       mavlink_frame = MAV_FRAME.GLOBAL_RELATIVE_ALT
    end
    return mavlink_frame
 end

local now_altitude = millis():tofloat() * 0.001
-- target.alt = new target altitude in meters
-- set_vehicle_target_altitude() Parameters
-- target.frame = Altitude frame MAV_FRAME, it's very important to get this right!
-- target.alt = altitude in meters to achieve
-- target.accel = z acceleration to altitude (1000.0 = max)
local function set_vehicle_target_altitude(target)
   local acceleration = target.accel or 1000.0 -- default to maximum z acceleration
   if math.floor(now) ~= math.floor(now_altitude) then
      now_altitude = millis():tofloat() * 0.001
   end
   if target.alt == nil then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": set_vehicle_target_altitude no altiude")
      return
   end
   -- GUIDED_CHANGE_ALTITUDE takes altitude in meters
   local mavlink_result = gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_ALTITUDE, {
                              frame = follow_frame_to_mavlink(target.frame),
                              p3 = acceleration,
                              z = target.alt })
    if mavlink_result > 0 then
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(": MAVLink GUIDED_CHANGE_ALTITUDE returned %d", mavlink_result))
        return false
    else
        return true
    end
end

local vehicle_mode = vehicle:get_mode()
local save_target_altitude = -1
local save_old_target_altitude = -1
local save_zgt_mode = -1
local save_vehicle_mode = -1

local function update()

    vehicle_mode = vehicle:get_mode()

    terrain_enable = TERRAIN_ENABLE:get()
    terrain_follow = TERRAIN_FOLLOW:get()
    zgt_mode = ZGT_MODE:get()
    if zgt_mode ~= save_zgt_mode then
        -- user changed modes, reset everything
        save_target_altitude = -1
        save_old_target_altitude = -1
    end
    save_zgt_mode = zgt_mode

    -- We should only reset the altitude if newly switched to guided mode
    if save_vehicle_mode ~= vehicle_mode and vehicle_mode == FLIGHT_MODE.GUIDED and terrain_enable == 1 and
        ((terrain_follow & 1) == 1 or (terrain_follow & (1 << 6)) == 64) then
        local target_location = vehicle:get_target_location()
        if target_location ~= nil then
            local target_location_frame = target_location:get_alt_frame()
            -- need to convert the target_location to ABOVE_TERRAIN so we have apples and apples
            local new_target_location = target_location:copy()
            local new_target_altitude = save_target_altitude
            local old_target_altitude = new_target_location:alt()/100

            if save_old_target_altitude ~= old_target_altitude then
                if target_location_frame ~= ALT_FRAME.ABOVE_TERRAIN then
                    if new_target_location:change_alt_frame(ALT_FRAME.ABOVE_TERRAIN) then
                        old_target_altitude = new_target_location:alt()/100
                    end
                end
                -- adjust target_location for home altitude
                local home_location = ahrs:get_home()
                if home_location ~= nil then
                    local home_amsl = terrain:height_amsl(home_location, true)
                    local above_home = (target_location:alt() * 0.01 - home_amsl)
                    local location = ahrs:get_location()
                    if location ~= nil then
                        if location:get_alt_frame() ~= ALT_FRAME.ABOVE_TERRAIN then
                            location:change_alt_frame(ALT_FRAME.ABOVE_TERRAIN)
                        end
                        local current_altitude = location:alt() * 0.01
                        -- @Description: When the GCS requests a guided altitude X above home
                        -- 1: reset to current terrain height ignore X
                        -- 2: reset to X above terrain
                        -- 3: reset to current alt + X
                        if zgt_mode == 1 then
                            new_target_altitude = current_altitude
                        elseif zgt_mode == 2 then
                            new_target_altitude = above_home
                        elseif zgt_mode == 3 then
                            new_target_altitude = current_altitude + above_home
                        end
                        new_target_altitude = constrain(new_target_altitude, ALTITUDE_MIN, ALTITUDE_MAX)
                end
                else
                    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: FAILED to get HOME terrain height", SCRIPT_NAME_SHORT))
                end

                if new_target_altitude > 0 and
                   set_vehicle_target_altitude({alt = new_target_altitude, frame = ALT_FRAME.ABOVE_TERRAIN}) then -- pass altitude in meters (location has it in cm)
                    if new_target_altitude ~= save_target_altitude then
                        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s: Reset to alt %.0fm above terrain", SCRIPT_NAME_SHORT,
                                    new_target_altitude
                            ))
                        save_target_altitude = new_target_altitude
                    end
                else
                    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: FAILED to set altitude ABOVE_TERRAIN ait %.0f", SCRIPT_NAME_SHORT,
                                new_target_altitude
                        ))
                end
            end
            save_old_target_altitude = old_target_altitude
        else
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: altitude not available", SCRIPT_NAME_SHORT))
        end
    else
        -- we switched out of guided, so forget what we thought we knew
        save_target_altitude = -1
        save_old_target_altitude = -1
    end
    save_vehicle_mode = vehicle_mode

   return update, 1000 * REFRESH_RATE
end

-- wrapper around update(). This calls update() at 1/REFRESHRATE Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 * REFRESH_RATE
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )

-- start running update loop
if FWVersion:type() == 3 and terrain_enable then
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return protected_wrapper()
else
    gcs:send_text(MAV_SEVERITY.NOTICE,string.format("%s: Must run on Plane with terrain follow", SCRIPT_NAME_SHORT))
end
