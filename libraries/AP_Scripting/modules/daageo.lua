--[[
    Geometry and airframe-capability helpers for planedaa.lua.

    Pure mechanism: nothing here decides what to DO about an obstacle, it only answers
    questions about angles, locations, and what turn the aircraft is actually capable of.
    Kept out of the applet so the applet stays the one file an integrator edits to change
    avoidance policy - see planedaa.md.

    Stateless: every function is a pure function of its arguments, so this module is
    plain functions on DAAgeometry directly, requireable from anywhere with no
    instantiation or configure() step - the caller (daacore.lua, daaobs.lua, planedaa.lua)
    owns whatever state it needs (e.g. a cached roll_limit_deg) and passes it in. This was
    an instantiable class until 2026-09-03; roll_limit_deg was the only actual state, and
    passing it as a parameter to the two functions that use it removed the need for an
    instance, a configure() call, and injecting the result into three different files.
--]]

local DAAgeometry = {}

DAAgeometry.SCRIPT_VERSION = "4.8.0-003"
DAAgeometry.SCRIPT_NAME = "DAA geometry"
DAAgeometry.SCRIPT_NAME_SHORT = "DAAgeo"

-- Load-banner severity only - this module does no other logging, so no full severity
-- table is needed; MAV_SEVERITY.INFO is a fixed MAVLink wire value (6).
local BANNER_SEVERITY = 6

local GRAVITY_MSS = 9.80665

function DAAgeometry.wrap_360(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

function DAAgeometry.wrap_180(angle)
    local res = DAAgeometry.wrap_360(angle)
    if res > 180 then
        res = res - 360
    end
    return res
end

--[[
    return true if two locations are identical
--]]
function DAAgeometry.locations_equal(loc1, loc2)
    if loc1 == nil and loc2 == nil then
        return true
    end
    if (loc1 == nil and loc2 ~= nil) or (loc1 ~= nil and loc2 == nil) then
        return false
    end
    return (loc1:lat() == loc2:lat()) and (loc1:lng() == loc2:lng())
            and (loc1:alt() == loc2:alt())
            and (loc1:get_alt_frame() == loc2:get_alt_frame())
end

-- copy the altitude (value and frame) from src into dest. A small Lua helper on top of
-- the existing get_alt_m/set_alt_m bindings, so we don't carry a Location:copy_alt_from()
-- binding just for this (no C++ flash cost). Reading in src's own frame needs no conversion.
local function copy_alt_from(dest, src)
    local frame = src:get_alt_frame()
    -- get_alt_m returns the altitude in `frame` (or nil if it can't convert, e.g. no terrain).
    -- Reading in src's own frame needs no conversion, so this normally succeeds.
    local alt_m = src:get_alt_m(frame)
    if alt_m ~= nil then
        dest:set_alt_m(alt_m, frame)
    end
end

-- Project forward from loc1 to a newlocation in the direction bearing_deg and distance m
-- the altitude of the new projected location should be based on alt_target_loc, including frame
function DAAgeometry.location_project(loc1, bearing_deg, distance, alt_target_loc)
    -- Create a copy of the location projected distance meters in bearing_deg direction
    -- the projection should be in the frame project_in_frame
    local loc2 = loc1:copy()
    loc2:offset_bearing(bearing_deg, distance)
    copy_alt_from(loc2, alt_target_loc)

    return loc2
end

-- Maximum achievable rate of turn (deg/s) in a level banked turn at roll_limit_deg:
-- omega = g * tan(phi) / V.  Both the startup sanity checks and the course-change
-- projection call this, so the two can never disagree.  Returns 0 when there is no
-- usable speed or roll limit, and the caller decides what that means.
function DAAgeometry.max_turn_rate_dps(speed_ms, roll_limit_deg)
    if speed_ms == nil or speed_ms < 1.0 or roll_limit_deg == nil or roll_limit_deg <= 0 then
        return 0.0
    end
    return math.deg(GRAVITY_MSS * math.tan(math.rad(roll_limit_deg)) / speed_ms)
end

-- Radius of the tightest level turn available at roll_limit_deg:
-- R = V^2 / (g * tan(phi)).  This is what the aircraft can actually fly, and so what
-- decides whether it can turn away from something in time.  WP_LOITER_RAD is a commanded
-- loiter setting, not a capability: ArduPlane's own documentation notes that the achieved
-- loiter radius is determined by ROLL_LIMIT_DEG when WP_LOITER_RAD is small.  Returns 0
-- when there is no usable speed or roll limit, and the caller decides what that means.
function DAAgeometry.turn_radius_m(speed_ms, roll_limit_deg)
    if speed_ms == nil or speed_ms < 1.0 or roll_limit_deg == nil or roll_limit_deg <= 0 then
        return 0.0
    end
    return (speed_ms * speed_ms) / (GRAVITY_MSS * math.tan(math.rad(roll_limit_deg)))
end

-- Bound on ACHIEVED roll rate (deg/s), used to model how long a bank reversal actually
-- takes rather than assuming it is instantaneous.  RLL2SRV_RMAX is the airframe's own
-- configured cap when set; ArduPlane ships it at 0 ("rate limit disabled") by default, so
-- the zero case is the actual case that needs handling here, not an edge case.  The
-- fallback, roll_limit_deg / tconst_s, is the rate that would cover the full commanded
-- roll range in one roll-controller time constant - the same order-of-magnitude estimate
-- AP_RollController's own P gain uses when RLL2SRV_P is left at zero (P = 1/TCONST).  Like
-- RLL2SRV_RMAX itself, this bounds DEMANDED rate, not a guarantee of ACHIEVED rate - but a
-- real, speed-and-airframe-derived number is a large improvement over the instantaneous
-- (infinite-rate) assumption it replaces.  Returns 0 when nothing usable is configured,
-- and the caller decides what that means.
function DAAgeometry.roll_rate_dps(rmax_dps, roll_limit_deg, tconst_s)
    if rmax_dps ~= nil and rmax_dps > 0 then
        return rmax_dps
    end
    if roll_limit_deg == nil or roll_limit_deg <= 0 or tconst_s == nil or tconst_s <= 0 then
        return 0.0
    end
    return roll_limit_deg / tconst_s
end

-- Displace from_loc along a constant-bank turn held for duration_s seconds at roll_deg,
-- starting on heading start_heading_deg and turning in direction turn_sign (+1 right /
-- -1 left).  Returns (end_loc, end_heading_deg).  Same R*sin(th)/R*(1-cos(th)) chord
-- decomposition as a full course-change projection, but parameterised by an explicit TIME
-- and ROLL ANGLE rather than derived from the course change itself - the primitive a
-- bank-aware, multi-segment transition (current bank -> wings level -> target bank ->
-- constant-bank arc) is built from one segment at a time.  Degrades to straight flight for
-- the same duration when there is no usable turn rate (roll_deg too small, or no speed) or
-- no time to cover.
function DAAgeometry.arc_projection(from_loc, start_heading_deg, turn_sign, duration_s,
                                     airspeed_ms, ground_speed_ms, roll_deg, to_loc)
    if duration_s <= 0 or ground_speed_ms <= 0 then
        return from_loc, start_heading_deg
    end
    local arc_length_m     = ground_speed_ms * duration_s
    local rate_of_turn_dps = DAAgeometry.max_turn_rate_dps(airspeed_ms, roll_deg)
    if rate_of_turn_dps <= 0 then
        return DAAgeometry.location_project(from_loc, start_heading_deg, arc_length_m, to_loc), start_heading_deg
    end
    local turn_angle_deg = turn_sign * rate_of_turn_dps * duration_s
    local turn_rad        = math.rad(math.abs(turn_angle_deg))
    if turn_rad < 1e-6 then
        return DAAgeometry.location_project(from_loc, start_heading_deg, arc_length_m, to_loc), start_heading_deg
    end
    local radius_m = arc_length_m / turn_rad
    local side_deg  = (turn_angle_deg >= 0) and (start_heading_deg + 90) or (start_heading_deg - 90)
    local mid_loc   = DAAgeometry.location_project(from_loc, start_heading_deg, radius_m * math.sin(turn_rad), to_loc)
    local end_loc   = DAAgeometry.location_project(mid_loc, side_deg, radius_m * (1.0 - math.cos(turn_rad)), to_loc)
    return end_loc, DAAgeometry.wrap_360(start_heading_deg + turn_angle_deg)
end

--[[
    calculate what our ground speed would be in a given direction, using wind estimate
--]]
function DAAgeometry.effective_groundspeed(airspeed, bearing_deg, wind_dir_rad, wind_speed)
    -- Ensure airspeed is at least 1.0
    airspeed = math.max(airspeed, 1.0)
    -- Convert bearing to radians
    local bearing_rad = math.rad(bearing_deg)
    -- Calculate the angle between wind direction and bearing
    local temp = math.pi - (wind_dir_rad - bearing_rad)
    local dangle = wind_speed * math.sin(temp) / airspeed
    -- If dangle is out of valid range, return 0
    if dangle > 1.0 or dangle < -1.0 then
        return 0
    end
    -- Calculate the angle alpha using arcsine
    local alpha = math.asin(dangle)
    -- Calculate yaw
    local yaw = bearing_rad - alpha
    -- Calculate beta, the angle between wind direction and yaw
    local beta = math.pi - (wind_dir_rad - yaw)
    -- Calculate ground speed squared (gs2)
    local gs2 = airspeed^2 + wind_speed^2 - 2 * airspeed * wind_speed * math.cos(beta)
    -- If gs2 is negative or zero, return 0
    if gs2 <= 0 then
        return 0
    end
    -- Calculate the final ground speed
    local gs = math.sqrt(gs2)
    return gs
end

gcs:send_text(BANNER_SEVERITY, string.format("%s %s module loaded", DAAgeometry.SCRIPT_NAME, DAAgeometry.SCRIPT_VERSION))

return DAAgeometry
