--[[
    Obstacle identification and lookup for planedaa.lua.

    Pure mechanism: this answers "what is out there, and how far" - it makes no decision
    about what to do in response.  The applet keeps that, so an integrator adding their own
    avoidance action only has to edit planedaa.lua (see planedaa.md).

    daageo is stateless (see that file), so it is required directly rather than injected -
    only location_project() is needed here, and it takes no configured state.  Everything
    else the applet knows is pushed in rather than reached for:
      configure()    - cached parameter values and the OBSTACLE_TYPE / ADSB_EMITTER enums,
                       refreshed with the applet's other parameters
      update_state() - the current vehicle location, once per cycle
    Both land in instance locals rather than fields, because find_closest_obstacle() runs on
    every candidate-heading probe - well over a hundred times per cycle in a full sweep.
--]]

local DAAobstacles = {}

DAAobstacles.SCRIPT_VERSION = "4.8.0-008"
DAAobstacles.SCRIPT_NAME = "DAA obstacles"
DAAobstacles.SCRIPT_NAME_SHORT = "DAAobs"

-- Load-banner severity only - this module does no other logging, so no full severity
-- table is needed; MAV_SEVERITY.INFO is a fixed MAVLink wire value (6).
local BANNER_SEVERITY = 6

local location_project = require("daageo").location_project

-- The obstacle taxonomy this module classifies against.  Owned here rather than injected:
-- AP_OAScripting returns these values, and this is the code that interprets them.
DAAobstacles.OBSTACLE_TYPE = {
    GENERAL                     = 0,    -- generic obstacle, we don't really know what it is
    MAV_SYSID                   = 1,    -- another MAVLINK drone with a MAV_SYSID
    CREWED_AIRCRAFT             = 2,
    -- 3, 4 and 5 were WEATHER, BIRD_MIGRATORY and BIRD_OF_PREY; removed, but the numbering
    -- is left alone because it is what DAAD.ObjT means in every log already recorded.    -- crewed aircraft, usually with an ICAO ADSB identifier
    FENCE_HOME                  = 3,    -- all fixed/unmovable fences
    FENCE_CIRCLE_INCLUSION      = 4,
    FENCE_CIRCLE_EXCLUSION      = 5,
    FENCE_POLYGON_INCLUSION     = 6,
    FENCE_POLYGON_EXCLUSION     = 7,
    FENCE_LUA                   = 8,
    PROXIMITY                   = 9,   -- detected by a proximty sensor, typically quite close
    AIS                         = 10,   -- Automatic Identification System for ship (maritime) vehicles
    FENCE_ALT_MAX               = 11,   -- max altitude fence (AC_FENCE_TYPE_ALT_MAX, FENCE_TYPE bit 0)
    FENCE_ALT_MIN               = 12,   -- min altitude fence (AC_FENCE_TYPE_ALT_MIN, FENCE_TYPE bit 3)
}

-- true for the horizontal fence obstacle types.  A fence is a boundary rather than a
-- point, so it carries no usable location and its real range has to come from
-- OAScripting:fence_distance().  The altitude fences (FENCE_ALT_MAX/MIN) are handled
-- separately and are deliberately not in this list - callers that also need to route
-- those (e.g. daacore.lua's resolver dispatch) OR them in explicitly.  The single
-- source of truth for "is this a fence" - daacore.lua and planedaa.lua both used to
-- carry their own copy of this list.
function DAAobstacles.is_fence_obstacle(obstacle_type)
    return obstacle_type == DAAobstacles.OBSTACLE_TYPE.FENCE_HOME
        or obstacle_type == DAAobstacles.OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION
        or obstacle_type == DAAobstacles.OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION
        or obstacle_type == DAAobstacles.OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION
        or obstacle_type == DAAobstacles.OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION
        or obstacle_type == DAAobstacles.OBSTACLE_TYPE.FENCE_LUA
end

-- ADS-B emitter categories (MAV_ADSB_EMITTER_TYPE), used only to label a contact.
DAAobstacles.ADSB_EMITTER = {
    NO_INFO                     = 0,
    LIGHT                       = 1,
    SMALL                       = 2,
    LARGE                       = 3,
    HIGH_VORTEX_LARGE           = 4,
    HEAVY                       = 5,
    HIGHLY_MANUV                = 6,
    ROTOCRAFT                   = 7,    -- this is Helicopter not a drone
    -- 8 Unassigned
    GLIDER                      = 9,
    LIGHTER_AIR                 = 10,
    PARACHUTE                   = 11,
    ULTRA_LIGHT                 = 12,
    AIRCRAFT_HIGH               = 13,
    UAV                         = 14,   -- this is drones
    SPACE                       = 15,   -- this is rockets
    --16 Unassigned

    -- Surface types
    EMERGENCY_SURFACE           = 17,
    SERVICE_SURFACE             = 18,

    -- Obstacle types
    POINT_OBSTACLE              = 19,
    CLUSTER_OBSTACLE            = 20,
    LINE_OBSTACLE               = 21,
    -- 22 - 39 Reserved

}


local FLT_MAX = 3.402823466e+38

function DAAobstacles.new()
    local self = {}

    local OBSTACLE_TYPE = DAAobstacles.OBSTACLE_TYPE
    local ADSB_EMITTER  = DAAobstacles.ADSB_EMITTER

    -- pushed in by configure().  Only the three that are read OUTSIDE configure() are kept
    -- as locals; every other margin exists solely to build the two lookup tables below, so
    -- it is read straight from the settings table and never becomes an upvalue.
    local well_clear_xy
    local wind_min_ms, wind_margin_per_ms
    local ground_alt_cm, ground_speed_ms
    -- pushed in by update_state()
    local current_loc
    local home_alt_cm

    -- Per-obstacle-type lookups rebuilt by configure().  These replace two long if-chains -
    -- one index instead of up to eight comparisons on every probe - and the table itself is
    -- the statement of which margin applies to what.
    local standoff_by_type        = {}   -- CPA keep-out radius, see get_standoff()
    local detect_margin_by_type   = {}   -- extra detection margin, see find_closest_obstacle()

    local function configure(settings)
        well_clear_xy        = settings.well_clear_xy
        wind_min_ms          = settings.wind_min_ms
        wind_margin_per_ms   = settings.wind_margin_per_ms
        ground_alt_cm        = settings.ground_alt_m * 100
        ground_speed_ms      = settings.ground_speed_ms

        standoff_by_type = {
            [OBSTACLE_TYPE.MAV_SYSID]               = settings.uav_clear_xy,   -- drone/UAV: AVD_UAV_XY
            [OBSTACLE_TYPE.PROXIMITY]               = settings.margin_proximity_m,
        }
        detect_margin_by_type = {
            [OBSTACLE_TYPE.CREWED_AIRCRAFT]         = settings.margin_crewed_m,
            [OBSTACLE_TYPE.MAV_SYSID]               = settings.margin_uav_m,
            [OBSTACLE_TYPE.AIS]                     = well_clear_xy + settings.margin_ais_m,
            [OBSTACLE_TYPE.PROXIMITY]               = settings.margin_proximity_m,
            [OBSTACLE_TYPE.FENCE_HOME]              = settings.margin_fence_m,
            [OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION]  = settings.margin_fence_m,
            [OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION]  = settings.margin_fence_m,
            [OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION] = settings.margin_fence_m,
            [OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION] = settings.margin_fence_m,
            [OBSTACLE_TYPE.FENCE_LUA]               = settings.margin_fence_m,
        }
    end

    -- the live vehicle position, pushed once per cycle: populate_obstacle() and
    -- obstacle_report_distance() report ranges from the aircraft, not from the probe origin.
    -- home_alt_cm is refreshed here too - once per cycle, not per probe - for
    -- is_grounded_traffic() below, which classify_threat() calls on every candidate.
    local function update_state(loc)
        current_loc  = loc
        home_alt_cm  = ahrs:get_home():alt()
    end

    -- True if a traffic contact (crewed aircraft or MAVLink drone) is parked/taxiing rather
    -- than flying: its own altitude is within DAA_GND_ALT_M of home AND its groundspeed is
    -- below DAA_GND_SPD_MS. Both AP_Avoidance obstacle sources (ADS-B and GLOBAL_POSITION_INT)
    -- populate location in Location::AltFrame::ABSOLUTE, same frame as home, so this is a plain
    -- subtraction - no frame conversion, no extra allocation. Fences/AIS/proximity are never
    -- "grounded" in this sense, so only the two traffic types are checked.
    --
    -- Without this, a stationary aircraft near the runway/pad reads as an airborne near-miss:
    -- avoidance forces a mode change away, which brings the vehicle straight back toward home
    -- and the same contact, re-triggers, and the vehicle never completes a landing or takeoff.
    local function is_grounded_traffic(obstacle_type, any_obstacle)
        if ground_alt_cm <= 0 then
            return false  -- DAA_GND_ALT_M = 0 disables the exemption entirely
        end
        if obstacle_type ~= OBSTACLE_TYPE.CREWED_AIRCRAFT and obstacle_type ~= OBSTACLE_TYPE.MAV_SYSID then
            return false
        end
        local loc = any_obstacle:location()
        if loc == nil or math.abs(loc:alt() - home_alt_cm) > ground_alt_cm then
            return false
        end
        local vel = any_obstacle:velocity_NED_ms()
        local ground_speed = math.sqrt(vel:x() * vel:x() + vel:y() * vel:y())
        return ground_speed <= ground_speed_ms
    end

    -- make obstacle labels a bit more meaningful for user especially for crewed aircraft and MAVLink vehicles
    local function pretty_label(script_obstacle)
        local obstacle_type = script_obstacle:obstacle_type()
        local emitter_type  = script_obstacle:emitter_type()

        -- a MAVLink drone (GLOBAL_POSITION_INT/FOLLOW_TARGET) carries a small MAV system id in
        -- src_id; an ADSB-sourced drone (emitter 14) carries a 24-bit ICAO address there instead,
        -- so show that in hex rather than a meaningless decimal "SYSID" (0xBFFF matches the C++ split)
        if emitter_type == ADSB_EMITTER.UAV then
            if script_obstacle:src_id() > 0xBFFF then
                return string.format("Drone:%06X", script_obstacle:icao_code())
            end
            return string.format("SYSID:%d", script_obstacle:src_id())

        -- this will have arrived as an ADSB_VEHICLE
        elseif obstacle_type == OBSTACLE_TYPE.CREWED_AIRCRAFT or emitter_type == 100 then
            return string.format("%06X", script_obstacle:icao_code())

        -- fake generated obstacles from mavproxy_genobstacles have these special case "emitters" for SITL/testing
        elseif emitter_type == 99 then
            return "Obstacle"
        elseif emitter_type == 101 then
            return "Drone"

        -- these obstacle types are returned by AP_OAScripting for fences
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
            return "Excl. Circle"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
            return "Incl. Circle"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION then
            return "Excl. Polygon"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION then
            return "Incl. Polygon"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_HOME then
            return "Tin Can"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_LUA then
            return "Lua Fence"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MAX then
            return "Alt Max Fence"
        elseif obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MIN then
            return "Alt Min Fence"
        end
        return "Unknown"
    end

    local function pretty_obstacle_type(type, src_id)
        if type == OBSTACLE_TYPE.GENERAL then
            return "general"
        end
        if type == OBSTACLE_TYPE.MAV_SYSID then
            -- a small src_id is a real MAVLink drone; a 24-bit ICAO (>0xBFFF) is an
            -- ADS-B drone (matches the label split in pretty_label)
            if src_id ~= nil and src_id > 0xBFFF then
                return "adsbdrone"
            end
            return "mavdrone"
        end
        if type == OBSTACLE_TYPE.CREWED_AIRCRAFT then
            return "aircraft"
        end
        if type == OBSTACLE_TYPE.FENCE_HOME then
            return "fence:home"
        end
        if type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
            return "fence:circle-inc"
        end
        if type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
            return "fence:circle-exc"
        end
        if type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION then
            return "fence:poly-inc"
        end
        if type == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION then
            return "fence:poly-exc"
        end
        if type == OBSTACLE_TYPE.FENCE_LUA then
            return "fence:lua"
        end
        if type == OBSTACLE_TYPE.FENCE_ALT_MAX then
            return "fence:alt-max"
        end
        if type == OBSTACLE_TYPE.FENCE_ALT_MIN then
            return "fence:alt-min"
        end
        if type == OBSTACLE_TYPE.PROXIMITY then
            return "proximity"
        end
        if type == OBSTACLE_TYPE.AIS then
            return "ship"
        end
        return "unknown"
    end

    local is_fence_obstacle = DAAobstacles.is_fence_obstacle

    local function populate_obstacle(distance_m, any_obstacle)
        local obstacle = {}
        obstacle.distance_m   = distance_m                      -- this is the Projected distance based on lookahead
        obstacle.sysid        = any_obstacle:src_id()
        obstacle.icao_code    = any_obstacle:icao_code()
        obstacle.type         = any_obstacle:obstacle_type()
        obstacle.timestamp_ms = any_obstacle:timestamp_ms()    -- last update time (millis); stale = laggy feed
        obstacle.label        = pretty_label(any_obstacle)
        obstacle.location     = any_obstacle:location()
        obstacle.pos_NED_m    = any_obstacle:position_NED_m()
        obstacle.vel_NED_ms   = any_obstacle:velocity_NED_ms()
        -- these are the actual distances based on current location with no lookahead.
        -- A fence is a boundary, not a point: C++ leaves its location un-set, and the binding
        -- hands back a Location userdata regardless (never nil), so a range computed from it
        -- would be a range to lat/lng 0,0.  Fall back to the bendy-ruler distance for every
        -- fence type; obstacle_report_distance() asks C++ for the real edge distance when a
        -- fence has to be reported to the pilot.
        if obstacle.location == nil or is_fence_obstacle(obstacle.type) then
            obstacle.distance_xy = obstacle.distance_m
            obstacle.distance_z  = 0
        else
            obstacle.distance_xy  = obstacle.location:get_distance(current_loc)
            obstacle.distance_z   = math.abs(obstacle.location:get_distance_NED(current_loc):z())
        end

        return obstacle
    end

    -- Real current horizontal distance (m) to the physical obstacle, for the AVOIDING message.
    -- distance_m is the bendy-ruler PROJECTED distance (along the lookahead ray), not a real range.
    -- Returns nil when we can't determine it simply (Lua fence, alt fence, no location) so the
    -- message omits the distance rather than mislead.  Called ONLY at announce time (not in the
    -- bendy-ruler sweep) so the per-call fence_distance loop stays out of the hot path.
    local function obstacle_report_distance(obstacle)
        if current_loc == nil then return nil end
        local t = obstacle.type
        if is_fence_obstacle(t) then
            -- horizontal fences carry no single usable "location"; ask C++ for the real edge distance
            -- to a fence of THIS type (so an "Excl. Circle" label reports the nearest exclusion
            -- circle, not a nearer polygon).  SIGNED - positive while clear, negative if this fence
            -- is already breached (rare, but then a true negative distance beats a misleading
            -- positive one). Returns nil if none.
            return OAScripting:fence_distance(current_loc, t)
        elseif t ~= OBSTACLE_TYPE.FENCE_ALT_MAX and t ~= OBSTACLE_TYPE.FENCE_ALT_MIN
                and obstacle.location ~= nil then
            -- point/traffic obstacle (aircraft/drone/bird/AIS/proximity): location is a real position
            return obstacle.location:get_distance(current_loc)
        end
        return nil
    end

    -- Every horizontal fence category, for nearest_fence_clearance_m() below.  Not built from
    -- is_fence_obstacle() at call time (that would mean scanning every OBSTACLE_TYPE value on
    -- every call) - listed once, matching is_fence_obstacle()'s own scope exactly.
    local FENCE_TYPES = {
        OBSTACLE_TYPE.FENCE_HOME,
        OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION,
        OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION,
        OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION,
        OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION,
        OBSTACLE_TYPE.FENCE_LUA,
    }

    -- Most severe horizontal fence clearance (m) at loc, independent of whatever obstacle
    -- actually won the single-obstacle sweep this cycle - diagnostic only (DAAR logging), so
    -- a fence that is present but currently masked by a closer moving obstacle is still
    -- visible.  SIGNED (positive clear, negative breached) - loops every fence category and
    -- asks C++ for its real edge clearance, keeping the smallest (most severe: a breach
    -- anywhere dominates a merely-close boundary elsewhere); nil if there is no fence of any
    -- category. Not on the hot sweep path - once per cycle, same cost class as
    -- obstacle_report_distance()'s own fence_distance() call.
    local function nearest_fence_clearance_m(loc)
        if loc == nil then return nil end
        local nearest_m = nil
        for _, t in ipairs(FENCE_TYPES) do
            local d = OAScripting:fence_distance(loc, t)
            if d ~= nil and (nearest_m == nil or d < nearest_m) then
                nearest_m = d
            end
        end
        return nearest_m
    end

    -- Keep-out ("well clear") radius (m) for the CPA conflict test, per obstacle type. This is the
    -- miss distance below which a moving obstacle is treated as a conflict; because the range check in
    -- assess_obstacle_motion uses the same value, it is also the range inside which avoidance is
    -- unconditional (the conservative "safer is better" floor). Distinct from the detection margin in
    -- find_closest_obstacle, which adds a look-ahead buffer on top so obstacles are picked up earlier.
    -- Aircraft/drone/plane share one CPA calculation; only this standoff differs.
    local function get_standoff(obstacle_type)
        -- AIRCRAFT, AIS and anything else: the aircraft well-clear radius (AVD_WCLR_XY)
        return standoff_by_type[obstacle_type] or well_clear_xy
    end

    -- Shared by find_closest_obstacle() and find_closest_fence(): applies the
    -- margin/wind-widening logic to a raw C++ threat-search result and either drops it
    -- (still outside the range we care about) or wraps it as a populated obstacle.
    --
    -- What distance_m already accounts for differs by where the obstacle came from, so the
    -- margin added here does too:
    --  * AP_Avoidance contacts (crewed aircraft, drones) - distance_to_obstacle() has ALREADY
    --    subtracted the protected radius for that emitter type (AVD_WCLR_XY / AVD_UAV_XY), so
    --    distance_m is clearance to the edge of the protected volume and only the extra margin
    --    belongs here.  Adding the radius back made the real trigger 2 x radius + margin: 1269 m
    --    for a crewed aircraft on defaults against the 660 m this file and planedaa.md document,
    --    and it disagreed with find_aircraft(), which passes the full standoff against a raw
    --    centre distance.
    --  * AP_OADatabase objects (AIS, proximity) - _distance_to_object() subtracts the object's
    --    own PHYSICAL radius, so a standoff still has to be added on top here.
    -- The per-type values are in detect_margin_by_type, built by configure(); a type that is
    -- not in it needs no margin.  Zero is a legitimate margin and stays one - only a missing
    -- entry falls through, because 0 is truthy in Lua.
    --
    -- NOTE: a breached fence is not dropped here.  OAScripting:find_threats()/
    -- find_fence_threats() already leave the breached fence categories out of their own
    -- search, because a breached fence reports a large negative clearance and would
    -- otherwise mask every other obstacle - including traffic - for as long as the
    -- breach lasted.
    local function classify_threat(distance_m, any_obstacle, wind_ms)
        if distance_m == nil or any_obstacle == nil then
            return FLT_MAX, nil
        end

        local obstacle_type_val = any_obstacle:obstacle_type()
        if is_grounded_traffic(obstacle_type_val, any_obstacle) then
            return FLT_MAX, nil
        end
        local obstacle_margin = detect_margin_by_type[obstacle_type_val] or 0
        -- widen the fence standoff in wind so the controller has buffer to absorb cross-track
        -- drift and is less likely to be blown across the boundary (DAA_WIND_MARG = 0 disables)
        if wind_ms ~= nil and wind_ms > wind_min_ms and is_fence_obstacle(obstacle_type_val) then
            obstacle_margin = obstacle_margin + wind_margin_per_ms * (wind_ms - wind_min_ms)
        end

        if distance_m > obstacle_margin then
            -- we are further away from the obstacle than we care about
            return FLT_MAX, nil
        end

        return distance_m, populate_obstacle(distance_m, any_obstacle)
    end

    local function find_closest_obstacle(loc1, loc2, lookahead_m, wind_ms)
        -- By projecting 1m along the line we avoid a problem with the
        -- exclusion avoidance being happy to skirt along a line parallel
        -- to an exclusion zone
        local bearing_deg   = math.deg(loc1:get_bearing(loc2))
        local loc1_shifted  = location_project(loc1, bearing_deg, 1, loc2)
        local distance_m, any_obstacle =
                OAScripting:find_threats(loc1_shifted, loc2, lookahead_m)
        return classify_threat(distance_m, any_obstacle, wind_ms)
    end

    -- Fence-only variant of find_closest_obstacle() - immune to being masked by a
    -- moving obstacle (ADS-B/MAVLink traffic) that happens to be closer along the same
    -- line. detect_impl()'s fence-only sweep and release validation use this so a
    -- fence never silently loses out just because a dismissible moving obstacle was
    -- this cycle's single winner.
    local function find_closest_fence(loc1, loc2, lookahead_m, wind_ms)
        local bearing_deg   = math.deg(loc1:get_bearing(loc2))
        local loc1_shifted  = location_project(loc1, bearing_deg, 1, loc2)
        local distance_m, any_obstacle =
                OAScripting:find_fence_threats(loc1_shifted, loc2, lookahead_m)
        return classify_threat(distance_m, any_obstacle, wind_ms)
    end

    -- the taxonomy is exposed on the instance as well as the class, so a caller that has
    -- an instance never needs a second handle on the class just to name an obstacle type
    self.OBSTACLE_TYPE            = DAAobstacles.OBSTACLE_TYPE
    self.ADSB_EMITTER             = DAAobstacles.ADSB_EMITTER

    self.configure                = configure
    self.update_state             = update_state
    self.pretty_obstacle_type     = pretty_obstacle_type
    self.populate_obstacle        = populate_obstacle
    self.is_grounded_traffic      = is_grounded_traffic
    self.obstacle_report_distance = obstacle_report_distance
    self.nearest_fence_clearance_m = nearest_fence_clearance_m
    self.get_standoff             = get_standoff
    self.find_closest_obstacle    = find_closest_obstacle
    self.find_closest_fence       = find_closest_fence
    self.is_fence_obstacle        = is_fence_obstacle

    return self
end

gcs:send_text(BANNER_SEVERITY, string.format("%s %s module loaded", DAAobstacles.SCRIPT_NAME, DAAobstacles.SCRIPT_VERSION))

return DAAobstacles
