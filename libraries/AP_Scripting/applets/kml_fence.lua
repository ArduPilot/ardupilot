--[[
    implement a set of complex polygon and altitude fences based on a
    lua boundaries file generated from a kml file

    to use first convert your kml file to a fence.lua file using
    the Tools/scripts/BVLOS/kml_to_lua.py python script.

    Place the resulting fence.lua in the APM/SCRIPTS/MODULES directory
    on your microSD card

    Then edit the limits table below
--]]

--[[ user specific limits table, this needs to be updated to reflect
    the actual limits that apply for your BVLOS instrument.
    NOTE! heights are in feet
--]]
local limits = {
    { name="Flight Geography", inclusion=true },
    { name="Contingency Volume", inclusion=true, multi_trigger=true, mode=20 },
    { name="River crossing area - 2200 ft AMSL", AGL=400, AMSL=2200 },
    { name="400ft AGL Altitude Limited Area", AGL=400, AMSL=nil },
    { name="Altitude Limit - 3900 ft AMSL", AGL=nil, AMSL=3900 },
    { name="Altitude Limit - 4900 ft AMSL", AGL=nil, AMSL=4900 },
    { name="Altitude Limit - 5900 ft AMSL", AGL=nil, AMSL=5900 },
}


local PARAM_TABLE_KEY = 120
local PARAM_TABLE_PREFIX = "FEN_KML_"

-- local MODE_AUTO = 10
local MODE_RTL = 11

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 14), 'could not add param table')

--[[
  // @Param: FEN_KML_ENABLE
  // @DisplayName: fence KML enable
  // @Description: fence KML enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local FEN_KML_ENABLE = bind_add_param('ENABLE', 1, 1)

triggered = false

local polygons = require("fence")

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded %d polygons from fence.lua", #polygons))

local EARTH_RADIUS_M = 6371000.0

-- Returns great-circle distance between two lat/lon points in meters
local function dist_latlon(lat1, lon1, lat2, lon2)
    local deg2rad = math.pi / 180
    local dlat = (lat2 - lat1) * deg2rad
    local dlon = (lon2 - lon1) * deg2rad
    local a = math.sin(dlat / 2)^2 +
              math.cos(lat1 * deg2rad) * math.cos(lat2 * deg2rad) *
              math.sin(dlon / 2)^2
    local c = 2 * math.atan(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c  -- Earth radius in meters
end

--[[
    check if we are outside a polygon
--]]
local function polygon_outside(polygon, lat, lon)
    if polygon.circle then
        -- it is a circle (from polygon simplification)
        local clat, clon, r = polygon.circle.lat, polygon.circle.lon, polygon.circle.radius
        return dist_latlon(lat, lon, clat, clon) > r
    end

    local n = #polygon.polygon
    if n < 3 then
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("Polygon %s has less than 3 points %d", polygon.name, n))
        return true
    end

    -- check if polygon is complete (last point == first point)
    local complete = (polygon.polygon[1].lat == polygon.polygon[n].lat and polygon.polygon[1].lon == polygon.polygon[n].lon)
    if complete then
        n = n - 1
    end

    local outside = true

    for i = 1, n do
        local j = i + 1
        if j > n then j = 1 end

        local vi = polygon.polygon[i]
        local vj = polygon.polygon[j]

        if ((vi[1] > lat) == (vj[1] > lat)) then
            goto continue
        end

        local dx1 = lon - vi[2]
        local dx2 = vj[2] - vi[2]
        local dy1 = lat - vi[1]
        local dy2 = vj[1] - vi[1]

        local dx1s = (dx1 < 0) and -1 or 1
        local dx2s = (dx2 < 0) and -1 or 1
        local dy1s = (dy1 < 0) and -1 or 1
        local dy2s = (dy2 < 0) and -1 or 1

        local m1 = dx1s * dy2s
        local m2 = dx2s * dy1s

        if dy2 < 0 then
            if m1 > m2 then
                outside = not outside
            elseif m1 == m2 then
                if dx1 * dy2 > dx2 * dy1 then
                    outside = not outside
                end
            end
        else
            if m1 < m2 then
                outside = not outside
            elseif m1 == m2 then
                if dx1 * dy2 < dx2 * dy1 then
                    outside = not outside
                end
            end
        end

        ::continue::
    end

    return outside
end

-- feet from meters
local function feet(m)
   return m / 0.3048
end

-- find a polygon from the loaded fence.lua module
local function find_polygon(name)
    for _, polygon in ipairs(polygons) do
        if polygon.name == name then
            return polygon
        end
    end
    return nil
end

--[[
    return margin to a fence limit
    nil means no limit applies
    negative means in breach
    positive means how much margin we have
--]]
local function check_alt_margin(limit, alt_amsl_ft, alt_agl_ft)
    local margin = nil
    if limit.AGL and alt_agl_ft then
        margin = limit.AGL - alt_agl_ft
    end

    if limit.AMSL then
        local m = limit.AMSL - alt_amsl_ft
        if not margin or m > margin then
            margin = m
        end
    end

    return margin
end

local function run_checks()
    local loc = ahrs:get_location()
    if not loc then
        return
    end
    local alt_amsl = loc:alt() * 0.01
    local alt_amsl_ft = feet(alt_amsl)
    local alt_agl_ft = nil
    local terrain_alt = terrain:height_amsl(loc, true)
    if terrain_alt then
        alt_agl_ft = feet(alt_amsl - terrain_alt)
    end
    local inside_one = false
    local within_limits = false
    local breach_index = nil
    local largest_margin = nil
    local boundary_ok = nil
    for i = 1, #limits do
        local limit = limits[i]
        local polygon = find_polygon(limit.name)
        assert(polygon, string.format("Polygon %s missing", limit.name))
        local lat, lon = loc:lat()*1.0e-7, loc:lng()*1.0e-7
        local inside = not polygon_outside(polygon, lat, lon)
        if inside then
            inside_one = true
            local margin = check_alt_margin(limit, alt_amsl_ft, alt_agl_ft)
            if margin ~= nil then
                if margin >= 0 then
                    within_limits = true
                end
                if not largest_margin or margin > largest_margin then
                    largest_margin = margin
                    if margin < 0 then
                        breach_index = i
                    end
                end
            end
            if limit.inclusion then
                boundary_ok = true
            end
            if limit.exclusion then
                boundary_ok = false
                breach_index = i
            end
        else
            if limit.inclusion then
                if not boundary_ok then
                    boundary_ok = false
                    breach_index = i
                end
            end
        end
    end
    if breach_index and boundary_ok == false then
        local limit = limits[breach_index]
        if not triggered or limit.multi_trigger then
            local new_mode = limit.mode or MODE_RTL
            if new_mode ~= vehicle:get_mode() then
                gcs:send_text(MAV_SEVERITY.ALERT, string.format("@Breached boundary %s", limit.name))
                triggered = true
                -- switch to RTL or specified mode
                vehicle:set_mode(limit.mode or MODE_RTL)
            end
        end
    end
    if breach_index and inside_one and not within_limits then
        local limit = limits[breach_index]
        if not triggered or limit.multi_trigger then
            local new_mode = limit.mode or MODE_RTL
            if new_mode ~= vehicle:get_mode() then
                gcs:send_text(MAV_SEVERITY.ALERT, string.format("@Breached %s %.0f %.0f", limit.name, alt_agl_ft, alt_amsl_ft))
                triggered = true
                -- switch to RTL or specified mode
                vehicle:set_mode(limit.mode or MODE_RTL)
            end
        end
    end
    if largest_margin then
        gcs:send_named_float("KML_MARGIN", largest_margin)
    end
end

-- Main update function, called at 1Hz
function update()
    if FEN_KML_ENABLE:get() == 1 then
        run_checks()
    else
        -- allow re-arming of checks
        if triggered then
            gcs:send_text(MAV_SEVERITY.INFO, "KML Fence re-armed")
            triggered = false
        end
    end
    return update, 1000
end

if FEN_KML_ENABLE:get() == 1 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded KML fence"))
end

-- Start running update loop
return update, 1000

