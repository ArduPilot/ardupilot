--[[
    implement a set of complex polygon and altitude fences based on a
    lua boundaries file generated from a kml file

    to use first convert your kml file to a fence.lua file using
    the kml_to_lua.py python script. Place fence.lua in the
    APM/SCRIPTS/MODULES directory on your microSD card
--]]

local PARAM_TABLE_KEY = 120
local PARAM_TABLE_PREFIX = "FEN_KML_"

local MODE_AUTO = 10
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
  // @DisplayName: fence AGL enable
  // @Description: fence AGL enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local FEN_KML_ENABLE = bind_add_param('ENABLE', 1, 1)

triggered = false

local polygons = require("fence")

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded %d polygons from fence.lua", #polygons))

local function polygon_outside(polygon, lat, lon)
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

local function feet_to_meters(feet)
   return feet * 0.3048
end

local function find_polygon(name)
    for i, polygon in ipairs(polygons) do
        if polygon.name == name then
            return polygon
        end
    end
    return nil
end

local limits = {
    { name="River crossing area - 2200 ft AMSL", AGL=400, AMSL=2200 },
    { name="400ft AGL Altitude Limited Area", AGL=400, AMSL=nil },
    { name="Altitude Limit - 3900 ft AMSL", AGL=nil, AMSL=3900 },
    { name="Altitude Limit - 4900 ft AMSL", AGL=nil, AMSL=4900 },
    { name="Altitude Limit - 5900 ft AMSL", AGL=nil, AMSL=5900 },
}

--[[
    return true if the polygon is breached by the given lat, lon, alt_amsl and alt_agl
--]]
local function check_polygon_alt_breach(limit, polygon, lat, lon, alt_amsl, alt_agl)
    if polygon_outside(polygon, lat, lon) then
        return false
    end
    if limit.AGL and alt_agl and alt_agl < limit.AGL then
        return false
    end

    if limit.AMSL and alt_amsl < limit.AMSL then
        return false
    end

    return true
end

local function run_checks()
    local loc = ahrs:get_location()
    if not loc then
        return
    end
    local alt_amsl = loc:alt() * 0.01
    local alt_agl = nil
    local terrain_alt = terrain:height_amsl(loc, true)
    if terrain_alt then
        alt_agl = alt_amsl - terrain_alt
    end
    for i = 1, #limits do
        local limit = limits[i]
        local polygon = find_polygon(limit.name)
        if polygon then
            local lat, lon = loc:lat()*1.0e-7, loc:lng()*1.0e-7
            if check_polygon_alt_breach(limit, polygon, lat, lon, alt_amsl, alt_agl) then
                if not triggered then
                    gcs:send_text(MAV_SEVERITY.ALERT, string.format("Breached %s", limit.name))
                    triggered = true
                end
                -- If we are in auto mode, switch to RTL
                if vehicle:get_mode() == MODE_AUTO then
                    vehicle:set_mode(MODE_RTL)
                end
            else
                triggered = false
            end
        end
    end
end

-- Main update function, called at 1Hz
function update()
    if FEN_KML_ENABLE:get() == 1 then
        run_checks()
    end
   return update, 1000
end

if FEN_KML_ENABLE:get() == 1 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded KML fence"))
end

-- Start running update loop
return update, 1000

