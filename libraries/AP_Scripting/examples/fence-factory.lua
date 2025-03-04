--[[
   ArduPilot lua script to create polyfences around the home position
--]]

-- setup param block for VTOL failsafe params
local PARAM_TABLE_KEY = 81
local PARAM_TABLE_PREFIX = "FENCEF_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: RTSW_ENABLE
  // @DisplayName: parameter reversion enable
  // @Description: Enable parameter reversion system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local NPOINTS      = bind_add_param('NPOINTS',         1, 20)
local RADIUS       = bind_add_param('RADIUS',          2, 20)
local MIN_RADIUS   = 10
--[[
   create an N point spiral mission
--]]
function create_poly_circle_fence(N)
   local center = ahrs:get_location()
   if not center then
      gcs:send_text(0, "Error: Need AHRS location for circular polyfence")
      return
   end

   local radius = RADIUS:get()
   if radius < MIN_RADIUS then
      radius = MIN_RADIUS
   end

   gcs:send_text(0, string.format("Creating circular polyfence of size %u", N))

    -- build fence points into this table
    local fence_points = {}

    for i=0, N-1 do
        fence_points[i+1] = AC_PolyFenceItem()
        fence_points[i+1]:type(98) -- POLYGON_INCLUSION
        fence_points[i+1]:vertex_count(N) -- N points in total

        local loc = center:copy()
        loc:offset_bearing(i * (360 / N), radius)

        local location = Vector2l()
        location:x(loc:lat())
        location:y(loc:lng())
        fence_points[i+1]:loc(location)
   end

   fence:load(table.unpack(fence_points))
   assert(fence:send(),'Failed to send fence')
   gcs:send_text(0, "Created circular polyfence of size " .. #fence_points)
end

function wait()
    -- wait until we have a valid home location
    home = ahrs:get_home()
    if home and home:lat() ~= 0 and home:lng() ~= 0 then
        return create_poly_circle_fence(NPOINTS:get())
    end

    return wait, 1000
end

return wait()
