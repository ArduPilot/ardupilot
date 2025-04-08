--[[
   create a mission of SCR_USER1 waypoints in a spiral pattern, for
   testing large mission transfer

   Runs on any change to SCR_USER1
--]]

local SCR_USER1 = Parameter("SCR_USER1") -- number of WPs
local SCR_USER2 = Parameter("SCR_USER2") -- radius
local SCR_USER3 = Parameter("SCR_USER3") -- center lat
local SCR_USER4 = Parameter("SCR_USER4") -- center lon
local SCR_USER5 = Parameter("SCR_USER5") -- altitude
local SCR_USER6 = Parameter("SCR_USER6") -- number of WPs for spiral compare

local last_SCR_USER1 = SCR_USER1:get()
local last_SCR_USER6 = SCR_USER6:get()

local NAV_WAYPOINT = 16
local FRAME_GLOBAL = 3

local ANGLE_INCREMENT = 5.0
local MIN_RADIUS = 10.0

--[[
   create a waypoint
--]]
function create_WP(i, center, radius, angle)
   local item = mavlink_mission_item_int_t()
   local loc = center:copy()
   loc:offset_bearing(radius, angle)

   item:seq(i)
   item:frame(FRAME_GLOBAL)
   item:command(NAV_WAYPOINT)
   item:param1(0)
   item:param2(0)
   item:param3(0)
   item:param4(0)
   item:x(loc:lat())
   item:y(loc:lng())
   item:z(100+loc:alt()*0.01)
   return item
end

--[[
   create a waypoint based on index
--]]
function create_WP_idx(center, idx)
   local radius = SCR_USER2:get()
   if radius < MIN_RADIUS then
      radius = MIN_RADIUS
   end
   return create_WP(idx, center, radius+idx, idx*ANGLE_INCREMENT)
end

--[[
   get center Location
--]]
function get_center()
   -- if USER3 and USER4 set then use those, allowing autotest to control exact position
   if SCR_USER3:get() ~= 0 and SCR_USER4:get() ~= 0 then
      local loc = Location()
      loc:lat(math.floor(SCR_USER3:get()*1.0e7))
      loc:lng(math.floor(SCR_USER4:get()*1.0e7))
      loc:alt(math.floor(SCR_USER5:get()*100))
      return loc
   end
   return ahrs:get_location()
end

--[[
   create an N point spiral mission
--]]
function create_spiral(N)
   local center = get_center()
   if not center then
      gcs:send_text(0, "Error: Need AHRS location for spiral center")
      return
   end
   mission:clear()

   gcs:send_text(0, string.format("Creating spiral of size %u", N))

   for i=0, N-1 do
      local item = create_WP_idx(center, i)
      if not mission:set_item(i, item) then
         gcs:send_text(0, string.format("Failed to create WP %u", i))
         N = i
         break
      end
   end
   gcs:send_text(0, string.format("Created spiral of size %u", N))
end

--[[
   compare current mission with spiral, used in autotest
--]]
function compare_spiral(N)
   local center = get_center()
   if not center then
      gcs:send_text(0, "Error: Need AHRS location for spiral center")
      return
   end
   gcs:send_text(0, string.format("Comparing spiral of size %u", N))

   -- start from index 1 as home can change
   for i=1, N-1 do
      local item1 = create_WP_idx(center, i)
      local item2 = mission:get_item(i)
      if not item2 then
         gcs:send_text(0, string.format("Failed to fetch WP %u", i))
         return
      end
      if item1:x() ~= item2:x() or item1:y() ~= item2:y() or item1:z() ~= item2:z() then
         gcs:send_text(0, string.format("Compared fail %u (%.8f %.8f %.2f) (%.8f %.8f %.2f)", i,
                                        item1:x()*1.0e-7,
                                        item1:y()*1.0e-7,
                                        item1:z(),
                                        item2:x()*1.0e-7,
                                        item2:y()*1.0e-7,
                                        item2:z()))
         return
      end
   end
   gcs:send_text(0, string.format("Compared spiral of size %u OK", N))
end

function update()
   if SCR_USER1:get() ~= last_SCR_USER1 and SCR_USER1:get() > 0 then
      last_SCR_USER1 = SCR_USER1:get();
      create_spiral(last_SCR_USER1);
   end
   if SCR_USER6:get() ~= last_SCR_USER6 and SCR_USER6:get() > 0 then
      last_SCR_USER6 = SCR_USER6:get();
      compare_spiral(last_SCR_USER6);
   end
   return update, 1000
end

gcs:send_text(0, "Loaded spiral mission creator")
return update, 1000
