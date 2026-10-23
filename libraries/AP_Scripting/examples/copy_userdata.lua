--[[
 An example of using the copy() method on userdata
--]]


local loc1 = Location()
loc1:lat(-35)
loc1:lng(-122)

-- if we did this as loc2 = loc1 then it actually takes a reference
-- by using copy() we get the intended behaviour
local loc2 = loc1:copy()
loc2:offset(3000,5000)

local diff = loc1:get_distance_NE(loc2)
gcs:send_text(0,string.format("locdiff=(%.2f,%.2f)", diff:x(), diff:y()))

local v1 = Vector2f()
v1:x(-35)
v1:y(-122)

local v2 = v1:copy()
v2:x(v2:x()+100)
v2:y(v2:y()+300)

diff = v2 - v1
gcs:send_text(0,string.format("vdiff=(%.2f,%.2f)", diff:x(), diff:y()))
