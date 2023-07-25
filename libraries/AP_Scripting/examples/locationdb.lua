-- This script demonstrates basic operations on the location database

local item = DBItem()
local pos = Vector3f() pos:x(100) pos:y(100) pos:z(100) -- pos in cm NEU w.r.t. ekf origin
local vel = Vector3f()
local accel = Vector3f()
local hdg = 0
local radius = 1
local key = locdb:construct_key_scripting(1)
item:init(key, millis(), pos, vel, accel, hdg, radius, 27)

-- add item
locdb:add_item(item)

-- get item
local ret = DBItem()
local success = locdb:get_item(key, ret) 
if success then
    gcs:send_text('6', "AP_LocationDB: Item retrieved successfully")
else
    gcs:send_text('0', "AP_LocationDB: Item could not be retrieved")
end

-- remove item
locdb:remove_item(key)

-- add again
locdb:add_item(item)

function update() -- this is the loop which periodically runs
    hdg = (hdg + 1000) % 36000
    item:init(key, millis(), pos, vel, accel, hdg, radius, 27)
    -- update item
    locdb:update_item(key, item)

    return update, 1000
end

return update()