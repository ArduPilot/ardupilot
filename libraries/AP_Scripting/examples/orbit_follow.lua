-- example that does an orbit of a vehicle that is being followed
-- by adjusting the follow offset X and Y parameters
-- user settable FOLL_ORB_TIME and FOLL_ORB_RADIUS parameters
-- are provided for the time for one orbit (in seconds) and the radius
-- of the orbit

-- setup param block for FOLL_ extensions
local PARAM_TABLE_KEY = 83
local PARAM_TABLE_PREFIX = "FOLL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

function bind_add_param(name, index, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, index,  name, default_value), string.format('could not add %s', PARAM_TABLE_PREFIX .. name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local FOLL_OFS_X = Parameter("FOLL_OFS_X")
local FOLL_OFS_Y = Parameter("FOLL_OFS_Y")
local FOLL_ORB_RADIUS = bind_add_param("ORB_RADIUS", 1, 5)
local FOLL_ORB_TIME = bind_add_param("ORB_TIME", 2, 10)

local t = 0

function update()
   t = t + 0.1
   --gcs:send_text(0, string.format("t=%.2f X=%.2f Y=%.2f", t, FOLL_OFS_X:get(), FOLL_OFS_Y:get()))
   if t > FOLL_ORB_TIME:get() then
      t = t - FOLL_ORB_TIME:get()
   end
   FOLL_OFS_X:set(math.cos((t/FOLL_ORB_TIME:get())*2*math.pi)*FOLL_ORB_RADIUS:get())
   FOLL_OFS_Y:set(math.sin((t/FOLL_ORB_TIME:get())*2*math.pi)*FOLL_ORB_RADIUS:get())
   return update, 100
end

gcs:send_text(0, string.format("orbit started"))
return update, 100


