--[[
    allow for force the position of the simulated vehicle when armed
--]]

local PARAM_TABLE_KEY = 16
local PARAM_TABLE_PREFIX = "SIM_APOS_"

-- setup package place specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local SIM_APOS_ENABLE = bind_add_param('ENABLE', 1, 0)
local SIM_APOS_POS_N = bind_add_param('POS_N', 2, 0)
local SIM_APOS_POS_E = bind_add_param('POS_E', 3, 0)
local SIM_APOS_POS_D = bind_add_param('POS_D', 4, 0)
local SIM_APOS_VEL_X = bind_add_param('VEL_X', 5, 0)
local SIM_APOS_VEL_Y = bind_add_param('VEL_Y', 6, 0)
local SIM_APOS_VEL_Z = bind_add_param('VEL_Z', 7, 0)
local SIM_APOS_RLL = bind_add_param('RLL', 8, 0)
local SIM_APOS_PIT = bind_add_param('PIT', 9, 0)
local SIM_APOS_YAW = bind_add_param('YAW', 10, 0)
local SIM_APOS_GX = bind_add_param('GX', 11, 0)
local SIM_APOS_GY = bind_add_param('GY', 12, 0)
local SIM_APOS_GZ = bind_add_param('GZ', 13, 0)
local SIM_APOS_MODE = bind_add_param('MODE', 14, -1)

local was_armed = false

local function update()
    if SIM_APOS_ENABLE:get() == 0 then
        return
    end
    local armed = arming:is_armed()
    local set_pos = armed and not was_armed

    if set_pos then
        gcs:send_text(0, "Forcing arm pose")
        local quat = Quaternion()
        quat:from_euler(math.rad(SIM_APOS_RLL:get()), math.rad(SIM_APOS_PIT:get()), math.rad(SIM_APOS_YAW:get()))

        local vel = Vector3f()
        vel:x(SIM_APOS_VEL_X:get())
        vel:y(SIM_APOS_VEL_Y:get())
        vel:z(SIM_APOS_VEL_Z:get())
        quat:earth_to_body(vel)

        local loc = ahrs:get_origin()
        if not loc then
            return
        end
        loc:offset(SIM_APOS_POS_N:get(), SIM_APOS_POS_E:get())
        loc:alt(loc:alt() - SIM_APOS_POS_D:get()*100)

        local gyro = Vector3f()
        gyro:x(math.rad(SIM_APOS_GX:get()))
        gyro:y(math.rad(SIM_APOS_GY:get()))
        gyro:z(math.rad(SIM_APOS_GZ:get()))

        sim:set_pose(0, loc, quat, vel, gyro)

        if SIM_APOS_MODE:get() >= 0 then
            vehicle:set_mode(SIM_APOS_MODE:get())
        end
    end

    was_armed = armed
end

local function loop()
    if SIM_APOS_ENABLE:get() == 0 then
        return loop, 500
    end
    update()
    return loop, 50
end

gcs:send_text(0, "Loaded arm pose")
return loop,1000
