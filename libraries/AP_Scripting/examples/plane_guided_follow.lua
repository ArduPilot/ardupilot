-- support follow in GUIDED mode in plane

local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "GFOLL_"

local MODE_GUIDED = 15

local ALT_FRAME_ABSOLUTE = 0

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup SHIP specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')
GFOLL_ENABLE     = bind_add_param('ENABLE', 1, 0)

-- current target
local target_pos
local current_pos

-- other state
local have_target = false

-- check key parameters
function check_parameters()
  --[[
     parameter values which are auto-set on startup
  --]]
   local key_params = {
      FOLL_ENABLE = 1,
      FOLL_OFS_TYPE = 1,
      FOLL_ALT_TYPE = 0,
   }

   for p, v in pairs(key_params) do
      local current = param:get(p)
      assert(current, string.format("Parameter %s not found", p))
      if math.abs(v-current) > 0.001 then
         param:set_and_save(p, v)
         gcs:send_text(0,string.format("Parameter %s set to %.2f was %.2f", p, v, current))
      end
   end
end

function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

-- update target state
function update_target()
   if not follow:have_target() then
      if have_target then
         gcs:send_text(0,"Lost beacon")
      end
      have_target = false
      return
   end
   if not have_target then
      gcs:send_text(0,"Have beacon")
   end
   have_target = true

   target_pos = follow:get_target_location_and_velocity_ofs()
end

-- main update function
function update()
   if GFOLL_ENABLE:get() < 1 then
      return
   end

   update_target()
   if not have_target then
      return
   end

   current_pos = ahrs:get_position()
   if not current_pos then
      return
   end
   current_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)

   if vehicle:get_mode() ~= MODE_GUIDED then
      return
   end

   local next_WP = vehicle:get_target_location()
   if not next_WP then
      -- not in a flight mode with a target location
      return
   end

   -- update the target position from the follow library, which includes the offsets
   target_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
   vehicle:update_target_location(next_WP, target_pos)
end

function loop()
   update()
   -- run at 20Hz
   return loop, 50
end

check_parameters()

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end

-- start running update loop
return protected_wrapper()

