-- support takeoff with velocity matching for quadplanes

local PARAM_TABLE_KEY = 35
local PARAM_TABLE_PREFIX = "SHIPV_"

local MODE_AUTO = 10

local NAV_TAKEOFF = 22
local NAV_VTOL_TAKEOFF = 84

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

-- setup SHIPV specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')
SHIPV_ENABLE     = bind_add_param('ENABLE', 1, 0)

local takeoff_vel = nil
local takeoff_pos = nil

-- main update function
function update()
   if SHIPV_ENABLE:get() < 1 then
      return
   end

   local vehicle_mode = vehicle:get_mode()

   if not arming:is_armed() then
      -- when not armed record position and velocity
      takeoff_vel = ahrs:get_velocity_NED()
      takeoff_pos = ahrs:get_position()
      takeoff_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
   else
      if vehicle_mode == MODE_AUTO and takeoff_pos and takeoff_vel then
         local id = mission:get_current_nav_id()
         if id == NAV_VTOL_TAKEOFF or id == NAV_TAKEOFF then
            local next_WP = vehicle:get_target_location()
            if not next_WP then
               return
            end
            vehicle:set_velocity_match(takeoff_vel:xy())
            local tpos = takeoff_pos:copy()
            tpos:alt(next_WP:alt())
            vehicle:update_target_location(next_WP, tpos)
         else
            takeoff_pos = nil
            takeoff_vel = nil
         end
      end
   end
end

function loop()
   update()
   -- run at 20Hz
   return loop, 50
end

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

gcs:send_text(0, "Loaded quadplane takeoff velmatch")

-- start running update loop
return protected_wrapper()

