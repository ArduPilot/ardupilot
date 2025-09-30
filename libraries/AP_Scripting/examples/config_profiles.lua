-- config_profiles.lua
-- Continuously watches CFG_* parameters and applies associated parameter sets

-- all_params must contain default values for any parameter present in
-- the parameter lists.  When switching between domain selections
-- these values are used if the domain doesn't have one.  The same
-- parameter must not exist in multiple domains!


-- TODO:
--   far flung future - change parameters on peripherals too


gcs:send_text(6, string.format("CFG: config_profiles v0.3 starting"))

local SEL_APPLY_DEFAULTS = 0
local SEL_DO_NOTHING = -1

local must_be_set = "must be set"

-- This is a marker for the start of the config_domains; it is used to swap these out for CI testing

-- a table of param indexes to help avoid annoying conflicts:
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["JOB"]       = 10,
   ["ARMS"]      = 11,
   ["BAT"]       = 12,
   ["PAY"]       = 13,
}

local config_domains = {
   JOB = {
      param_name = "JOB",
      all_param_defaults = {
         ["FLTMODE1"] = must_be_set,
         ["FLTMODE2"] = must_be_set,
         ["FLTMODE3"] = must_be_set,
         ["FLTMODE4"] = must_be_set,
         ["FLTMODE5"] = must_be_set,
         ["FLTMODE6"] = must_be_set,
         ["LOG_BITMASK"] = 180223,
      },
      default_sel_value = 1,
      profiles = {
         [1] = {
            name = "DeliveryDefault",
            params = {
               ["FLTMODE1"] = 2,
               ["FLTMODE2"] = 2,
               ["FLTMODE3"] = 5,
               ["FLTMODE4"] = 5,
               ["FLTMODE5"] = 6,
               ["FLTMODE6"] = 6,
               ["LOG_BITMASK"] = 180222,
            }
         },
         [2] = {
            name = "FSO",
            params = {
               ["FLTMODE1"] = 5,
               ["FLTMODE2"] = 5,
               ["FLTMODE3"] = 3,
               ["FLTMODE4"] = 3,
               ["FLTMODE5"] = 6,
               ["FLTMODE6"] = 6,
            },
         },
         [3] = {
            name = "Testing",
            params = {
               ["FLTMODE1"] = 0,
               ["FLTMODE2"] = 0,
               ["FLTMODE3"] = 2,
               ["FLTMODE4"] = 2,
               ["FLTMODE5"] = 5,
               ["FLTMODE6"] = 5,
            },
         },
      },
   },
   ARMS = {
      param_name = "ARMS",
      all_param_defaults = {  -- all parameters present in the params for each option
         ["ACRO_BAL_PITCH"] = 1,
         ["ATC_ACCEL_P_MAX"] = 30000,
      },
      default_sel_value = 50,
      profiles = {
         [50] = {
            name = "Callisto 50",
            params = {
               ["ATC_ACCEL_P_MAX"] = 40000,
            },
         },
         [25] = {
            name = "Callisto 25",
            params = {
               ["ATC_ACCEL_P_MAX"] = 50000,
            },
         },
      },
   },

   BAT = {
      param_name = "BAT",
      all_param_defaults = {
         ["BATT_CAPACITY"] = must_be_set,
         ["BATT_CRT_MAH"] = must_be_set,
         ["BATT_CRT_VOLT"] = must_be_set,
         ["BATT_LOW_MAH"] = must_be_set,
         ["BATT_LOW_VOLT"] = must_be_set,
      },
      default_sel_value = 16,
      profiles = {
         [16] = {
            name = "16Ah",
            params = {
               ["BATT_CAPACITY"] = 32000,
               ["BATT_CRT_MAH"] = 6400,
               ["BATT_CRT_VOLT"] = 44.5,
               ["BATT_LOW_MAH"] = 9600,
               ["BATT_LOW_VOLT"] = 45.0,
            },
         },
         [22] = {
            name = "22Ah",
            params = {
               ["BATT_CAPACITY"] = 44000,
               ["BATT_CRT_MAH"] = 8800,
               ["BATT_CRT_VOLT"] = 43.25,
               ["BATT_LOW_MAH"] = 13200,
               ["BATT_LOW_VOLT"] = 43.75,
            },
         },
         [40] = {
            name = "40Ah",
            params = {
               ["BATT_CAPACITY"] = 80000,
               ["BATT_CRT_MAH"] = 16000,
               ["BATT_CRT_VOLT"] = 43.75,
               ["BATT_LOW_MAH"] = 24000,
               ["BATT_LOW_VOLT"] = 44.25,
            },
         },
      },
   },

   PAY = {
      param_name = "PAY",
      all_param_defaults = {
         ["GRIP_ENABLE"] = 0,
         ["GRIP_GRAB"] = 1000,
         ["GRIP_NEUTRAL"] = 1000,
         ["GRIP_REGRAB"] = 0,
         ["GRIP_RELEASE"] = 2000,
         ["GRIP_TYPE"] = 1,
         ["MNT1_DEFLT_MODE"] = 3,     -- 3 = RC Targeting
         ["MNT1_LEAD_PTCH"] = 0,
         ["MNT1_LEAD_RLL"] = 0,
         ["MNT1_NEUTRAL_X"] = 0,
         ["MNT1_NEUTRAL_Y"] = 0,
         ["MNT1_NEUTRAL_Z"] = 0,
         ["MNT1_PITCH_MAX"] = 20,
         ["MNT1_PITCH_MIN"] = -90,
         ["MNT1_RC_RATE"] = 90,
         ["MNT1_RETRACT_X"] = 0,
         ["MNT1_RETRACT_Y"] = 0,
         ["MNT1_RETRACT_Z"] = 0,
         ["MNT1_ROLL_MAX"] = 30,
         ["MNT1_ROLL_MIN"] = -30,
         ["MNT1_TYPE"] = 0,
         ["MNT1_YAW_MAX"] = 180,
         ["MNT1_YAW_MIN"] = -180,
         ["RC13_OPTION"] = 0,
         ["RC13_REVERSED"] = 0,
         ["RC14_OPTION"] = 0,
         ["RC14_REVERSED"] = 0,
         ["RC8_OPTION"] = 0,        -- disabled
         ["SERIAL5_BAUD"] = 115,      -- 115 = 115200 baud
         ["SERIAL5_OPTIONS"] = 0,
         ["SERIAL5_PROTOCOL"] = 2,    -- 2 = MAVLink2
         ["SERVO10_FUNCTION"] = -1,
         ["SERVO10_MAX"] = 2000,
         ["SERVO10_MIN"] = 1000,
         ["SERVO10_REVERSED"] = 0,
         ["SERVO10_TRIM"] = 1500,
         ["SERVO11_FUNCTION"] = -1,
         ["SERVO12_FUNCTION"] = -1,
         ["SERVO13_FUNCTION"] = -1,
         ["SERVO14_FUNCTION"] = -1,
         ["SERVO9_FUNCTION"] = -1,
         ["SERVO9_MAX"] = 2000,
         ["SERVO9_MIN"] = 1000,
         ["SERVO9_REVERSED"] = 0,
         ["SERVO9_TRIM"] = 1500,
      },
      default_sel_value = SEL_APPLY_DEFAULTS,
      profiles = {
         [1] = {
            name = "HookFixed",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,          -- 19 = Gripper (RC switch triggers grab/release)
               ["SERVO9_FUNCTION"] = 28,     -- 28 = gripper
            }
         },
         [2] = {
            name = "HookSlung",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,           -- 19 = Gripper (RC switch triggers grab/release)
               ["SERVO9_FUNCTION"] = 146,     -- 146 = k_rcin7_mapped?!
               ["SERVO9_REVERSED"] = 1,
               ["SERVO10_FUNCTION"] = 28,     -- 28 = gripper
            },
         },
         [3] = {
            name = "Gimbal - AVT_CM62",
            params = {
               ["MNT1_TYPE"] = 6,           -- 6 = Mount type MAVLinkv2 (or Custom)
               ["RC13_OPTION"] = 214,       -- 214 = Mount Yaw
               ["RC14_OPTION"] = 213,       -- 213 = Mount Pitch
            },
         },
         [4] = {
            name = "Gimbal - HookFixed",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,            -- 19 = Gripper

               ["SERVO9_FUNCTION"] = 28,       -- 28 = Mount Shutter (e.g. camera trigger)
               ["SERVO10_FUNCTION"] = 64,      -- 64 = Landing Gear
               ["SERVO11_FUNCTION"] = 63,      -- 63 = Mount Tilt
               ["SERVO12_FUNCTION"] = 62,      -- 62 = Mount Roll
               ["SERVO13_FUNCTION"] = 58,      -- 58 = Mount Zoom
               ["SERVO14_FUNCTION"] = 57,      -- 57 = Mount Focus
            },
         },
         [5] = {
            name = "Gimbal - Hook Slung",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,        -- gripper
               ["SERVO9_FUNCTION"] = 146,  -- k_rcin7_mapped?!
               ["SERVO9_REVERSED"] = 1,
               ["SERVO10_FUNCTION"] = 28,  -- gripper
               ["SERVO10_TRIM"] = 2000,
               ["SERVO11_FUNCTION"] = 64,  -- rcin 14
               ["SERVO12_FUNCTION"] = 63,  -- rcin 13
               ["SERVO13_FUNCTION"] = 61,  -- rcin 11
               ["SERVO14_FUNCTION"] = 56,  -- rcin 6
            }
         },
      },
   },
}
-- This is a marker for the end of the config_domains; it is used to swap these out for CI testing

-- a list of parameters which can still be set even if we are
-- filtering which parameter values can bet set.  We also permit the
-- configuration paramters to be dynamically set.
local parameters_which_can_be_set = {
    ["CFG_FLT_PM_SET"] = true,

    ["MAV_OPTIONS"] = true,

    ["BATT_ARM_MAH"] = true,
    ["BATT_ARM_VOLT"] = true,
    ["BATT_FS_CRT_ACT"] = true,
    ["BATT_FS_LOW_ACT"] = true,

    ["BRD_OPTIONS"] = true,
    ["COMPASS_USE3"] = true,

    ["FENCE_ACTION"] = true,
    ["FENCE_ALT_MAX"] = true,
    ["FENCE_ENABLE"] = true,
    ["FENCE_RADIUS"] = true,
    ["FENCE_TYPE"] = true,

    ["LIGHTS_ON"] = true,

    ["LOG_BITMASK"] = true,
    ["LOG_DISARMED"] = true,
    ["LOG_FILE_DSRMROT"] = true,

    ["RTL_ALT"] = true,
    ["RTL_LOIT_TIME"] = true,
    ["RTL_SPEED"] = true,
}

local msg_pfx = "CFG: "

-- set up for denying arming when problems occur:
local auth_id = arming:get_aux_auth_id() or 0

local function set_aux_auth_failed(reason)
   arming:set_aux_auth_failed(auth_id, msg_pfx .. reason)
end

local function set_aux_auth_passed()
   arming:set_aux_auth_passed(auth_id)
end

set_aux_auth_failed("Validation pending")

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(5, 1)

-- register message id to receive
local PARAM_SET_ID = 23
mavlink:register_rx_msgid(PARAM_SET_ID)

-- initialise our knowledge of the GCS's allow-set-parameters state.
--   We do not want to fight over setting this GCS state via other
--   mechanisms (eg. an auxiliary function), so we keep this state
--   around to track what we last set:
local gcs_allow_set = gcs:get_allow_param_set()

local function send_text(severity, message)
   gcs:send_text(severity, msg_pfx .. message)
end

local function validate_unique_parameters_across_domains()
   -- ensure that no parameter exists in multiple domains:
   local all_domain_parameters = {}
   for _, domain in pairs(config_domains) do
      for param_name, _ in pairs(domain.all_param_defaults) do
         local first_domain_name = all_domain_parameters[param_name]
         if first_domain_name ~= nil then
            send_text(3, string.format("%s exists in two domains (%s and %s)", param_name, first_domain_name, domain.param_name))
            return false
         end
         all_domain_parameters[param_name] = domain.param_name
      end
   end
   return true
end

local function validate_params_in_param_defaults()
   -- ensure that any parameter in the params tables are also in all_params.
   -- Also ensure that it doesn't have the same value as the default
   -- also ensure all defaults are over-ridden in one of the profiles
   for _, domain in pairs(config_domains) do
--      local default_overridden = {}
      for profile_num, profile in pairs(domain.profiles) do
         for param_name, param_value in pairs(profile.params) do
 --            default_overridden[param_name] = true
            local param_default = domain.all_param_defaults[param_name]
            if param_default == nil then
               send_text(3, string.format("%s exists in %s[%f](%s) but not in %s.all_param_defaults", param_name, domain.param_name, profile_num, profile.name, domain.param_name))
               return false
            end
            if param_default == param_value then
               send_text(3, string.format("%s in %s[%f](%s) has the same value as %s.all_param_defaults", param_name, domain.param_name, profile_num, profile.name, domain.param_name))
               return false
            end
         end
      end
-- turns out this is a pain as you need to find values to put into all_param_defaults which is annoying:
--      local failed_defaults_used = false
--      for param_name, _ in pairs(domain.all_param_defaults) do
         --if default_overridden[param_name] == nil then
--            send_text(3, string.format("%s.all_param_defaults contains %s but no profile overrides it", domain.param_name, param_name))
--            failed_defaults_used = true
--         end
--      end
--      if failed_defaults_used then
--         return false
--      end
   end
   return true
end

local function validate_must_be_set_parameters()
   -- ensure that if a parameter has a "must be set" value in the
   -- defaults that every domain defines a value for that default
   -- also assert that at least one profile has a different value to the others
   for _, domain in pairs(config_domains) do
      for param_name, value in pairs(domain.all_param_defaults) do
         if value ~= must_be_set then
            goto vmbsp_next_param
         end
         local first_value = nil
         local at_least_two_values = false
         for profile_num, profile in pairs(domain.profiles) do
            param_value = profile.params[param_name]
            if param_value == nil then
               send_text(3, string.format("%s exists in %s.all_param_defaults with must-be-set-value but profile %s(%s) params does not have a value for it", param_name, domain.param_name, profile_num, profile.name))
               return false
            end
            if first_value == nil then
               first_value = param_value
            else
               if param_value ~= first_value then
                  at_least_two_values = true
               end
            end
         end
         if not at_least_two_values then
            send_text(3, string.format("%s in %s is must-be-set but all profiles have the same value", param_name, domain.param_name))
            return false
         end
         ::vmbsp_next_param::
      end
   end
   return true
end

local function validate_domain_attributes()
   -- check direct domain attributes are valid
   for _, domain in pairs(config_domains) do
      -- check default selection is valid
      default_sel_value = domain.default_sel_value
      if default_sel_value == nil then
         goto good_sel_value
      end
      if default_sel_value == SEL_DO_NOTHING or default_sel_value == SEL_APPLY_DEFAULTS then
         goto good_sel_value
      end
      
      sel = domain.profiles[default_sel_value]
      if sel ~= nil then
         goto good_sel_value
      end

      if true then  -- needed to fool lua checker
         send_text(3, string.format("%s.default_sel_value is invalid", domain.param_name))
         return false
      end

      ::good_sel_value::
   end
   return true
end

-- this is a runtime check:
local function validate_apply_defaults_no_must_be_sets()
   -- ensure that if a parameter is marked as must-be-supplied in the
   -- defaults that the domain isn't in "apply defaults" mode
   for _, domain in pairs(config_domains) do
      local sel_value = domain.sel_param:get()
      if sel_value == nil then
         send_text(3, string.format("Bad sel parameter for %s", domain.param_name))
         return false
      end
      if sel_value ~= SEL_APPLY_DEFAULTS then
         goto vadnn_next_domain
      end
      for param_name, value in pairs(domain.all_param_defaults) do
         if value == must_be_set then
            send_text(3, string.format("%s mode is set-values-from-defaults but %s is marked must-be-set in %s.all_param_defaults", domain.param_name, param_name, domain.param_name))
            return false
         end
      end
      ::vadnn_next_domain::
   end
   return true
end

local function validate_param_profiles()
   for _, domain in pairs(config_domains) do
      for profile_num, profile in pairs(domain.profiles) do
         if profile.name == nil then
            send_text(3, string.format("%s profile is missing a name", domain.param_name))
            return false
         end
         if profile_num == 0 then
            send_text(3, string.format("%s profile (%s) has zero index, which conflicts with the domain \"apply defaults\" option", domain.param_name, profile.name))
            return false
         end
      end
   end
   return true
end

-- validate_config_domains - step through the domain configuration
-- structure and ensure that no mistake has been made in the setup of
-- the configuration sets
local function validate_config_domains()
   validators = {
      validate_unique_parameters_across_domains,
      validate_params_in_param_defaults,
      validate_must_be_set_parameters,
      validate_apply_defaults_no_must_be_sets,
      validate_domain_attributes,
      validate_param_profiles,
   }
   for _, validator in pairs(validators) do
      if validator == nil then
         set_aux_auth_failed("Invalid validator")
         return false
      end
      if not validator() then
         set_aux_auth_failed("Domain configuration invalid")
         return false
      end
   end
   return true
end

-- parameter setup
--
local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "CFG_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: CFG_ENABLE
  // @DisplayName: Enable configuration script
  // @Description: Script immediately exits if this is zero
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local CFG_ENABLE = bind_add_param("ENABLE", param_index["enable"], 1)

--[[
  // @Param: CFG_FLTR_PM_SET
  // @DisplayName: Filter parameter setting
  // @Description: Disallows setting of parameters outside whitelist
  // @Values: 0:Do not filter,1:Filter
  // @User: Standard
--]]
local FLT_PM_SET = bind_add_param("FLT_PM_SET", param_index["pm_filter"], 0)

local function add_param_for_domain(domain, index, name, default)
   pname = domain.param_name .. "_" .. name
   -- send_text(6, string.format("Registering %s at index=%s with default %s", pname, index, default))
   return bind_add_param(pname, index, default)
end

for _, domain in pairs(config_domains) do
   domain_sel_default = domain.default_sel_value
   if domain_sel_default == nil then
      domain_sel_default = 0
   end
   domain.sel_param = add_param_for_domain(domain, param_index[domain.param_name], "SEL", domain_sel_default)
end

local a_parameter_was_ever_set = false


-- utility: apply one profile
local function apply_parameters(domain, params)
   for param_name, new_value in pairs(domain.all_param_defaults) do
      local profile_param_value = params[param_name]
      if profile_param_value ~= nil then
         new_value = profile_param_value
      end
      if new_value == nil then
         send_text(3, string.format("Validation code has failed as %s has nil value", param_name))
         return
      end
      if new_value == must_be_set then
         send_text(3, string.format("Validation code has failed as %s is must_be_set", param_name))
         return
      end
      local old_value = param:get(param_name)
      if old_value == nil then
         send_text(3, string.format("Unable to fetch parameter %s", param_name))
         return
      end
      if old_value ~= new_value then
         send_text(6, string.format("Set %s=%s (was %.3f)", param_name, new_value, old_value))
         param:set_and_save(param_name, new_value)
         a_parameter_was_ever_set = true
      end
   end
end

-- check each domain configuration parameter, act if any has changed
local last_active_domains_print_time = millis()
local function handle_show_domain_statuses()
   local now = millis()
   if now - last_active_domains_print_time < 30000 then
      return
   end
   last_active_domains_print_time = now
   for _, domain in pairs(config_domains) do
      local sel_value = domain.sel_param:get()
      if sel_value == SEL_DO_NOTHING then
         send_text(6, string.format("%s: doing nothing", domain.param_name))
         goto hsds_next_domain
      end
      if sel_value == SEL_APPLY_DEFAULTS then
         send_text(6, string.format("%s: applying defaults", domain.param_name))
         goto hsds_next_domain
      end
      local profile = domain.profiles[sel_value]
      if profile == nil then
         send_text(6, string.format("%s: Invalid profile selected", domain.param_name))
         goto hsds_next_domain
      end
      send_text(6, string.format("%s: applying profile %s", domain.param_name, profile.name))
      ::hsds_next_domain::
   end
end

local selected_profile_was_nil = false
local function handle_domains()
   local success = true
   for _, domain in pairs(config_domains) do
      local sel_value = domain.sel_param:get()
      local sel_value_changed = false
      if sel_value ~= domain.last_sel_value then
         domain.last_sel_value = sel_value
         sel_value_changed = true
      end

      if sel_value == SEL_DO_NOTHING then
         if sel_value_changed then
            send_text(6, string.format("Doing nothing for %s", domain.param_name))
         end
         goto cd_next_domain
      end

      if sel_value == SEL_APPLY_DEFAULTS then
         if sel_value_changed then
            send_text(6, string.format("Applying %s defaults", domain.param_name))
         end
         apply_parameters(domain, {})
         goto cd_next_domain
      end

      local profile = domain.profiles[sel_value]
      if profile == nil then
         if not selected_profile_was_nil then
            send_text(6, string.format("Invalid profile selected for %s", domain.param_name))
            selected_profile_was_nil = true
         end
         set_aux_auth_failed(string.format("Invalid profile selected for %s", domain.param_name))
         success = false
         goto cd_next_domain
      end
      selected_profile_was_nil = false

      if domain.last_sel_value ~= sel_value or sel_value_changed then
         domain.last_sel_value = sel_value
         send_text(6, string.format("Applying %s profile: %s", domain.param_name, profile.name))
      end
      apply_parameters(domain, profile.params)
      goto cd_next_domain

      ::cd_next_domain::
   end
   if a_parameter_was_ever_set then
      set_aux_auth_failed("Reboot needed for config change")
      return
   end
   if not success then
      return
   end

   set_aux_auth_passed()

   handle_show_domain_statuses()
end

-- support for parameter-set-filtering:
local function should_set_parameter_id(param_id)
   -- first check the static list:
   if parameters_which_can_be_set[param_id] ~= nil then
      return parameters_which_can_be_set[param_id]
   end

   -- now check to see if this is one of the configuration selection
   -- parameters:
   for _, domain in pairs(config_domains) do
      domain_sel_param_name = string.format("CFG_%s_SEL", domain.param_name)
      if param_id == domain_sel_param_name then
         return true
      end
   end

   return false
end

local function handle_param_set(name, value)
    -- we will not receive packets in here for the wrong system ID /
    --   component ID; this is handled by ArduPilot's MAVLink routing
    --   code

    -- check for this specific ID:
    if not should_set_parameter_id(name) then
       send_text(3, string.format("param set denied (%s)", name))
        return
    end

    param:set_and_save(name, value)
    gcs:send_text(3, string.format("param set applied"))
end

local function handle_param_setting()
   if gcs_allow_set and FLT_PM_SET:get() == 1 then
      -- this script is filtering, disallow setting via normal means (once):
      gcs:set_allow_param_set(false)
      gcs_allow_set = false
   elseif not gcs_allow_set and FLT_PM_SET:get() == 0 then
      -- this script is not filtering, allow setting via normal means (once):
      gcs:set_allow_param_set(true)
      gcs_allow_set = true
   end

   while true do
      -- we consume all mavlink messages even when we won't set parameters
      local msg, _ = mavlink:receive_chan()
      if msg == nil then
         break
      end

      if gcs_allow_set == false then -- we are in change of param setting
         local param_value, _, _, param_id, _ = string.unpack("<fBBc16B", string.sub(msg, 13, 36))
         param_id = string.gsub(param_id, string.char(0), "")

         handle_param_set(param_id, param_value)
      end
   end
end

-- update function
local domains_valid = false
function update()
   if CFG_ENABLE:get() == 0 then
      -- permanently exit
        if not gcs_allow_set then
          gcs:set_allow_param_set(true)
          gcs_allow_set = true
        end
      send_text(3, string.format("exitting"))
      return
   end

   handle_param_setting()

   -- do nothing if armed:
   if arming:is_armed() then
      return update, 1000
   end

   if not domains_valid then
      if validate_config_domains() then
         domains_valid = true
         set_aux_auth_failed("Profile value validation pending")
      else
         return update, 10000  -- rerun periodically to make the problem clear
      end
   end

   handle_domains()
   return update, 1000  -- recheck every second
end

return update()
