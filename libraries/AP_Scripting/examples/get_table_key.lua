--[[

get_table_key.lua - ArduPilot Lua script
  
This script is useful to demonstrate and test the binding and function get_table_key

IT'S STRONGLY RECOMMENDED TO USE IT ONLY IN SITL !!!
Save the parameters bevore you start !!!

CAUTION: 
  That script will fill the parameter-space with senseless parameters. So after running maybe you want to clear it as described here:
  https://ardupilot.org/dev/docs/common-scripting-parameters.html#how-parameters-work-in-ardupilot
  https://ardupilot.org/plane/docs/common-parameter-reset.html

  Use this script AT YOUR OWN RISK.

-- Willy Zehnder -- 21.12.2022

]]
-------- SCRIPT 'CONSTANTS' --------
local SCRIPT_NAME           = 'gtk'         -- abbreviation of scriptname for messages on GCS
local START_MS              = 20000         -- interval before start
local RUN_MS                = 500           -- interval if script is running normally
local INTERRUPTED_MS        = 10000         -- interval if script is paused/interrupted by a problem

local WARN                  =   4           -- MAV_SEVERITY_WARNING
local INFO                  =   6           -- MAV_SEVERITY_INFO

-------- SCRIPT PARAMETERS --------
local param_prefix          = 'A'
local prefix_count          = 0

local PARAM_TABLE           = {
--  {       name , default value },
    { 'PARAM_1',             1 },
    { 'PARAM_2',             2 },
    { 'PARAM_3',             3 },
    { 'PARAM_4',             4 },
    { 'PARAM_5',             5 },
}
-----------------------------------

local function send_msg(msg_type, msg)
  -- wrapper for sending messages to the GCS
  gcs:send_text(msg_type, string.format('%s: %s', SCRIPT_NAME, msg))
end
  
local function add_params(prefix, tbl)
  -- add script-specific parameter table
  local key = param:get_table_key(prefix, #tbl)
  if key == 0xFF then
    send_msg(WARN, string.format('Could not get key for table %s', prefix))
    return false
  else
    send_msg(INFO, string.format('table %s at key %d', prefix, key))
  end
  for num, data in ipairs(tbl) do
      assert(param:add_param(key, num, data[1], data[2]), string.format('%s: Could not add %s%s.', SCRIPT_NAME, prefix, data[1]))
  end
  return true
end

local function read_param(param_name)
  -- reading the selected option in parameter and finding corresponding rc-channel for the switch
  local option_value = param:get(param_name)
  if option_value then
      send_msg(INFO, string.format('parameter %s: %i', param_name, option_value))
  else
    send_msg(WARN, string.format('get parameter %s failed', param_name))
  end
  return option_value
end


local function init()

  local table_prefix = string.format('%s%d_', param_prefix, prefix_count)

  -- add script-specific parameter-table
  if not add_params(table_prefix, PARAM_TABLE) then
      return init, INTERRUPTED_MS
  end

  if not read_param(string.format('%sPARAM_1', table_prefix)) then return init, INTERRUPTED_MS end
  if not read_param(string.format('%sPARAM_5', table_prefix)) then return init, INTERRUPTED_MS end

  prefix_count = prefix_count + 1
  if prefix_count == 10 then
    send_msg(INFO, 'script ended')
    return
  end

  return init, RUN_MS
end

send_msg(INFO, 'script started')
return init, START_MS
