--[[

SM_SwitchMission.lua - ArduPilot Lua script

That script is derived from MissionSelector.lua in  
https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets  
and the examples in https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples  
  
This script is useful to switch between different missions on SD-card vis RC-channel

The switching can be done by multi-position-switch or pushbutton.
The switching is allowed: 
  -disarmed, 
  -in modes that are not mode-AUTO if they are activated by RC or GCS
  -in mode-AUTO when the current mission is complete

CAUTION: Use this script AT YOUR OWN RISK.

-- Willy Zehnder -- 21.12.2022

]]
-------- SCRIPT 'CONSTANTS' --------
local SCRIPT_NAME           = 'SM'          -- abbreviation of scriptname for messages on GCS
local SUB_DIR_APM           = '/APM/scripts'-- path to scripts on SD-card
local SUB_DIR_SITL          = './scripts'   -- path to scripts when using SITL
local SUB_DIR_MISSIONS      = 'missions'    -- subdirectory below scripts-directory where the mission-files are stored
local FILE_PREFIX           = 'Mission#'    -- filename-prefix of all mission files
local FILE_EXTENSION        = 'waypoints'   -- file-extension of all mission files
local LONG_PRESS_MS         = 1000          -- time to long press pushbutton to start loading the mission
local SHORT_INTERV_MS       = 20            -- interval to find param_table_key
local RUN_INTERVAL_MS       = 100           -- interval to check input-changes
local SBY_INTERVAL_MS       = 1000          -- interval if script is inactive i.e. num_sw_positions is lower than 1
local INTERRUPTED_MS        = 10000         -- interval if script is paused/interrupted by a problem

local WARN                  =   4           -- MAV_SEVERITY_WARNING
local INFO                  =   6           -- MAV_SEVERITY_INFO

local MODE_AUTO             =  10
local REASON_RC             =   1
local REASON_GCS            =   2
local REASON_INITIALIZED    =  26

-------- SCRIPT PARAMETERS --------
local PARAM_PREFIX          = 'SM_'
local PARAM_TABLE           = {
--  {       name , default value },
    { 'RC_OPTION',           300 },         -- defines the selected RCx_Option (300..307)
    { 'POSITIONS',             3 },         -- defines the selection-method and/or amount of switch-positions
}

-------- SCRIPT VARIABLES --------
local sub_dir                               -- where to store the Missions
local file_name                             -- filename of the current mission
local num_sw_positions                      -- number of select-switch positions
local num_files                             -- number of detected mission-files
local scripting_rc                          -- RC-input-channel of selectet RXx_OPTION
local last_sw_position      = nil           -- last position of RC-switch
local last_sw_change        = uint32_t(0)
local file_index            = 0
local wait_finger_off       = false
local param_table_key       = 0             -- start-value for searching free or still used table-key between 0 and 200


local function send_msg(msg_type, msg)
  -- wrapper for sending messages to the GCS
  gcs:send_text(msg_type, string.format('%s: %s', SCRIPT_NAME, msg))
end
  
local function add_params(key, prefix, tbl)
-- add script-specific parameter table
  if not param:add_table(key, prefix, #tbl) then
    return false
  end
  for num, data in ipairs(tbl) do
      assert(param:add_param(key, num, data[1], data[2]), string.format('%s: Could not add %s%s.', SCRIPT_NAME, prefix, data[1]))
  end
  return true
end


local function read_mission(file_path)
-- try to read the mission from file and write it into FC
  local file = io.open(file_path)
  if file then
    -- check file-header
    local headline = file:read('l')
    if not headline then
      send_msg(WARN,string.format('%s is empty', file_name))
      return
    end
    if string.find(headline, 'QGC WPL 110') ~= 1 then
      send_msg(WARN,string.format('%s incorrect format', file_name))
      return
    end
  else
    send_msg(WARN,string.format('%s not existing', file_name))
    return
  end
  -- clear any existing mission
  assert(mission:clear(), string.format('%s: Could not clear current mission', SCRIPT_NAME))

  -- read each line of file and write item to FC
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          -- reached correct end of file
          send_msg(INFO, string.format('%s (%d items) loaded', file_name, index-1))
          return
        else
          -- in case of problems
          mission:clear() -- clear partly loaded mission
          error(string.format('%s: failed to read file', SCRIPT_NAME))
        end
      end
    end

    -- fill the item-structure
    item:seq(data[1])
    item:frame(data[3])
    item:command(data[4])
    item:param1(data[5])
    item:param2(data[6])
    item:param3(data[7])
    item:param4(data[8])
    item:x(data[9]*10^7)
    item:y(data[10]*10^7)
    item:z(data[11])

    -- write the mission-item to FC
    if not mission:set_item(index,item) then
      mission:clear() -- clear partly loaded mission
      error(string.format('%s failed to set mission item %i', SCRIPT_NAME, index))
    end
    index = index + 1
  end
end


local function change_mission_allowed()
-- check, if loading a new Mission is allowed
  local mode = vehicle:get_mode()
  local mode_reason = vehicle:get_control_mode_reason()

  if ((mode ~= MODE_AUTO)
   and
   ((mode_reason == REASON_RC) or (mode_reason == REASON_GCS) or (mode_reason == REASON_INITIALIZED) or (not arming:is_armed())))
   or
   (mission:state() == mission.MISSION_COMPLETE)
  then
    return true
  else
    send_msg(WARN, string.format('Mission change not allowed (mode:%d reason:%d)',mode, mode_reason))
    return false
  end
end


-- ### selection and loading of mission by momentary-pushbutton ###
local function do_momentary_select()
  -- check if pushbutton changed and react
  local previous_switch_position = last_sw_position
  local switch_position          = scripting_rc:get_aux_switch_pos()
  last_sw_position = switch_position
  -- unpressed and no change
  if (switch_position == 0) and (previous_switch_position == 0) then 
    wait_finger_off = false
    return
  end
  if wait_finger_off then return end
  -- just pressed
  if (switch_position >= 1) and (previous_switch_position == 0) then
    last_sw_change = millis()
    return
  end
  -- check if long-pressed
  local long_press = (millis() - last_sw_change) > LONG_PRESS_MS
  if long_press then
    if change_mission_allowed() then
      file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, file_index, FILE_EXTENSION)
      read_mission(string.format('%s/%s', sub_dir, file_name))
      wait_finger_off = true
    end
    return
  -- just released
  elseif (switch_position == 0) and (previous_switch_position >= 1) then
    file_index = (file_index + 1) % num_files
    send_msg(INFO, string.format('Selected %s%d', FILE_PREFIX, file_index))
  end
  return
end


-- ### selection and loading of mission by multi-position-switch ###
local function switch_has_changed(new_sw_pos)
-- action if switch has changed
  if change_mission_allowed() then
    file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, new_sw_pos, FILE_EXTENSION)
    read_mission(string.format('%s/%s', sub_dir, file_name))
    last_sw_position = new_sw_pos
  end
end


local function do_multi_select()
-- check, if switch position has changed
  sw_pos = (scripting_rc:norm_input_ignore_trim() + 1) * (num_sw_positions - 1) / 2
  sw_pos = tonumber(string.format('%.0f', sw_pos)) or -1

  if sw_pos ~= last_sw_position then
    switch_has_changed(sw_pos)
  end
  return
end


local function standby()
  -- standby or switch to the selected input-variant
    if num_sw_positions > 1 then
      do_multi_select()
      return standby, RUN_INTERVAL_MS
    end
    if num_sw_positions == 1 then
      do_momentary_select()
      return standby, RUN_INTERVAL_MS
    end
    return standby, SBY_INTERVAL_MS
  end
  

-- ### initialisation of all necessary environment ###
function init()

  -- add script-specific parameter-table
  if not add_params(param_table_key, PARAM_PREFIX, PARAM_TABLE) then
    if param_table_key < 200 then
      param_table_key = param_table_key + 1
      return init, SHORT_INTERV_MS
    else
      send_msg(WARN, string.format('Could not add table %s', PARAM_PREFIX))
      return init, INTERRUPTED_MS
    end
  else
    send_msg(INFO, string.format('Param table %s at key %d', PARAM_PREFIX, param_table_key))
  end

  -- reading  parameter: number of switch positions
  num_sw_positions = param:get('SM_POSITIONS')
  if num_sw_positions then
    if num_sw_positions == 1 then
      send_msg(INFO, string.format('pushbutton-multiselect'))
    elseif num_sw_positions > 1 then
      send_msg(INFO, string.format('multiposition-switch: %i positions',num_sw_positions))
    end
  else
    send_msg(WARN, string.format('get parameter %s failed', 'SM_POSITIONS'))
    return init, INTERRUPTED_MS
  end

  -- reading the selected option in SM_RC_OPTION parameter and finding corresponding rc-channel for the switch
  local option_value = param:get('SM_RC_OPTION')
  if option_value then
    if (option_value >= 300) and (option_value <= 307) then
      send_msg(INFO, string.format('selected RC_Option: %i',option_value))
    else
      send_msg(WARN, string.format('selected RC_Option: %i out of range (300..307)',option_value))
      return init, INTERRUPTED_MS
    end
  else
    send_msg(WARN, string.format('get parameter %s failed', 'SM_RC_OPTION'))
    return init, INTERRUPTED_MS
  end

  -- finding correct sub_dir for SITL or SD-card
  sub_dir = string.format('%s/%s', SUB_DIR_APM, SUB_DIR_MISSIONS)
  file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, 0, FILE_EXTENSION)
  local file = io.open(string.format('%s/%s', sub_dir, file_name))
  if file then
    file:close()
  else
    sub_dir = string.format('%s/%s', SUB_DIR_SITL, SUB_DIR_MISSIONS)
    file = io.open(string.format('%s/%s', sub_dir, file_name))
    if file then
      file:close()
    else
      send_msg(WARN, string.format('No file %s found ...', file_name))
      send_msg(WARN, string.format('...in SubDir /%s', SUB_DIR_MISSIONS))
      return init, INTERRUPTED_MS
    end
  end
  send_msg(INFO, string.format('sub_dir: %s', sub_dir))

  -- count available missions
  num_files = 0
  while true do
    file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, num_files, FILE_EXTENSION)
    file = io.open(string.format('%s/%s', sub_dir, file_name))
    if file then
      num_files = num_files + 1
      file:close()
    else
      send_msg(INFO, string.format('%d missions found', num_files))
      break
    end
  end
  file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, 0, FILE_EXTENSION)

  if (num_sw_positions > 1) and (num_files < num_sw_positions) then
    send_msg(WARN, string.format('only %d files found', num_files))
    return init, INTERRUPTED_MS
  end  

  -- find channel for mission-switching
  scripting_rc = rc:find_channel_for_option(option_value)
  if scripting_rc then
    send_msg(INFO, 'Script is running')
    return standby, SBY_INTERVAL_MS -- init done -> regular loop start
  else
    send_msg(WARN, string.format('RCx_OPTION = %d not found', option_value))
    return init, INTERRUPTED_MS
  end
end

return init()
