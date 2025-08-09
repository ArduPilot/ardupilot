--[[
    Switch EFI Generator on/off based on reported Load
    This maybe useful when aircraft is flown using the gas engine and the electric generator is to power payloads and other auxilary equipment
    Tested to be working in parallel with the  EFI_SkyPower lua script
--]]

-- Variables
local lastSwitchTime_ms = millis() -- Time when the last switch occurred
local generatorState = nil -- Current state of the generator (true = on, false = off)
local aboveThresholdTime_ms = 0 -- Time (in ms) engine load has remained above the threshold
local belowThresholdTime_ms = 0 -- Time (in ms) engine load has remained below the threshold
local update_rate_ms = 100

PARAM_TABLE_KEY = 120
PARAM_TABLE_PREFIX = "EFIGEN_"

-- bind a parameter to a variable given
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

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'could not add EFIGEN param table')

--[[
  // @Param: EFIGEN_ENABLE
  // @DisplayName: Enable automatic EFI Generator on/off switch
  // @Description: Enable automatic EFI Generator on/off switch
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFIGEN_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: EFIGEN_GEN_FN
  // @DisplayName: EFI generator control function
  // @Description: EFI generator control function.
  //               This is the RCn_OPTION value to use to find the R/C channel used for controlling generator start/stop
  // @Values: 0:Disabled,300:300,301:301,302:302,303:303,304:304,305:305,306:306,307:307
  // @User: Standard
--]]
local EFIGEN_FN = bind_add_param('FN', 2, 0) -- generator control function (RC option)

--[[
  // @Param: EFIGEN_MIN
  // @DisplayName: EFI Generator Min Load
  // EFI Generator will switch on if load is less than this parameter for EFIGEN_TIMER seconds
  // @Range: 20 50
  // @User: Standard
--]]
local EFIGEN_MIN = bind_add_param('MIN', 3, 45) -- generator control function (RC option)

--[[
  // @Param: EFIGEN_MAX
  // @DisplayName: EFI Generator Max Load
  // @Description: EFI Generator will switch off if load is greater than this parameter for EFIGEN_TIMER seconds
  // @Range: 40 80
  // @User: Standard
--]]
local EFIGEN_MAX = bind_add_param('MAX', 4, 55)


--[[
  // @Param: EFIGEN_TIMER
  // @DisplayName: EFI Generator Switch Timer
  // @Description: EFI Generator Load has to be greater than EFIGEN_MIN or less than EFIGEN_MAX for this many seconds
  // @Range: 0 10
  // @User: Standard
--]]
local EFIGEN_TIMER = bind_add_param('TIMER', 5, 2)


--[[
  // @Param: EFIGEN_TIMOU
  // @DisplayName: EFI Generator Switch Timeout
  // @Description: EFI Generator will not be switched on/off if it was previously switched within this many seconds
  // @Range: 0 10
  // @User: Standard
--]]
local EFIGEN_TIMOU = bind_add_param('TIMOU', 6, 5)


function switchGenerator(state, load)
  generatorState = state
  lastSwitchTime_ms = millis()
  if state then
    gcs:send_text(2,"Generator switched ON. Current Load: " .. tostring(load))
    rc:run_aux_function(EFIGEN_FN:get(), 2)
  else
    gcs:send_text(2,"Generator switched OFF. Current Load: " .. tostring(load))
    rc:run_aux_function(EFIGEN_FN:get(), 0)
  end
end

-- Function to check the EFI engine load
function checkEngineLoad(loadPercent)
  if loadPercent >= EFIGEN_MAX:get() then
    aboveThresholdTime_ms = aboveThresholdTime_ms + update_rate_ms
    belowThresholdTime_ms = 0
    if aboveThresholdTime_ms >= EFIGEN_TIMER:get()*1000 and (generatorState or generatorState == nil) then
      switchGenerator(false, loadPercent) -- Switch off the generator
    end
  elseif loadPercent <= EFIGEN_MIN:get() then
    belowThresholdTime_ms = belowThresholdTime_ms + update_rate_ms
    aboveThresholdTime_ms = 0
    if belowThresholdTime_ms >= EFIGEN_TIMER:get()*1000 and (not generatorState or generatorState == nil) then
      switchGenerator(true, loadPercent) -- Switch on the generator
    end
  else
    aboveThresholdTime_ms = 0
    belowThresholdTime_ms = 0
  end
end

-- Function to check if enough time has passed since the last switch
function enoughTimePassed()
  local elapsedTime = millis() - lastSwitchTime_ms
  return elapsedTime >= EFIGEN_TIMOU:get()*1000
end

function update()
  if EFIGEN_ENABLE:get() == 0 then
      return update, update_rate_ms
  end

  local efi_state = efi:get_state()
  local load_percentage = efi_state:engine_load_percent()
  if not load_percentage then
      return update, update_rate_ms
  end

  if enoughTimePassed() then
    checkEngineLoad(load_percentage)
  end

  return update, update_rate_ms
end

gcs:send_text(6, "Started EFI Gen Load Checker")
return update()
