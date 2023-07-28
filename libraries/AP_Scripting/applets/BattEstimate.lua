--[[
   battery state of charge (SOC) estimator based on resting voltage

   See Tools/scripts/battery_fit.py for a tool to calculate the coefficients from a log
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 14
local PARAM_TABLE_PREFIX = "BATT_SOC"

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

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 32), 'could not add param table')

--[[
  // @Param: BATT_SOC_COUNT
  // @DisplayName: Count of SOC estimators
  // @Description: Number of battery SOC estimators
  // @Range: 0 4
  // @User: Standard
--]]
local BATT_SOC_COUNT     = bind_add_param('_COUNT', 1, 0)

if BATT_SOC_COUNT:get() <= 0 then
   return
end

--[[
  // @Param: BATT_SOC1_IDX
  // @DisplayName: Battery estimator index
  // @Description: Battery estimator index
  // @Range: 0 4
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC1_NCELL
  // @DisplayName: Battery estimator cell count
  // @Description: Battery estimator cell count
  // @Range: 0 48
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC1_C1
  // @DisplayName: Battery estimator coefficient1
  // @Description: Battery estimator coefficient1
  // @Range: 100 200
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC1_C2
  // @DisplayName: Battery estimator coefficient2
  // @Description: Battery estimator coefficient2
  // @Range: 2 5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC1_C3
  // @DisplayName: Battery estimator coefficient3
  // @Description: Battery estimator coefficient3
  // @Range: 0.01 0.5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC2_IDX
  // @DisplayName: Battery estimator index
  // @Description: Battery estimator index
  // @Range: 0 4
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC2_NCELL
  // @DisplayName: Battery estimator cell count
  // @Description: Battery estimator cell count
  // @Range: 0 48
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC2_C1
  // @DisplayName: Battery estimator coefficient1
  // @Description: Battery estimator coefficient1
  // @Range: 100 200
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC2_C2
  // @DisplayName: Battery estimator coefficient2
  // @Description: Battery estimator coefficient2
  // @Range: 2 5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC2_C3
  // @DisplayName: Battery estimator coefficient3
  // @Description: Battery estimator coefficient3
  // @Range: 0.01 0.5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC3_IDX
  // @DisplayName: Battery estimator index
  // @Description: Battery estimator index
  // @Range: 0 4
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC3_NCELL
  // @DisplayName: Battery estimator cell count
  // @Description: Battery estimator cell count
  // @Range: 0 48
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC3_C1
  // @DisplayName: Battery estimator coefficient1
  // @Description: Battery estimator coefficient1
  // @Range: 100 200
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC3_C2
  // @DisplayName: Battery estimator coefficient2
  // @Description: Battery estimator coefficient2
  // @Range: 2 5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC3_C3
  // @DisplayName: Battery estimator coefficient3
  // @Description: Battery estimator coefficient3
  // @Range: 0.01 0.5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC4_IDX
  // @DisplayName: Battery estimator index
  // @Description: Battery estimator index
  // @Range: 0 4
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC4_NCELL
  // @DisplayName: Battery estimator cell count
  // @Description: Battery estimator cell count
  // @Range: 0 48
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC4_C1
  // @DisplayName: Battery estimator coefficient1
  // @Description: Battery estimator coefficient1
  // @Range: 100 200
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC4_C2
  // @DisplayName: Battery estimator coefficient2
  // @Description: Battery estimator coefficient2
  // @Range: 2 5
  // @User: Standard
--]]

--[[
  // @Param: BATT_SOC4_C3
  // @DisplayName: Battery estimator coefficient3
  // @Description: Battery estimator coefficient3
  // @Range: 0.01 0.5
  // @User: Standard
--]]

local params = {}
local last_armed_ms = 0

--[[
   add parameters for an estimator
--]]
function add_estimator(i)
   id = string.format("%u_", i)
   pidx = 2+(i-1)*5
   params[i] = {}
   params[i]['IDX']   = bind_add_param(id .. "IDX",     pidx+0, 0)
   params[i]['NCELL'] = bind_add_param(id .. "NCELL", pidx+1, 0)
   params[i]['C1']    = bind_add_param(id .. "C1", pidx+2, 111.56)
   params[i]['C2']    = bind_add_param(id .. "C2", pidx+3, 3.65)
   params[i]['C3']    = bind_add_param(id .. "C3", pidx+4, 0.205)
end

local count = math.floor(BATT_SOC_COUNT:get())
for i = 1, count do
   add_estimator(i)
end

local function constrain(v, vmin, vmax)
   return math.max(math.min(v, vmax), vmin)
end

--[[
   simple model of state of charge versus resting voltage.
   With thanks to Roho for the form of the equation
   https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery
--]]
local function SOC_model(cell_volt, c1, c2, c3)
    local p0 = 80.0
    local soc = c1*(1.0-1.0/(1+(cell_volt/c2)^p0)^c3)
    return constrain(soc, 0, 100)
end

--[[
   update one estimator
--]]
local function update_estimator(i)
   local idx = math.floor(params[i]['IDX']:get())
   local ncell = math.floor(params[i]['NCELL']:get())
   if idx <= 0 or ncell <= 0 then
      return
   end
   local C1 = params[i]['C1']:get()
   local C2 = params[i]['C2']:get()
   local C3 = params[i]['C3']:get()
   local num_batts = battery:num_instances()
   if idx > num_batts then
      return
   end
   local voltR = battery:voltage_resting_estimate(idx-1)
   local soc = SOC_model(voltR/ncell, C1, C2, C3)
   battery:reset_remaining(idx-1, soc)
end

--[[
   main update function, called at 1Hz
--]]
function update()
   local now_ms = millis()
   if arming:is_armed() then
      last_armed_ms = now_ms
      return update, 1000
   end
   -- don't update for 10s after disarm, to get logging of charge recovery
   if now_ms - last_armed_ms < 10000 then
      return update, 1000
   end
   for i = 1, #params do
      update_estimator(i)
   end
   return update, 1000
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded BattEstimate for %u batteries", #params))

-- start running update loop
return update, 1000

