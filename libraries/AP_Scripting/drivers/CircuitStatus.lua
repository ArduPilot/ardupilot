--[[
   driver for DroneCAN CircuitStatus battery monitors

   listens for uavcan.equipment.power.CircuitStatus messages and maps
   them onto scripting battery monitor instances
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local CIRCUITSTATUS_ID = 1091
local CIRCUITSTATUS_SIGNATURE = uint64_t(0x8313D33D, 0x0DDDA115)

local MAX_CIRCUITS = 9

local PARAM_TABLE_KEY = 50
local PARAM_TABLE_PREFIX = "DCS"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1+2*MAX_CIRCUITS), 'could not add param table')

--[[
  // @Param: DCS_NUM_CIRCUITS
  // @DisplayName: Number of CircuitStatus monitors
  // @Description: Number of DroneCAN CircuitStatus battery monitor instances. The per-instance DCSx_ parameters are created based on this value.
  // @Range: 0 9
  // @RebootRequired: True
  // @User: Standard
--]]
local DCS_NUM_CIRCUITS = bind_add_param('_NUM_CIRCUITS', 1, 0)

local num_circuits = math.floor(DCS_NUM_CIRCUITS:get())
if num_circuits <= 0 then
    return
end
if num_circuits > MAX_CIRCUITS then
    num_circuits = MAX_CIRCUITS
end

--[[
  // @Param: DCS1_CIRCUIT_ID
  // @DisplayName: CircuitStatus circuit ID
  // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
  // @Range: -1 65535
  // @User: Standard
--]]

--[[
  // @Param: DCS1_BATT_IDX
  // @DisplayName: CircuitStatus battery index
  // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
  // @Range: 0 9
  // @User: Standard
--]]

local circuit_id_param = {}
local batt_idx_param = {}
for i = 1, num_circuits do
    circuit_id_param[i] = bind_add_param(string.format('%u_CIRCUIT_ID', i), 2*i, -1)
    batt_idx_param[i] = bind_add_param(string.format('%u_BATT_IDX', i), 2*i+1, 0)
end

-- receive handles, one per CAN driver
local handles = {}
for bus = 0, 1 do
    local h = DroneCAN_Handle(bus, CIRCUITSTATUS_SIGNATURE, CIRCUITSTATUS_ID)
    if h and h:subscribe() then
        table.insert(handles, h)
    end
end
if #handles == 0 then
    gcs:send_text(MAV_SEVERITY.ERROR, "CircuitStatus: no DroneCAN driver")
    return
end

-- last error_flags seen per instance, for change reporting
local last_error_flags = {}
local last_idx_warn_ms = uint32_t(0)

--[[
   unpack a float16 into a floating point number
--]]
local function unpackFloat16(v16)
    local sign     = (v16 >> 15) & 0x1
    local exponent = (v16 >> 10) & 0x1F
    local fraction = v16 & 0x3FF

    local value
    if exponent == 0 then
        -- zero or subnormal
        value = (fraction / 1024.0) * 2.0^-14
    elseif exponent == 0x1F then
        if fraction == 0 then
            value = math.huge
        else
            value = 0/0
        end
    else
        value = (1 + fraction / 1024.0) * 2.0^(exponent - 15)
    end

    if sign == 1 then
        value = -value
    end

    return value
end

--[[
   handle one CircuitStatus message
--]]
local function handle_circuit_status(payload)
    if #payload ~= 7 then
        return
    end
    local circuit_id, voltage16, current16, error_flags = string.unpack("<HHHB", payload)
    local voltage = unpackFloat16(voltage16)
    local current = unpackFloat16(current16)
    if voltage ~= voltage or voltage < 0 or voltage == math.huge then
        -- reject NaN, negative and infinite voltage
        return
    end
    for i = 1, num_circuits do
        if circuit_id_param[i]:get() == circuit_id then
            local batt_idx = math.floor(batt_idx_param[i]:get())
            if batt_idx >= 1 and batt_idx <= 9 then
                local state = BattMonitorScript_State()
                state:healthy(true)
                state:voltage(voltage)
                if current == current and current > -math.huge and current < math.huge then
                    -- leave current unset when not finite
                    state:current_amps(current)
                end
                if not battery:handle_scripting(batt_idx-1, state) then
                    local now = millis()
                    if now - last_idx_warn_ms > 10000 then
                        last_idx_warn_ms = now
                        local pname = batt_idx == 1 and "BATT" or string.format("BATT%u", batt_idx)
                        gcs:send_text(MAV_SEVERITY.WARNING, string.format("CircuitStatus: %s_MONITOR must be 29", pname))
                    end
                end
                if error_flags ~= last_error_flags[i] then
                    last_error_flags[i] = error_flags
                    if error_flags ~= 0 then
                        gcs:send_text(MAV_SEVERITY.WARNING, string.format("CircuitStatus: circuit %u error 0x%02x", circuit_id, error_flags))
                    end
                end
            end
        end
    end
end

local function update()
    for _, h in ipairs(handles) do
        -- drain all pending messages
        while true do
            local payload, _ = h:check_message()
            if not payload then
                break
            end
            handle_circuit_status(payload)
        end
    end
    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("CircuitStatus: loaded %u circuits", num_circuits))

return update, 100
