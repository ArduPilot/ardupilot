---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

-- ============================================================
-- ECU TURBINE TELEMETRY
-- Vspeak / Xicoy
-- ============================================================

local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = 'ECU_'

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30),
       'could not add param table')

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 1 = Jakadofsky
-- 2 = EvoJet
-- 3 = PBS
-- 4 = Hornet
-- 5 = JetCat
-- 6 = KingTech
-- 7 = AMT
-- 8 = Xicoy telem adapter
-- 9 = JetCentral
-- 10 = Kolibri NG
-- 11 = Swiwin
-- 12 = Linton

local ECU_TYPE = bind_add_param('TYPE', 1, 1)
local XICOY_ID = 0x1B
local VSPEAK_ID = 0x1C
local port = serial:find_serial(0)
local BAUD = 57600
local POLL_START = 0x7E
local POLL_INTERVAL = 100
local LOG_INTERVAL = 100
local POLL_ID = VSPEAK_ID
local raw_bytes = {}
local last_poll = 0
local last_report = 0
local rx_bytes = 0
local frames = 0
local EGT = 0
local RPM_val = 0
local Fuel = 0
local Vbat = 0
local Vpump = 0
local Status = 0
local Status_last = 0
local ECU_Status = {}

-- ============================================================
-- UINT32
-- ============================================================

local function to_uint32(val)
    if val < 0 then
        return val + 4294967296
    end

    return val
end


-- Status table


local function initECU_Status()

    local ecu_type = ECU_TYPE:get()

    ECU_Status = {}

    if POLL_ID == XICOY_ID then

        ECU_Status[0] = "HighTemp"
        ECU_Status[1] = "Trim Low"
        ECU_Status[2] = "SetIdle!"
        ECU_Status[3] = "Ready"
        ECU_Status[4] = "Ignition"
        ECU_Status[5] = "FuelRamp"
        ECU_Status[6] = "Glow Test"
        ECU_Status[7] = "Running"
        ECU_Status[8] = "Stop"
        ECU_Status[9] = "FlameOut"
        ECU_Status[10] = "SpeedLow"
        ECU_Status[11] = "Cooling"
        ECU_Status[12] = "Ignit.Bad"
        ECU_Status[13] = "Start.Fail"
        ECU_Status[14] = "AccelFail"
        ECU_Status[15] = "Start On"
        ECU_Status[16] = "UserOff"
        ECU_Status[17] = "Failsafe"
        ECU_Status[18] = "Low RPM"
        ECU_Status[19] = "Reset"
        ECU_Status[20] = "RXPwFail"
        ECU_Status[21] = "PreHeat"
        ECU_Status[22] = "Battery!"
        ECU_Status[23] = "Time Out"
        ECU_Status[24] = "Overload"
        ECU_Status[25] = "Ign.Fail"
        ECU_Status[26] = "Burner On"
        ECU_Status[27] = "Starting"
        ECU_Status[28] = "SwitchOv"
        ECU_Status[29] = "Cal.Pump"
        ECU_Status[30] = "PumpLimi"
        ECU_Status[31] = "NoEngine"
        ECU_Status[32] = "PwrBoost"
        ECU_Status[33] = "Run-Idle"
        ECU_Status[34] = "Run-Max"
        ECU_Status[35] = "Restart"

        return
    end

    -- ========================================================
    -- VSPEAK / TURBINE ECU STATUS
    -- ========================================================

    if ecu_type == 1 or ecu_type == 2 then

        ECU_Status[-30] = " ERROR"
        ECU_Status[-20] = "TH:-off"
        ECU_Status[-10] = "TH:cool"
        ECU_Status[-1] = "TH:lock"
        ECU_Status[0] = "TH:stop"
        ECU_Status[10] = "TH:run-"
        ECU_Status[20] = "TH:rel-"
        ECU_Status[25] = "TH:glow"
        ECU_Status[30] = "TH:spin"
        ECU_Status[40] = "TH:fire"
        ECU_Status[45] = "TH:ignt"
        ECU_Status[50] = "TH:heat"
        ECU_Status[60] = "TH:acce"
        ECU_Status[65] = "TH:cal."
        ECU_Status[70] = "TH:idle"

    elseif ecu_type == 3 then

        ECU_Status[-30] = "ERROR"
        ECU_Status[-10] = "COOL"
        ECU_Status[-1] = "LOCK"
        ECU_Status[0] = "STOP"
        ECU_Status[10] = "TH:Idle"
        ECU_Status[20] = "TH:-Rel"
        ECU_Status[25] = "TEST"
        ECU_Status[40] = "FIRE"
        ECU_Status[70] = "IDLE"

    elseif ecu_type == 4 then

        ECU_Status[-30] = "DEV.DELAY"
        ECU_Status[-20] = "FLAME OUT"
        ECU_Status[-10] = "EMERGENCY"
        ECU_Status[0] = "OFF"
        ECU_Status[1] = "ON"
        ECU_Status[10] = "Cool Down"
        ECU_Status[20] = "Slow Down"
        ECU_Status[30] = "STANDBY"
        ECU_Status[31] = "PROP IGNIT"
        ECU_Status[32] = "PROP HEAT"
        ECU_Status[33] = "Pump Start"
        ECU_Status[34] = "BURNER ON"
        ECU_Status[35] = "FUEL IGNIT"
        ECU_Status[36] = "FUEL HEAT"
        ECU_Status[37] = "Ramp Delay"
        ECU_Status[38] = "RAMP UP"
        ECU_Status[40] = "STEADY"
        ECU_Status[41] = "CAL IDLE"
        ECU_Status[42] = "CALIBRATE"
        ECU_Status[43] = "WAIT ACC"
        ECU_Status[44] = "GO IDLE"
        ECU_Status[50] = "AUTO"
        ECU_Status[51] = "AUTO HC"

    elseif ecu_type == 5 then

        ECU_Status[-40] = "Failure"
        ECU_Status[-32] = "ECU reboot"
        ECU_Status[-31] = "ClutchFail"
        ECU_Status[-30] = "Low Rpm"
        ECU_Status[-28] = "Out Fuel"
        ECU_Status[-27] = "Pump Comm"
        ECU_Status[-26] = "WrongPump"
        ECU_Status[-25] = "No Pump!"
        ECU_Status[-24] = "OverCurr"
        ECU_Status[-23] = "No-OIL"
        ECU_Status[-22] = "2nd Comm"
        ECU_Status[-21] = "2nd Diff"
        ECU_Status[-20] = "2nd EngF"
        ECU_Status[-19] = "Rpm2Fail"
        ECU_Status[-18] = "FuelFail"
        ECU_Status[-17] = "TempFail"
        ECU_Status[-16] = "PowerFail"
        ECU_Status[-15] = "IgnTimOut"
        ECU_Status[-14] = "FailSafe"
        ECU_Status[-13] = "WatchDog"
        ECU_Status[-12] = "GlowPlug!"
        ECU_Status[-11] = "HiTempOff"
        ECU_Status[-10] = "LowTempOff"
        ECU_Status[-9] = "OverTemp"
        ECU_Status[-8] = "BatteryLow"
        ECU_Status[-7] = "Low-Rpm"
        ECU_Status[-6] = "Over-Rpm"
        ECU_Status[-5] = "Acc. Slow"
        ECU_Status[-4] = "AccTimOut"
        ECU_Status[-3] = "Manual Off"
        ECU_Status[-2] = "Auto-Off"
        ECU_Status[-1] = "RC-Off"
        ECU_Status[0] = "-OFF-"
        ECU_Status[1] = "SlowDown"
        ECU_Status[2] = "SwitchOff"
        ECU_Status[3] = "Stby/START"
        ECU_Status[4] = "PreHeat1"
        ECU_Status[5] = "PreHeat2"
        ECU_Status[6] = "Ignite..."
        ECU_Status[7] = "AccelrDly"
        ECU_Status[8] = "MainFStrt"
        ECU_Status[9] = "Keros.Full"
        ECU_Status[10] = "accelerate"
        ECU_Status[11] = "Stabilise"
        ECU_Status[12] = "LearnLO"
        ECU_Status[13] = "RUN (reg.)"
        ECU_Status[14] = "SpeedCtrl"
        ECU_Status[15] = "Rpm2Ctrl"

    elseif ecu_type == 6 then

        ECU_Status[-20] = "- ERROR -"
        ECU_Status[-19] = "Unknown"
        ECU_Status[-13] = "CAB-Lost"
        ECU_Status[-12] = "FlameOut"
        ECU_Status[-11] = "TempHigh"
        ECU_Status[-10] = "SpeedLow"
        ECU_Status[-9] = "Failsafe"
        ECU_Status[-8] = "RxPwFail"
        ECU_Status[-7] = "Overload"
        ECU_Status[-6] = "Low Batt"
        ECU_Status[-5] = "StartBad"
        ECU_Status[-4] = "Weak Gas"
        ECU_Status[-3] = "Time Out"
        ECU_Status[-2] = "Ign.Fail"
        ECU_Status[-1] = "Glow Bad"
        ECU_Status[0] = "Trim Low"
        ECU_Status[1] = "User Off"
        ECU_Status[2] = "Stop"
        ECU_Status[3] = "Cooling"
        ECU_Status[4] = "Ready"
        ECU_Status[5] = "GdReady"
        ECU_Status[6] = "GlowTest"
        ECU_Status[7] = "StickLo!"
        ECU_Status[8] = "PrimeVap"
        ECU_Status[9] = "BurnerOn"
        ECU_Status[10] = "Start On"
        ECU_Status[11] = "Ignition"
        ECU_Status[12] = "Stage1"
        ECU_Status[13] = "Stage2"
        ECU_Status[14] = "Stage3"
        ECU_Status[20] = "Running"
        ECU_Status[22] = "ReStart"

    elseif ecu_type == 7 then

        ECU_Status[-20] = "- ERROR -"
        ECU_Status[-8] = "Supply ASS"
        ECU_Status[-7] = "Supply low"
        ECU_Status[-6] = "RPM high"
        ECU_Status[-5] = "EGT error"
        ECU_Status[-4] = "Throttle fail"
        ECU_Status[-3] = "Switch failure"
        ECU_Status[-2] = "RPM low"
        ECU_Status[-1] = "No serial input"
        ECU_Status[0] = "No Start Clear"
        ECU_Status[1] = "Start Clear"
        ECU_Status[2] = "Starting"
        ECU_Status[3] = "Started up"
        ECU_Status[4] = "Calibrated"
        ECU_Status[5] = "Auto stop"
        ECU_Status[6] = "Running"
        ECU_Status[7] = "Max. RPM"

    elseif ecu_type == 8 then

        ECU_Status[0]  = "HighTemp"
        ECU_Status[1]  = "Trim Low"
        ECU_Status[2]  = "SetIdle!"
        ECU_Status[3]  = "Ready"
        ECU_Status[4]  = "Ignition"
        ECU_Status[5]  = "FuelRamp"
        ECU_Status[6]  = "Glow Test"
        ECU_Status[7]  = "Running"
        ECU_Status[8]  = "Stop"
        ECU_Status[9]  = "FlameOut"
        ECU_Status[10] = "SpeedLow"
        ECU_Status[11] = "Cooling"
        ECU_Status[12] = "Ignit.Bad"
        ECU_Status[13] = "Start.Fail"
        ECU_Status[14] = "AccelFail"
        ECU_Status[15] = "Start On"
        ECU_Status[16] = "UserOff"
        ECU_Status[17] = "Failsafe"
        ECU_Status[18] = "Low RPM"
        ECU_Status[19] = "Reset"
        ECU_Status[20] = "RXPwFail"
        ECU_Status[21] = "PreHeat"
        ECU_Status[22] = "Battery!"
        ECU_Status[23] = "Time Out"
        ECU_Status[24] = "Overload"
        ECU_Status[25] = "Ign.Fail"
        ECU_Status[26] = "Burner On"
        ECU_Status[27] = "Starting"
        ECU_Status[28] = "SwitchOv"
        ECU_Status[29] = "Cal.Pump"
        ECU_Status[30] = "PumpLimi"
        ECU_Status[31] = "NoEngine"
        ECU_Status[32] = "PwrBoost"
        ECU_Status[33] = "Run-Idle"
        ECU_Status[34] = "Run-Max "
        ECU_Status[35] = "Restart "
        ECU_Status[36] = "No Status"

    elseif ecu_type == 9 then

        ECU_Status[-21] = "ERROR"
        ECU_Status[-20] = "- no data -"
        ECU_Status[-19] = "MaxPump"
        ECU_Status[-18] = "RPMSensorErr"
        ECU_Status[-17] = "LowRPM"
        ECU_Status[-16] = "MaxAmpers"
        ECU_Status[-15] = "MaxTemp"
        ECU_Status[-14] = "ComTurbErr"
        ECU_Status[-13] = "TempSensErr"
        ECU_Status[-12] = "RxSetupError"
        ECU_Status[-11] = "Failsafe"
        ECU_Status[-10] = "NoRx"
        ECU_Status[-9] = "LowECUBatt"
        ECU_Status[-8] = "LowRXBatt"
        ECU_Status[-7] = "IgniterBad"
        ECU_Status[-6] = "AccelerateBad"
        ECU_Status[-5] = "ToIdle Error"
        ECU_Status[-4] = "StarterError"
        ECU_Status[-3] = "SwitchOverErr"
        ECU_Status[-2] = "PreheatError"
        ECU_Status[-1] = "IgnitionError"
        ECU_Status[0] = "OFF"
        ECU_Status[1] = "ManCooling"
        ECU_Status[2] = "AutoCooling"
        ECU_Status[6] = "GlowTest"
        ECU_Status[7] = "StarterTest"
        ECU_Status[8] = "PrimeFuel"
        ECU_Status[9] = "PrimeBurner"
        ECU_Status[10] = "Standby"
        ECU_Status[11] = "Start"
        ECU_Status[12] = "IgniterHeat"
        ECU_Status[13] = "Ignition"
        ECU_Status[14] = "Preheat"
        ECU_Status[15] = "Switchover"
        ECU_Status[16] = "To Idle"
        ECU_Status[17] = "MinPumpOk"
        ECU_Status[18] = "MaxPumpOk"
        ECU_Status[19] = "Running"
        ECU_Status[20] = "Full"

    elseif ecu_type == 10 then

        ECU_Status[-10] = "FLAME OUT"
        ECU_Status[-2] = "BATT LOW!"
        ECU_Status[-1] = "GLOW PLUG!"
        ECU_Status[0] = "OFF"
        ECU_Status[1] = "ON"
        ECU_Status[2] = "COOL-DOWN"
        ECU_Status[3] = "SLOW-DOWN"
        ECU_Status[10] = "STANDBY"
        ECU_Status[11] = "PROPIGNIT"
        ECU_Status[12] = "PROP-HEAT"
        ECU_Status[13] = "PUMPSTART"
        ECU_Status[14] = "FUELHEAT"
        ECU_Status[15] = "RAMP-UP"
        ECU_Status[20] = "AUTO"

    elseif ecu_type == 11 then

        ECU_Status[-30] = "no data"
        ECU_Status[-18] = "Engine Offline"
        ECU_Status[-17] = "Current overload"
        ECU_Status[-16] = "Clutch failure"
        ECU_Status[-15] = "Pump Temp High"
        ECU_Status[-14] = "StarterTempHigh"
        ECU_Status[-13] = "Lost Signal"
        ECU_Status[-12] = "Fuel Valve Bad"
        ECU_Status[-11] = "Gas Valve Bad"
        ECU_Status[-10] = "TempSensorfail"
        ECU_Status[-9] = "Low Temp"
        ECU_Status[-8] = "High Temp"
        ECU_Status[-7] = "RPM Instability"
        ECU_Status[-6] = "RPM Low"
        ECU_Status[-5] = "Starter failure"
        ECU_Status[-4] = "Pump Anomaly"
        ECU_Status[-3] = "GlowPlug Bad"
        ECU_Status[-2] = "Low Battery"
        ECU_Status[-1] = "Time Out"
        ECU_Status[0] = "Stop"
        ECU_Status[1] = "Cooling"
        ECU_Status[5] = "TestGlowPlug"
        ECU_Status[6] = "TestFuelValve"
        ECU_Status[7] = "TestGasValve"
        ECU_Status[8] = "TestPump"
        ECU_Status[9] = "TestStarter"
        ECU_Status[10] = "Ready"
        ECU_Status[11] = "Ignition"
        ECU_Status[12] = "Preheat"
        ECU_Status[13] = "Fuelramp"
        ECU_Status[20] = "Running"
        ECU_Status[21] = "Restart"

    elseif ecu_type == 12 then

        ECU_Status[-40] = "-no data-"
        ECU_Status[-39] = "Error Alarms"
        ECU_Status[-35] = "Power limit"
        ECU_Status[-34] = "Restart Fail"
        ECU_Status[-27] = "Pump bubble"
        ECU_Status[-18] = "Fuel Fail"
        ECU_Status[-17] = "RPM Low"
        ECU_Status[-16] = "RPM Err"
        ECU_Status[-15] = "Rc Lost Off"
        ECU_Status[-14] = "Pump Fail"
        ECU_Status[-13] = "PumpCurrent"
        ECU_Status[-12] = "Pump Open"
        ECU_Status[-11] = "CTH Fail"
        ECU_Status[-10] = "MotorCurrent"
        ECU_Status[-9] = "Motor Open"
        ECU_Status[-8] = "IGT Short"
        ECU_Status[-7] = "IGT Open"
        ECU_Status[-6] = "EGT Warn"
        ECU_Status[-5] = "TT Trans"
        ECU_Status[-4] = "TT Open"
        ECU_Status[-3] = "OverCurrent"
        ECU_Status[-2] = "Volt High"
        ECU_Status[-1] = "Volt Low"
        ECU_Status[0] = "Ready"
        ECU_Status[1] = "Ready start"
        ECU_Status[2] = "Temp high"
        ECU_Status[3] = "Start.."
        ECU_Status[4] = "Burner.."
        ECU_Status[5] = "Success"
        ECU_Status[6] = "Heating1.."
        ECU_Status[7] = "Heating2.."
        ECU_Status[8] = "Heating3.."
        ECU_Status[9] = "Heating4.."
        ECU_Status[10] = "Heating5.."
        ECU_Status[11] = "Heating6.."
        ECU_Status[12] = "Pump Acc.."
        ECU_Status[13] = "CTH.."
        ECU_Status[18] = "Idling.."
        ECU_Status[19] = "Acc.."
        ECU_Status[20] = "Dec.."
        ECU_Status[21] = "Speed.."
        ECU_Status[22] = "Max Speed.."
        ECU_Status[23] = "RC Learn"
        ECU_Status[24] = "RC Learning.."
        ECU_Status[25] = "RC Successful"
        ECU_Status[26] = "Restart"
        ECU_Status[27] = "Restart.."
        ECU_Status[28] = "Cooling.."
        ECU_Status[29] = "Error Status"
    end
end


-- Parser function


local function parse_window(win)

    local app_id = win[1] + (win[2] * 256)

    -- VSpeak telemetry adapter

    if POLL_ID == VSPEAK_ID then

        if app_id ~= 0x0400 and
           app_id ~= 0x0500 and
           app_id ~= 0x0600 and
           app_id ~= 0x0900 and
           app_id ~= 0x0910 and
           app_id ~= 0x0410 then

            return false
        end

        local b1 = win[3] or 0
        local b2 = win[4] or 0
        local b3 = win[5] or 0
        local b4 = win[6] or 0

        local raw_value =
            b1 +
            (b2 * 256) +
            (b3 * 65536) +
            (b4 * 16777216)

        local value = to_uint32(raw_value)

        if app_id == 0x0400 then
            if value > 1000 then
                return false
            end
            EGT = value

        elseif app_id == 0x0500 then
            if value > 250000 then
                return false
            end
            RPM_val = value

        elseif app_id == 0x0600 then
            Fuel = value

        elseif app_id == 0x0900 then
            if value < 100000 then
                Vbat = value / 100
            end

        elseif app_id == 0x0910 then
            if value < 100000 then
                Vpump = value / 100
            end

        elseif app_id == 0x0410 then
            if value < 100 then
                Status = value
            end
        end

        frames = frames + 1

        return true
    end

    -- XICOY telemetry adapter

    if POLL_ID == XICOY_ID then

        if app_id ~= 0x4400 and
           app_id ~= 0x4401 and
           app_id ~= 0x4403 and
           app_id ~= 0x4406 then

            return false
        end

        local b1 = win[3] or 0
        local b2 = win[4] or 0
        local b3 = win[5] or 0
        local b4 = win[6] or 0

        local raw_value =
            b1 +
            (b2 * 256) +
            (b3 * 65536) +
            (b4 * 16777216)

        local value = to_uint32(raw_value)

        if app_id == 0x4400 then
            if value > 1000 then
                return false
            end
            EGT = value

        elseif app_id == 0x4401 then
            if value > 250000 then
                return false
            end
            RPM_val = value

        elseif app_id == 0x4403 then
            if value < 100000 then
                Vbat = value / 10
            end

        elseif app_id == 0x4406 then
            if value < 100 then
                Status = value
            end
        end

        frames = frames + 1

        return true
    end

    return false
end

-- read S.Port stream

local function read_stream()

    local unescape_next = false

    while port:available() > 0 do

        local b = port:read()

        rx_bytes = rx_bytes + 1

        if b == 0x7D then

            unescape_next = true

        else

            if unescape_next then

                if b == 0x5E then
                    b = 0x7E

                elseif b == 0x5D then
                    b = 0x7D
                end

                unescape_next = false
            end

            table.insert(raw_bytes, b)

            while #raw_bytes >= 8 do

                local snapshot = {
                    raw_bytes[1],
                    raw_bytes[2],
                    raw_bytes[3],
                    raw_bytes[4],
                    raw_bytes[5],
                    raw_bytes[6],
                    raw_bytes[7],
                    raw_bytes[8]
                }

                if parse_window(snapshot) then

                    for _ = 1, 8 do
                        table.remove(raw_bytes, 1)
                    end

                else

                    table.remove(raw_bytes, 1)
                end
            end
        end
    end
end

local function send_poll()

    port:write(POLL_START)
    port:write(POLL_ID)
end

--main loop

function update()

    if not port then

        gcs:send_text(3, "NO SERIAL PORT")

        return update, 1000
    end

    local now = millis()

    read_stream()

    if now - last_poll >= POLL_INTERVAL then

        send_poll()

        last_poll = now
    end

    if now - last_report >= LOG_INTERVAL then
        logger:write('TURB', 'EGT,RPM,Fuel,Vbat,Vpmp,Stat', 'ffffff', EGT, RPM_val, Fuel, Vbat, Vpump, Status)
        --gcs:send_text(5, string.format("EGT:%d RPM:%d Vbat:%.1f", EGT, RPM_val, Vbat))

        -- Debug opzionale:
        -- gcs:send_text(6,
        --     string.format(
        --         "RX:%d FRAMES:%d BUF:%d",
        --         rx_bytes,
        --         frames,
        --         #raw_bytes
        --     )
        -- )

        last_report = now
    end

    if Status ~= Status_last then

        local msg =
            string.format(
                "ECU: %s",
                ECU_Status[Status] or
                ("Unknown(" .. Status .. ")")
            )

        gcs:send_text(6, msg)

        Status_last = Status
    end

    return update, 1
end


-- Initialization


if port then

    local ecu_type = ECU_TYPE:get()

    if ecu_type == 8 then
        POLL_ID = XICOY_ID
    else
        POLL_ID = VSPEAK_ID
    end

    port:begin(BAUD)

    initECU_Status()

    gcs:send_text(
        5,
        string.format(
            "ECU TELEMETRY RUNNING ID:0x%02X",
            POLL_ID
        )
    )

    send_poll()

    last_poll = millis()
end

return update, 1