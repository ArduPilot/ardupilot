-- Run in ArduPilot Lua via QuadPlane.ServoFailDetectRegression.
-- Simulated inputs make scheduling and fault cases deterministic; timestamps
-- use the firmware's uint32_t binding, including across millis() rollover.

local applet = "scripts/modules/ServoFailDetect.lua"
local file = assert(io.open(applet, "r"))
local applet_source = file:read("a")
file:close()
local override_ms = 120

local function new_sim(parameters, config, version)
    version = version or {4, 7}
    local servo_telem_one_indexed = (version[1] < 4) or
                                    ((version[1] == 4) and (version[2] < 7))
    local sim = {
        now = uint32_t(0), armed = true, safety = false, vtol = true, assisted = false,
        params = {
            Q_M_PWM_MIN = 1000, Q_TILT_MASK = 20,
            SERVO15_FUNCTION = 75, SERVO16_FUNCTION = 76,
            SERVO11_FUNCTION = 35, SERVO13_FUNCTION = 37,
            SERVO15_MIN = 1000, SERVO15_MAX = 2000,
            SERVO16_MIN = 1000, SERVO16_MAX = 2000,
        },
        pwm = {[11] = 1100, [13] = 1100, [15] = 1000, [16] = 1000},
        position = {[15] = -57.5, [16] = -57.5},
        telem = {}, telem_requests = {}, overrides = {}, expiries = 0, cuts = {}, logs = {}, messages = {},
    }
    for k, v in pairs(parameters or {}) do
        sim.params[k] = v
    end

    local function expire_overrides()
        for channel, start_ms in pairs(sim.overrides) do
            if (sim.now - start_ms):tofloat() >= override_ms then
                sim.overrides[channel] = nil
                sim.pwm[channel] = 1100
                sim.expiries = sim.expiries + 1
            end
        end
    end

    local env = setmetatable({}, {__index = _ENV})
    function env.millis() return sim.now end
    function env.Parameter(name)
        assert(sim.params[name] ~= nil, "missing parameter " .. name)
        return {get = function () return sim.params[name] end}
    end
    env.param = {
        get = function (_, name) return sim.params[name] end,
    }
    env.arming = {is_armed = function () return sim.armed end}
    env.quadplane = {
        in_vtol_mode = function () return sim.vtol end,
        in_assisted_flight = function () return sim.assisted end,
    }
    env.FWVersion = {
        major = function () return version[1] end,
        minor = function () return version[2] end,
    }
    env.gcs = {send_text = function (_, _severity, message) sim.messages[#sim.messages + 1] = message end}
    env.logger = {write = function (_, name, _labels, _format, ...)
        local values = {...}
        if name == "SCHK" then
            sim.logs[values[3] + 1] = {expected = values[4], connected = values[5], state = values[6]}
        end
        -- Simulate descheduling midway through a callback.
        if sim.stall_in_log then
            sim.now = sim.now + sim.stall_in_log
            sim.stall_in_log = nil
        end
    end}
    env.servo_telem = {get_telem = function (_, channel)
        sim.telem_requests[#sim.telem_requests + 1] = channel
        local output_channel = channel + (servo_telem_one_indexed and 0 or 1)
        if sim.on_telem_get ~= nil then sim:on_telem_get(output_channel) end
        return sim.telem[output_channel]
    end}
    env.SRV_Channels = {
        get_safety_state = function () return sim.safety end,
        get_output_pwm_chan = function (_, channel)
            expire_overrides()
            return sim.pwm[channel + 1]
        end,
        set_output_pwm_chan_timeout = function (_, channel, pwm, timeout)
            channel = channel + 1
            expire_overrides()
            -- The firmware writes PWM even when timeout is zero.
            sim.pwm[channel] = pwm
            if timeout == 0 then
                sim.overrides[channel] = nil
            else
                assert(timeout == override_ms and pwm == sim.params.Q_M_PWM_MIN)
                for c in pairs(sim.overrides) do
                    assert(c == channel, "overlapping motor shutdowns")
                end
                sim.overrides[channel] = sim.now
                sim.cuts[#sim.cuts + 1] = {channel = channel, time = sim.now}
            end
        end,
    }

    local source = applet_source
    if config then
        local rows = {"local servo_config = {"}
        for _, row in ipairs(config) do
            rows[#rows + 1] = string.format("{%s, %q, %s, %s, %s},", tostring(row[1]), row[2],
                                           tostring(row[3]), tostring(row[4]), tostring(row[5]))
        end
        rows[#rows + 1] = "}"
        local replacements
        source, replacements = source:gsub("local servo_config = %b{}", function () return table.concat(rows, "\n") end)
        assert(replacements == 1)
    end
    local update = assert(load(source, applet, "t", env))()
    function sim:step(dt, fresh)
        self.now = self.now + (dt or 20)
        expire_overrides()
        if fresh ~= false then
            for channel, pos in pairs(self.position) do
                local timestamp = self.now
                self.telem[channel] = {
                    last_update_ms = function () return timestamp end,
                    measured_position = function () return pos end,
                }
            end
        end
        local delay
        update, delay = update()
        assert(delay == 20)
    end
    function sim:run(duration, dt, fresh)
        for _ = 1, math.ceil(duration / (dt or 20)) do self:step(dt, fresh) end
    end
    function sim:healthy()
        self:run(2200)
        assert(self.logs[15].state == 1 and self.logs[16].state == 1)
        assert(#self.cuts == 0)
    end
    return sim
end

local tests = {}

function tests.defaults_and_healthy()
    local s = new_sim()
    s:healthy()
    assert(s.messages[1] == "SRVFAIL: Left Tilt servo 15 motor 11")
    assert(s.messages[2] == "SRVFAIL: Right Tilt servo 16 motor 13")
    s:run(10000, 80)
    assert(#s.cuts == 0)
end

function tests.telemetry_index_versions()
    for _, case in ipairs({
        {{4, 6}, 15, 16},
        {{4, 7}, 14, 15},
        {{5, 0}, 14, 15},
    }) do
        local s = new_sim(nil, nil, case[1])
        s:step()
        assert(s.telem_requests[1] == case[2] and s.telem_requests[2] == case[3],
               "incorrect servo telemetry indexing")
        s:healthy()
    end
end

function tests.elapsed_time_model()
    local s = new_sim()
    s:healthy()
    s.pwm[15] = 2000
    for _, dt in ipairs({20, 40, 80, 100, 60}) do
        local before = s.logs[15].expected
        s:step(dt)
        local expected_change = dt * 0.001 * 500 * 115 / 1000
        assert(math.abs(s.logs[15].expected - before - expected_change) < 0.00001)
    end
end

function tests.individual_failures()
    for _, side in ipairs({{15, 11}, {16, 13}}) do
        local s = new_sim()
        s:healthy()
        s.pwm[side[1]] = 2000
        s:run(5000)
        assert(#s.cuts > 0 and s.overrides[side[2]])
        for _, cut in ipairs(s.cuts) do assert(cut.channel == side[2]) end
        s.position[side[1]] = 57.5
        s:step()
        assert(next(s.overrides) == nil)
    end
end

function tests.both_failed_and_pending_error()
    local s = new_sim()
    s:healthy()
    s.pwm[15], s.pwm[16] = 2000, 2000
    s:run(5000)
    assert(s.logs[15].state == 2 and s.logs[16].state == 2 and #s.cuts == 0)
    s.position[16] = 57.5
    s:step()
    assert(s.overrides[11])
    -- A pending fault on the other side already releases the existing cut.
    s.position[16] = -57.5
    s:step()
    assert(next(s.overrides) == nil)
    s:run(2200)
    local cuts = #s.cuts
    s:run(1000)
    assert(#s.cuts == cuts)
    s.position[15] = 57.5
    s:run(100)
    assert(s.overrides[13])
end

function tests.scheduling_gap_restarts_detection()
    local s = new_sim()
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    assert(s.overrides[11])
    local expiries = s.expiries
    for _, gap in ipairs({40, 60, 80, 100}) do
        s:step(gap)
        assert(s.overrides[11] and s.pwm[11] == 1000)
        assert(s.expiries == expiries, "motor shutdown expired during an accepted callback gap")
    end
    local cuts = #s.cuts
    s:step(5000)
    assert(#s.cuts == cuts and next(s.overrides) == nil)
    assert(s.pwm[11] == 1100, "release reapplied shutdown PWM after expiry")
    s:run(3900)
    assert(#s.cuts == cuts)
    s:run(400)
    assert(#s.cuts > cuts)
    -- Exercise healthy servos moving during repeated 1--8 second stalls.
    s = new_sim()
    s:healthy()
    for _, gap in ipairs({1000, 8000, 5000, 2000, 5000}) do
        s.pwm[15], s.pwm[16] = 2000, 2000
        s.position[15], s.position[16] = 57.5, 57.5
        s:step(gap)
    end
    s:run(5000)
    assert(#s.cuts == 0)
end

function tests.stall_inside_callback()
    local s = new_sim()
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    local cuts = #s.cuts
    s.stall_in_log = 5000
    s:step()
    assert(#s.cuts == cuts and next(s.overrides) == nil)
    assert(s.pwm[11] == 1100, "release reapplied shutdown PWM after expiry")
    s:run(3900)
    assert(#s.cuts == cuts)

    -- The time since the last renewal matters even when neither the callback
    -- interval nor the time spent in this callback exceeds MAX_GAP_MS alone.
    s = new_sim()
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    cuts = #s.cuts
    local expiries = s.expiries
    s.stall_in_log = 70
    s:step(60)
    assert(#s.cuts == cuts and next(s.overrides) == nil)
    assert(s.expiries == expiries + 1 and s.pwm[11] == 1100,
           "expired shutdown was renewed after a combined scheduling delay")
end

function tests.telemetry_loss_and_recovery()
    local s = new_sim()
    s:healthy()
    s.position[15] = nil
    s:run(2900)
    assert(#s.cuts == 0)
    s:run(300)
    assert(s.logs[15].state == 3 and s.overrides[11])
    s.position[15] = -57.5
    s:step()
    assert(next(s.overrides) == nil and s.logs[15].state == 1)
    s:run(2200)
    assert(s.logs[15].state == 1)
    s.position[15], s.position[16] = nil, nil
    local cuts = #s.cuts
    s:run(5000)
    assert(#s.cuts == cuts and next(s.overrides) == nil)

    local function check_failed_servo_loss(position)
        s = new_sim()
        s:healthy()
        s.pwm[15] = 2000
        s:run(5000)
        assert(s.overrides[11])
        s.position[15] = position
        s:step()
        assert(s.overrides[11] and s.logs[15].state == 2)
        s.position[15] = -57.5
        s:step()
        assert(s.overrides[11] and s.logs[15].state == 2,
               "telemetry recovery released a confirmed servo failure")
        s.position[15] = position
        s:run(4000)
        assert(s.overrides[11] and s.logs[15].state == 3)
        s.position[15] = -57.5
        s:step()
        assert(s.overrides[11] and s.logs[15].state == 3,
               "telemetry recovery released a disconnected servo")
        s.position[15] = 57.5
        s:step()
        assert(next(s.overrides) == nil and s.logs[15].state == 1)
    end
    check_failed_servo_loss(nil)
    check_failed_servo_loss(0/0)
end

function tests.repeated_and_invalid_feedback()
    local s = new_sim()
    s:healthy()
    s.on_telem_get = function (self, channel)
        if channel ~= 15 then return end
        self.now = self.now + 1
        local timestamp = self.now
        local pos = self.position[channel]
        self.telem[channel] = {
            last_update_ms = function () return timestamp end,
            measured_position = function () return pos end,
        }
        self.on_telem_get = nil
    end
    s:step(20, false)
    assert(s.logs[15].connected and s.logs[15].state == 1,
           "telemetry newer than the callback start was treated as stale")
    s.pwm[15] = 2000
    s:run(3000, 20, false)
    assert(#s.cuts == 0 and s.logs[15].state ~= 2)
    s = new_sim()
    s:healthy()
    s.position[15] = 0/0
    s:run(2200)
    assert(s.logs[15].state == 3 and s.overrides[11])
end

function tests.interrupted_position_error()
    local s = new_sim()
    s:healthy()
    s.position[15] = 57.5
    s:run(1500)
    s.position[15] = -57.5
    s:step()
    s.position[15] = 57.5
    s:run(1900)
    assert(#s.cuts == 0)
    s:run(200)
    assert(s.overrides[11])
end

function tests.startup_loss_and_gap()
    local s = new_sim()
    s.position[15] = nil
    s:run(1900)
    assert(#s.cuts == 0)
    s:run(300)
    assert(s.overrides[11])
    local cuts = #s.cuts
    s:step(5000)
    s:run(1900)
    assert(#s.cuts == cuts and next(s.overrides) == nil)
    s:run(300)
    assert(s.overrides[11])
end

function tests.invalid_pwm_restarts_detection()
    local s = new_sim()
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    assert(s.overrides[11])
    s.pwm[15] = 0
    s:step()
    assert(next(s.overrides) == nil)
    local cuts = #s.cuts
    s.pwm[15] = 2000
    s:run(3900)
    assert(#s.cuts == cuts)
    s:run(400)
    assert(s.overrides[11])
end

function tests.safety_and_arming()
    for _, flag in ipairs({"safety", "armed"}) do
        local s = new_sim()
        s:healthy()
        s.pwm[15] = 2000
        s:run(5000)
        assert(s.overrides[11])
        if flag == "safety" then s.safety = true end
        if flag == "armed" then s.armed = false end
        local cuts = #s.cuts
        s:run(100)
        assert(#s.cuts == cuts and next(s.overrides) == nil)
        s.safety, s.armed = false, true
        s:run(3900)
        assert(#s.cuts == cuts)
        s:run(400)
        assert(#s.cuts > cuts)
    end
end

function tests.flight_modes()
    local s = new_sim()
    s.vtol, s.assisted = false, true
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    assert(#s.cuts == 0) -- unknown transition history at startup
    s.vtol = true
    s:step()
    assert(s.overrides[11])
    local cuts = #s.cuts
    s.vtol = false
    s:run(100)
    assert(#s.cuts == cuts and next(s.overrides) == nil) -- forward transition
    s.assisted = false
    s:run(100)
    s.assisted = true
    s:step()
    assert(s.overrides[11]) -- assisted flight after completed transition
    cuts = #s.cuts
    s:step(5000)
    s:run(5000)
    assert(#s.cuts == cuts) -- history invalidated by the gap
end

function tests.reversed_calibration_and_rollover()
    local s = new_sim(nil, {{15, "Left Tilt", 11, 57.5, -57.5}, {16, "Right Tilt", 13, 57.5, -57.5}})
    s.now = uint32_t(0) - 1000
    s.position[15], s.position[16] = 57.5, 57.5
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    assert(s.overrides[11])
    s.position[15] = -57.5
    s:step()
    assert(next(s.overrides) == nil)
end

function tests.explicit_channels_and_motor9_mapping()
    local s = new_sim({Q_TILT_MASK = 768, SERVO11_FUNCTION = 82, SERVO13_FUNCTION = 83},
                      {{15, "Left Tilt", 13, -57.5, 57.5}, {16, "Right Tilt", 11, -57.5, 57.5}})
    s:healthy()
    s.pwm[15] = 2000
    s:run(5000)
    assert(s.overrides[13])
    s = new_sim({Q_TILT_MASK = 768, SERVO11_FUNCTION = 82, SERVO13_FUNCTION = 83},
                {{0, "Left Tilt", 0, -57.5, 57.5}, {0, "Right Tilt", 0, -57.5, 57.5}})
    s:healthy()
    assert(s.messages[1] == "SRVFAIL: Left Tilt servo 15 motor 11")
    assert(s.messages[2] == "SRVFAIL: Right Tilt servo 16 motor 13")
end

function tests.monitor_only_and_independent_calibration()
    local s = new_sim(nil, {{15, "Left Tilt", nil, -45, 65}, {16, "Right Tilt", 13, -57.5, 57.5}})
    s.position[15] = -45
    s:healthy()
    assert(s.messages[1] == "SRVFAIL: Left Tilt servo 15 motor none")
    s.pwm[15] = 2000
    s:run(5000)
    assert(s.logs[15].state == 2 and #s.cuts == 0)
    s.position[15] = 65
    s.pwm[16] = 2000
    s:run(5000)
    assert(s.overrides[13])
    s = new_sim(nil, {{15, "Left Tilt", nil, -57.5, 57.5}, {16, "Right Tilt", nil, -57.5, 57.5}})
    s:healthy()
    s.position[15] = nil
    s:run(5000)
    assert(s.logs[15].state == 3 and #s.cuts == 0)
end

function tests.invalid_configuration()
    for _, params in ipairs({
        {Q_TILT_MASK = 0}, {Q_TILT_MASK = 1}, {Q_TILT_MASK = 7},
        {SERVO15_MAX = 1000}, {SERVO11_FUNCTION = 33},
    }) do
        local ok, err = pcall(function () new_sim(params) end)
        assert(not ok and tostring(err):find("SRVFAIL:"), "invalid configuration was accepted")
    end
    for _, config in ipairs({
        {{-1, "Left Tilt", 11, -57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{15, "Left Tilt", 11.5, -57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{15, "Left Tilt", 33, -57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{16, "Left Tilt", 11, -57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{15, "Left Tilt", 13, -57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{15, "Left Tilt", 15, -57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{15, "Left Tilt", 11, 57.5, 57.5}, {16, "Right Tilt", 13, -57.5, 57.5}},
        {{15, "Left Tilt", 11, -57.5, 57.5}},
    }) do
        local ok, err = pcall(function () new_sim(nil, config) end)
        assert(not ok and tostring(err):find("SRVFAIL:"), "invalid servo table was accepted")
    end
    for _, params in ipairs({{SERVO14_FUNCTION = 75}, {SERVO15_FUNCTION = 0}}) do
        local ok, err = pcall(function ()
            new_sim(params, {{0, "Left Tilt", 0, -57.5, 57.5}, {0, "Right Tilt", 0, -57.5, 57.5}})
        end)
        assert(not ok and tostring(err):find("SRVFAIL:"), "ambiguous/missing automatic mapping was accepted")
    end
end

local names = {}
for name in pairs(tests) do
    names[#names + 1] = name
end
table.sort(names)

local index = 0
local function run_next()
    index = index + 1
    local name = names[index]
    if name == nil then
        gcs:send_text(6, string.format("Passed %u ServoFailDetect regression tests", #names))
        return
    end
    gcs:send_text(6, "SFD test: " .. name)
    tests[name]()
    return run_next, 100
end

return run_next, 100
