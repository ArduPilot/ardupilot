
-- sub_test_synthetic_seafloor.lua
-- A simulated range finder driver returns distances based on the
-- vehicle location over a simulated seafloor topography. Used for
-- testing bottom tracking modes in ArduSub.
--
-- The synthetic seafloor topography is defined as a 1-D height profile (h=f(x)) that is extruded 
-- in the perpendicular direction to define a 2-D height function (h=f(x,y)).
--
-- The following Parameters can be set by the test script to control how this driver behaves
--  SCR_USER1 is an index into a table of configuration bundles.
--  SCR_USER2 is the average bottom depth in meters
--  SCR_USER3 is a bit field that controls driver logging.
--


local UPDATE_PERIOD_MS = 50

local TEST_ID_STR = "STSS"
local SCRIPT_NAME = "sub_test_synthetic_seafloor.lua"

local enable_logger_write = true
local enable_gcs_send_data = false
local enable_gcs_send_range = false

-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Type {}.
local RNGFND_TYPE_LUA = 36.0
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Status {}.
local RNGFND_STATUS_NO_DATA = 1
local RNGFND_STATUS_GOOD = 4
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h
local SIGNAL_QUALITY_MIN = 0
local SIGNAL_QUALITY_MAX = 100



-------------------------------------------------------------------------------

-- gcs messaging function. Pass two strings and the function will drop frequent messages

local gcs_send_funcfactory = function(name, eat_messages_period_s, msg_severity)
    local gcs_send_times = {}
    local gcs_eaten_count = {}

    ---@param str1 string
    ---@param str2 string|nil
    return function(str1, str2)
        if not str1 or #str1 == 0 then return end
        if not msg_severity then msg_severity = 6 end
        if not eat_messages_period_s then eat_messages_period_s = 1.0 end
        
        local send_str

        if not str2 or #str2 == 0 then
            send_str = string.format("%s: %s", name, str1)
        
        else
            local time_curr_s = millis():tofloat() / 1000.0
            local time_first_s = gcs_send_times[str1]
            if (time_first_s) then
                local dur_since_first = time_curr_s - time_first_s
                if dur_since_first < eat_messages_period_s then
                    if not gcs_eaten_count[str1] then
                        gcs_eaten_count[str1] = 0
                    end
                    gcs_eaten_count[str1] = gcs_eaten_count[str1] + 1
                    return
                end
            end

            local eaten_count = gcs_eaten_count[str1]
            if eaten_count then
                gcs_eaten_count[str1] = nil
                send_str = string.format("%s: %s %s (+%i)", name, str1, str2, eaten_count)
            else
                send_str = string.format("%s: %s %s", name, str1, str2)
            end

            gcs_send_times[str1] = time_curr_s
        end

        gcs:send_text(msg_severity, send_str)
    end
end

local send = gcs_send_funcfactory(TEST_ID_STR, 1.0, 3)
-- A send function used to send true range messages at a different rate.
local send_quick = gcs_send_funcfactory(TEST_ID_STR, 0.5, 3)

local function fatal_error(error)
    send(string.format("FATAL ERROR '%s': %s", SCRIPT_NAME, error))
end

-------------------------------------------------------------------------------

-- Profile definition and intersection used in Sea Floor Model

local function section_factory(x0, z0, x1, z1)

    -- Use cross product to see if self intersects with seg.
    -- Lines are defined by the two points (x0, z0) and (x0 + vx, z0 + vz).
    local function intersect(self, seg)
        local den = - self.vx*seg.vz + self.vz*seg.vx
        if math.abs(den) < 1.0e-6 then
            return nil
        end
        local x1m0 = seg.x0 - self.x0
        local z1m0 = seg.z0 - self.z0
        local t0 = - x1m0*seg.vz + z1m0*seg.vx
        local t1 = self.vx*z1m0 - self.vz*x1m0
        -- The returned values are the distance along the lines where the intersection occurs.
        -- The first is along self and the second is along seg
        return t0 / den, t1 / den
    end

    return {
        x0 = x0,
        z0 = z0,
        vx = x1 - x0,
        vz = z1 - z0,
        intersect = intersect,
    }
end

-- psi is an angle in radians rotating around the y axis with zero at the z axis
local function ray_factory(x, z, psi)
    return section_factory(x, z, x + math.sin(psi), z + math.cos(psi))
end

local function profile_factory(vertices)
    local sections = {}

    -- The first section covers to -infinity
    if #vertices > 0 then
        local vertex1 = vertices[1]
        table.insert(sections, section_factory(vertex1[1], vertex1[2], vertex1[1] - 1, vertex1[2]))
    end

    local last_vertex
    for _, vertex in pairs(vertices) do
        if not last_vertex then
            last_vertex = vertex
        else
            local section = section_factory(last_vertex[1], last_vertex[2], vertex[1], vertex[2])
            if math.abs(section.vx) > 1.0e-6 or math.abs(section.vy) > 1.0e-6 then
                -- Only add section if it has non-zero length, otherwise ignore this vertex.
                table.insert(sections, section)
                last_vertex = vertex
            end
        end
    end

    -- The last section covers to +infinity
    if last_vertex then
        table.insert(sections, section_factory(last_vertex[1], last_vertex[2], last_vertex[1] + 1, last_vertex[2]))
    end


    -- Returns the distance to the closest section.
    sections.intersect = function(self, ray)
        local d

        for i, segment in ipairs(self) do

            -- Return the distance from the start of the ray to the intersection
            -- with the segment. If there is no intersection or the intersection is
            -- not on the segment s < 0 or s > 1 then return nil. Note: the r distance
            -- can be negative if the intersection is on the ray before its start (r < 0)
            local function find_valid_intersection()
                local s, r = segment:intersect(ray)
                if not s then
                    return nil
                end
                if s < 0 then
                    return nil
                end
                if i ~= 1 and i ~= #self then
                    if s > 1 then
                        return nil
                    end
                end
                return r
            end

            local d1 = find_valid_intersection()
            if d1 then
                -- Found a valid intersection, look for the shortest distance
                if not d or d > d1 then
                    d = d1
                end
            end
        end

        return d
    end

    return sections
end

do  -- Some code to test functionality and illustrate usage
    local section = section_factory(0, 1, 1, 1)

    local function test(x, z, psi, s_expected, r_expected)
        local ray = ray_factory(x, z, psi)

        -- Test the section:intersect method.
        local s_actual, r_actual = section:intersect(ray)

        if not s_actual or not s_expected then
            if s_actual or s_expected then
                local actual_str = not s_actual and "nil" or string.format("%.2f", s_actual)
                local expected_str = not s_expected and "nil" or string.format("%.2f", s_expected)
                send(string.format("intersect not nil  : x %.2f, z %.2f, psi %.2f", x, z, psi) ..
                    string.format(" : s_actual %s, s_expected %s", actual_str, expected_str))
            end
        else
            if math.abs(s_actual - s_expected) > 1.0e-6 or math.abs(r_actual - r_expected) > 1.0e-6 then
                send(string.format("intersect failed  : x %.2f, z %.2f, psi %.2f", x, z, psi) ..
                    string.format(" : s_actual %.2f, s_expected %.2f, r_actual %.2f, r_expected %.2f", 
                        s_actual, s_expected, r_actual, r_expected))
            end
        end
    end

    test(-1, 0, 0,  -1, 1)
    test(0, 0, 0,  0, 1)
    test(1, 0, 0,  1, 1)
    test(2, 0, 0,  2, 1)

    test(-1, 0.5, 0,  -1, .5)
    test(0, 0.5, 0,  0, .5)
    test(1, 0.5, 0,  1, .5)
    test(2, 0.5, 0,  2, .5)

    test(-1, 1, 0,  -1, 0)
    test(0, 1, 0,  0, 0)
    test(1, 1, 0,  1, 0)
    test(2, 1, 0,  2, 0)

    test(-1, 2, 0,  -1, -1)
    test(0, 2, 0,  0, -1)
    test(1, 2, 0,  1, -1)
    test(2, 2, 0,  2, -1)

    test(0, 0, math.pi/2,  nil, 0)
    test(0, 1, math.pi/2,  nil, 0)
    test(1, 1, math.pi/2,  nil, 0)
    test(1, 2, math.pi/2,  nil, 0)

    test(0, 0, math.pi/4,  1, math.sqrt(2))
    test(1, 0, math.pi/4,  2, math.sqrt(2))
    test(0, 0, -math.pi/4,  -1, math.sqrt(2))
    test(1, 0, -math.pi/4,  0, math.sqrt(2))
end

do  -- Some code to test functionality and illustrate usage
    local profile = profile_factory({{0, 1}, {1, 2}})

    local function test(x, z, psi, d_expected)
        local ray = ray_factory(x, z, psi)

        -- Test the profile:intersect method.
        local d_actual = profile:intersect(ray)

        if math.abs(d_actual - d_expected) > 1.0e-6 then
            send(string.format("intersect failed x %.2f, z %.2f, psi %.2f, d_actual %.2f, d_expected %.2f",
                x, z, psi, d_actual, d_expected))
        end
    end

    test(-20, 0, 0, 1)
    test(-1, 0, 0, 1)
    test(0, 0, 0, 1)
    test(0.5, 0, 0, 1.5)
    test(1, 0, 0, 2)
    test(1.5, 0, 0, 2)
    test(20, 0, 0, 2)

    test(-20, 1, 0, 0)
    test(-1, 1, 0, 0)
    test(0, 1, 0, 0)
    test(0.5, 1, 0, 0.5)
    test(1, 1, 0, 1)
    test(1.5, 1, 0, 1)
    test(20, 1, 0, 1)

    test(-20, 2, 0, -1)
    test(-1, 2, 0, -1)
    test(0, 2, 0, -1)
    test(0.5, 2, 0, -0.5)
    test(1, 2, 0, 0)
    test(1.5, 2, 0, 0)
    test(20, 2, 0, 0)

    test(-20, 0, math.pi/4, math.sqrt(2))
    test(-1, 0, math.pi/4, math.sqrt(2))
    test(-0.99, 0, math.pi/4, 2*math.sqrt(2))
    test(0, 0, math.pi/4, 2*math.sqrt(2))
    test(0.5, 0, math.pi/4, 2*math.sqrt(2))
    test(1, 0, math.pi/4, 2*math.sqrt(2))
    test(1.5, 0, math.pi/4, 2*math.sqrt(2))
    test(20, 0, math.pi/4, 2*math.sqrt(2))

    test(-20, 0, -math.pi/4, math.sqrt(2))
    test(-1, 0, -math.pi/4, math.sqrt(2))
    test(0, 0, -math.pi/4, math.sqrt(2))
    test(0.5, 0, -math.pi/4, math.sqrt(2))
    test(1, 0, -math.pi/4, math.sqrt(2))
    test(1.5, 0, -math.pi/4, 1.25*math.sqrt(2))
    test(2, 0, -math.pi/4, 1.5*math.sqrt(2))
    test(3, 0, -math.pi/4, 2*math.sqrt(2))
    test(20, 0, -math.pi/4, 2*math.sqrt(2))
end


-------------------------------------------------------------------------------

-- NoiseModel

---@class NoiseModelConfig
---@field mean number                   -- Mean of the noise distribution (mean == 0 and std_dev == 0 => no gaussian noise)
---@field std_dev number                -- Standard deviation of the noise distribution
---@field outlier_rate_ops number       -- Rate of outliers outliers/second (0 => no outliers)
---@field outlier_mean number           -- Mean of outliers distribution
---@field outlier_std_dev number        -- Stardard deviation of outliers distribution
---@field delay_s number                -- Delay between measurement request and measurement return (0 => no delay)
---@field callback_interval_ms number   -- Delay between calls of the add noise function

-- This factory creates a function that will take a measurement m and add noise to it
---@param config NoiseModelConfig
local function add_noise_funcfactory(config)

    local function identity_funcfactory(pre_func)
        if pre_func then
            return pre_func
        end
        return function(m) return m end
    end


    local function noise_funcfactory(mean, std_dev, rate_ops, callback_interval_ms, pre_func)

        -- Use the Box-Muller algorithm to generate normally distributed error that is added to the sample.
        local function box_muller_func(m)
            return m + mean + std_dev * math.sqrt(-2 * math.log(math.random())) * math.cos(2 * math.pi * math.random())
        end

        local function gaussian_noise_funcfactory()
            if std_dev == 0.0 and pre_func then
                return function(m) return pre_func(m) + mean end
            end

            if std_dev == 0.0 then
                return function(m) return m + mean end
            end

            if pre_func then
                return function(m) return box_muller_func(pre_func(m)) end
            end

            return box_muller_func
        end

        -- Just simple normally distributed noise
        if rate_ops == 0.0 then
            return gaussian_noise_funcfactory()
        end

        -- Use poisson distribution to generate outliers

        -- Create a function to generate outliers
        local outlier_func = gaussian_noise_funcfactory()

        -- Rate of outlier events in a callback interval
        local rate_opi = rate_ops * callback_interval_ms / 1000.0

        -- Poisson probability of zero events in an interval - Poisson formula is just exp in this case
        local poisson_prob_zero = math.exp(-rate_opi)

        return function(m)
            -- Poisson probability of 1 or more events in this interval is 1-P(0)
            if math.random() > poisson_prob_zero then
                return outlier_func(m)  -- NOTE: pre_func is invoked in outlier_func.
            end
            if pre_func then
                return pre_func(m)
            end
            return m
        end
    end


    local function delay_funcfactory(delay_s, callback_interval_ms, pre_func)

        if delay_s == 0.0 or callback_interval_ms == 0 then
            return identity_funcfactory(pre_func)
        end

        local delay_line = {}
        local delay_count = math.ceil(delay_s / callback_interval_ms * 1000.0)
        if delay_count <= 0 then
            return identity_funcfactory(pre_func)
        end

        local next_idx = -1
        local function delay_func(m)
            if pre_func then
                m = pre_func(m)
            end

            if next_idx < 1 then
                for i = 1, delay_count do
                    delay_line[i] = m
                end
                next_idx = 1
            end

            local m_delay = delay_line[next_idx]
            delay_line[next_idx] = m
            next_idx = next_idx + 1
            if next_idx > #delay_line then
                next_idx = 1
            end
            return m_delay
        end

        return delay_func
    end

    local func

    -- Check for adding gaussian noise to measrement
    if config.mean ~= 0.0 or config.std_dev ~= 0.0 then
        func = noise_funcfactory(config.mean, config.std_dev, 0.0, 0.0)
    end

    -- Check for adding an outlier measurement
    if config.outlier_rate_ops ~= 0 then
        func = noise_funcfactory(config.outlier_mean, config.outlier_std_dev,
            config.outlier_rate_ops, config.callback_interval_ms, func)
    end

    -- Check for delaying the measurement
    if config.delay_s ~= 0.0 then
        func = delay_funcfactory(config.delay_s, config.callback_interval_ms, func)
    end

    if func == nil then
        func = identity_funcfactory()
    end

    return func
end

-------------------------------------------------------------------------------

-- Range Model

---@class RangeModel
---@field get_range function(RangeModel, location_ud): number
---@field sub_z_m number
---@field bottom_z_m number
---@field range_m number
---@field set_origin function(RangeModel, location_ud)
---@field is_origin_valid function(RangeModel)

---@return RangeModel
local function range_model_factory(model_bearing_N_rad, model_depth_m, vertices)

    ---@type Location_ud
    local origin_loc

    local profile = profile_factory(vertices)
  
    ---@param sub_loc Location_ud
    local function get_range(self, sub_loc)

        -- Figure out the depth of the sub in absolute frame
        sub_loc:change_alt_frame(0)
        self.sub_z_m = -sub_loc:alt()/100

        -- If the origin has not been set then do not use the profile.
        -- The seafloor model needs to have an origin. The origin of the model is set to
        -- the location of the sub when it is armed. If the model’s origin hasn’t been set,
        -- the driver returns distances based on a constant depth sea floor.
        if not origin_loc then
            self.bottom_z_m = model_depth_m
            self.range_m = model_depth_m - self.sub_z_m
            return self.range_m
        end

        -- The model origin has been set so use the profile.
        -- N in name means relative to Earth frame, M in name means relative to Sea floor model frame.
        local sub_bearing_N_rad = origin_loc:get_bearing(sub_loc)
        local sub_distance_M_m = origin_loc:get_distance(sub_loc)

        local sub_bearing_M_rad = sub_bearing_N_rad - model_bearing_N_rad
        local sub_northly_M_m = math.cos(sub_bearing_M_rad) * sub_distance_M_m

        -- profile:intersect has an origin at zero so we have to add in the model
        -- depth to get the seafloor depth at this location.
        -- For the sea floor depth:
        -- bottom_z = model_depth_m - profile(0)
        -- Positive z is down, positive profile is up
        local bottom_ray = ray_factory(sub_northly_M_m, 0, 0)
        self.bottom_z_m = model_depth_m - profile:intersect(bottom_ray)

        -- For the sub range measurement:
        -- model_range = model_depth_m - sub_z
        self.range_m = self.bottom_z_m - self.sub_z_m
        return self.range_m
    end

    return {
        get_range = get_range,
        sub_z_m = 0,
        bottom_z_m = model_depth_m,
        range_m = model_depth_m,
        set_origin = function(origin) origin_loc = origin end,
        is_origin_valid = function() return origin_loc ~= nil end,
    }
end

-------------------------------------------------------------------------------

-- Range Finder Driver

-- The range finder backend is initialized in the update_init function.
---@type AP_RangeFinder_Backend_ud
local rngfnd_backend

-- The range_model and add_noise_func are initialized when vehicle is armed
---@type RangeModel
local range_model

local measurement_noise_func
local signal_quality_noise_func



local function range_finder_driver(sub_loc)
    local rf_state = RangeFinder_State()

    -- The full state udata must be initialized.
    rf_state:last_reading(millis():toint())
    rf_state:voltage(0)

    -- If no location, then return no data
    if not sub_loc then
        rf_state:status(RNGFND_STATUS_NO_DATA)
        rf_state:range_valid_count(0)
        rf_state:distance(0)
        rf_state:signal_quality(SIGNAL_QUALITY_MIN)
        rngfnd_backend:handle_script_msg(rf_state) -- state as arg
        return
    end

    -- Generate a simulated range measurement
    local true_range_m = range_model:get_range(sub_loc)
    local range_m = measurement_noise_func(true_range_m)
    local signal_quality = signal_quality_noise_func(SIGNAL_QUALITY_MAX)

    -- Return this measurement to the range finder backend
    rf_state:status(RNGFND_STATUS_GOOD)
    rf_state:range_valid_count(10)
    rf_state:distance(range_m)
    rf_state:signal_quality(signal_quality)
    rngfnd_backend:handle_script_msg(rf_state) -- state as arg

    -- Log this data
    if enable_logger_write then
        logger:write('RNFN', 'sub_z,bottom_z,true_range,range,quality', 'fffff', 'mmmmm', '-----',
            range_model.sub_z_m, range_model.bottom_z_m, true_range_m, range_m, signal_quality)
        -- This data can be viewed in mavexplorer with the following command:
        -- graph RNFN.sub_z RNFN.bottom_z RNFN.true_range RNFN.range
    end
    if enable_gcs_send_data then
        send("RNGFND", string.format("true range %.2f, range %.2f, sub_z %.2f, bottom_z %.2f",
            true_range_m, range_m, range_model.sub_z_m, range_model.bottom_z_m))
    end
    if enable_gcs_send_range then
        send_quick("#TR#", string.format("%7.2f", true_range_m))
    end
end

-------------------------------------------------------------------------------

local function initialize_model()

    -- query SCR_USERx for parameters
    -- SCR_USER1 is a code for which config bundle to use
    local config_index = param:get('SCR_USER1')
    if not config_index then
        config_index = 1
    end
    -- SCR_USER2 is the bottom depth
    local bottom_depth_m = param:get('SCR_USER2')
    if not bottom_depth_m or bottom_depth_m < 1 then
        bottom_depth_m = 50
    end
    -- SCR_USER3 contains bits for logging
    local logging_bits = param:get('SCR_USER3')
    if not logging_bits then
        logging_bits = 1
    end


    -- Set logging flags from logging_bits
    local lb_str = tostring(math.floor(logging_bits))
    enable_logger_write = string.sub(lb_str, -1, -1) == '1'
    enable_gcs_send_data = string.sub(lb_str, -2, -2) == '1'
    enable_gcs_send_range = string.sub(lb_str, -3, -3) == '1'

    local config_simple_ridge = {{5, 0}, {30, 10}, {40, 10}, {50, 0}}
    local config_ridge_plateau = {{5, 0}, {30, 10}, {40, 10}, {50, 0}, {70, 0}, {90, -10}}

    local config_range_model = {
        model_bearing_N_rad = math.pi,
        vertices = config_simple_ridge,
    }

    local config_measurement_noise = {
        mean = 0.0,
        std_dev = 0.0,
        outlier_rate_ops = 0.0,
        outlier_mean = 0.0,
        outlier_std_dev = 0.0,
        delay_s = 0.0,
        callback_interval_ms = UPDATE_PERIOD_MS,
    }

    local config_signal_quality_noise = {
        mean = 0.0,
        std_dev = 0.0,
        outlier_rate_ops = 0.0,
        outlier_mean = 0.0,
        outlier_std_dev = 0.0,
        delay_s = 0.0,
        callback_interval_ms = UPDATE_PERIOD_MS,
    }

    -- config_index = 1 is default with no noise
    -- config_index = 2 has a little noise
    if config_index == 2 then
        config_range_model.vertices = config_ridge_plateau
        config_measurement_noise.std_dev = .1
        config_measurement_noise.outlier_rate_ops = .2
        config_measurement_noise.outlier_mean = 5
        config_measurement_noise.outlier_std_dev = 2
        config_measurement_noise.delay_s = 0.00
    end

    range_model = range_model_factory(config_range_model.model_bearing_N_rad,
    bottom_depth_m, config_range_model.vertices)
    measurement_noise_func = add_noise_funcfactory(config_measurement_noise)


    -- Constrain signal quality values
    local signal_quality_noise_pre = add_noise_funcfactory(config_signal_quality_noise)
    signal_quality_noise_func = function(m)
        m = signal_quality_noise_pre(m)
        if m > SIGNAL_QUALITY_MAX then
            return SIGNAL_QUALITY_MAX
        end
        if m < SIGNAL_QUALITY_MIN then
            return SIGNAL_QUALITY_MIN
        end
        return m
    end

end

-------------------------------------------------------------------------------

-- update functions

local function update_run()

    local loc_c = ahrs:get_location()

    -- Check if we have to set or clear the origin
    if arming:is_armed() ~= range_model.is_origin_valid() then
        if arming:is_armed() then
            if loc_c then
                send("Starting to use the sea floor model for range data.")
                range_model.set_origin(loc_c)
            end
        else
            send("Stopping sea floor model range data. Starting to use flat sea floor for range data.")
            range_model.set_origin(nil)
        end
    end

    -- Update with range finder driver
    range_finder_driver(loc_c)

    return update_run, UPDATE_PERIOD_MS
end

local function update_init()
    if Parameter('RNGFND1_TYPE'):get() ~= RNGFND_TYPE_LUA then
        return fatal_error("LUA range finder driver not enabled")
    end
    if rangefinder:num_sensors() < 1 then
        return fatal_error("LUA range finder driver not connected")
    end
    rngfnd_backend = rangefinder:get_backend(0)
    if not rngfnd_backend then
        return fatal_error("Range Finder 1 does not exist")
    end
    if (rngfnd_backend:type() ~= RNGFND_TYPE_LUA) then
        return fatal_error("Range Finder 1 is not a LUA driver")
    end

    initialize_model()

    if not range_model or not measurement_noise_func or not signal_quality_noise_func then
        return fatal_error("Could not initialize model")
     end

    return update_run, 0
end

send(string.format("Loaded %s", SCRIPT_NAME))

return update_init, 0
