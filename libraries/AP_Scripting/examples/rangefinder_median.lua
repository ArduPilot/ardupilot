
local RANGEFINDER_TYPE_SCRIPT = 36
local ERROR_DIST = 130

local RNGFND_STATUS = {
    NotConnected   = 0,
    NoData         = 1,
    OutOfRangeLow  = 2,
    OutOfRangeHigh = 3,
    Good           = 4
}

local CALC_STATUS = {
    Median = 1,
    Mean = 2,
    SingleValue = 3,
    AllOutOfRangeHigh = 4,
    AllOutOfRangeLow = 5,
    NoData = 6
}

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

-- Setup params
local PARAM_TABLE_KEY = 76
assert(param:add_table(PARAM_TABLE_KEY, "ML_", 1), 'could not add param table')

assert(param:add_param(PARAM_TABLE_KEY, 1, 'SEND_VALS', 0), 'could not add ML_SEND_VALS')
-- assert(param:add_param(PARAM_TABLE_KEY, 2, 'TEST2', 5.7), 'could not add param2')

local send_vals = Parameter("ML_SEND_VALS")

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("Rngfnd Med Scr: Could not find %s parameter", name))
    return p
end

-- Rangefinder "Class"
local function Lidar(inst, angle_deg)
    local self = {}

    local status = RNGFND_STATUS.NoData

    -- Sanity check the installed angle to avoid mistakes
    if (math.abs(angle_deg) > 85) then
        error("Rangefinder [" .. inst .. "] install angle > 85 is unrealistic")
    end

    -- Store the installed angle in radians
    local install_angle = angle_deg / 180 * math.pi

    -- Setup the rangefinder backend
    local backend = rangefinder:get_backend(inst)
    if (backend == nil) then
        error("Rangefinder [" .. inst .. "] is nil")
    end

    -- get min max params
    local param_min = bind_param(string.format("RNGFND%i_MIN", inst+1))
    local param_max = bind_param(string.format("RNGFND%i_MAX", inst+1))

    function self.get_reading()

        -- Update status
        status = backend:status()
        if (status == nil) then
            status = RNGFND_STATUS.NoData
            return nil
        end

        -- Note: We do not discard out of range low and high values. Instead we use them and correct for the installed angle.
        -- This will help for smooth transitions for the median value in- and out- of range.
        if ((status == RNGFND_STATUS.NoData) or (status == RNGFND_STATUS.NotConnected)) then
            return nil
        end

        -- Calculate the corrected distance to account for the install angle
        local dist = backend:distance()

        -- Handle the out of range cases because rangefinder does not do the constraining for us
        if (status == RNGFND_STATUS.OutOfRangeLow) then
            local min_dist = param_min:get()
            if (min_dist ~= nil) then
                dist = min_dist
            else
                status = RNGFND_STATUS.NoData
                return nil
            end
        end
        if (status == RNGFND_STATUS.OutOfRangeHigh) then
            local max_dist = param_max:get()
            if (max_dist ~= nil) then
                dist = max_dist
            else
                status = RNGFND_STATUS.NoData
                return nil
            end
        end

        if (dist ~= nil) then
            return dist * math.cos(install_angle)
        end

        -- If we got this far, something went wrong
        status = RNGFND_STATUS.NoData
        return nil

    end

    function self.min_val()
        return param_min:get()
    end

    function self.max_val()
        return param_max:get()
    end

    function self.status()
        return status
    end

    return self
end



-- Setup rangefinders
range_finders = {}
local installation_angles = {25, 5, -15}
local n_rngfnd = #installation_angles
-- index 0 is going to be the scripting index
for i=1, n_rngfnd do
    table.insert(range_finders, Lidar(i, installation_angles[i]))
end

-- Check that the first instance is the lua scripting backend instance
local output_rngfnd = rangefinder:get_backend(0)
if ((output_rngfnd == nil) or (output_rngfnd:type() ~= RANGEFINDER_TYPE_SCRIPT)) then
    error("Rangefinder1 type needs to be scripting (36)")
end

gcs:send_text(MAV_SEVERITY.INFO, "LiDaR script setup succesfully")

local range_valid_count = 0
local function update()

    local distances = {}
    local out_of_range_low_count = 0
    local out_of_range_high_count = 0
    local low_range_value = ERROR_DIST
    local high_range_value = 0

    for i=1, #range_finders do
        local dist = range_finders[i]:get_reading()

        -- Optionally report reading over telem
        if (send_vals:get() > 0) then
            -- Guard against trying to send nils via named float
            local send_dist = dist
            if (send_dist == nil) then
                send_dist = ERROR_DIST
            end
            gcs:send_named_float(string.format("MLD%i", i), send_dist)
        end

        -- update the min max values for this rangefinder based on all of the others
        low_range_value = math.min(low_range_value, range_finders[i]:min_val())
        high_range_value = math.max(high_range_value, range_finders[i]:max_val())

        -- Track out of range status to catch when all inputs are out of range
        local status = range_finders[i]:status()
        if (status == RNGFND_STATUS.OutOfRangeLow) then
            out_of_range_low_count = out_of_range_low_count + 1
        end
        if (status == RNGFND_STATUS.OutOfRangeHigh) then
            out_of_range_high_count = out_of_range_high_count + 1
        end

        -- only commit the distance to the table if its a valid reading
        if ((dist ~= nil) and (status == RNGFND_STATUS.Good)) then
            table.insert(distances, dist)
        end

        -- Logging
        -- function logger:write(name, labels, format, units, multipliers, ...)
        local log_dist = dist
        if (log_dist == nil) then
            log_dist = ERROR_DIST
        end
        logger:write('RNGM', 'I,Dist,Case,Count', 'BfBB', '#m--', '-0-0', i, log_dist, 0, 0)
    end

    -- init rangefinder state
    local this_state = RangeFinder_State()
    this_state:status(RNGFND_STATUS.NoData)
    this_state:voltage(0)

    -- handle all the out of range case
    if (out_of_range_low_count >= n_rngfnd) then
        -- all range finders are out of range low, report this same status
        this_state:status(RNGFND_STATUS.OutOfRangeLow)
    end
    if (out_of_range_high_count >= n_rngfnd) then
        -- all range finders are out of range high, report this same status
        this_state:status(RNGFND_STATUS.OutOfRangeHigh)
    end
    if (out_of_range_low_count + out_of_range_high_count >= n_rngfnd) then
        -- We are in a weird state here, maybe a bit of structure or wire is obscuring one or more rangefinders
        -- we can only do our best to and go with the majority vote
        if (out_of_range_low_count >= out_of_range_high_count) then
            this_state:status(RNGFND_STATUS.OutOfRangeLow)
        else
            this_state:status(RNGFND_STATUS.OutOfRangeHigh)
        end
    end

    -- Report signal quality as percentage of the number of rangefinders giving non-nil readings
    local qual = math.floor((#distances / n_rngfnd) * 100)
    if (qual ~= nil) then
        this_state:signal_quality(qual)
    else
        this_state:signal_quality(0.0)
    end


    -- calculate the median (if we can)
    local this_dist
    local case_num
    table.sort(distances)
    if (#distances == n_rngfnd) then
        -- the median is valid
        local median = distances[2]
        this_dist = median
        range_valid_count = range_valid_count + 3
        case_num = CALC_STATUS.Median

    elseif (#distances == 2) then
        -- we will have to do the best we can with an mean
        local mean = (distances[1] + distances[2]) / 2
        this_dist = mean
        range_valid_count = range_valid_count + 2
        case_num = CALC_STATUS.Mean

    elseif (#distances == 1) then
        -- we are relying on this one last rangefinder
        this_dist = distances[1]
        range_valid_count = range_valid_count + 1
        case_num = CALC_STATUS.SingleValue

    elseif (this_state:status() == RNGFND_STATUS.OutOfRangeHigh) then
        this_dist = high_range_value
        range_valid_count = 0
        case_num = CALC_STATUS.AllOutOfRangeHigh

    elseif (this_state:status() == RNGFND_STATUS.OutOfRangeLow) then
        this_dist = low_range_value
        range_valid_count = 0
        case_num = CALC_STATUS.AllOutOfRangeLow

    else
        -- Its gone really wrong and we have no rangefinders
        -- Report error state
        this_dist = ERROR_DIST
        range_valid_count = 0
        case_num = CALC_STATUS.NoData
    end

    -- Final check for nil
    if (this_dist == nil) then
        this_dist = ERROR_DIST
        range_valid_count = 0
        this_state:status(RNGFND_STATUS.NoData)
    end

    -- Set distance in range finder state
    this_state:distance(this_dist)

    -- Optionally report reading over telem
    if (send_vals:get() > 0) then
        gcs:send_named_float("MLDM", this_dist)
    end

    -- Check if the sensor reading can be considered good
    if (range_valid_count >= 10) then
        range_valid_count = 10
        this_state:status(RNGFND_STATUS.Good)
    end

    this_state:range_valid_count(range_valid_count)
    this_state:last_reading(millis():toint())

    -- send output to pending state
    output_rngfnd:handle_script_msg(this_state)

    -- Log calculated value
    logger:write('RNGM', 'I,Dist,Case,Count', 'BfBB', '#m--', '-0-0', n_rngfnd+1, this_dist, case_num, range_valid_count)

    return update, 50
end

return update, 200
