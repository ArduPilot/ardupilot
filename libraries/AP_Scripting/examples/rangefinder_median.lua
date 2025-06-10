
local RANGEFINDER_TYPE_SCRIPT = 36
local RANGEFINDER_STATUS_GOOD = 4
local ERROR_DIST = 130

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

-- Rangefinder "Class"
local function Lidar(inst, angle_deg)
    local self = {}

    -- Store the installed angle in radians
    local install_angle = angle_deg / 180 * math.pi

    if (math.abs(angle_deg) > 85) then
        error("Rangefinder [" .. i .. "] install angle > 85 deg risk of div by zero")
    end

    -- setup the rangefinder backend
    local backend = rangefinder:get_backend(inst)
    if (backend == nil) then
        error("Rangefinder [" .. i .. "] is nil")
    end

    function self.get_reading()

        local status = backend:status()

        if ((status == nil) or (status ~= RANGEFINDER_STATUS_GOOD)) then
            return nil
        end

        -- Calculate the corrected distance to account for the install angle
        local dist = backend:distance()
        if (dist ~= nil) then
            return dist * math.cos(install_angle)
        end

        -- If we got this far, something went wrong
        return nil

    end

    return self
end



-- Setup rangefinders
range_finders = {}
local installation_angles = {25, 5, -15}
-- index 0 is going to be the scripting index
for i=1, #installation_angles do
    table.insert(range_finders, Lidar(i, installation_angles[i]))
end

-- Check that the first instance is the lua scripting backend instance
local output_rngfnd = rangefinder:get_backend(0)
if ((output_rngfnd == nil) or (output_rngfnd:type() ~= RANGEFINDER_TYPE_SCRIPT)) then
    error("Rangefinder1 type needs to be scripting (36)")
end


gcs:send_text(MAV_SEVERITY.INFO, "LiDaR script setup succesfully")

local function update()

    local distances = {}
    for i=1, #range_finders do
        local dist = range_finders[i]:get_reading()

        if (send_vals:get() > 0) then
            -- Guard against trying to send nils via named float
            local send_dist = dist
            if (send_dist == nil) then
                send_dist = ERROR_DIST
            end
            gcs:send_named_float(string.format("MLD%i", i), send_dist)
        end

        distances[i] = dist
    end

    -- TODO handle case with more then 1 nil

    -- calculate the median
    table.sort(distances)
    local median = distances[2]

    -- send output to pending state
    local redundant_dist = ERROR_DIST
    if (median ~= nil) then
        redundant_dist = median
    end

    output_rngfnd:handle_script_msg(redundant_dist)

    return update, 200
end



return update, 200
