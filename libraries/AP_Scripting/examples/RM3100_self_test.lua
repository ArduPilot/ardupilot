-- Runs the Built-In Self Test on the RM3100 LR circuits
-- Note COMPASS_DISBLMSK should have the 16th bit set to 1 (RM3100)

-- Init RM3100 on bus 0
local rm3100 = i2c:get_device(0, 0x20)
assert(rm3100 ~= nil, "i2c get_device error, cannot run RM3100 self test")

-- Queues a Built-In Self Test
function queue_test()
    gcs:send_text(1, "Running RM3100 self test")

    -- Queue a self test by setting BIST register
    local ret = rm3100:transfer("\x33\x8F", 0)
    if ret == nil then
        gcs:send_text(1, "Rm3100 BIST transfer failed")
        return queue_test, 1000
    end

    -- Send a POLL request to run a BIST
    ret = rm3100:transfer("\x00\x70", 0)
    if ret == nil then
        gcs:send_text(1, "Rm3100 POLL transfer failed")
        return queue_test, 1000
    end

    -- As a measurement takes time, delay a bit by scheduling a different function
    return read_test, 1000
end


-- Reads back values from a Built-In Self Test
function read_test()
    -- Read the BIST results
    local results_str = rm3100:transfer("\x33", 1)
    if results_str ~= nil then
        local results = results_str:byte()

        if results & (1 << 4) == 0 then
            gcs:send_text(1, "RM3100 X is unhealthy")
        else
            gcs:send_text(1, "RM3100 X is OK")
        end

        if results & (1 << 5) == 0 then
            gcs:send_text(1, "RM3100 Y is unhealthy")
        else
            gcs:send_text(1, "RM3100 Y is OK")
        end

        if results & (1 << 6) == 0 then
            gcs:send_text(1, "RM3100 Z is unhealthy")
        else
            gcs:send_text(1, "RM3100 Z is OK")
        end
    else
        gcs:send_text(1, "Rm3100 BIST read transfer failed")
        return queue_test, 1000
    end

    -- Reset the BIST register
    local ret = rm3100:transfer("\x33\x0F", 0)
    if ret == nil then
        gcs:send_text(1, "Rm3100 BIST reset transfer failed")
        return queue_test, 1000
    end

    -- Send a POLL request to take a data point
    ret = rm3100:transfer("\x00\x70", 0)
    if ret == nil then
        gcs:send_text(1, "Rm3100 POLL data transfer failed")
        return queue_test, 1000
    end

    -- As a measurement takes time, delay a bit by scheduling a different function
    return read_data, 1000
end

-- Reads data from the RM3100
function read_data()
    -- Check that data is ready for a read
    local status_str = rm3100:transfer("\x34", 1)
    if status_str ~= nil then
        local status = status_str:byte()
        if status & (1 << 7) == 0 then
            gcs:send_text(1, "RM3100 data not ready for reading")
            return queue_test, 1000
        end
    else
        gcs:send_text(1, "Rm3100 BIST status reg transfer failed")
        return queue_test, 1000
    end

    -- Read measured values
    local measurements_str = rm3100:transfer("\x24", 9)
    if measurements_str ~= nil then
        local MX, MY, MZ = string.unpack(">i3>i3>i3", measurements_str)
        gcs:send_text(6, string.format("RM3100 Mag: X=%8d Y=%8d Z=%8d", MX, MY, MZ))
    else
        gcs:send_text(1, "Rm3100 data read transfer failed")
        return queue_test, 1000
    end

    -- Loop back to the first function to run another set of tests
    return queue_test, 1000
end

return queue_test, 1000
