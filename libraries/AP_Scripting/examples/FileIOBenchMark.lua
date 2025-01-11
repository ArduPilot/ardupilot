
local SIZE = 2097152 -- 2^21
local BLOCKS = { 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 }

local name = "@9P2000/test.txt"
local tests = 10

local function test_write(block_size)

    -- Generate a random block
    local data = ""
    while #data < block_size do
        data = data .. string.char(math.random(255))
    end
    assert(#data == block_size)

    local file = assert(io.open(name, "w+"))

    -- Repeat the block to make up the file size
    local i = 0
    while i < SIZE do
        file:write(data)
        i = i + block_size
    end
    assert(i == SIZE)

    file:close()
end

local function test_read(block_size)

    local file = assert(io.open(name, "r"))

    local i = 0
    while i < SIZE do
        local data = file:read(block_size)
        if data ~= nil then
            i = i + #data
        end
    end
    assert(i == SIZE)

    file:close()

end

local block_index = 0
local function update()

    block_index = block_index + 1
    if block_index > #BLOCKS then
        -- Done
        return
    end

    local min
    local max
    local average = 0

    for _ = 1, tests do
        local start_us = micros()
        test_write(BLOCKS[block_index])
        local dt_us = (micros() - start_us):tofloat() * 10^-6

        if (min == nil) or (dt_us < min) then
            min = dt_us
        end
        if (max == nil) or (dt_us > max) then
            max = dt_us
        end
        average = average + dt_us
    end

    print(string.format("Write block size: %i took %f seconds (%f/%f)", BLOCKS[block_index], average / tests, min, max))

    min = nil
    max = nil
    average = 0

    for _ = 1, tests do
        local start_us = micros()
        test_read(BLOCKS[block_index])
        local dt_us = (micros() - start_us):tofloat() * 10^-6

        if (min == nil) or (dt_us < min) then
            min = dt_us
        end
        if (max == nil) or (dt_us > max) then
            max = dt_us
        end
        average = average + dt_us
    end

    print(string.format("Read block size: %i took %f seconds (%f/%f)", BLOCKS[block_index], average / tests, min, max))

    return update, 0
end

return update, 1000
