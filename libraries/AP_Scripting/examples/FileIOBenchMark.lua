
local SIZE = 2097152 -- 2^21
local BLOCKS = { 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536 }

local name = "test.txt"
--local name = "@9P2000/test.txt"
local tests = 100

local function test_write(block_size)

    -- Generate a random block
    local data = ""
    while #data < block_size do
        data = data .. string.char(math.random(255))
    end
    assert(#data == block_size)

    local start_us = micros()

    local file = assert(io.open(name, "w+"))

    -- Repeat the block to make up the file size
    local i = 0
    while i < SIZE do
        file:write(data)
        i = i + #data
    end
    assert(i == SIZE)

    file:close()

    return (micros() - start_us):tofloat() * 10^-6
end

local function test_read(block_size)
    local start_us = micros()

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

    return (micros() - start_us):tofloat() * 10^-6
end

local block_index = 0
local function update()

    block_index = block_index + 1
    if block_index > #BLOCKS then
        -- Done
        return
    end

    local write = {
        min = nil,
        max = nil,
        average = 0,
    }
    local read = {
        min = nil,
        max = nil,
        average = 0,
    }

    for _ = 1, tests do
        local write_dt = test_write(BLOCKS[block_index])
        if (write.min == nil) or (write_dt < write.min) then
            write.min = write_dt
        end
        if (write.max == nil) or (write_dt > write.max) then
            write.max = write_dt
        end
        write.average = write.average + write_dt

        local read_dt = test_read(BLOCKS[block_index])
        if (read.min == nil) or (read_dt < read.min) then
            read.min = read_dt
        end
        if (read.max == nil) or (read_dt > read.max) then
            read.max = read_dt
        end
        read.average = read.average + read_dt
    end

    print(string.format("Block size: %i", BLOCKS[block_index]))
    print(string.format("Read: %f, %f, %f", read.min, read.average / tests, read.max))
    print(string.format("Write: %f, %f, %f", write.min, write.average / tests, write.max))

    return update, 0
end

return update, 1000
