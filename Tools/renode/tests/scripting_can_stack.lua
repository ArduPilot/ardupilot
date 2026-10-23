-- Exercise delivery to enough scripting CAN buffers to overflow the
-- CANSensor thread stack if the native linked list is walked recursively.

local buffer_count = 256
local buffers = {}
local received = {}
local received_count = 0
local announced_ms = uint32_t(0)

for i = 1, buffer_count do
    buffers[i] = CAN:get_device(1)
    if not buffers[i] then
        gcs:send_text(2, string.format("CAN stack allocation failed %u", i))
        return
    end
end

local function update()
    for i = 1, buffer_count do
        if not received[i] and buffers[i]:read_frame() then
            received[i] = true
            received_count = received_count + 1
        end
    end

    if received_count == buffer_count then
        gcs:send_text(6, string.format("CAN stack test passed %u", buffer_count))
        return
    end

    local now = millis()
    if now - announced_ms >= 1000 then
        gcs:send_text(6, string.format("CAN stack test ready %u", buffer_count))
        announced_ms = now
    end
    return update, 10
end

return update, 10
