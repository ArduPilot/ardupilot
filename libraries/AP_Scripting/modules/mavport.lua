-- class to cast mavlink SERIAL_CONTROL message functionality as a serial port.
-- currently always uses the "dev shell" device (which is ignored by Ardupilot).
-- designed for use with the Lua REPL applet.
-- note that flush() must be called to empty internal transmit buffers.

local mavport = {}

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local SERIAL_CONTROL_DATA_LEN = 70
local SERIAL_CONTROL = require("MAVLink/mavlink_msg_SERIAL_CONTROL")
local msg_map = {
    [SERIAL_CONTROL.id] = "SERIAL_CONTROL",
}

function mavport:begin(_)
    local data = {}
    for i = 1, SERIAL_CONTROL_DATA_LEN do
        data[i] = 0
    end
    self._tx_template = {
        baudrate = 0,
        timeout = 0,
        device = 10, -- dev shell
        flags = 1, -- is reply
        count = 0,
        data = data,
    }

    self._tx_buf = data
    self._tx_count = 0
    self._tx_msg = nil

    self._chan = 0 -- send to channel 0 by default

    self._rx_buf = nil
    self._rx_pos = 1

    mavlink.init(4, 1) -- only one message we care about, don't need huge queue
    mavlink.register_rx_msgid(SERIAL_CONTROL.id) -- register it
end

function mavport:write(value)
    return self:writestring(string.char(value))
end

function mavport:writestring(str)
    if self._tx_msg then -- message already queued?
        self:flush()
        if self._tx_msg then return 0 end -- reject if flush failed
    end

    local count = self._tx_count
    local buf = self._tx_buf
    for ci = 1, #str do
        if count == SERIAL_CONTROL_DATA_LEN then break end
        count = count + 1
        buf[count] = str:byte(ci)
    end
    local sent = count - self._tx_count
    self._tx_count = count

    if count == SERIAL_CONTROL_DATA_LEN then
        self:flush()
    end

    return sent
end

function mavport:flush() -- send queued data if possible
    if not self._tx_msg and self._tx_count > 0 then
        -- encode the message and store it for transmission
        local msg = self._tx_template
        msg.count = self._tx_count
        _, self._tx_msg = mavlink_msgs.encode("SERIAL_CONTROL", msg)
        self._tx_count = 0
    end

    if self._tx_msg then -- message to send?
        if mavlink.send_chan(self._chan, SERIAL_CONTROL.id, self._tx_msg) then
            self._tx_msg = nil -- successfully sent
        end
    end
end

function mavport:read()
    if not self._rx_buf then
        self:_receive()
        if not self._rx_buf then return -1 end
    end

    local b = self._rx_buf
    local pos = self._rx_pos
    local c = b[pos]

    self._rx_pos = pos + 1
    if pos == #b then
        self._rx_buf = nil
    end

    return c 
end

function mavport:readstring(count)
    local avail = self:available() -- also fills rx buf
    if avail == 0 then return "" end
    if count > avail then count = avail end

    local b = self._rx_buf
    local pos = self._rx_pos
    local s = string.char(table.unpack(b, pos, pos+count-1))

    pos = pos + count
    if pos > #b then
        self._rx_buf = nil
    end
    self._rx_pos = pos

    return s
end

function mavport:_receive()
    local msg, chan
    while true do
        msg, chan = mavlink.receive_chan()
        if not msg then return end -- no new messages

        -- decode message and handle if it's for us
        msg = mavlink_msgs.decode(msg, msg_map)
        if msg.device == 10 then -- for the dev shell?
            self._chan = chan -- reply on the same channel
            break
        end
    end

    local data = msg.data
    local count = msg.count
    -- remove trailing nulls, they shouldn't happen but they do
    while data[count] == 0 do
        count = count - 1
    end

    -- store received bytes
    if count > 0 then
        if count < SERIAL_CONTROL_DATA_LEN then -- remove trailing junk
            data = table.move(data, 1, count, 1, {})
        end
        self._rx_buf = data
        self._rx_pos = 1
    end
end

function mavport:available()
    if not self._rx_buf then
        self:_receive()
        if not self._rx_buf then return 0 end
    end

    return #self._rx_buf - self._rx_pos + 1
end

-- for completeness
function mavport.set_flow_control(_, _)
end

return mavport
