--[[
   MAVLink Broadcast - send messages to all MAVLink channels

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   -----------------------------------------------------------------------

   This module provides functions for discovering and broadcasting
   to MAVLink channels in ArduPilot Lua scripts.

   Background:
   ArduPilot assigns MAVLink channels based on SERIAL port configuration.
   Channel 0 is the first SERIAL port configured for MAVLink (SERIALx_PROTOCOL = 1 or 2),
   channel 1 is the second, etc. The actual channel number depends on hardware
   setup and parameter configuration.

   When using mavlink:send_chan(chan, msgid, payload), the message is sent directly
   on that specific channel. This means messages won't automatically reach devices
   on other channels, even if addressed via target_system/target_component.

   Use cases:
   - Sending commands to companion computers when channel assignment is unknown
   - Broadcasting messages to all connected GCS/devices
   - Discovering which MAVLink channels are active

   Usage:
     local mavlink_broadcast = require("MAVLink/mavlink_broadcast")

     -- Discover available channels once at startup
     local channels = mavlink_broadcast.get_available_channels()
     gcs:send_text(6, string.format("Found %d MAVLink channels", #channels))

     -- Send a message to all available channels
     local msgid, payload = mavlink_msgs.encode("COMMAND_LONG", cmd)
     mavlink_broadcast.send_to_all_channels(channels, msgid, payload)

--]]

local mavlink_broadcast = {}

-- Maximum number of MAVLink channels to probe
-- ArduPilot typically supports up to 8 channels depending on build configuration
local MAX_CHANNELS = 8

---Get a list of available MAVLink channels
---
---Probes channels 0 through MAX_CHANNELS-1 and returns a table of
---channel numbers where send_chan() succeeds (doesn't return nil).
---
---Note: This function sends a HEARTBEAT on each channel to test availability.
---The HEARTBEAT is harmless and standard MAVLink behavior. Call this once
---at script startup and cache the result for efficiency.
---
---@return table -- Array of available channel numbers (e.g., {0, 1, 2})
function mavlink_broadcast.get_available_channels()
    local channels = {}

    -- Send a minimal HEARTBEAT to probe each channel
    -- HEARTBEAT is the standard MAVLink message and is harmless
    local mavlink_msgs = require("MAVLink/mavlink_msgs")
    local heartbeat = {
        custom_mode = 0,
        type = 0,               -- MAV_TYPE_GENERIC
        autopilot = 0,          -- MAV_AUTOPILOT_GENERIC
        base_mode = 0,
        system_status = 0,      -- MAV_STATE_UNINIT
        mavlink_version = 3,    -- MAVLink 2
    }
    local msgid, payload = mavlink_msgs.encode("HEARTBEAT", heartbeat)

    for chan = 0, MAX_CHANNELS - 1 do
        -- send_chan returns: true (success), false (failed), nil (channel doesn't exist)
        local result = mavlink:send_chan(chan, msgid, payload)
        if result ~= nil then
            table.insert(channels, chan)
        end
    end

    return channels
end

---Send an encoded MAVLink message to all specified channels
---
---Use this with a channel list from get_available_channels() for efficient
---repeated broadcasts without re-probing channels each time.
---
---@param channels table -- Array of channel numbers from get_available_channels()
---@param msgid integer -- MAVLink message ID
---@param payload string -- Encoded message payload from mavlink_msgs.encode()
---@return integer -- Number of channels the message was successfully sent on
function mavlink_broadcast.send_to_all_channels(channels, msgid, payload)
    local sent_count = 0
    for _, chan in ipairs(channels) do
        local result = mavlink:send_chan(chan, msgid, payload)
        if result == true then
            sent_count = sent_count + 1
        end
    end
    return sent_count
end

---Send an encoded MAVLink message to all available channels (convenience function)
---
---This combines get_available_channels() and send_to_all_channels() for one-off
---broadcasts. For repeated sends, cache the channel list from get_available_channels()
---and use send_to_all_channels() directly for better efficiency.
---
---@param msgid integer -- MAVLink message ID
---@param payload string -- Encoded message payload from mavlink_msgs.encode()
---@return integer -- Number of channels the message was sent on
function mavlink_broadcast.broadcast_to_all(msgid, payload)
    local channels = mavlink_broadcast.get_available_channels()
    return mavlink_broadcast.send_to_all_channels(channels, msgid, payload)
end

return mavlink_broadcast
