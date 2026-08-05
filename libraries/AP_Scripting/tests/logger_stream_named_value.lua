-- test streaming of dataflash log message fields to the GCS as
-- NAMED_VALUE_FLOAT/NAMED_VALUE_INT/NAMED_VALUE_STRING messages; used
-- with autotest

local FLOAT_VALUE = 122.25
local INT_VALUE = -1234567
local STRING_VALUE = "speedy"

local ALL_CHANNELS = 0xFFFFFFFF
-- mavlink channel 1 is SERIAL1, tcp port 5762 under SITL:
local CHANNEL_1 = 2

local configured = false

local function write_message()
   logger:write('LSNV', 'fval,ival,sval', 'fiN', FLOAT_VALUE, INT_VALUE, STRING_VALUE)
end

local function fail(message)
   gcs:send_text(0, "LSNV: FAILED: " .. message)
end

local function configure()
   if not logger:stream_named_value('LSNV', 'fval', ALL_CHANNELS) then
      -- the message may not have been written yet; try again shortly
      return false
   end
   if logger:stream_named_value('XXXX', 'fval', ALL_CHANNELS) then
      fail("unknown message accepted")
      return false
   end
   if logger:stream_named_value('LSNV', 'noval', ALL_CHANNELS) then
      fail("unknown field accepted")
      return false
   end
   if logger:stream_named_value('LSNV', 'ival', 0) then
      fail("zero channel mask accepted")
      return false
   end
   -- adding the same field again must succeed without creating a new stream:
   if not logger:stream_named_value('LSNV', 'fval', ALL_CHANNELS) then
      fail("duplicate add failed")
      return false
   end
   if not logger:stream_named_value('LSNV', 'ival', ALL_CHANNELS) then
      fail("could not add int field")
      return false
   end
   if not logger:stream_named_value('LSNV', 'sval', ALL_CHANNELS) then
      fail("could not add string field")
      return false
   end
   -- fields of static structures can also be streamed.  Pitch is
   -- only streamed on mavlink channel 1:
   if not logger:stream_named_value('ATT', 'Roll', ALL_CHANNELS) then
      fail("could not add ATT.Roll")
      return false
   end
   if not logger:stream_named_value('ATT', 'Pitch', CHANNEL_1) then
      fail("could not add ATT.Pitch")
      return false
   end
   if not logger:stream_named_value('ATT', 'Yaw', ALL_CHANNELS) then
      fail("could not add sixth stream")
      return false
   end
   gcs:send_text(6, "LSNV: configured")
   return true
end

function update()
   write_message()
   if not configured then
      configured = configure()
   end
   return update, 100
end

return update()
