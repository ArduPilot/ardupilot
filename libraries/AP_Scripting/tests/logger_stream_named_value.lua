-- test streaming of dataflash log message fields to the GCS as
-- NAMED_VALUE_FLOAT/NAMED_VALUE_INT/NAMED_VALUE_STRING messages; used
-- with autotest

local FLOAT_VALUE = 122.25
local INT_VALUE = -1234567
local STRING_VALUE = "speedy"
local INST0_VALUE = 17.5
local INST1_VALUE = 42.5

local ALL_CHANNELS = 0xFFFFFFFF
-- mavlink channel 1 is SERIAL1, tcp port 5762 under SITL:
local CHANNEL_1 = 2
local ANY_INSTANCE = -1
local DEFAULT_RATE = 0
local STOP = -1

local configured = false

local function write_messages()
   logger:write('LSNV', 'fval,ival,sval', 'fiN', FLOAT_VALUE, INT_VALUE, STRING_VALUE)
   -- an instanced message; the '#' unit marks the instance field:
   logger:write('LSNI', 'I,val', 'Bf', '#-', '--', 0, INST0_VALUE)
   logger:write('LSNI', 'I,val', 'Bf', '#-', '--', 1, INST1_VALUE)
end

local function fail(message)
   gcs:send_text(0, "LSNV: FAILED: " .. message)
end

local function configure()
   if not logger:stream_named_value('LSNV', 'fval', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      -- the message may not have been written yet; try again shortly
      return false
   end
   if logger:stream_named_value('XXXX', 'fval', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("unknown message accepted")
      return false
   end
   if logger:stream_named_value('LSNV', 'noval', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("unknown field accepted")
      return false
   end
   if logger:stream_named_value('LSNV', 'ival', ANY_INSTANCE, '', DEFAULT_RATE, 0) then
      fail("zero channel mask accepted")
      return false
   end
   -- adding the same field again must succeed without creating a new stream:
   if not logger:stream_named_value('LSNV', 'fval', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("duplicate add failed")
      return false
   end
   if not logger:stream_named_value('LSNV', 'ival', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add int field")
      return false
   end
   if not logger:stream_named_value('LSNV', 'sval', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add string field")
      return false
   end
   -- instance-restricted streams; messages without an instance field
   -- must reject instance requests:
   if logger:stream_named_value('LSNV', 'fval', 0, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("instance accepted for message without units")
      return false
   end
   if logger:stream_named_value('ATT', 'Roll', 0, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("instance accepted for non-instanced message")
      return false
   end
   -- the streamed names are truncated to 10 characters, eg "LSNI[0].va":
   if not logger:stream_named_value('LSNI', 'val', 0, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add LSNI[0].val")
      return false
   end
   if not logger:stream_named_value('LSNI', 'val', 1, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add LSNI[1].val")
      return false
   end
   -- fields of static structures can also be streamed.  Pitch is
   -- only streamed on mavlink channel 1:
   if not logger:stream_named_value('ATT', 'Roll', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add ATT.Roll")
      return false
   end
   if not logger:stream_named_value('ATT', 'Pitch', ANY_INSTANCE, '', DEFAULT_RATE, CHANNEL_1) then
      fail("could not add ATT.Pitch")
      return false
   end
   if not logger:stream_named_value('ATT', 'Yaw', ANY_INSTANCE, '', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add ATT.Yaw")
      return false
   end
   -- a requested name is used in place of the derived name:
   if not logger:stream_named_value('LSNV', 'ival', ANY_INSTANCE, 'myint', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add ival under a requested name")
      return false
   end
   -- a name already streaming a different field must be refused:
   if logger:stream_named_value('LSNV', 'ival', ANY_INSTANCE, 'LSNV.fval', DEFAULT_RATE, ALL_CHANNELS) then
      fail("name streaming a different field accepted")
      return false
   end
   -- stopping a name which was never streamed must fail:
   if logger:stream_named_value('LSNV', 'fval', ANY_INSTANCE, 'unknown', STOP, ALL_CHANNELS) then
      fail("stop of unknown name accepted")
      return false
   end
   -- a stopped stream's name may be re-bound to a different field:
   if not logger:stream_named_value('LSNV', 'fval', ANY_INSTANCE, 'stopme', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not add fval under a requested name")
      return false
   end
   if not logger:stream_named_value('LSNV', 'fval', ANY_INSTANCE, 'stopme', STOP, ALL_CHANNELS) then
      fail("could not stop a stream")
      return false
   end
   if not logger:stream_named_value('LSNV', 'sval', ANY_INSTANCE, 'stopme', DEFAULT_RATE, ALL_CHANNELS) then
      fail("could not re-bind a stopped stream")
      return false
   end
   if not logger:stream_named_value('LSNV', 'sval', ANY_INSTANCE, 'stopme', STOP, ALL_CHANNELS) then
      fail("could not stop a re-bound stream")
      return false
   end
   gcs:send_text(6, "LSNV: configured")
   return true
end

function update()
   write_messages()
   if not configured then
      configured = configure()
   end
   return update, 100
end

return update()
