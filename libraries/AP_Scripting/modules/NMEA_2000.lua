--[[
   NMEA 2000 parser as lua module
   with thanks to https://canboat.github.io/canboat/canboat.html

   caller must setup a PGN expected size table with set_PGN_table()
--]]

local M = {}

M.PGN_table = {}

-- multi-frame pending data
M.pending = { pgn = nil, data = "", count = 0, expected_size = 0 }

-- Extract the PGN (Parameter Group Number) from the message ID
local function extract_pgn(message_id)
   local PF = (message_id >> 16) & 0xFF
   local RDP = (message_id >> 24) & 0x3
   if PF < 0xF0 then
      return (RDP << 16) | (PF << 8)
   else
      local PS = (message_id >> 8) & 0xFF
      return (RDP << 16) | (PF << 8) | PS
   end
end

--[[
   extract data from a CAN frame as a lua binary string
--]]
local function extract_data(frame, max_len)
   local ret = ""
   local dlc = frame:dlc()
   local len = math.min(dlc, max_len)
   for ofs = 1, len do
      ret = ret .. string.char(frame:data(ofs-1))
   end
   return ret
end

--[[
   set table of PGNs that we are interested in along with the expected packet size

   The table should be indexed by the PGN and give the expected size
   of that PGN any frames with PGNs not in this table will be
   discarded
--]]
function M.set_PGN_table(t)
   M.PGN_table = t
end

-- Parse CAN frame and reassemble messages
function M.parse(can_frame)
   if not can_frame:isExtended() then
      -- NMEA 2000 frame are always extended (29 bit address)
      return nil
   end
   local message_id = can_frame:id_signed()

   local pgn = extract_pgn(message_id)
   local dlc = can_frame:dlc()

   local exp_size = M.PGN_table[pgn]
   if not exp_size then
      -- discard unwated frame and reset pending
      M.pending.pgn = nil
      return nil
   end

   if exp_size <= 8 and exp_size > dlc then
      -- discard short frame
      M.pending.pgn = nil
      return nil
   end

   if exp_size <= 8 then
      -- single frame
      local data = extract_data(can_frame, exp_size)
      M.pending.pgn = nil
      return pgn, data
   end

   -- multi-frame
   local data = extract_data(can_frame, dlc)
   local subframe = string.byte(data, 1) & 0x1F
   if M.pending.pgn ~= pgn or subframe ~= M.pending.count then
      -- reset
      M.pending.pgn = nil
      M.pending.data = ""
      M.pending.count = 0

      if subframe ~= 0 then
         -- discard, lost first frame or out of order
         return nil
      end
   end

   if subframe == 0 then
      M.pending.expected_size = string.byte(data, 2)
      if M.pending.expected_size < exp_size then
         M.pending.pgn = nil
         return nil
      end
      M.pending.data = M.pending.data .. string.sub(data, 3, #data)
   else
      M.pending.data = M.pending.data .. string.sub(data, 2, #data)
   end
   M.pending.pgn = pgn
   M.pending.count = M.pending.count + 1

   -- do we have a complete frame
   if #M.pending.data >= M.pending.expected_size then
      M.pending.pgn = nil
      return pgn, M.pending.data
   end

   return nil
end

return M
