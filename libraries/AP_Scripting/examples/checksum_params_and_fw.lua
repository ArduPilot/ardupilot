--[[
   This script:
   - reads the values of fixed list of parameters
   - checksums them with SHA256
   - compares them against a known-good checksum value
   - emits a statustext message containing the checksum and the expected value
   - appends a line to a file on SD card containing the results
   - blocks arming if the checksums don't match

   It does this 10 seconds after boot.  And any time millis() wraps.

   It only runs once.
--]]

local required_parameters_checksum = "b43bf49dbfff1e46a675057fb532218e4d5cfa14a53b3c94aefb75983fd956ad"
local parameters_to_checksum = {
   "VISO_TYPE",  -- 0
   "TUNE",       -- 0
   "RELAY1_PIN"  -- 12
}

local required_firmware_checksum = "AD341DDC6A50B0C2D8CDABDC2D362D3AFC443776C99C5A90E0F9AFFAC47D2C5E"

local flash_file_path = "@SYS/flash.bin"

local sdcard_file_name = "checksum_parameters.txt"

local prefix = "cpaf: "

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- SHA256 round constants
local sha256_k = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
}

local function rrot32(x, n)
    return ((x >> n) | (x << (32 - n))) & 0xFFFFFFFF
end

local function add32(a, b)
    return (a + b) & 0xFFFFFFFF
end

local function sha256_process_block(ctx, block)
    local w = {}
    for i = 1, 16 do
        local j = (i - 1) * 4
        w[i] = (block[j+1] << 24) | (block[j+2] << 16) | (block[j+3] << 8) | block[j+4]
    end
    for i = 17, 64 do
        local s0 = rrot32(w[i-15], 7) ~ rrot32(w[i-15], 18) ~ (w[i-15] >> 3)
        local s1 = rrot32(w[i-2], 17) ~ rrot32(w[i-2], 19) ~ (w[i-2] >> 10)
        w[i] = add32(add32(add32(w[i-16], s0), w[i-7]), s1)
    end

    local a, b, c, d, e, f, g, h = table.unpack(ctx.hash)

    for i = 1, 64 do
        local S1 = rrot32(e, 6) ~ rrot32(e, 11) ~ rrot32(e, 25)
        local ch = (e & f) ~ (~e & g)
        local temp1 = add32(add32(add32(add32(h, S1), ch), sha256_k[i]), w[i])
        local S0 = rrot32(a, 2) ~ rrot32(a, 13) ~ rrot32(a, 22)
        local maj = (a & b) ~ (a & c) ~ (b & c)
        local temp2 = add32(S0, maj)

        h = g
        g = f
        f = e
        e = add32(d, temp1)
        d = c
        c = b
        b = a
        a = add32(temp1, temp2)
    end

    ctx.hash[1] = add32(ctx.hash[1], a)
    ctx.hash[2] = add32(ctx.hash[2], b)
    ctx.hash[3] = add32(ctx.hash[3], c)
    ctx.hash[4] = add32(ctx.hash[4], d)
    ctx.hash[5] = add32(ctx.hash[5], e)
    ctx.hash[6] = add32(ctx.hash[6], f)
    ctx.hash[7] = add32(ctx.hash[7], g)
    ctx.hash[8] = add32(ctx.hash[8], h)
end

local function sha256_init()
    return {
        hash = {
            0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
            0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
        },
        buf = {},
        len = 0
    }
end

-- feed a table of uint8 byte values into the SHA256 context
local function sha256_update(ctx, bytes)
    local buf = ctx.buf
    for i = 1, #bytes do
        buf[#buf + 1] = bytes[i]
        ctx.len = ctx.len + 1
        if #buf == 64 then
            sha256_process_block(ctx, buf)
            ctx.buf = {}
            buf = ctx.buf
        end
    end
end

-- feed a Lua string into the SHA256 context
local function sha256_update_string(ctx, s)
    local bytes = {}
    for i = 1, #s do
        bytes[i] = s:byte(i)
    end
    sha256_update(ctx, bytes)
end

-- finalise and return the digest as a 64-character lowercase hex string
local function sha256_final(ctx)
    local buf = ctx.buf
    local len = ctx.len

    buf[#buf + 1] = 0x80
    while #buf ~= 56 do
        if #buf == 64 then
            sha256_process_block(ctx, buf)
            ctx.buf = {}
            buf = ctx.buf
        else
            buf[#buf + 1] = 0x00
        end
    end

    -- message length in bits as a 64-bit big-endian integer
    local bit_len = len * 8
    buf[57] = (bit_len >> 56) & 0xFF
    buf[58] = (bit_len >> 48) & 0xFF
    buf[59] = (bit_len >> 40) & 0xFF
    buf[60] = (bit_len >> 32) & 0xFF
    buf[61] = (bit_len >> 24) & 0xFF
    buf[62] = (bit_len >> 16) & 0xFF
    buf[63] = (bit_len >>  8) & 0xFF
    buf[64] =  bit_len        & 0xFF

    sha256_process_block(ctx, buf)

    local result = ""
    for i = 1, 8 do
        result = result .. string.format("%08x", ctx.hash[i])
    end
    return result
end

-- state
local param_sha256_ctx = sha256_init()
local param_checksummed = false
local param_start_ms = nil

local fw_checksummed = false

local param_checksum_match = false
local firmware_checksum_match = false

local CALC_RESULT_INCOMPLETE = 0
local CALC_RESULT_DONE = 1

local auth_id = arming:get_aux_auth_id()

function append_to_sdcard_file(text)
   local file = io.open(sdcard_file_name, 'a')
   if file ~= nil then
      file.write(file, text .. "\n")
      io.close(file)
   end
end

function update_param_calculation()
   if param_start_ms == nil then
      param_start_ms = millis()
   end
   local count = 0
   for i = #parameters_to_checksum, 1, -1 do
      local s = parameters_to_checksum[i]
      local parameter_value = param:get(s)
      if parameter_value == nil then
         gcs:send_text(MAV_SEVERITY.ERROR, prefix .. "Invalid Parameter: " .. s)
         param_checksummed = true
         return CALC_RESULT_DONE
      end
      table.remove(parameters_to_checksum, i)

      -- pack the value as a 4-byte little-endian IEEE 754 float
      sha256_update_string(param_sha256_ctx, string.pack("<f", parameter_value))

      count = count + 1
      -- limit parameters processed per loop to avoid overflowing vmcount
      if count >= 10 then
         return CALC_RESULT_INCOMPLETE
      end
   end

   local digest = sha256_final(param_sha256_ctx)
   local elapsed = string.format("(%.1fs)", (millis() - param_start_ms):tofloat() / 1000.0)
   local notification_severity = MAV_SEVERITY.NOTICE
   local notification_string
   if digest:lower() == required_parameters_checksum:lower() then
      notification_string = prefix .. "params sha256 OK: " .. digest .. " " .. elapsed
      param_checksum_match = true
   else
      notification_string = prefix .. "params sha256 BAD got=" .. digest .. " want=" .. required_parameters_checksum .. " " .. elapsed
      notification_severity = MAV_SEVERITY.ERROR
      if auth_id ~= nil then
         arming:set_aux_auth_failed(auth_id, prefix .. "params sha256 BAD")
      end
   end
   gcs:send_text(notification_severity, notification_string)
   append_to_sdcard_file(notification_string)

   param_checksummed = true
   return CALC_RESULT_DONE
end

function update_firmware_calculation()
   local start_ms = millis()
   local digest = fs:sha256(flash_file_path, 128*1024)
   local notification_severity = MAV_SEVERITY.NOTICE
   local notification_string
   if digest == nil then
      notification_string = prefix .. "firmware sha256 failed"
      notification_severity = MAV_SEVERITY.ERROR
      if auth_id ~= nil then
         arming:set_aux_auth_failed(auth_id, notification_string)
      end
   else
      local elapsed = string.format("(%.1fs)", (millis() - start_ms):tofloat() / 1000.0)
      if digest:lower() == required_firmware_checksum:lower() then
         notification_string = prefix .. "firmware sha256 OK: " .. digest .. " " .. elapsed
         firmware_checksum_match = true
      else
         notification_string = prefix .. "firmware sha256 BAD got=" .. digest .. " want=" .. required_firmware_checksum .. " " .. elapsed
         notification_severity = MAV_SEVERITY.ERROR
         if auth_id ~= nil then
            arming:set_aux_auth_failed(auth_id, prefix .. "firmware sha256 BAD")
         end
      end
   end
   gcs:send_text(notification_severity, notification_string)
   append_to_sdcard_file(notification_string)

   fw_checksummed = true
   return CALC_RESULT_DONE
end

function update()
   -- wait until we've been booted 10 seconds before starting:
   local now = millis()
   if now < 10000 then
      return update, 100
   end

   if not param_checksummed then
      if update_param_calculation() == CALC_RESULT_INCOMPLETE then
         return update, 100
      end
   end

   if not fw_checksummed then
      update_firmware_calculation()
   end

   if param_checksum_match and firmware_checksum_match then
      if auth_id ~= nil then
         arming:set_aux_auth_passed(auth_id)
      end
   end

   return -- one-shot script
end

if auth_id ~= nil then
   arming:set_aux_auth_failed(auth_id, prefix .. "Checksums pending")
end

return update()
