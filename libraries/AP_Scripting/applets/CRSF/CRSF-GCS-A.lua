-- Ardupilot Parameter Configurator Server
-- Lua 5.3


--=========================================================
-- Protocol config (shared EdgeTx & Ardupilot)
--=========================================================
local CRSF_DEST_FC = 0xC8 -- flight controller destination
local CRSF_DEST_RADIO = 0xEA -- radio destination
local CRSF_TYPE = 0x80 -- type

local prot = {}
prot.tokenRadio = "GCS-E"
prot.tokenFC = "GCS-A"
prot.def = {
-- [cmd_byte] = {tag, {arguments}, pack_format}
  [0] = {"hello", {"token"}, ">z"},
  [1] = {"helloR", {"token"}, ">z"}, --helloR = helloReply
  [2] = {"paGet", {"tag"}, ">z"},
  [3] = {"paGetR", {"val","tag"}, ">fz"},
  [4] = {"paSet", {"val","tag"}, ">fz"},
  [5] = {"paSetR", {"val","tag"}, ">fz"},
}


--=========================================================
-- Ardupilot command encode/decode
--=========================================================
--encode command + arg table into crsf byte string, nil on fail
function prot.encode(cmd, arg)
  local code
  for k,v in pairs(prot.def) do
    if v[1] == cmd then
      code = k
      break
    end
  end
  if code then
    local def = prot.def[code]
    local parg = {}
    for k,v in pairs(def[2]) do
      parg[k] = arg[v]
    end
    local pkt = string.char(prot.dest) .. string.char(code) .. string.pack(def[3], table.unpack(parg))
    if string.sub(def[3],-1) == "z" and string.byte(pkt,-1) == 0 then
      pkt = string.sub(pkt,1,-2) --remove trailing 0
    end
    return pkt
  end
end

function prot._decode(s)
  local cmd = string.byte(s,2)
  local def = prot.def[cmd]
  local u = { string.unpack(def[3], s, 3) }
  if #u-1 == #def[2] then
    local d = {} --d = decode
    for k,v in pairs(def[2]) do
      d[v] = u[k]
    end
    d.cmd = def[1]
    d.dest = string.byte(s,1)
    return d
  end
end

--decode crsf byte string into {cmd="cmdtext", arg1=xxx, arg2=yyy, ...}, nil on fail
function prot.decode(s)
  local ok, d = pcall(prot._decode, s)
  if ok then return d end
end


--=========================================================
-- MAIN Functions
--=========================================================
--enable/disable CRSF Telemetry, call without enable parameter to get current value
local function csrfTelemEnable(enable)
  local opt = param:get("RC_OPTIONS")
  local mask = (1<<8)
  local was_enabled = (math.floor(opt) & mask) ~= 0
  if enable == false and was_enabled then
    param:set("RC_OPTIONS",opt - mask)
  end
  if enable == true and not was_enabled then
    param:set("RC_OPTIONS",opt + mask)
  end
  return was_enabled
end

--return parameter value or bit value for tag = "paramname:bitno"
local function param_get(tag)
  local pos = string.find(tag,":")
  if pos then
    local bit = tonumber(string.sub(tag, pos+1))
    tag = string.sub(tag, 1, pos-1)
    local pval = param:get(tag)
    if not bit or not pval then return end
    pval = math.floor(pval) & (1<<bit)
    if pval ~= 0 then pval = 1 end
    return pval
  else
    return param:get(tag)
  end
end

--set parameter value or bit value for tag = "paramname:bitno"
local function param_set(tag, val)
  local pos = string.find(tag,":")
  if pos then
    local bit = tonumber(string.sub(tag, pos+1))
    tag = string.sub(tag, 1, pos-1)
    local pval = param:get(tag)
    if not bit or not pval then return end
    pval = math.floor(pval) & (1<<bit)
    if pval ~= 0 and val == 0 then
      val = pval - (1<<bit)
    elseif pval == 0 and val ~= 0 then
      val = pval + (1<<bit)
    else
      return
    end
  end
  param:set(tag, val)
end


--=========================================================
-- MAIN
--=========================================================
prot.dest = CRSF_DEST_RADIO
local name = "CRSF-GCS-A.lua"

local was_enabled = csrfTelemEnable(false)
if was_enabled then
  gcs:send_text(0,"### "..name.." disabled RC_OPTIONS CRSF Telem")
else
  gcs:send_text(0,"### "..name)
end

local function update()
  --receive and reply
  local cmd, data = crsf.pop()
  if cmd == CRSF_TYPE and data and data:byte(1) == CRSF_DEST_FC then
    local d = prot.decode(data)
    if d.cmd == "hello" and d.token == prot.tokenRadio then
      gcs:send_text(0,"### rx hello")
      crsf.push(CRSF_TYPE, prot.encode("helloR", {token=prot.tokenFC}))
      connected = true
    elseif d.cmd == "helloR" and d.token == prot.tokenRadio then
      gcs:send_text(0,"### rx helloR")
      connected = true
    end
    if not connected then
      --(re)connect
      crsf.push(CRSF_TYPE, prot.encode("hello", {token=prot.tokenFC}))
    else
      if d.cmd == "paGet" then
        local val = param_get(d.tag)
        gcs:send_text( 0,"### rx paGet("..tostring(d.tag)..")="..tostring(val) )
        if val then
          crsf.push(CRSF_TYPE, prot.encode("paGetR", {tag=d.tag,val=val}))
        end
      elseif d.cmd == "paSet" then
        param_set(d.tag, d.val)
        local val = param_get(d.tag)
        gcs:send_text(0,"### rx paSet("..tostring(d.tag)..","..tostring(d.val)..")="..tostring(val) )
        if val then
          crsf.push(CRSF_TYPE, prot.encode("paSetR", {tag=d.tag,val=val}))
        end
      end
    end
  end

  return update, 10
end

return update()
