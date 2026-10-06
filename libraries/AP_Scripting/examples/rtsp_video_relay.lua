--[[
   RTSP Video Relay example

   this relays RTSP video between a camera and a media server such as mediamtx
--]]

-- ==== Defaults you can override via parameters ====
local RTSP_PATH      = "/0/SUB"     -- default to H.264 substream
local RTSP_USER      = "user1"
local RTSP_PASSWORD  = "yyy"

local MEDIAMTX_PUBLISH_USER = "publisher"
local MEDIAMTX_PUBLISH_PASS = "xxx"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- convenient CRLF
local CRLF = string.char(13,10)

local MAX_BUFFER = 200000

-- ==== Parameters ====
PARAM_TABLE_KEY = 94
PARAM_TABLE_PREFIX = "VID_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 63), 'could not add param table')

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), 'could not add param '..name)
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: VID_ENABLE
  // @DisplayName: Video enable
  // @Description: Enable or disable the video relay
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local VID_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
   // @Param: VID_CAM_IP0
   // @DisplayName: Camera IP 0
   // @Description: First octet of the camera IP address
   // @Range: 0 255
   // @User: Standard
--]]
--[[
   // @Param: VID_CAM_IP1
   // @DisplayName: Camera IP 1
   // @Description: Second octet of the camera IP address
   // @Range: 0 255
   // @User: Standard
--]]
--[[
   // @Param: VID_CAM_IP2
   // @DisplayName: Camera IP 2
   // @Description: Third octet of the camera IP address
   // @Range: 0 255
   // @User: Standard
--]]
--[[
   // @Param: VID_CAM_IP3
   // @DisplayName: Camera IP 3
   // @Description: Fourth octet of the camera IP address
   // @Range: 0 255
   // @User: Standard
--]]
local VID_CAM_IP = {
   bind_add_param('CAM_IP0',  2, 192),
   bind_add_param('CAM_IP1',  3, 168),
   bind_add_param('CAM_IP2',  4, 144),
   bind_add_param('CAM_IP3',  5, 25),
}

--[[
   // @Param: VID_CAM_PORT
   // @DisplayName: Camera port
   // @Description: RTSP TCP port camera
   // @Range: 0 65535
   // @User: Standard
--]]
local VID_CAM_PORT = bind_add_param('CAM_PORT', 6, 554)

--[[
   // @Param: VID_MTX_IP0
   // @DisplayName: mediamtx IP 0
   // @Description: First octet of the mediamtx IP address
   // @Range: 0 255
   // @User: Standard
--]]
--[[
   // @Param: VID_MTX_IP1
   // @DisplayName: mediamtx IP 1
   // @Description: Second octet of the mediamtx IP address
   // @Range: 0 255
   // @User: Standard
--]]
--[[
   // @Param: VID_MTX_IP2
   // @DisplayName: mediamtx IP 2
   // @Description: Third octet of the mediamtx IP address
   // @Range: 0 255
   // @User: Standard
--]]
--[[
   // @Param: VID_MTX_IP3
   // @DisplayName: mediamtx IP 3
   // @Description: Fourth octet of the mediamtx IP address
   // @Range: 0 255
   // @User: Standard
--]]
local VID_MTX_IP = {
   bind_add_param('MTX_IP0', 10, 192),
   bind_add_param('MTX_IP1', 11, 168),
   bind_add_param('MTX_IP2', 12, 144),
   bind_add_param('MTX_IP3', 13, 1),
}

--[[
   // @Param: VID_MTX_PORT
   // @DisplayName: mediamtx port
   // @Description: RTSP TCP port for mediamtx
   // @Range: 0 65535
   // @User: Standard
--]]
local VID_MTX_PORT = bind_add_param('MTX_PORT', 14, 8554)

--[[
   // @Param: VID_DEBUG
   // @DisplayName: video debug
   // @Description: debug level
   // @Range: 0 3
   // @User: Standard
--]]
local VID_DEBUG = bind_add_param('DEBUG', 15, 2)

--[[
   // @Param: VID_RETRY_MS
   // @DisplayName: video retry timeout
   // @Description: video retry timeout
   // @Units: ms
   // @Range: 1000 10000
   // @User: Standard
--]]
local VID_RETRY_MS = bind_add_param('RETRY_MS', 16, 3000)

-- ==== State ====
local cam_rtsp, mtx_rtsp = nil, nil       -- TCP sockets (also carry interleaved data)
local cam_state, mtx_state = 'DISCONNECTED', 'DISCONNECTED'
local cseq_cam, cseq_mtx = 1, 1
local cam_session, mtx_session = nil, nil
local last_retry_ms, last_mtx_retry_ms, last_ka_cam, last_ka_mtx = uint32_t(0), uint32_t(0), uint32_t(0), uint32_t(0)
local last_options_send = uint32_t(0)
local pkt_count, last_pkt_report = 0, uint32_t(0)

-- Interleaved buffers
local cam_ibuf, mtx_ibuf = '', ''

-- ==== Utils ====
local function ip4(o)
   return string.format('%u.%u.%u.%u', o[1]:get(), o[2]:get(), o[3]:get(), o[4]:get())
end
local function get_cam_ip()
   return ip4(VID_CAM_IP)
end
local function get_mtx_ip()
   return ip4(VID_MTX_IP)
end
local function get_cam_port()
   return math.floor(VID_CAM_PORT:get())
end
local function get_mtx_port()
   return math.floor(VID_MTX_PORT:get())
end

local function get_cam_user()
   return RTSP_USER
end
local function get_cam_pass()
   return RTSP_PASSWORD
end
local function get_cam_path()
   return RTSP_PATH
end

local function b64(data)
   local t='ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
   return ((data:gsub('.', function(x)
                         local r,b='',x:byte(); for i=8,1,-1 do r=r..(b%2^i-b%2^(i-1)>0 and '1' or '0') end; return r
                     end)..'0000'):gsub('%d%d%d?%d?%d?%d?', function(x)
                                           if #x<6 then return '' end; local c=0; for i=1,6 do c=c+(x:sub(i,i)=='1' and 2^(6-i) or 0) end; return t:sub(c+1,c+1)
                                       end)..({ '', '==', '=' })[#data%3+1])
end

local function dbg(level, msg)
   if VID_DEBUG:get() >= level then
      local sev = MAV_SEVERITY.INFO; if level==0 then sev=MAV_SEVERITY.ERROR elseif level==1 then sev=MAV_SEVERITY.WARNING end
      gcs:send_text(sev, 'VID: '..msg)
   end
end

--[[
   wrapper around a TCP socket with write buffer
--]]
local function socket_wrapper(sockname)
  local obj = {}
  local sock = Socket(0)
  local q, q_head, q_tail, q_bytes = {}, 1, 1, 0
  local dead = false
  local name = sockname

  sock:set_blocking(false)

  function obj:send(data)
     if dead or not data or #data == 0 then
        return
     end
     q[q_tail] = data
     q_bytes = q_bytes + #data
     q_tail = q_tail + 1
     if q_bytes > MAX_BUFFER then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: dropping %d bytes", name, q_bytes))
        q, q_head, q_tail, q_bytes = {}, 1, 1, 0
        return
     end
     self:write_pending()
  end

  function obj:write_pending()
    if dead or not sock then
        return
    end
    -- only flush when the TCP connection is up
    if not sock:is_connected() then
       return
    end

    while q[q_head] ~= nil do
      if not sock:pollout(0) then
         return
      end
      local chunk = q[q_head]
      local nsent = sock:send(chunk, #chunk)
      if not nsent or nsent <= 0 then
         gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: EOF write", name))
         self:close()
         return
      elseif nsent < #chunk then
        q[q_head] = string.sub(chunk, nsent + 1)
        q_bytes = q_bytes - nsent
        return
      else
        q_bytes = q_bytes - #chunk
        q[q_head] = nil
        q_head = q_head + 1
      end
    end
  end

  function obj:is_dead() -- luacheck: ignore unused argument self
     return dead
  end
  function obj:close() -- luacheck: ignore unused argument self
    if sock then
       sock:close()
       ---@diagnostic disable-next-line: cast-local-type
       sock = nil
    end
    q, q_head, q_tail, q_bytes, dead = {}, 1, 1, 0, true
  end

  function obj:pollout(t) -- luacheck: ignore unused argument self
     return sock and sock:pollout(t)
  end
  function obj:is_connected() -- luacheck: ignore unused argument self
     return sock and sock:is_connected()
  end
  function obj:connect(ip, port) -- luacheck: ignore unused argument self
     return sock and sock:connect(ip, port)
  end
  function obj:is_pending() -- luacheck: ignore unused argument self
     return sock and sock:is_pending()
  end
  function obj:recv(n)
     if not sock or dead then
        return nil
     end
     if not sock:pollin(0) then
        return nil
     end
     local ret = sock:recv(n)
     if not ret or #ret <= 0 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: EOF read", name))
        self:close()
        return nil
     end
     return ret
  end
  return obj
end

--[[
   send some data over a RTSP link
--]]
local function rtsp_send(sock, method, uri, cseq, headers, body)
   headers = headers or {}
   local lines = { string.format('%s %s RTSP/1.0', method, uri), string.format('CSeq: %d', cseq) }
   for i=1,#headers do if headers[i] and headers[i] ~= '' then lines[#lines+1] = headers[i] end end
      if body and #body > 0 then
         lines[#lines+1] = 'Content-Type: application/sdp'
         lines[#lines+1] = string.format('Content-Length: %d', #body)
      end
      local req = table.concat(lines, CRLF) .. CRLF .. CRLF
      if body and #body > 0 then req = req .. body end
      sock:send(req)
end

--[[
   parse RTSP messages
--]]
local function read_messages(buf)
   local msgs = {}
   while true do
      if #buf == 0 then break end
      local b1 = string.byte(buf,1)
      if b1 == 0x24 then
         if #buf < 4 then break end
         local ch = string.byte(buf,2)
         local ln = string.byte(buf,3)*256 + string.byte(buf,4)
         if #buf < 4 + ln then break end
         msgs[#msgs+1] = {kind='ilv', ch=ch, payload=string.sub(buf,5,4+ln)}
         buf = string.sub(buf,5+ln)
      else
         local sep = CRLF..CRLF
         local he = buf:find(sep, 1, true); if not he then break end
         local head_end = he + #sep - 1
         local head = string.sub(buf,1,head_end)
         local headers = {}
         for k,v in head:gmatch('[\r\n]+([%w%-]+):%s*([^\r\n]+)') do headers[string.lower(k)] = v end
         local cl = tonumber(headers['content-length'] or '0') or 0
         local body_start = head_end + 1
         local need = head_end + cl
         if #buf < need then break end
         local body = cl>0 and string.sub(buf, body_start, body_start + cl - 1) or ''
         local code = tonumber(head:match('RTSP/1%.0%s+(%d+)') or '0') or 0
         msgs[#msgs+1] = {kind='rtsp', head=head, headers=headers, body=body, code=code}
         buf = string.sub(buf, need + 1)
      end
   end
   return msgs, buf
end

-- Camera URI
local function cam_uri()
   return string.format('rtsp://%s:%d%s', get_cam_ip(), get_cam_port(), get_cam_path())
end

-- auth string for publishing
local function cam_auth()
   local u=get_cam_user()
   local p=get_cam_pass()
   if u=='' and p=='' then
      return ''
   end
   return 'Authorization: Basic '..b64(u..':'..p)
end

--[[
   update link to camera
--]]
local function cam_update()
   local now = millis()

   if cam_rtsp then
      cam_rtsp:write_pending()
   end

   if cam_state=='DISCONNECTED' or cam_rtsp == nil then
      local connected = false
      if cam_rtsp then
         -- must be a pending connection
         if not cam_rtsp:pollout(10) then
            return
         end
         if cam_rtsp:is_connected() then
            connected = true
         else
            cam_rtsp:close()
            cam_rtsp = nil
         end
      end
      if connected or now - last_retry_ms > VID_RETRY_MS:get() then
         last_retry_ms=now
         if not connected then
            dbg(2, 'Cam connect '..get_cam_ip()..':'..get_cam_port()..get_cam_path())
            assert(not cam_rtsp, "cam socket leak")
            cam_rtsp = socket_wrapper("cam")
            if not cam_rtsp then
               return
            end
            connected = cam_rtsp:connect(get_cam_ip(), get_cam_port())
            if not connected and cam_rtsp:is_pending() then
               return
            end
         end
         if connected then
            dbg(2, 'Cam send OPTIONS')
            cam_state='OPTIONS'
            cseq_cam=1
            last_options_send = now
            rtsp_send(cam_rtsp,'OPTIONS',cam_uri(),cseq_cam,{cam_auth(),'User-Agent: ArduPilot-Lua'})
            cseq_cam=cseq_cam+1
         else
            dbg(1,'Cam connect failed')
            if cam_rtsp then
               cam_rtsp:close()
               cam_rtsp = nil
            end
         end
      end
      return
   end

   local chunk = cam_rtsp:recv(8192); if chunk and #chunk>0 then cam_ibuf = cam_ibuf..chunk end
   local msgs; msgs, cam_ibuf = read_messages(cam_ibuf)
   for i=1,#msgs do
      local m=msgs[i]
      if m.kind=='rtsp' then
         if m.code == 200 then
            if cam_state=='OPTIONS' then
               rtsp_send(cam_rtsp,'DESCRIBE',cam_uri(),cseq_cam,{cam_auth(),'User-Agent: ArduPilot-Lua','Accept: application/sdp'})
               cseq_cam=cseq_cam+1
               cam_state='DESCRIBE'
            elseif cam_state=='DESCRIBE' then
               rtsp_send(cam_rtsp,'SETUP',cam_uri(),cseq_cam,{cam_auth(),'User-Agent: ArduPilot-Lua','Transport: RTP/AVP/TCP;unicast;interleaved=0-1'})
               cseq_cam=cseq_cam+1
               cam_state='SETUP'
            elseif cam_state=='SETUP' then
               local sess = (m.headers and m.headers['session']) and m.headers['session']:match('([^;]+)') or nil; if sess then cam_session=sess end
               rtsp_send(cam_rtsp,'PLAY',cam_uri(),cseq_cam,{cam_auth(),'User-Agent: ArduPilot-Lua', cam_session and ('Session: '..cam_session) or ''})
               cseq_cam=cseq_cam+1
               cam_state='PLAY'
            elseif cam_state=='PLAY' then
               dbg(1,'Camera streaming (TCP interleaved)')
               cam_state='STREAMING'
               last_ka_cam=now
            end
         end
      else -- interleaved from camera
         if cam_state=='STREAMING' then
            if not mtx_rtsp or mtx_state~='PUBLISHING' then goto continue end
            if m.ch==0 then
               -- forward RTP to MediaMTX interleaved ch 0
               local hdr = string.char(0x24,0, math.floor(#m.payload/256)%256, #m.payload%256)
               mtx_rtsp:send(hdr..m.payload)
               pkt_count = pkt_count + 1
            elseif m.ch==1 then
               -- forward RTCP to ch 1
               local hdr = string.char(0x24,1, math.floor(#m.payload/256)%256, #m.payload%256)
               mtx_rtsp:send(hdr..m.payload)
            end
            local n = millis()
            if n - last_pkt_report > 1000 then
               gcs:send_named_float('VID_FRAMES', pkt_count)
               last_pkt_report=n
            end
         end
      end
      ::continue::
   end

   if cam_state=='OPTIONS' and (now - last_options_send > 5000) then
      cam_state='DISCONNECTED'
      cam_rtsp:close()
      cam_rtsp = nil
   end

   if cam_state=='STREAMING' and (now - last_ka_cam > 25000) then
       rtsp_send(cam_rtsp,'GET_PARAMETER',cam_uri(),cseq_cam,{cam_auth(),'User-Agent: ArduPilot-Lua', cam_session and ('Session: '..cam_session) or ''})
       cseq_cam=cseq_cam+1
       last_ka_cam=now
   end
end

-- MTX URI
local function mtx_uri_base()
   return string.format('rtsp://%s:%d/relay_stream', get_mtx_ip(), get_mtx_port())
end
local function mtx_auth()
   return 'Authorization: Basic '..b64(MEDIAMTX_PUBLISH_USER..':'..MEDIAMTX_PUBLISH_PASS)
end

local function announce_sdp_h264()
   -- Minimal H264 SDP
   return table.concat({
         'v=0',
         'o=- 0 0 IN IP4 0.0.0.0',
         's=Relay Stream',
         'c=IN IP4 0.0.0.0',
         't=0 0',
         'm=video 0 RTP/AVP 96',
         'a=rtpmap:96 H264/90000',
         'a=control:trackID=0'
                       }, CRLF) .. CRLF
end

--[[
   update link to mediamtx server
--]]
local function mtx_update()
   local now = millis()
   if mtx_rtsp then
      mtx_rtsp:write_pending()
   end
   if mtx_state=='DISCONNECTED' or mtx_rtsp == nil then
      if cam_state=='STREAMING' then
         local connected = false
         if mtx_rtsp then
            -- must be a pending connection
            if not mtx_rtsp:pollout(10) then
               return
            end
            if mtx_rtsp:is_connected() then
               connected = true
            else
               mtx_rtsp:close()
               mtx_rtsp = nil
            end
         end
         if not connected and now - last_mtx_retry_ms < VID_RETRY_MS:get() then
            return
         end
         if not connected then
            dbg(2, 'Connect MediaMTX '..get_mtx_ip()..':'..get_mtx_port())
            assert(not mtx_rtsp, "mtx socket leak")
            mtx_rtsp = socket_wrapper("mtx")
            if not mtx_rtsp then
               return
            end
            connected = mtx_rtsp:connect(get_mtx_ip(), get_mtx_port())
            if not connected and mtx_rtsp:is_pending() then
               return
            end
         end
         if connected then
            mtx_state='ANNOUNCE'
            cseq_mtx=1
            local sdp = announce_sdp_h264()
            rtsp_send(mtx_rtsp,'ANNOUNCE',mtx_uri_base(),cseq_mtx,{mtx_auth(),'User-Agent: ArduPilot-Lua'}, sdp)
            cseq_mtx=cseq_mtx+1
         else
            dbg(1,'MediaMTX connect failed')
            last_mtx_retry_ms=now
            if mtx_rtsp then
               mtx_rtsp:close()
               mtx_rtsp = nil
            end
         end
      end
      return
   end

   local chunk = mtx_rtsp:recv(4096);
   if chunk and #chunk>0 then
      mtx_ibuf = mtx_ibuf..chunk
   end
   local msgs; msgs, mtx_ibuf = read_messages(mtx_ibuf)
   for i=1,#msgs do local m=msgs[i]
      if m.kind=='rtsp' then
         if m.code == 200 then
            if mtx_state=='ANNOUNCE' then
               local track = mtx_uri_base()..'/trackID=0'
               rtsp_send(mtx_rtsp,'SETUP',track,cseq_mtx,{mtx_auth(),'User-Agent: ArduPilot-Lua','Transport: RTP/AVP/TCP;unicast;interleaved=0-1;mode=record'})
               cseq_mtx=cseq_mtx+1
               mtx_state='SETUP'
            elseif mtx_state=='SETUP' then
               local sess = (m.headers and m.headers['session']) and m.headers['session']:match('([^;]+)') or nil; if sess then mtx_session=sess end
               rtsp_send(mtx_rtsp,'RECORD',mtx_uri_base(),cseq_mtx,{mtx_auth(),'User-Agent: ArduPilot-Lua', mtx_session and ('Session: '..mtx_session) or ''}); cseq_mtx=cseq_mtx+1; mtx_state='RECORD'
            elseif mtx_state=='RECORD' then
               dbg(1,'Publishing to MediaMTX (TCP interleaved)'); mtx_state='PUBLISHING'; last_ka_mtx=now
            end
         end
      end
   end

   if mtx_state=='PUBLISHING' and (now - last_ka_mtx > 25000) then
      rtsp_send(mtx_rtsp,'GET_PARAMETER',mtx_uri_base(),cseq_mtx,{mtx_auth(),'User-Agent: ArduPilot-Lua', mtx_session and ('Session: '..mtx_session) or ''}); cseq_mtx=cseq_mtx+1; last_ka_mtx=now
   end
end

local function reset_state()
      cam_state='DISCONNECTED'
      mtx_state='DISCONNECTED'
      if cam_rtsp then
         cam_rtsp:close();
         cam_rtsp=nil
      end
      if mtx_rtsp then
         mtx_rtsp:close()
         mtx_rtsp=nil
      end
end

-- main loop
local function update()
   if VID_ENABLE:get() <= 0 then
      reset_state()
      return
   end
   if cam_rtsp and cam_rtsp:is_dead() then
      reset_state()
   end
   if mtx_rtsp and mtx_rtsp:is_dead() then
      reset_state()
   end
   cam_update()
   mtx_update()
end

function protected_wrapper()
   local ok, err = pcall(update)
   if not ok then
      gcs:send_text(MAV_SEVERITY.ERROR, 'VID error: '..tostring(err))
      reset_state()
      return protected_wrapper, 1000
   end
   return protected_wrapper, 10
end

-- ==== Init ====
dbg(1,'Video relay starting (both legs TCP interleaved)')
dbg(2,('Camera: %s:%d%s'):format(get_cam_ip(), get_cam_port(), get_cam_path()))
dbg(2,('MediaMTX: %s:%d'):format(get_mtx_ip(), get_mtx_port()))
dbg(2,('Publish user: %s'):format(MEDIAMTX_PUBLISH_USER))

return protected_wrapper, 1000
