--[[
   example script to test lua socket API
--]]

PARAM_TABLE_KEY = 47
PARAM_TABLE_PREFIX = "WEB_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'net_test: could not add param table')

--[[
  // @Param: WEB_ENABLE
  // @DisplayName: enable web server
  // @Description: enable web server
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local WEB_ENABLE = bind_add_param('ENABLE',  1, 1)

--[[
  // @Param: WEB_BIND_PORT
  // @DisplayName: web server TCP port
  // @Description: web server TCP port
  // @Range: 1 65535
  // @User: Standard
--]]
local WEB_BIND_PORT = bind_add_param('BIND_PORT', 2, 80)

--[[
  // @Param: WEB_DEBUG
  // @DisplayName: web server debugging
  // @Description: web server debugging
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
--]]
local WEB_DEBUG = bind_add_param('DEBUG', 3, 0)

--[[
  // @Param: WEB_BLOCK_SIZE
  // @DisplayName: web server block size
  // @Description: web server block size for download
  // @Range: 1 65535
  // @User: Advanced
--]]
local WEB_BLOCK_SIZE = bind_add_param('BLOCK_SIZE', 4, 10240)

--[[
  // @Param: WEB_TIMEOUT
  // @DisplayName: web server timeout
  // @Description: timeout for inactive connections
  // @Units: s
  // @Range: 0.1 60
  // @User: Advanced
--]]
local WEB_TIMEOUT = bind_add_param('TIMEOUT', 5, 2.0)

--[[
  // @Param: WEB_SENDFILE_MIN
  // @DisplayName: web server minimum file size for sendfile
  // @Description: sendfile is an offloading mechanism for faster file download. If this is non-zero and the file is larger than this size then sendfile will be used for file download
  // @Range: 0 10000000
  // @User: Advanced
--]]
local WEB_SENDFILE_MIN = bind_add_param('SENDFILE_MIN', 6, 100000)

if WEB_ENABLE:get() ~= 1 then
   periph:can_printf("WebServer: disabled")
   return
end

periph:can_printf(string.format("WebServer: starting on port %u", WEB_BIND_PORT:get()))

local sock_listen = Socket(0)
local clients = {}

local DOCTYPE = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 3.2 Final//EN\">"
local SERVER_VERSION = "net_webserver 1.0"
local CONTENT_TEXT_HTML = "text/html;charset=UTF-8"
local CONTENT_OCTET_STREAM = "application/octet-stream"

local HIDDEN_FOLDERS = { "@SYS", "@ROMFS", "@MISSION", "@PARAM" }

local MNT_PREFIX = "/mnt"
local MNT_PREFIX2 = MNT_PREFIX .. "/"

local MIME_TYPES = {
   ["apj"] = CONTENT_OCTET_STREAM,
   ["dat"] = CONTENT_OCTET_STREAM,
   ["o"] = CONTENT_OCTET_STREAM,
   ["obj"] = CONTENT_OCTET_STREAM,
   ["lua"] = "text/x-lua",
   ["py"] = "text/x-python",
   ["shtml"] = CONTENT_TEXT_HTML,
   ["js"] = "text/javascript",
   -- thanks to https://developer.mozilla.org/en-US/docs/Web/HTTP/Basics_of_HTTP/MIME_types/Common_types
   ["aac"] = "audio/aac",
   ["abw"] = "application/x-abiword",
   ["arc"] = "application/x-freearc",
   ["avif"] = "image/avif",
   ["avi"] = "video/x-msvideo",
   ["azw"] = "application/vnd.amazon.ebook",
   ["bin"] = "application/octet-stream",
   ["bmp"] = "image/bmp",
   ["bz"] = "application/x-bzip",
   ["bz2"] = "application/x-bzip2",
   ["cda"] = "application/x-cdf",
   ["csh"] = "application/x-csh",
   ["css"] = "text/css",
   ["csv"] = "text/csv",
   ["doc"] = "application/msword",
   ["docx"] = "application/vnd.openxmlformats-officedocument.wordprocessingml.document",
   ["eot"] = "application/vnd.ms-fontobject",
   ["epub"] = "application/epub+zip",
   ["gz"] = "application/gzip",
   ["gif"] = "image/gif",
   ["htm"] = CONTENT_TEXT_HTML,
   ["html"] = CONTENT_TEXT_HTML,
   ["ico"] = "image/vnd.microsoft.icon",
   ["ics"] = "text/calendar",
   ["jar"] = "application/java-archive",
   ["jpeg"] = "image/jpeg",
   ["json"] = "application/json",
   ["jsonld"] = "application/ld+json",
   ["mid"] = "audio/x-midi",
   ["mjs"] = "text/javascript",
   ["mp3"] = "audio/mpeg",
   ["mp4"] = "video/mp4",
   ["mpeg"] = "video/mpeg",
   ["mpkg"] = "application/vnd.apple.installer+xml",
   ["odp"] = "application/vnd.oasis.opendocument.presentation",
   ["ods"] = "application/vnd.oasis.opendocument.spreadsheet",
   ["odt"] = "application/vnd.oasis.opendocument.text",
   ["oga"] = "audio/ogg",
   ["ogv"] = "video/ogg",
   ["ogx"] = "application/ogg",
   ["opus"] = "audio/opus",
   ["otf"] = "font/otf",
   ["png"] = "image/png",
   ["pdf"] = "application/pdf",
   ["php"] = "application/x-httpd-php",
   ["ppt"] = "application/vnd.ms-powerpoint",
   ["pptx"] = "application/vnd.openxmlformats-officedocument.presentationml.presentation",
   ["rar"] = "application/vnd.rar",
   ["rtf"] = "application/rtf",
   ["sh"] = "application/x-sh",
   ["svg"] = "image/svg+xml",
   ["tar"] = "application/x-tar",
   ["tif"] = "image/tiff",
   ["tiff"] = "image/tiff",
   ["ts"] = "video/mp2t",
   ["ttf"] = "font/ttf",
   ["txt"] = "text/plain",
   ["vsd"] = "application/vnd.visio",
   ["wav"] = "audio/wav",
   ["weba"] = "audio/webm",
   ["webm"] = "video/webm",
   ["webp"] = "image/webp",
   ["woff"] = "font/woff",
   ["woff2"] = "font/woff2",
   ["xhtml"] = "application/xhtml+xml",
   ["xls"] = "application/vnd.ms-excel",
   ["xlsx"] = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
   ["xml"] = "default.",
   ["xul"] = "application/vnd.mozilla.xul+xml",
   ["zip"] = "application/zip",
   ["3gp"] = "video",
   ["3g2"] = "video",
   ["7z"] = "application/x-7z-compressed",
}

--[[
 builtin dynamic pages
--]]
local DYNAMIC_PAGES = {

-- main home page
["/"] = [[
<!doctype html>
<html lang="en">

<head>
    <meta charset="utf-8">
    <title>ArduPilot</title>
    <script>
      <?lstr JS_LIBRARY['dynamic_load']?>
    </script>
</head>

<h2>ArduPilot PPP Gateway</h2>
<body onload="javascript: dynamic_load('board_status','/@DYNAMIC/board_status.shtml',1000)">

  <div id="main">
    <ul>
      <li><a href="mnt/">Filesystem Access</a></li>
      <li><a href="?FWUPDATE">Reboot for Firmware Update</a></li>
    </ul>
    </div>
<h2>Controller Status</h2>
  <div id="board_status"></div>
</body>
</html>
]],    
   
-- board status section on front page
["@DYNAMIC/board_status.shtml"] = [[
         <table>
         <tr><td>Firmware</td><td><?lstr FWVersion:string() ?></td></tr>
         <tr><td>GIT Hash</td><td><?lstr FWVersion:hash() ?></td></tr>
         <tr><td>Uptime</td><td><?lstr hms_uptime() ?></td></tr>
         <tr><td>IP</td><td><?lstr networking:address_to_str(networking:get_ip_active()) ?></td></tr>
         <tr><td>Netmask</td><td><?lstr networking:address_to_str(networking:get_netmask_active()) ?></td></tr>
         <tr><td>Gateway</td><td><?lstr networking:address_to_str(networking:get_gateway_active()) ?></td></tr>
         </table>
]]
}

reboot_counter = 0

local ACTION_PAGES = {
   ["/?FWUPDATE"] = function()
      periph:can_printf("Rebooting for firmware update")
      reboot_counter = 50
   end
}

--[[
 builtin javascript library functions
--]]
JS_LIBRARY = {
   ["dynamic_load"] = [[
      function dynamic_load(div_id, uri, period_ms) {
          var xhr = new XMLHttpRequest();
          xhr.open('GET', uri);
          
          xhr.setRequestHeader("Cache-Control", "no-cache, no-store, max-age=0");
          xhr.setRequestHeader("Expires", "Tue, 01 Jan 1980 1:00:00 GMT");
          xhr.setRequestHeader("Pragma", "no-cache");
          
          xhr.onload = function () {
              if (xhr.status === 200) {
                 var output = document.getElementById(div_id);
                 if (uri.endsWith('.shtml') || uri.endsWith('.html')) {
                  output.innerHTML = xhr.responseText;
                 } else {
                  output.textContent = xhr.responseText;
                 }
              }
              setTimeout(function() { dynamic_load(div_id,uri, period_ms); }, period_ms);
          }
          xhr.send();
     }
]]
}

if not sock_listen:bind("0.0.0.0", WEB_BIND_PORT:get()) then
   periph:can_printf(string.format("WebServer: failed to bind to TCP %u", WEB_BIND_PORT:get()))
   return
end

if not sock_listen:listen(20) then
   periph:can_printf("WebServer: failed to listen")
   return
end

function hms_uptime()
   local s = (millis()/1000):toint()
   local min = math.floor(s / 60) % 60
   local hr = math.floor(s / 3600)
   return string.format("%u hours %u minutes %u seconds", hr, min, s%60)
end

--[[
   split string by pattern
--]]
local function split(str, pattern)
   local ret = {}
   for s in string.gmatch(str, pattern) do
      table.insert(ret, s)
   end
   return ret
end

--[[
   return true if a string ends in the 2nd string
--]]
local function endswith(str, s)
   local len1 = #str
   local len2 = #s
   return string.sub(str,1+len1-len2,len1) == s
end

--[[
   return true if a string starts with the 2nd string
--]]
local function startswith(str, s)
   return string.sub(str,1,#s) == s
end

local debug_count=0

function DEBUG(txt)
   if WEB_DEBUG:get() ~= 0 then
      periph:can_printf(txt .. string.format(" [%u]", debug_count))
      debug_count = debug_count + 1
   end
end

--[[
   return index of element in a table
--]]
function table_index(t,el)
   for i,v in ipairs(t) do
      if v == el then
         return i
      end
   end
   return nil
end

--[[
   return true if a table contains a given element
--]]
function table_contains(t,el)
   local i = table_index(t, el)
   return i ~= nil
end

function is_hidden_dir(path)
   return table_contains(HIDDEN_FOLDERS, path)
end

local DAYS = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" }
local MONTHS = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" }

function isdirectory(path)
   local s = fs:stat(path)
   return s and s:is_directory()
end

--[[
   time string for directory listings
--]]
function file_timestring(path)
   local s = fs:stat(path)
   if not s then
      return ""
   end
   local mtime = s:mtime()
   local year, month, day, hour, min, sec, _ = rtc:clock_s_to_date_fields(mtime)
   if not year then
      return ""
   end
   return string.format("%04u-%02u-%02u %02u:%02u", year, month+1, day, hour, min, sec)
end

--[[
   time string for Last-Modified
--]]
function file_timestring_http(mtime)
   local year, month, day, hour, min, sec, wday = rtc:clock_s_to_date_fields(mtime)
   if not year then
      return ""
   end
   return string.format("%s, %02u %s %u %02u:%02u:%02u GMT",
                        DAYS[wday+1],
                        day,
                        MONTHS[month+1],
                        year,
                        hour,
                        min,
                        sec)
end

--[[
   parse a http time string to a uint32_t seconds timestamp
--]]
function file_timestring_http_parse(tstring)
   local dayname, day, monthname, year, hour, min, sec =
      string.match(tstring,
                   '(%w%w%w), (%d+) (%w%w%w) (%d%d%d%d) (%d%d):(%d%d):(%d%d) GMT')
   if not dayname then
      return nil
   end
   local mon = table_index(MONTHS, monthname)
   return rtc:date_fields_to_clock_s(year, mon-1, day, hour, min, sec)
end

--[[
   return true if path exists and is not a directory
--]]
function file_exists(path)
   local s = fs:stat(path)
   if not s then
      return false
   end
   return not s:is_directory()
end

--[[
   substitute variables of form {xxx} from a table
   from http://lua-users.org/wiki/StringInterpolation
--]]
function substitute_vars(s, vars)
   s = (string.gsub(s, "({([^}]+)})",
                    function(whole,i)
                       return vars[i] or whole
   end))
   return s
end

--[[
  lat or lon as a string, working around limited type in ftoa_engine
--]]
function latlon_str(ll)
   local ipart = tonumber(string.match(tostring(ll*1.0e-7), '(.*[.]).*'))
   local fpart = math.abs(ll - ipart*10000000)
   return string.format("%d.%u", ipart, fpart, ipart*10000000, ll)
end

--[[
 location string for home page
--]]
function location_string(loc)
   return substitute_vars([[<a href="https://www.google.com/maps/search/?api=1&query={lat},{lon}" target="_blank">{lat} {lon}</a> {alt}]],
      { ["lat"] = latlon_str(loc:lat()),
        ["lon"] = latlon_str(loc:lng()),
        ["alt"] = string.format("%.1fm", loc:alt()*1.0e-2) })
end

--[[
   client class for open connections
--]]
local function Client(_sock, _idx)
   local self = {}

   self.closed = false

   local sock = _sock
   local idx = _idx
   local have_header = false
   local header = ""
   local header_lines = {}
   local header_vars = {}
   local run = nil
   local protocol = nil
   local file = nil
   local start_time = millis()
   local offset = 0

   function self.read_header()
      local s = sock:recv(2048)
      if not s then
         local now = millis()
         if not sock:is_connected() or now - start_time > WEB_TIMEOUT:get()*1000 then
            -- EOF while looking for header
            DEBUG(string.format("%u: EOF", idx))
            self.remove()
            return false
         end
         return false
      end
      if not s or #s == 0 then
         return false
      end
      header = header .. s
      local eoh = string.find(s, '\r\n\r\n')
      if eoh then
         DEBUG(string.format("%u: got header", idx))
         have_header = true
         header_lines = split(header, "[^\r\n]+")
         -- blocking for reply
         sock:set_blocking(true)
         return true
      end
      return false
   end

   function self.sendstring(s)
      sock:send(s, #s)
   end

   function self.sendline(s)
      self.sendstring(s .. "\r\n")
   end

   --[[
      send a string with variable substitution using {varname}
   --]]
   function self.sendstring_vars(s, vars)
      self.sendstring(substitute_vars(s, vars))
   end
   
   function self.send_header(code, codestr, vars)
      self.sendline(string.format("%s %u %s", protocol, code, codestr))
      self.sendline(string.format("Server: %s", SERVER_VERSION))
      for k,v in pairs(vars) do
         self.sendline(string.format("%s: %s", k, v))
      end
      self.sendline("Connection: close")
      self.sendline("")
   end

   -- get size of a file
   function self.file_size(fname)
      local s = fs:stat(fname)
      if not s then
         return 0
      end
      local ret = s:size():toint()
      DEBUG(string.format("%u: size of '%s' -> %u", idx, fname, ret))
      return ret
   end


   --[[
      return full path with .. resolution
   --]]
   function self.full_path(path, name)
      DEBUG(string.format("%u: full_path(%s,%s)", idx, path, name))
      local ret = path
      if path == "/" and startswith(name,"@") then
         return name
      end
      if name == ".." then
         if path == "/" then
            return "/"
         end
         if endswith(path,"/") then
            path = string.sub(path, 1, #path-1)
         end
         local dir, _ = string.match(path, '(.*/)(.*)')
         if not dir then
            return path
         end
         return dir
      end
      if not endswith(ret, "/") then
         ret = ret .. "/"
      end
      ret = ret .. name
      DEBUG(string.format("%u: full_path(%s,%s) -> %s", idx, path, name, ret))
      return ret
   end
   
   function self.directory_list(path)
      sock:set_blocking(true)
      if startswith(path, "/@") then
         path = string.sub(path, 2, #path-1)
      end
      DEBUG(string.format("%u: directory_list(%s)", idx, path))
      local dlist = dirlist(path)
      if not dlist then
         dlist = {}
      end
      if not table_contains(dlist, "..") then
         -- on ChibiOS we don't get ..
         table.insert(dlist, "..")
      end
      if path == "/" then
         for _,v in ipairs(HIDDEN_FOLDERS) do
            table.insert(dlist, v)
         end
      end

      table.sort(dlist)
      self.send_header(200, "OK", {["Content-Type"]=CONTENT_TEXT_HTML})
      self.sendline(DOCTYPE)
      self.sendstring_vars([[
<html>
 <head>
  <title>Index of {path}</title>
 </head>
 <body>
<h1>Index of {path}</h1>
  <table>
   <tr><th align="left">Name</th><th align="left">Last modified</th><th align="left">Size</th></tr>
]], {path=path})
      for _,d in ipairs(dlist) do
         local skip = d == "."
         if not skip then
            local fullpath = self.full_path(path, d)
            local name = d
            local sizestr = "0"
            local stat = fs:stat(fullpath)
            local size = stat and stat:size() or 0
            if is_hidden_dir(fullpath) or (stat and stat:is_directory()) then
               name = name .. "/"
            elseif size >= 100*1000*1000 then
               sizestr = string.format("%uM", (size/(1000*1000)):toint())
            else
               sizestr = tostring(size)
            end
            local modtime = file_timestring(fullpath)
            self.sendstring_vars([[<tr><td align="left"><a href="{name}">{name}</a></td><td align="left">{modtime}</td><td align="left">{size}</td></tr>
]], { name=name, size=sizestr, modtime=modtime })
         end
      end
      self.sendstring([[
</table>
</body>
</html>
]])
   end

   -- send file content
   function self.send_file()
      if not sock:pollout(0) then
         return
      end
      local chunk = WEB_BLOCK_SIZE:get()
      local b = file:read(chunk)
      sock:set_blocking(true)
      if b and #b > 0 then
         local sent = sock:send(b, #b)
         if sent == -1 then
            run = nil
            self.remove()
            return
         end
         if sent < #b then
            file:seek(offset+sent)
         end
         offset = offset + sent
      end
      if not b or #b < chunk then
         -- EOF
         DEBUG(string.format("%u: sent file", idx))
         run = nil
         self.remove()
         return
      end
   end

   --[[
      load whole file as a string
   --]]
   function self.load_file()
      local chunk = WEB_BLOCK_SIZE:get()
      local ret = ""
      while true do
         local b = file:read(chunk)
         if not b or #b == 0 then
            break
         end
         ret = ret .. b
      end
      return ret
   end
   
   --[[
      evaluate some lua code and return as a string
   --]]
   function self.evaluate(code)
      local eval_code = "function eval_func()\n" .. code .. "\nend\n"
      local f, errloc, err = load(eval_code, "eval_func", "t", _ENV)
      if not f then
         DEBUG(string.format("load failed: err=%s errloc=%s", err, errloc))
         return nil
      end
      local success, err2 = pcall(f)
      if not success then
         DEBUG(string.format("pcall failed: err=%s", err2))
         return nil
      end
      local ok, s2 = pcall(eval_func)
      eval_func = nil
      if ok then
         return s2
      end
      return nil
   end

   --[[
      process a file as a lua CGI
   --]]
   function self.send_cgi()
      sock:set_blocking(true)
      local contents = self.load_file()
      local s = self.evaluate(contents)
      if s then
         self.sendstring(s)
      end
      self.remove()
   end

   --[[
      send file content with server side processsing
      files ending in .shtml can have embedded lua lika this:
      <?lua return "foo" ?>
      <?lstr 2.6+7.2 ?>

      Using 'lstr' a return tostring(yourcode) is added to the code
      automatically
   --]]
   function self.send_processed_file(dynamic_page)
      sock:set_blocking(true)
      local contents
      if dynamic_page then
         contents = file
      else
         contents = self.load_file()
      end
      while #contents > 0 do
         local pat1 = "(.-)[<][?]lua[ \n](.-)[?][>](.*)"
         local pat2 = "(.-)[<][?]lstr[ \n](.-)[?][>](.*)"
         local p1, p2, p3 = string.match(contents, pat1)
         if not p1 then
            p1, p2, p3 = string.match(contents, pat2)
            if not p1 then
               break
            end
            p2 = "return tostring(" .. p2 .. ")"
         end
         self.sendstring(p1)
         local s2 = self.evaluate(p2)
         if s2 then
            self.sendstring(s2)
         end
         contents = p3
      end
      self.sendstring(contents)
      self.remove()
   end
   
   -- return a content type
   function self.content_type(path)
      if path == "/" then
         return MIME_TYPES["html"]
      end
      local _, ext = string.match(path, '(.*[.])(.*)')
      ext = string.lower(ext)
      local ret = MIME_TYPES[ext]
      if not ret then
         return CONTENT_OCTET_STREAM
      end
      return ret
   end

   -- perform a file download
   function self.file_download(path)
      if startswith(path, "/@") then
         path = string.sub(path, 2, #path)
      end
      DEBUG(string.format("%u: file_download(%s)", idx, path))
      file = DYNAMIC_PAGES[path]
      dynamic_page = file ~= nil
      if not dynamic_page then
         file = io.open(path,"rb")
         if not file then
            DEBUG(string.format("%u: Failed to open '%s'", idx, path))
            return false
         end
      end
      local vars = {["Content-Type"]=self.content_type(path)}
      local cgi_processing = startswith(path, "/cgi-bin/") and endswith(path, ".lua")
      local server_side_processing = endswith(path, ".shtml")
      local stat = fs:stat(path)
      if not startswith(path, "@") and
         not server_side_processing and
         not cgi_processing and stat and
         not dynamic_page then
         local fsize = stat:size()
         local mtime = stat:mtime()
         vars["Content-Length"]= tostring(fsize)
         local modtime = file_timestring_http(mtime)
         if modtime then
            vars["Last-Modified"] = modtime
         end
         local if_modified_since = header_vars['If-Modified-Since']
         if if_modified_since then
            local tsec = file_timestring_http_parse(if_modified_since)
            if tsec and tsec >= mtime then
               DEBUG(string.format("%u: Not modified: %s %s", idx, modtime, if_modified_since))
               self.send_header(304, "Not Modified", vars)
               return true
            end
         end
      end
      self.send_header(200, "OK", vars)
      if server_side_processing or dynamic_page then
         DEBUG(string.format("%u: shtml processing %s", idx, path))
         run = self.send_processed_file(dynamic_page)
      elseif cgi_processing then
         DEBUG(string.format("%u: CGI processing %s", idx, path))
         run = self.send_cgi
      elseif stat and
         WEB_SENDFILE_MIN:get() > 0 and
         stat:size() >= WEB_SENDFILE_MIN:get() and
         sock:sendfile(file) then
         return true
      else
         run = self.send_file
      end
      return true
   end

   function self.not_found()
      self.send_header(404, "Not found", {})
   end

   function self.moved_permanently(relpath)
      if not startswith(relpath, "/") then
         relpath = "/" .. relpath
      end
      local location = string.format("http://%s%s", header_vars['Host'], relpath)
      DEBUG(string.format("%u: Redirect -> %s", idx, location))
      self.send_header(301, "Moved Permanently", {["Location"]=location})
   end
   
   -- process a single request
   function self.process_request()
      local h1 = header_lines[1]
      if not h1 or #h1 == 0 then
         DEBUG(string.format("%u: empty request", idx))
         return
      end
      local cmd = split(header_lines[1], "%S+")
      if not cmd or #cmd < 3 then
         DEBUG(string.format("bad request: %s", header_lines[1]))
         return
      end
      if cmd[1] ~= "GET" then
         DEBUG(string.format("bad op: %s", cmd[1]))
         return
      end
      protocol = cmd[3]
      if protocol ~= "HTTP/1.0" and protocol ~= "HTTP/1.1" then
         DEBUG(string.format("bad protocol: %s", protocol))
         return
      end
      local path = cmd[2]
      DEBUG(string.format("%u: path='%s'", idx, path))

      -- extract header variables
      for i = 2,#header_lines do
         local key, var = string.match(header_lines[i], '(.*): (.*)')
         if key then
            header_vars[key] = var
         end
      end

      if ACTION_PAGES[path] ~= nil then
         DEBUG(string.format("Running ACTION %s", path))
         local fn = ACTION_PAGES[path]
         self.send_header(200, "OK", {["Content-Type"]=CONTENT_TEXT_HTML})
         self.sendstring([[
<html>
<head>
<meta http-equiv="refresh" content="2; url=/">
</head>
</html>
]])
         fn()
         return
      end

      if DYNAMIC_PAGES[path] ~= nil then
         self.file_download(path)
         return
      end

      if path == MNT_PREFIX then
         path = "/"
      end
      if startswith(path, MNT_PREFIX2) then
         path = string.sub(path,#MNT_PREFIX2,#path)
      end

      if isdirectory(path) and
         not endswith(path,"/") and
         header_vars['Host'] and
         not is_hidden_dir(path) then
         self.moved_permanently(path .. "/")
         return
      end

      if path ~= "/" and endswith(path,"/") then
         path = string.sub(path, 1, #path-1)
      end

      if startswith(path,"/@") then
         path = string.sub(path, 2, #path)
      end

      -- see if we have an index file
      if isdirectory(path) and file_exists(path .. "/index.html") then
         DEBUG(string.format("%u: found index.html", idx))
         if self.file_download(path .. "/index.html") then
            return
         end
      end
      
      -- see if it is a directory
      if (path == "/" or
         DYNAMIC_PAGES[path] == nil) and
         (endswith(path,"/") or
          isdirectory(path) or
          is_hidden_dir(path)) then
         self.directory_list(path)
         return
      end
      
      -- or a file
      if self.file_download(path) then
         return
      end
      self.not_found(path)
   end

   -- update the client
   function self.update()
      if run then
         run()
         return
      end
      if not have_header then
         if not self.read_header() then
            return
         end
      end
      self.process_request()
      if not run then
         -- nothing more to do
         self.remove()
      end
   end

   function self.remove()
      DEBUG(string.format("%u: removing client OFFSET=%u", idx, offset))
      sock:close()
      self.closed = true
   end

   -- return the instance
   return self
end

--[[
   see if any new clients want to connect
--]]
local function check_new_clients()
   while sock_listen:pollin(0) do
      local sock = sock_listen:accept()
      if not sock then
         return
      end
      -- non-blocking for header read
      sock:set_blocking(false)
      -- find free client slot
      for i = 1, #clients+1 do
         if clients[i] == nil then
            local idx = i
            local client = Client(sock, idx)
            DEBUG(string.format("%u: New client", idx))
            clients[idx] = client
         end
      end
   end
end

--[[
   check for client activity
--]]
local function check_clients()
   for idx,client in ipairs(clients) do
      if not client.closed then
         client.update()
      end
      if client.closed then
         table.remove(clients,idx)
      end
   end
end

local function update()
   check_new_clients()
   check_clients()
   if reboot_counter then
      reboot_counter = reboot_counter - 1
      if reboot_counter == 0 then
         periph:can_printf("Rebooting")
         periph:reboot(true)
      end
   end
   return update,5
end

return update,100
