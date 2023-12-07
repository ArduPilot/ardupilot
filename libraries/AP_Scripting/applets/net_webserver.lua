--[[
   example script to test lua socket API
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

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
local WEB_ENABLE = bind_add_param('ENABLE',  1, 0)

--[[
  // @Param: WEB_BIND_PORT
  // @DisplayName: web server TCP port
  // @Description: web server TCP port
  // @Range: 1 65535
  // @User: Standard
--]]
local WEB_BIND_PORT = bind_add_param('BIND_PORT', 2, 8080)

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

gcs:send_text(MAV_SEVERITY.INFO, string.format("WebServer: starting on port %u", WEB_BIND_PORT:get()))

local counter = 0
local sock_listen = SocketAPM(0)
local clients = {}

local DOCTYPE = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 3.2 Final//EN\">"
local SERVER_VERSION = "net_webserver 1.0"
local CONTENT_TEXT_HTML = "text/html;charset=UTF-8"
local CONTENT_OCTET_STREAM = "application/octet-stream"

local HIDDEN_FOLDERS = { "@SYS", "@ROMFS", "@MISSION", "@PARAM" }

local CGI_BIN_PATH = "cgi-bin"

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


if not sock_listen:bind("0.0.0.0", WEB_BIND_PORT:get()) then
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("WebServer: failed to bind to TCP %u", WEB_BIND_PORT:get()))
   return
end

if not sock_listen:listen(20) then
   gcs:send_text(MAV_SEVERITY.ERROR, "WebServer: failed to listen")
   return
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
   local len1 = #str
   local len2 = #s
   return string.sub(str,1,len2) == s
end

local debug_count=0

function DEBUG(txt)
   if WEB_DEBUG:get() ~= 0 then
      gcs:send_text(MAV_SEVERITY.DEBUG, txt .. string.format(" [%u]", debug_count))
      debug_count = debug_count + 1
   end
end

--[[
   return true if a table contains a given element
--]]
function contains(t,el)
   for _,v in ipairs(t) do
      if v == el then
         return true
      end
   end
   return false
end

function is_hidden_dir(path)
   return contains(HIDDEN_FOLDERS, path)
end

function file_exists(path)
   local f = io.open(path, "rb")
   if f then
      return true
   end
   return false
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
      from http://lua-users.org/wiki/StringInterpolation
   --]]
   function self.sendstring_vars(s, vars)
      s = (string.gsub(s, "({([^}]+)})",
                          function(whole,i)
                             return vars[i] or whole
      end))
      self.sendstring(s)
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
      DEBUG(string.format("%u: size of '%s'", idx, fname))
      local f = io.open(fname, "rb")
      if not f then
         return -1
      end
      local ret = f:seek("end")
      f:close()
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
         local dir, file = string.match(path, '(.*/)(.*)')
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
      if not contains(dlist, "..") then
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
   <tr><th>Name</th><th>Size</th></tr>
]], {path=path})
      for _,d in ipairs(dlist) do
         local skip = d == "."
         if path == "/" and d == ".." then
            skip = true
         end
         if not skip then
            local fullpath = self.full_path(path, d)
            local name = d
            local size = 0
            if is_hidden_dir(fullpath) or isdirectory(fullpath) then
               name = name .. "/"
            else
               size = math.max(self.file_size(fullpath),0)
            end
            self.sendstring_vars([[<tr><td><a href="{name}">{name}</a></td><td>{size}</td></tr>
]], { name=name, size=tostring(size) })
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
         DEBUG(string.format("load failed: err=%s", err))
         return nil
      end
      local success, err = pcall(f)
      if not success then
         DEBUG(string.format("pcall failed: err=%s", err))
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
   function self.send_processed_file()
      sock:set_blocking(true)
      local contents = self.load_file()
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
      local file, ext = string.match(path, '(.*[.])(.*)')
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
      file = io.open(path,"rb")
      if not file then
         DEBUG(string.format("%u: Failed to open '%s'", idx, path))
         return false
      end
      local vars = {["Content-Type"]=self.content_type(path)}
      local cgi_processing = startswith(path, "/cgi-bin/") and endswith(path, ".lua")
      local server_side_processing = endswith(path, ".shtml")
      if not startswith(path, "@") and not server_side_processing and not cgi_processing then
         local fsize = self.file_size(path)
         vars["Content-Length"]= tostring(fsize)
      end
      self.send_header(200, "OK", vars)
      if server_side_processing then
         DEBUG(string.format("%u: shtml processing %s", idx, path))
         run = self.send_processed_file
      elseif cgi_processing then
         DEBUG(string.format("%u: CGI processing %s", idx, path))
         run = self.send_cgi
      elseif sock:sendfile(file) then
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

      if isdirectory(path) and not endswith(path,"/") and header_vars['Host'] and not is_hidden_dir(path) then
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
      if endswith(path,"/") or isdirectory(path) or is_hidden_dir(path) then
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
   return update,5
end

return update,100
