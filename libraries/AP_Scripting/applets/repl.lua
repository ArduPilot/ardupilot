-- Interactive REPL (read-evaluate-print-loop) for the Lua scripting engine
-- accessible over serial, with line editing, history, and output formatting.

local PORT_IDX = 0 -- 0-based index of Scripting protocol port to use
local MAX_HISTORY = 50 -- number of lines of history to keep (must be >= 1)

local port = serial:find_serial(PORT_IDX)
assert(port, "scripting port not configured")

-- scan through parameters to find our port and grab its baud rate
do
  local serial_info = ""
  local baud = 115200
  local port_num = 0
  while PORT_IDX >= 0 and port_num <= 9 do
    local protocol = param:get(("SERIAL%d_PROTOCOL"):format(port_num))
    port_num = port_num + 1
    if protocol == 28 then PORT_IDX = PORT_IDX - 1 end
  end
  if PORT_IDX == -1 then -- correct port index found
    port_num = port_num - 1
    baud = param:get(("SERIAL%d_BAUD"):format(port_num)) or 115200
    serial_info = (" on SERIAL%d, BAUD=%d"):format(port_num, baud)
  end
  -- if we can't find the right port, the baud probably does not matter
  -- (e.g. CAN or network port)
  port:begin(baud)

  gcs:send_text(6, "Lua REPL starting"..serial_info)
end

-- grab things we want from the environment in case the user messes them up
local string = string
local table = table
local print_real = print

-- declaration of main state variable and functions
local state_func, state_read, state_eval, state_print

-- write the string s to the port, buffering if not all could be written
local tx_buf = {"\r\n\r\nLua REPL started.\r\n> "} -- first message and prompt
local writestring
if port.writestring ~= nil then -- use more efficient method if we have it
  writestring = function(s)
    if tx_buf then -- stuff already in the buffer?
      tx_buf[#tx_buf+1] = s -- this needs to go after
    else
      local written = port:writestring(s)
      if written < #s then
        -- write was short i.e. system buffer is full. buffer the rest of the
        -- string and transmit it later
        tx_buf = { s:sub(written+1) }
      end
    end
  end
else
  writestring = function(s)
    if tx_buf then -- stuff already in the buffer?
      tx_buf[#tx_buf+1] = s -- this needs to go after
    else
      for ci = 1, #s do
        if port:write(s:byte(ci)) == 0 then
          -- write failed i.e. system buffer is full
          -- we now buffer the rest of the string and transmit it later
          tx_buf = { s:sub(ci) }
          break
        end
      end
    end
  end
end

-- substitute print function for within the REPL that prints to the port
function print(...) -- luacheck: ignore
  local t = table.pack(...)
  for i = 1, t.n do
    writestring(tostring(t[i]))
    writestring((i ~= t.n) and "\t" or "\r\n")
  end
end
local print = print_real -- luacheck: ignore

-- write the character c to the port, buffering if failed
local function writechar(c)
  -- buffer character if stuff already in buffer or write fails
  if tx_buf or port:write(c) == 0 then
    tx_buf[#tx_buf+1] = string.char(c) -- massive overhead...
  end
end

local function writeobj(o)
  if type(o) == "table" then
    writestring("{ ")
    for k, v in pairs(o) do
      if type(k) ~= "number" then k = '"'..k..'"' end
      writestring("["..k.."] = ")
      writeobj(v)
      writestring(", ")
    end
    writestring("}")
  else
    writestring(tostring(o))
  end
end

local curr_line = nil -- table of line bytes, or nil if viewing history
local curr_pos = 1 -- position the next character will be put at
local curr_esc = nil -- table of escape sequence bytes

local eval_pieces = {} -- pieces of code being evaluated

local history_lines = {""} -- lines in the history (one is always being edited)
local history_pos = 1 -- position in the history being edited

local function writeprompt()
  writestring((#eval_pieces > 0) and ">> " or "> ")
end

local function movehistory(dir)
  if curr_line ~= nil then -- current line was edited, store it in history
    history_lines[history_pos] = string.char(table.unpack(curr_line))
    curr_line = nil
  end

  history_pos = history_pos + dir -- move to new position

  writestring("\x1B[2K\r") -- erase line and return cursor to start
  writeprompt() -- draw prompt
  local line = history_lines[history_pos] -- and current line from history
  writestring(line)
  curr_pos = #line + 1
end

local function readesc(c)
  assert(curr_esc) -- only called if curr_esc isn't nil

  if c == 27 then -- another escape, clear line and buffer and exit escape mode
    curr_line = nil
    curr_pos = 1
    curr_esc = nil
    eval_pieces = {}
    history_pos = #history_lines
    history_lines[history_pos] = ""
    writestring("\r\n")
    writeprompt()
    return
  else
    curr_esc[#curr_esc+1] = c
  end

  if curr_esc[1] ~= 91 then -- not a [, exit escape mode
    curr_esc = nil
    return
  end

  if #curr_esc < 2 then return end -- command character not yet present

  local line_len = #history_lines[history_pos]
  if curr_line ~= nil then line_len = #curr_line end

  -- c is now the command character
  if c == 65 then -- up
    if history_pos > 1 then
      movehistory(-1)
    end
  elseif c == 66 then -- down
    if history_pos < #history_lines then
      movehistory(1)
    end
  elseif c == 67 then -- right
    if curr_pos < line_len + 1 then
      writestring("\x1B[C")
      curr_pos = curr_pos + 1
    end
  elseif c == 68 then -- left
    if curr_pos > 1 then
      writestring("\x1B[D")
      curr_pos = curr_pos - 1
    end
  elseif c == 72 then -- home
    if curr_pos > 1 then
      writestring(("\x1B[%dD"):format(curr_pos-1))
      curr_pos = 1
    end
  elseif c == 70 then -- end
    if curr_pos < line_len + 1 then
      writestring(("\x1B[%dC"):format(line_len-curr_pos+1))
      curr_pos = line_len + 1
    end
  end

  curr_esc = nil -- exit escape mode, handling complete
end

state_read = function ()
  while true do
    local c = port:read()
    if c == -1 then return end -- no new character, give time for more to come

    if curr_esc ~= nil then -- in escape sequence
      readesc(c)
    elseif c == 27 then -- escape, start of a control sequence
      curr_esc = {} -- engage escape sequence handler
    elseif c == 13 then -- \r, line complete (\n is ignored)
      writestring("\r\n")
      break
    elseif c == 8 or c == 127 then -- backspace
      if curr_pos > 1 then -- a character to delete?
        if curr_line == nil then -- retrieve line for editing
          curr_line = table.pack(history_lines[history_pos]:byte(1, -1))
        end
        table.remove(curr_line, curr_pos-1)
        writechar(8) -- back cursor up
        curr_pos = curr_pos - 1
        if curr_pos <= #curr_line then -- draw the characters after that one
          writestring(string.char(table.unpack(
            curr_line, curr_pos, #curr_line)))
        end
        writechar(32) -- blank out trailing character
        -- back cursor up to insertion point
        writestring(("\x1B[%dD"):format(#curr_line-curr_pos+2))
      end
    elseif c >= 32 and c <= 126 then -- a character to type
      if curr_line == nil then -- retrieve line for editing
        curr_line = table.pack(history_lines[history_pos]:byte(1, -1))
      end
      table.insert(curr_line, curr_pos, c) -- store character in the line
      if curr_pos < #curr_line then -- draw the new character and any after
        writestring(string.char(table.unpack(curr_line, curr_pos, #curr_line)))
      else
        writechar(c)
      end
      curr_pos = curr_pos + 1
      if curr_pos <= #curr_line then -- back cursor up to insertion point
        writestring(("\x1B[%dD"):format(#curr_line-curr_pos+1))
      end
    end

    if tx_buf then return end -- give time to flush if buffer full
  end

  -- loop break, line is complete!
  local line = history_lines[history_pos]
  if curr_line ~= nil then
    line = string.char(table.unpack(curr_line)) -- store line for processing
    curr_line = nil
  end

  if #line == 0 then -- line is empty, ignore it
    writeprompt()
    return
  end

  -- if this line is different to the last one added (one before the last entry)
  if history_lines[#history_lines-1] ~= line then
    history_lines[#history_lines] = line -- insert it at history end
    history_lines[#history_lines+1] = "" -- create empty entry for next line
    if #history_lines > MAX_HISTORY then table.remove(history_lines, 1) end
  else -- don't create a new entry with a duplicate line
    history_lines[#history_lines] = "" -- just clear and reuse the last entry
  end
  history_pos = #history_lines -- now editing the last entry
  curr_pos = 1

  eval_pieces[#eval_pieces+1] = line -- evaluate the line
  state_func = state_eval
end

local function to_chunk(pieces)
  local num = 1+#pieces
  local pos = 1

  local function next_piece()
    if pos == num then return nil end -- no more pieces
    local piece = pieces[pos]
    pos = pos + 1
    return piece
  end

  return next_piece
end

local eval_results
state_eval = function ()
  local func, err
  -- try to compile a single line as "return %s;" assuming it could be an
  -- expression. technique borrowed from the official Lua REPL.
  if #eval_pieces == 1 then
    local expr_pieces = {"return ", eval_pieces[1], ";"}
    func = load(to_chunk(expr_pieces), "=input", "t", _ENV)
  end
  if func == nil then -- compilation unsuccessful, load normally
    func, err = load(to_chunk(eval_pieces), "=input", "t", _ENV)
  end

  -- if there is an error at the end of the statement, assume we need more to
  -- complete it. technique borrowed from the official Lua REPL.
  -- note that err is not nil if func is nil
  ---@diagnostic disable-next-line: need-check-nil
  if func == nil and err:sub(-5, -1) == "<eof>" then
    -- add a newline and get another line from the user
    eval_pieces[#eval_pieces+1] = "\n"
    writeprompt()
    state_func = state_read
    return
  end

  eval_pieces = {} -- destroy to make room for result
  if func == nil then -- result was the error message
    eval_results = { err, n = 1 }
  else
    local success, result = pcall(function () return table.pack(func()) end)
    if success == true then
      eval_results = result
    else -- result was the error message
      eval_results = { result, n = 1 }
    end
  end
  state_func = state_print
end

state_print = function ()
  for i = 1, eval_results.n do
    writeobj(eval_results[i]) -- write each result separated by tabs
    writestring((i ~= eval_results.n) and "\t" or "\r\n")
    eval_results[i] = nil -- destroy to make room for stringified version
  end
  eval_results = nil

  writeprompt()
  state_func = state_read -- loop back to read
end

state_func = state_read
local function update()
  if tx_buf then -- write out stuff in tx buffer if present
    local old_buf = tx_buf
    tx_buf = nil
    for _, s in ipairs(old_buf) do -- re-write all data
      writestring(s)
    end
  else -- otherwise we have time to process
    state_func()
  end

  return update, 10
end

return update()
