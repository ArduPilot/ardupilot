-- load default parameters depending upon if the board is a left or a right

defaults = {}

forced = {{"BATT_SERIAL_NUM",   2,   3},
          {"CAN_NODE",        109, 110},
          {"ESC_NUMBER",        2,   0},
          {"ESC_NUMBER2",       1,   3},
          {"RELAY1_FUNCTION",  17,  18},
          {"RELAY2_FUNCTION",  19,  20},
          {"OUT1_FUNCTION",    35,  33},
          {"OUT2_FUNCTION",    34,  36}}

local index
if gpio:read (4) then -- id pin is pin 4, high if it's a left
  index = 2
else
  index = 3
end

function set_table(fn, table)
  for _,values in ipairs (table) do
    assert(fn(param, values[1], values[index]), "not set the parameter: " .. values[index])
  end
end

set_table(param.set_default, defaults)
set_table(param.set_and_save, forced)
