std = "lua53"  -- Lua 5.3 standard

max_line_length = 678

ignore = {
  "111",  -- setting non-standard global
  "112",  -- mutating an undefined global variable.
  "113",  -- accessing undefined variable
  "212",  -- Unused argument
  "214",  -- used variable with unused hint
  "611",  -- line consists only of whitespace
  "612",  -- trailing whitespace
  "613",  -- trailing whitespace in string
  "614",  -- trailing whitespace in comment
  "621",  -- inconsistent indentation
}
