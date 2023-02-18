
-- Don't check globals yet. This requires us to add a list of all the AP bindings
global = false

-- https://luacheck.readthedocs.io/en/stable/warnings.html
ignore = {"631", -- Line is too long
          "611", -- A line consists of nothing but whitespace.
          "612", -- A line contains trailing whitespace.
          "614"} -- Trailing whitespace in a comment.

-- These lua scripts are not for running on AP
exclude_files = {"Tools/CHDK-Scripts/*", "modules/*"}
