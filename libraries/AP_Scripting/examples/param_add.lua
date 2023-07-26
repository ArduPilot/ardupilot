-- example for adding parameters to a lua script

-- the table key must be used by only one script on a particular flight
-- controller. If you want to re-use it then you need to wipe your old parameters
-- the key must be a number between 0 and 200. The key is persistent in storage
local PARAM_TABLE_KEY = 72

-- create a parameter table with 2 parameters in it. A table can have
-- at most 63 parameters. The table size for a particular table key
-- cannot increase without a reboot. The prefix "MY_" is used with
-- every parameter in the table. This prefix is used to ensure another
-- script doesn't use the same PARAM_TABLE_KEY.
assert(param:add_table(PARAM_TABLE_KEY, "MY_", 2), 'could not add param table')

-- create two parameters. The param indexes (2nd argument) must
-- be between 1 and 63. All added parameters are floats, with the given
-- default value (4th argument).
assert(param:add_param(PARAM_TABLE_KEY, 1, 'TEST', 3.14), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TEST2', 5.7), 'could not add param2')

gcs:send_text(0, string.format("Added two parameters"))

local param1 = Parameter("MY_TEST")
local param2 = Parameter("MY_TEST2")

gcs:send_text(0, string.format("param1=%f", param1:get()))
gcs:send_text(0, string.format("param2=%f", param2:get()))
