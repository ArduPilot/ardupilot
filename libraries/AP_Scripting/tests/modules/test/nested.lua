-- Description: Test require()ing another script
local nested = {}
nested.top = require('test/top')

-- create some test functions to call
function nested.call_fn()
    return "nested"
end

return nested
