-- Description: Test require()ing another script
local top = {}

-- create some test functions to call
function top.call_fn()
    return "top"
end

return top
