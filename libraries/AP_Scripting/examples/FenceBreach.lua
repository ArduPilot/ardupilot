-- Example of checking and reporting on geo-fence breach

local breach_lookup = {
  [1] = "Maximim altitude",
  [2] = "Circle",
  [4] = "Polygon",
  [8] = "Minimum altitude"
}

-- Lookup brach type bitmask and return the names of any breached fences
local function get_breach_names(breaches)
  local msg = ""
  for bitmaks, name in pairs(breach_lookup) do
    if (breaches & bitmaks) ~= 0 then
      -- And + delimiter between types if more than one
      if (string.len(msg) > 0) then
        msg = msg .. " + "
      end
      msg = msg .. name
    end
  end

  return msg
end

function update()

  local breaches = fence:get_breaches()
  if breaches ~= 0 then
    -- Time passed since fence breach
    local breach_time = millis() - fence:get_breach_time()

    gcs:send_text(0, string.format("Breached: %s fence, outside for %0.2f seconds", get_breach_names(breaches), breach_time:tofloat() * 0.001))
  end

  return update, 1000
end

return update()
