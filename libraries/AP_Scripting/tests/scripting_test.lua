-- this is a script which is primarily intended to test the scripting bindings and be used with autotest
-- however it may be useful to some as a demo of how the API's can be used, this example is much less
-- heavily commented, and may require quite a bit more RAM to run in

local loop_time = 500 -- number of ms between runs

function is_equal(v1, v2, tolerance)
  return math.abs(v1 - v2) < (tolerance or 0.0001)
end

function test_offset(ofs_e, ofs_n)
  local manipulated_pos = Location()
  manipulated_pos:offset(ofs_e, ofs_n)
  local distance_offset = manipulated_pos:get_distance(Location())
  if distance_offset < 0 then
    gcs:send_text(0, "Location:get_distance() returned a negative number")
    return false
  end
  local error_dist = distance_offset - math.sqrt((ofs_e*ofs_e)+(ofs_n*ofs_n))
  if error_dist > 0.1 then
    gcs:send_text(0, string.format("Failed offset %.1f, %.1f with an error of %f", ofs_e, ofs_n, error_dist))
    return false
  end
  local from_origin_NE = Location():get_distance_NE(manipulated_pos)
  if (not is_equal(ofs_e, from_origin_NE:x(), 0.01)) or (not is_equal(ofs_n, from_origin_NE:y(), 0.01)) then
    gcs:send_text(0, string.format("Failed to offset %.1f, %.1f %.1f %.1f", ofs_e, ofs_n, from_origin_NE:x(), from_origin_NE:y()))
    return false
  end
  local from_origin_NED = Location():get_distance_NED(manipulated_pos)
  if (not is_equal(from_origin_NED:x(), from_origin_NE:x())) or (not is_equal(from_origin_NED:y(), from_origin_NE:y())) then 
    gcs:send_text(0, string.format("Distance NED != NE %.1f, %.1f %.1f %.1f", from_origin_NED:x(), from_origin_NED:y(), from_origin_NE:x(), from_origin_NE:y()))
    return false
  end
  return true
end

function update()
  local all_tests_passed = true
  -- each test should run then and it's result with the previous ones
  all_tests_passed = test_offset(500, 200) and all_tests_passed

  if all_tests_passed then
    gcs:send_text(3, "Internal tests passed")
  end
  return update, loop_time
end

return update() -- run immediately before starting to reschedule
