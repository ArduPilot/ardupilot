-- report the position sent by the GCS via GLOBAL_POSITION_INT
--
-- The GCS must be configured as the GCS position source with the
-- MAV_GCS_POS_SRC parameter. The position is kept when the GCS stops
-- sending, so the age is reported as well.

local MAX_AGE_MS = 5000

function update()
    local loc = gcs:get_gcs_location()
    if not loc then
        gcs:send_text(6, "GCS position: none received")
        return update, 1000
    end
    local age_ms = gcs:get_gcs_location_age_ms():toint()
    if age_ms > MAX_AGE_MS then
        gcs:send_text(4, string.format("GCS position stale: %.1fs", age_ms / 1000))
    else
        gcs:send_text(6, string.format("GCS position %.7f %.7f alt %.0fm",
                      loc:lat() / 1e7, loc:lng() / 1e7, loc:alt() / 100))
    end
    return update, 1000
end

return update()
