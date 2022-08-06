
--returns active source set used by EKF3
-- can be used for infering the current source set in use without RC for autonomous source switching

function update() -- this is the loop which periodically runs

  gcs:send_text(0, string.format("current source  set in use :%d", ahrs:get_posvelyaw_source_set()))

  return update, 1000 -- 1000ms reschedules the loop (1Hz)
end

return update() -- run immediately before starting to reschedule
