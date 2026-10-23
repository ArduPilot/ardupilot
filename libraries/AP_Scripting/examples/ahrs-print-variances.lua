-- This script displays the ahrs variances at 1hz
-- get_variances provides the innovations normalised using the innovation variance
-- a value of 0 indicates perfect consistency between the measurement and the EKF solution
-- a value of 1 is the maximum inconsistency that will be accepted by the filter
-- nil is returned for all arguments if variances are not available

function update() -- this is the loop which periodically runs)
  vel_variance, pos_variance, height_variance, mag_variance, airspeed_variance = ahrs:get_variances()
  if vel_variance then
    gcs:send_text(0, string.format("Variances Pos:%.1f Vel:%.1f Hgt:%.1f Mag:%.1f", pos_variance, vel_variance, height_variance, mag_variance:length()))
  else
    gcs:send_text(0, string.format("Failed to retrieve variances"))
  end
  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
