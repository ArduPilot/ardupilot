--[[
set scale factor for RPM on some ESCs to allow for different pole count on some ESCs
--]]

-- set ESC 4 (index 3) to 2.0 times reported RPM
esc_telem:set_rpm_scale(3, 2.0)

-- set ESC 6 (index 5) to 2.0 times reported RPM
esc_telem:set_rpm_scale(5, 2.0)

gcs:send_text(0,"Setup motor poles")
