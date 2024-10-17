local CAMERA_TRACKING_IMAGE_STATUS = {}
CAMERA_TRACKING_IMAGE_STATUS.id = 275
CAMERA_TRACKING_IMAGE_STATUS.fields = {
             { "point_x", "<f" },
             { "point_y", "<f" },
             { "radius", "<f" },
             { "rec_top_x", "<f" },
             { "rec_top_y", "<f" },
             { "rec_bottom_x", "<f" },
             { "rec_bottom_y", "<f" },
             { "tracking_status", "<B" },
             { "tracking_mode", "<B" },
             { "target_data", "<B" },
             }
return CAMERA_TRACKING_IMAGE_STATUS
