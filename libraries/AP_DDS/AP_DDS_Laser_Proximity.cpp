#include "AP_DDS_Laser_Proximity.h"

#if AP_DDS_ENABLED
#include "AP_Proximity/AP_Proximity_ROS.h"

// Handle incoming laser scan messages
void AP_DDS_Laser_Proximity::handle_laser_scan(const sensor_msgs_msg_LaserScan &msg)
{
#if HAL_PROXIMITY_ENABLED
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity != nullptr)
    {
        for (uint8_t i = 0; i < proximity->num_sensors(); i++)
        {
#if AP_PROXIMITY_ROS_ENABLED
            if (proximity->get_type(i) == AP_Proximity::Type::ROS)
            {
                AP_Proximity_ROS *proximity_ros = static_cast<AP_Proximity_ROS *>(proximity->get_backend(i));
                if (proximity_ros != nullptr)
                {
                    proximity_ros->handle_laser_scan(msg);
                }
            }
#endif
        }
    }
#endif
}

#endif // AP_DDS_ENABLED
