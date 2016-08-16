#include "AP_WaterDetector.h"
#include "AP_WaterDetector_Backend.h"

AP_WaterDetector_Backend::AP_WaterDetector_Backend(AP_WaterDetector &_water_detector, AP_WaterDetector::WaterDetector_State &_state) :
water_detector(_water_detector),
state(_state)
{

}
