#pragma once

#include "AP_HAL_RP.h"
#include "HAL_RP_Class.h"
#include "AP_HAL_RP_Private.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_Notify/GPIO_LED_1.h>

namespace RP {

class LED
{
public:
	bool init(void);
	void update(void);
private:
	GPIO_LED_1 _led;
};
} // namespace RP
