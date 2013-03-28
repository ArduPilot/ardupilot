// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_sonar(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    sonar.Init(&adc);
    sonar2.Init(&adc);
#else
    sonar.Init(NULL);
    sonar2.Init(NULL);
#endif
}

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void read_battery(void)
{
	if(g.battery_monitoring == 0) {
		battery_voltage1 = 0;
		return;
	}
	
    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        // this copes with changing the pin at runtime
        batt_volt_pin->set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_pin);
    }
    if(g.battery_monitoring == 4) {
        // this copes with changing the pin at runtime
        batt_curr_pin->set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_pin);
        current_total1   += current_amps1 * (float)delta_ms_medium_loop * 0.0002778;                                    // .0002778 is 1/3600 (conversion to hours)
    }
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->read_average();
    receiver_rssi = constrain_int16(ret, 0, 255);
}

// read the sonars
static void read_sonars(void)
{
    if (!sonar.enabled()) {
        // this makes it possible to disable sonar at runtime
        return;
    }

    if (sonar2.enabled()) {
        // we have two sonars
        float sonar1_dist_cm = sonar.distance_cm();
        float sonar2_dist_cm = sonar2.distance_cm();
        if (sonar1_dist_cm <= g.sonar_trigger_cm &&
            sonar1_dist_cm <= sonar2_dist_cm)  {
            // we have an object on the left
            obstacle.detected = true;
            obstacle.turn_angle = g.sonar_turn_angle;
        } else if (sonar2_dist_cm <= g.sonar_trigger_cm) {
            // we have an object on the right
            obstacle.detected = true;
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
    } else {
        // we have a single sonar
        float sonar_dist_cm = sonar.distance_cm();
        if (sonar_dist_cm <= g.sonar_trigger_cm)  {
            // obstacle detected in front 
            obstacle.detected = true;
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }

    // no object detected - reset after the turn time
    if (hal.scheduler->millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) { 
        obstacle.detected = false;
    }
}
