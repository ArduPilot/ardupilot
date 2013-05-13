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
    float ret = rssi_analog_source->voltage_average() * 50;
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
        obstacle.sonar1_distance_cm = sonar.distance_cm();
        obstacle.sonar2_distance_cm = sonar2.distance_cm();
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm &&
            obstacle.sonar2_distance_cm <= (uint16_t)obstacle.sonar2_distance_cm)  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar1 obstacle %.0fcm"),
                                  obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        } else if (obstacle.sonar2_distance_cm <= (uint16_t)g.sonar_trigger_cm) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar2 obstacle %.0fcm"),
                                  obstacle.sonar2_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
    } else {
        // we have a single sonar
        obstacle.sonar1_distance_cm = sonar.distance_cm();
        obstacle.sonar2_distance_cm = 0;
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm)  {
            // obstacle detected in front 
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar obstacle %.0fcm"),
                                  obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }

    Log_Write_Sonar();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.sonar_debounce &&
        hal.scheduler->millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) { 
        gcs_send_text_fmt(PSTR("Obstacle passed"));
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}
