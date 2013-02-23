// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


void set_home_is_set(bool b)
{
    ap.home_is_set 	= b;

	if(b) Log_Write_Event(DATA_SET_HOME);
}

// ---------------------------------------------
void set_armed(bool b)
{
	ap.armed = b;
	if(b){
		Log_Write_Event(DATA_ARMED);

	}else{
		Log_Write_Event(DATA_DISARMED);
	}
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
	ap.auto_armed = b;
	if(b){
		Log_Write_Event(DATA_AUTO_ARMED);
	}
}

// ---------------------------------------------
static void set_failsafe(bool mode)
{
    // only act on changes
    // -------------------
    if(ap.failsafe != mode) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        ap.failsafe = mode;

        if (ap.failsafe == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_off_event();
			Log_Write_Event(DATA_FAILSAFE_OFF);

        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_on_event();
            Log_Write_Event(DATA_FAILSAFE_ON);
        }
    }
}


// ---------------------------------------------
void set_low_battery(bool b)
{
	if(ap.low_battery != b && true == b){
		Log_Write_Event(DATA_LOW_BATTERY);
	}
	ap.low_battery = b;
}

// ---------------------------------------------
void set_takeoff_complete(bool b)
{
}

// ---------------------------------------------
void set_land_complete(bool b)
{
}

// ---------------------------------------------
void set_rtl_reached_alt(bool b)
{
}

// ---------------------------------------------

void set_alt_change(uint8_t flag){
}

void set_compass_healthy(bool b)
{
	if(ap.compass_status != b){
		if(false == b){
			Log_Write_Event(DATA_LOST_COMPASS);
		}
	}
	ap.compass_status = b;
}

void set_gps_healthy(bool b)
{
	if(ap.gps_status != b){
		if(false == b){
			Log_Write_Event(DATA_LOST_GPS);
		}
	}
	ap.gps_status = b;
}

void dump_state()
{
    cliSerial->printf("st: %u\n",ap.value);
	//cliSerial->printf("%u\n", *(uint16_t*)&ap);
}
