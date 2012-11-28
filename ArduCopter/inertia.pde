/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    static uint8_t log_counter_inav = 0;

    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    if( motors.armed() && g.log_bitmask & MASK_LOG_INAV ) {
        log_counter_inav++;
        if( log_counter_inav >= 10 ) {
            log_counter_inav = 0;
            Log_Write_INAV(G_Dt);
        }
    }
#endif
}