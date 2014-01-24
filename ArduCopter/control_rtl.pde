/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_rtl.pde - init and run calls for RTL flight mode
 */

// rtl_init - initialise rtl controller
static bool rtl_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        do_RTL();
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
static void rtl_run()
{
    verify_RTL();
}

