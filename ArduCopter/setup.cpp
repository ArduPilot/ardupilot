#include "Copter.h"

// report_compass - displays compass information.  Also called by compassmot.pde
void Copter::report_compass()
{
    hal.console->printf("Compass\n");
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    hal.console->printf("Mag Dec: %4.4f\n",
            (double)degrees(compass.get_declination()));

    // mag offsets
    Vector3f offsets;
    for (uint8_t i=0; i<compass.get_count(); i++) {
        offsets = compass.get_offsets(i);
        // mag offsets
        hal.console->printf("Mag%d off: %4.4f, %4.4f, %4.4f\n",
                        (int)i,
                        (double)offsets.x,
                        (double)offsets.y,
                        (double)offsets.z);
    }

    // motor compensation
    hal.console->printf("Motor Comp: ");
    if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        hal.console->printf("Off\n");
    }else{
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            hal.console->printf("Throttle");
        }
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            hal.console->printf("Current");
        }
        Vector3f motor_compensation;
        for (uint8_t i=0; i<compass.get_count(); i++) {
            motor_compensation = compass.get_motor_compensation(i);
            hal.console->printf("\nComMot%d: %4.2f, %4.2f, %4.2f\n",
                        (int)i,
                        (double)motor_compensation.x,
                        (double)motor_compensation.y,
                        (double)motor_compensation.z);
        }
    }
    print_blanks(1);
}

void Copter::print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        hal.console->printf("\n");
    }
}

void Copter::print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        hal.console->printf("-");
    }
    hal.console->printf("\n");
}

void Copter::print_enabled(bool b)
{
    if(b)
        hal.console->printf("en");
    else
        hal.console->printf("dis");
    hal.console->printf("abled\n");
}

void Copter::report_version()
{
    hal.console->printf("FW Ver: %d\n",(int)(g.k_format_version));
    print_divider();
    print_blanks(2);
}
