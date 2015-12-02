/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static Compass compass;

uint32_t timer;

void setup() {
    hal.console->println("Compass library test");

    if (!compass.init()) {
        hal.console->println("compass initialisation failed!");
        while (1) ;
    }
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    compass.set_and_save_offsets(0,0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0f)); // set local difference between magnetic north and true north

    hal.scheduler->delay(1000);
    timer = AP_HAL::micros();
}

void loop()
{
    static float min[3], max[3], offset[3];

    compass.accumulate();

    if((AP_HAL::micros()- timer) > 100000L)
    {
        timer = AP_HAL::micros();
        compass.read();
        unsigned long read_time = AP_HAL::micros() - timer;
        float heading;

        if (!compass.healthy()) {
            hal.console->println("not healthy");
            return;
        }
	Matrix3f dcm_matrix;
	// use roll = 0, pitch = 0 for this example
	dcm_matrix.from_euler(0, 0, 0);
        heading = compass.calculate_heading(dcm_matrix);
        compass.learn_offsets();

        // capture min
        const Vector3f &mag = compass.get_field();
        if( mag.x < min[0] )
            min[0] = mag.x;
        if( mag.y < min[1] )
            min[1] = mag.y;
        if( mag.z < min[2] )
            min[2] = mag.z;

        // capture max
        if( mag.x > max[0] )
            max[0] = mag.x;
        if( mag.y > max[1] )
            max[1] = mag.y;
        if( mag.z > max[2] )
            max[2] = mag.z;

        // calculate offsets
        offset[0] = -(max[0]+min[0])/2;
        offset[1] = -(max[1]+min[1])/2;
        offset[2] = -(max[2]+min[2])/2;

        // display all to user
        hal.console->printf("Heading: %.2f (%3d,%3d,%3d) i2c error: %u",
			    ToDeg(heading),
			    (int)mag.x,
			    (int)mag.y,
			    (int)mag.z, 
			    (unsigned)hal.i2c->lockup_count());

        // display offsets
        hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
                      offset[0], offset[1], offset[2]);

        hal.console->printf(" t=%u", (unsigned)read_time);

        hal.console->println();
    } else {
	    hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
