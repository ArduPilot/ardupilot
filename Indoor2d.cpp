// My first file here
// Neuer Flugmodus
// holds the position for 2 directions with 4 Ultrasonic sensors

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


void setup()
// The setup() function is called when the board boots
{
    hal.console->printf("Hallo, Ronny ist hier\n");
	gcs().send_text(MAV_SEVERITY_CRITICAL, "hello ronny ist hier! %5.3f", (double)3.142f);
}

// stabilize_init - initialise stabilize controller
bool Indoor2d::ModeHold2DIndoor::init(bool ignore_checks)
{
	// Hier könnten Dinge ausgeschlossen werden, die den Modus verhindern
	// if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !_copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

bool Indoor2d::ModeHold2DIndoor::run()
// Wird 400 mal pro Sekunde aufgerufen
// und kann die laufenden Arbeiten übernehmen
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;
	
	
	// Das sind die Beisiele wie man die Position des Hubschraubers einstellt

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilots desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilots desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilots throttle
	// Ronny: Hier könnte man das Gas manuell einstellen !!!! Huraaaa Hier kann ich eingreifen
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
	
	// Der Ultraschall-Pin könnte hier in einer Dauerschleife abgefragt werden, bis er anspricht:
	// Die Zeit, die vergangen ist, wird mit dieser Funktion in Microsekunden abgefragt:
	hal.scheduler->micros();
	hal.scheduler->millis();
	
	// Dies ist ein Sleep();
	// hal.scheduler->delay() 
	// hal.scheduler->delay_microseconds() 
	// hal.gpio->pinMode(), hal.gpio->read() and hal.gpio->write() for accessing GPIO pins	
}

// Timer registieren:
hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&ModeHold2DIndoor::_update));

void ModeHold2DIndoor::_update()
// Dieser Timer wird 100 x pro Sekunde angesprungen
// Mit 1 KHz also alle Millisekunden
{
	
}


AP_HAL_MAIN();



