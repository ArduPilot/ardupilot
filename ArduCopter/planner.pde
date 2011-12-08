// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	planner_gcs(uint8_t argc, 		const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
const struct Menu::command planner_menu_commands[] PROGMEM = {
	{"gcs",		planner_gcs},
};

// A Macro to create the Menu
MENU(planner_menu, "planner", planner_menu_commands);

static int8_t
planner_mode(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Planner Mode\nNot intended for manual use\n\n"));
	planner_menu.run();
	return (0);
}

static int8_t
planner_gcs(uint8_t argc, const Menu::arg *argv)
{
	gcs0.init(&Serial);

	int loopcount = 0;

	while (1) {
		if (millis()-fast_loopTimer > 19) {
			fast_loopTimer			= millis();

			gcs_update();

            read_radio();

            gcs_data_stream_send(45, 1000);

            if ((loopcount % 5) == 0) // 10 hz
                gcs_data_stream_send(5, 45);

            if ((loopcount % 16) == 0) { // 3 hz
                gcs_data_stream_send(1, 5);
                gcs_send_message(MSG_HEARTBEAT);
            }

			loopcount++;
		}
	}
	return 0;
}

