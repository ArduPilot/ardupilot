// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

bool no_RC_in;

float _aft_rpm;
float _fwd_rpm;


//Counters and timers
uint8_t killswitch_counter;

uint16_t spoolup_timer;
uint16_t spoolup_watcher;

uint16_t start_rpm_comp_time;


// Triggers
bool timer_trigger;


// Auto Tune
int8_t num_battery;
float payload_weight;
float vehicle_weight;


// Herelink decoding of buttons
uint32_t ch7_timer;
uint32_t ch9_timer;
uint32_t ch10_timer;
uint32_t ch11_timer;
uint32_t ch12_timer;

bool cam_button_pressed;
bool ch9_button_pressed;
bool ch10_button_pressed;
bool ch11_button_pressed;
bool ch12_button_pressed;

bool ch7_button_pressed;
bool long_press_flag_ch7;
bool short_press_flag_ch7;
bool ch7_button_hold;

bool long_press_flag_ch9;
bool long_press_flag_ch10;
bool long_press_flag_ch11;
bool long_press_flag_ch12;

bool short_press_flag_ch9;
bool short_press_flag_ch10;
bool short_press_flag_ch11;
bool short_press_flag_ch12;

bool ch9_button_hold;
bool ch10_button_hold;
bool ch11_button_hold;
bool ch12_button_hold;

uint8_t function_counter;
uint8_t cam_button_debounce_timer;
bool flight_mode_switch;

bool zoom_out;


enum vehicle_state{
	disarm,
	spoolup,
	land,
	takeoff,
	hover,
	landing,

}spirit_state;




#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


