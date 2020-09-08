// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES


uint8_t killswitch_counter;

float _aft_rpm;
float _fwd_rpm;

float vehicle_weight;

//uint8_t touchdown_accel_counter;

//bool take_off_complete;
uint16_t spoolup_timer;
bool timer_trigger;

Vector2f adv_ratio;

uint32_t ch9_timer;
uint32_t ch10_timer;
uint32_t ch11_timer;
uint32_t ch12_timer;

int8_t speed_setting;

bool cam_button_pressed;
bool ch9_button_pressed;
bool ch10_button_pressed;
bool ch11_button_pressed;
bool ch12_button_pressed;

bool long_press_flag_ch9;
bool long_press_flag_ch10;
bool long_press_flag_ch11;
bool long_press_flag_ch12;

bool short_press_flag_ch9;
bool short_press_flag_ch10;
bool short_press_flag_ch11;
bool short_press_flag_ch12;

uint8_t function_counter;
uint8_t cam_button_debounce_timer;
bool flight_mode_switch;

bool ch9_button_hold;
bool ch10_button_hold;
bool ch11_button_hold;
bool ch12_button_hold;


enum vehicle_state{
	disarm,
	spoolup,
	takeoff,
	hover,
	//fwd_flight,
	//brake,
	land
}spirit_state;
#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


