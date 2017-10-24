/*******************************************************************************
* rc_dsm.c
*******************************************************************************/
#define _GNU_SOURCE
#include "roboticscape/roboticscape.h"
#include "roboticscape/rc_defs.h"
#include "roboticscape/preprocessor_macros.h"
#include "mmap/rc_mmap_gpio_adc.h"
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#define MAX_DSM_CHANNELS 9
#define PAUSE 115	//microseconds
// don't ask me why, but this is the default range for spektrum and orange
#define DEFAULT_MIN 1142
#define DEFAULT_MAX 1858

#define DSM_UART_BUS	4
#define DSM_BAUD_RATE	115200
#define DSM_PACKET_SIZE	16

/*******************************************************************************
* Local Global Variables
*******************************************************************************/
int running;
int rc_channels[MAX_DSM_CHANNELS];
int rc_maxes[MAX_DSM_CHANNELS];
int rc_mins[MAX_DSM_CHANNELS];
int num_channels; // actual number of channels being sent
int resolution; // 10 or 11
int new_dsm_flag;
int dsm_frame_rate;
uint64_t last_time;
pthread_t serial_parser_thread;
int listening; // for calibration routine only
void (*dsm_ready_func)();
int rc_is_dsm_active_flag; 

/*******************************************************************************
* Local Function Declarations
*******************************************************************************/
int load_default_calibration();
void* serial_parser(void *ptr); //background thread
void* calibration_listen_func(void *ptr);

/*******************************************************************************
* int rc_initialize_dsm()
* 
* returns -1 for failure or 0 for success
* This starts the background thread serial_parser_thread which listens
* for serials packets on that interface.
*******************************************************************************/ 
int rc_initialize_dsm(){
	int i;
	//if calibration file exists, load it and start spektrum thread
	FILE *cal;
	char file_path[100];

	// construct a new file path string
	strcpy(file_path, CONFIG_DIRECTORY);
	strcat(file_path, DSM_CAL_FILE);
	
	// open for reading
	cal = fopen(file_path, "r");

	if (cal == NULL) {
		printf("\ndsm Calibration File Doesn't Exist Yet\n");
		printf("Run calibrate_dsm example to create one\n");
		printf("Using default values for now\n");
		load_default_calibration();
	}
	else{
		for(i=0;i<MAX_DSM_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
		}
		#ifdef DEBUG
		printf("DSM Calibration Loaded\n");
		#endif
		fclose(cal);
	}

	rc_set_pinmux_mode(DSM_PIN, PINMUX_UART);
	
	
	dsm_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	rc_is_dsm_active_flag = 0;
	rc_set_dsm_data_func(&rc_null_func);
	
	if(rc_uart_init(DSM_UART_BUS, DSM_BAUD_RATE, 0.1)){
		printf("Error, failed to initialize UART%d for dsm\n", DSM_UART_BUS);
	}
	
	pthread_create(&serial_parser_thread, NULL, serial_parser, (void*) NULL);
	#ifdef DEBUG
	printf("dsm Thread Started\n");
	#endif

	rc_usleep(10000); // let thread start
	return 0;
}

/*******************************************************************************
* @ int rc_stop_dsm_service()
* 
* signals the serial_parser_thread to stop and allows up to 1 second for the 
* thread to  shut down before returning.
*******************************************************************************/
int rc_stop_dsm_service(){
	int ret = 0;

	if(running){
		running = 0; // this tells serial_parser_thread loop to stop
		// allow up to 0.3 seconds for thread cleanup
		timespec thread_timeout;
		clock_gettime(CLOCK_REALTIME, &thread_timeout);
		rc_timespec_add(&thread_timeout, 0.3);
		int thread_err = 0;
		thread_err = pthread_timedjoin_np(serial_parser_thread, NULL, 
															   &thread_timeout);
		if(thread_err == ETIMEDOUT){
			printf("WARNING: dsm serial_parser_thread exit timeout\n");
			ret = -1;
		}
		#ifdef DEBUG
		printf("DSM thread exited successfully\n");
		#endif
		
		running = 0;
	}
	return ret;
}

/*******************************************************************************
* @ int rc_get_dsm_ch_raw(int ch)
* 
* Returns the pulse width in microseconds commanded by the transmitter for a
* particular channel. The user can specify channels 1 through 9 but non-zero 
* values will only be returned for channels the transmitter is actually using. 
* The raw values in microseconds typically range from 900-2100us for a standard
* radio with default settings.
*******************************************************************************/
int rc_get_dsm_ch_raw(int ch){
	if(ch<1 || ch > MAX_DSM_CHANNELS){
		printf("please enter a channel between 1 & %d",MAX_DSM_CHANNELS);
		return -1;
	}
	else{
		new_dsm_flag = 0;
		return rc_channels[ch-1];
	}
}

/*******************************************************************************
* float rc_get_dsm_ch_normalized(int ch)
* 
* Returns a scaled value from -1 to 1 corresponding to the min and max values
* recorded during calibration. The user
* MUST run the clalibrate_dsm example to ensure the normalized values returned
* by this function are correct.
*******************************************************************************/
float rc_get_dsm_ch_normalized(int ch){
	if(ch<1 || ch > MAX_DSM_CHANNELS){
		printf("please enter a channel between 1 & %d",MAX_DSM_CHANNELS);
		return -1;
	}
	float range = rc_maxes[ch-1]-rc_mins[ch-1];
	if(range!=0 && rc_channels[ch-1]!=0) {
		new_dsm_flag = 0;
		float center = (rc_maxes[ch-1]+rc_mins[ch-1])/2;
		return 2*(rc_channels[ch-1]-center)/range;
	}
	else{
		return 0;
	}
}

/*******************************************************************************
* @ int rc_is_new_dsm_data()
* 
* returns 1 if new data is ready to be read by the user. 
* otherwise returns 0
*******************************************************************************/
int rc_is_new_dsm_data(){
	return new_dsm_flag;
}

/*******************************************************************************
* @ int rc_set_dsm_data_func(void (*func)(void))
* 
* sets the 
*******************************************************************************/
int rc_set_dsm_data_func(void (*func)(void)){
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to new_dsm_data_func\n");
		return -1;
	}
	dsm_ready_func = func;
	return 0;
}

/*******************************************************************************
* @ uint64_t rc_nanos_since_last_dsm_packet()
* 
* returns the number of milliseconds since the last dsm packet was received.
* if no packet has ever been received, return -1;
*******************************************************************************/
uint64_t rc_nanos_since_last_dsm_packet(){
	// if global variable last_time ==0 then no packet must have arrived yet
	if (last_time==0){
		return UINT64_MAX;
	}	
	return rc_nanos_since_epoch()-last_time;
}

/*******************************************************************************
* @ int rc_get_dsm_resolution()
* 
* returns 10 or 11 indicating 10-bit or 11-bit resolution
* returns a 0 if no packet has been received yet
*******************************************************************************/
int rc_get_dsm_resolution(){
	return resolution;
}

/*******************************************************************************
* @ int rc_num_dsm_channels()
* 
* returns the number of ds2 channels currently being sent
* returns 0 if no packets have been received yet.
*******************************************************************************/
int rc_num_dsm_channels(){
	return num_channels;
}

/*******************************************************************************
* @ int rc_is_dsm_active()
* 
* returns 1 if packets are arriving in good health without timeouts.
* returns 0 otherwise.
*******************************************************************************/
int rc_is_dsm_active(){
	return rc_is_dsm_active_flag;
}

/*******************************************************************************
* @ void* serial_parser(void *ptr)
* 
* This is a local function that is started as a background thread by 
* rc_initialize_dsm(). This monitors the serial port and interprets data
* for each packet, it determines 10 or 11 bit resolution
* Radios with more than 7 channels split data across multiple packets. Thus, 
* new data is not committed until a full set of channel data is received.
*******************************************************************************/
void* serial_parser( __unused void *ptr){
	char buf[DSM_PACKET_SIZE];
	int i, ret, available;
	int new_values[MAX_DSM_CHANNELS]; // hold new values before committing
	int detection_packets_left; // use first 4 packets just for detection
	unsigned char ch_id;
	int16_t value;
	int is_complete;
	unsigned char max_channel_id_1024 = 0; // max channel assuming 1024 decoding
	unsigned char max_channel_id_2048 = 0; // max channel assuming 2048 decoding
	char channels_detected_1024[MAX_DSM_CHANNELS];
	char channels_detected_2048[MAX_DSM_CHANNELS];
	memset(channels_detected_1024,0,MAX_DSM_CHANNELS);
	memset(channels_detected_2048,0,MAX_DSM_CHANNELS);

	/***************************************************************************
	* First packets that come in are read just to detect resolution and channels
	* start assuming 1024 bit resolution, if any of the first 4 packets appear
	* to break 1024 mode then swap to 2048
	***************************************************************************/
DETECTION_START:
	rc_uart_flush(DSM_UART_BUS); // flush first
	detection_packets_left = 4;
	while(detection_packets_left>0 && running && rc_get_state()!=EXITING){
		rc_usleep(5000);
		available = rc_uart_bytes_available(DSM_UART_BUS);

		// nothing yet, go back to sleep
		if(available == 0) continue;
		// halfway through packet, sleep for 1ms
		if(available <  DSM_PACKET_SIZE){
			rc_usleep(1000);
			available = rc_uart_bytes_available(DSM_UART_BUS);
		}
		// read or flush depending on bytes available		
		if(available == DSM_PACKET_SIZE){
			ret = rc_uart_read_bytes(DSM_UART_BUS, DSM_PACKET_SIZE, buf);
			if(ret!=DSM_PACKET_SIZE){
				#ifdef DEBUG
					printf("WARNING: read the wrong number of bytes: %d\n", ret);
				#endif
				rc_uart_flush(DSM_UART_BUS); // flush
				continue;
			}
		}
		else{
			// got out of sync or read nonsense, flush and try again
			rc_uart_flush(DSM_UART_BUS); // flush
			continue;
		}

		// first check each channel id assuming 1024/22ms mode
		// where the channel id lives in 0b01111000 mask
		// if one doesn't make sense, must be using 2048/11ms mode
		#ifdef DEBUG
			printf("1024-mode: ");
		#endif
		for(i=1;i<8;i++){
			// last few words in buffer are often all 1's, ignore those
			if((buf[2*i]!=0xFF) || (buf[(2*i)+1]!=0xFF)){
				// grab channel id from first byte assuming 1024 mode
				ch_id = (buf[i*2]&0b01111100)>>2; 
				if(ch_id>max_channel_id_1024){
					max_channel_id_1024 = ch_id;
				} 
				if(ch_id<MAX_DSM_CHANNELS){
					channels_detected_1024[ch_id] = 1;
				} 
				#ifdef DEBUG
					printf("%d ", ch_id);
				#endif
			}
		}

		#ifdef DEBUG
			printf("   2048-mode: ");
		#endif
		
		for(i=1;i<8;i++){
			// last few words in buffer are often all 1's, ignore those
			if((buf[2*i]!=0xFF) || (buf[(2*i)+1]!=0xFF)){
				// now grab assuming 2048 mode
				ch_id = (buf[i*2]&0b01111000)>>3; 
				if(ch_id>max_channel_id_2048){
					 max_channel_id_2048 = ch_id;
				}
				if(ch_id<MAX_DSM_CHANNELS){
					channels_detected_2048[ch_id] = 1;
				}
				#ifdef DEBUG
					printf("%d ", ch_id);
				#endif
			}
		}

		// raw debug mode spits out all ones and zeros
		#ifdef DEBUG
		printf("ret=%d : ", ret);
		for(i=0; i<(DSM_PACKET_SIZE/2); i++){
			printf(rc_byte_to_binary(buf[2*i]));
			printf(" ");
			printf(rc_byte_to_binary(buf[(2*i)+1]));
			printf("   ");
		}
		printf("\n");
		#endif


		detection_packets_left --;
	}

	// do an exit check here since there is a jump above this code
	if(!running || rc_get_state()==EXITING) return 0;

	/***************************************************************************
	* now determine which mode from detection data
	***************************************************************************/
	if(max_channel_id_1024 >= MAX_DSM_CHANNELS){
		// probbaly 2048 if 1024 was invalid
		resolution = 2048;

		// still do some checks
		if(max_channel_id_2048 >= MAX_DSM_CHANNELS){
			printf("WARNING: too many DSM channels detected, trying again\n");
			goto DETECTION_START;
		}
		else num_channels = max_channel_id_2048+1;
		// now make sure every channel was actually detected up to max
		for(i=0;i<num_channels;i++){
			if(channels_detected_2048[i]==0){
				printf("WARNING: Missing DSM channel, trying again\n");
				goto DETECTION_START;
			}
		}
	}
	// if not 2048, must be 1024 but do some checks anyway
	else{
		resolution = 1024;

		// still do some checks
		if(max_channel_id_1024 >= MAX_DSM_CHANNELS){
			printf("WARNING: too many DSM channels detected, trying again\n");
			goto DETECTION_START;
		}
		else num_channels = max_channel_id_1024+1;
		// now make sure every channel was actually detected up to max
		for(i=0;i<num_channels;i++){
			if(channels_detected_1024[i]==0){
				printf("WARNING: Missing DSM channel, trying again\n");
				goto DETECTION_START;
			}
		}
	}

	/***************************************************************************
	* normal operation loop
	***************************************************************************/
START_NORMAL_LOOP:
	while(running && rc_get_state()!=EXITING){
		rc_usleep(5000);
		available = rc_uart_bytes_available(DSM_UART_BUS);

		// nothing yet, go back to sleep
		if(available == 0) continue;
		// halfway through packet, sleep for 1ms
		if(available <  DSM_PACKET_SIZE){
			rc_usleep(1000);
			available = rc_uart_bytes_available(DSM_UART_BUS);
		}

		#ifdef DEBUG
			printf("bytes available: %d\n", available);
		#endif

		// read or flush depending on bytes available		
		if(available == DSM_PACKET_SIZE){
			ret = rc_uart_read_bytes(DSM_UART_BUS, DSM_PACKET_SIZE, buf);
			if(ret!=DSM_PACKET_SIZE){
				#ifdef DEBUG
					printf("WARNING: read the wrong number of bytes: %d\n", ret);
				#endif
				rc_is_dsm_active_flag=0;
				rc_uart_flush(DSM_UART_BUS); // flush
				continue;
			}
		}
		else{
			// got out of sync, flush and try again
			rc_uart_flush(DSM_UART_BUS); // flush
			continue;
		}

		// raw debug mode spits out all ones and zeros
		#ifdef DEBUG
		printf("ret=%d : ", ret);
		for(i=0; i<(DSM_PACKET_SIZE/2); i++){
			printf(rc_byte_to_binary(buf[2*i]));
			printf(" ");
			printf(rc_byte_to_binary(buf[(2*i)+1]));
			printf("   ");
		}
		printf("\n");
		#endif

		
		// okay, must have a full packet now	
		// packet is 16 bytes, 8 words long
		// first word doesn't have channel data, so iterate through last 7 words
		for(i=1;i<=7;i++){
			// unused words are 0xFF
			// skip if one of them
			if(buf[2*i]!=0xFF || buf[(2*i)+1]!=0xFF){
				// grab channel id from first byte
				// and value from both bytes
				if(resolution == 1024){
					ch_id = (buf[i*2]&0b01111100)>>2; 
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000011)<<8) + buf[(2*i)+1];
					value += 989; // shift range so 1500 is neutral
				}
				else if(resolution == 2048){
					ch_id = (buf[i*2]&0b01111000)>>3; 
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000111)<<8) + buf[(2*i)+1];
					// extra bit of precision means scale is off by factor of 
					// two, also add 989 to center channels around 1500
					value = (value/2) + 989; 
				}
				else{
					printf("ERROR: dsm resolution incorrect\n");
					return NULL;
				}
				
				#ifdef DEBUG
				printf("%d %d  ",ch_id,value);
				#endif
				
				if((ch_id+1)>MAX_DSM_CHANNELS){
					#ifdef DEBUG
					printf("error: bad channel id\n");
					#endif
					goto START_NORMAL_LOOP;
				}
				// record new value
				new_values[ch_id] = value;
			}
		}

		// check if a complete set of channel data has been received
		// otherwise wait for another packet with more data
		is_complete = 1;
		for(i=0;i<num_channels;i++){
			if (new_values[i]==0){
				is_complete=0;
				#ifdef DEBUG
				printf("waiting for rest of data in next packet\n");
				#endif
				break;
			}
		}
		if(is_complete){
			#ifdef DEBUG
			printf("all data complete now\n");
			#endif
			new_dsm_flag=1;
			rc_is_dsm_active_flag=1;
			last_time = rc_nanos_since_epoch();
			for(i=0;i<num_channels;i++){
				rc_channels[i]=new_values[i];
				new_values[i]=0;// put local values array back to 0
			}
			// run the dsm ready function.
			// this is null unless user changed it
			dsm_ready_func();
		}
		
		#ifdef DEBUG
		printf("\n");
		#endif
	}
	return NULL;
}

/*******************************************************************************
* int rc_bind_dsm()
*
* the user doesn't need to call this function. Just use the rc_bind_dsm example
* program instead.
*
* DSM satellite receivers are put into bind mode by sending them a sequence of
* pulses right after it receives power and starts up. This program puts the 
* normally UART signal pin into GPIO pulldown mode temporarily, detects when the 
* user unplugs and plugs back in the receiver, then sends the binding pulses. 
*
* the number of pulses dictates the mode the satellite receiver will request
* the transmitter to use. The transmitter may bind but use a different mode.
* I suggest configuring your radio to use DSMX 11ms fast mode if it allows that.
*
* 2048 & 1024 indicates 10 or 11 bit resolution.
* 11ms & 22ms indicates the time period between the transmitter sending frames.
* 11ms is required for transmitters with 8 or more channels.
* 
* Testing done with DX7s, DX6i, DX8, and Orange T-SIX
* 
* Table of Bind Modes
*  pulses      mode        
*   3      dsm 1024/22ms 
*   5  	dsm 2048/11ms
*   7  	DSMX 1024/22ms: 
*   9  	DSMx 2048/11ms: 
*******************************************************************************/

int rc_bind_dsm(){
	int value;
	int i;
	char c = 0; // for reading user input
	// default to dsmx 11ms mode for most applications
	int pulses = 9; 
	int delay = 200000;
	
	if(initialize_mmap_gpio()<0){
		printf("ERROR: can't proceed without mmap \n");
		return -1;
	}

	// first set the pin as input (pulldown) to detect when receiver is attached
	if(rc_set_pinmux_mode(DSM_PIN, PINMUX_GPIO_PD)<0){
		printf("ERROR: pinmux helper not enabled for P9_11\n");
		return -1;
	}

	//export GPIO pin to userspace
	if(rc_gpio_export(DSM_PIN)){
		printf("error exporting gpio pin\n");
		return -1;
	}
	rc_gpio_set_dir(DSM_PIN, INPUT_PIN);
	
	// give user instructions
	printf("\n\nYou must choose which DSM mode to request from your transmitter\n");
	printf("Note that your transmitter may actually bind in a different mode\n");
	printf("depending on how it is configured.\n");
	printf("We suggest option 1 for 6-channel dsm radios,\n");
	printf("and option 4 for 7-9 channel DSMX radios\n");
	printf("\n");
	printf("1: Spektrum  DSM2 10-bit 22ms framerate\n");
	printf("2: Spektrum  DSM2 11-bit 11ms framerate\n"); 
	printf("3: Spektrum  DSMX 10-bit 22ms framerate\n"); 
	printf("4: Spektrum  DSMX 11-bit 11ms framerate\n"); 
	printf("5: Orange/JR DSM2 10-bit 22ms framerate\n");
	printf("\n"); 
	printf("Enter mode 1-5: ");
	
	// wait for user input
enter:
	c = getchar();
 
	switch(c){
		case '1':
			pulses = 3;
			break;
		case '2':
			pulses = 5;
			break;
		case '3':
			pulses = 7;
			break;
		case '4':
			pulses = 9;
			break;
		case '5':
			pulses = 9;
			delay = 50000;
			break;
		case '\n':
			goto enter;
			break;
		default:
			printf("incorrect mode number\n");
			getchar();
			goto enter;
			break;
	}
		
    printf("Using mode %c\n", c);

	// wait for user to hit enter before continuing
	printf("\nDisconnect your dsm satellite receiver if it is still connected\n");
	printf("Plug it into the cape quickly and firmly to begin binding.\n");
	
	// wait for the receiver to be disconnected
	value = 1;
	while(value==1){ //pin will go low when disconnected
		value=rc_gpio_get_value(DSM_PIN);
		rc_usleep(500);
	}
	rc_usleep(100000);
	
	//wait for the receiver to be plugged in
	//receiver will pull pin up when connected
	while(value==0){ 
		value=rc_gpio_get_value(DSM_PIN);
		rc_usleep(500);
	}
	
	// now set pin as output
	rc_gpio_set_dir(DSM_PIN, OUTPUT_PIN);
	rc_gpio_set_value_mmap(DSM_PIN, HIGH);
	rc_set_pinmux_mode(DSM_PIN, PINMUX_GPIO);
	
	// wait as long as possible before sending pulses
	// in case the user plugs in the receiver slowly at an angle
	// which would delay the power pin from connecting 
	rc_usleep(delay); 
	
	for(i=0; i<pulses; i++){
		rc_gpio_set_value_mmap(DSM_PIN, LOW);
		rc_usleep(PAUSE);
		rc_gpio_set_value_mmap(DSM_PIN, HIGH);
		rc_usleep(PAUSE);
	}
	
	rc_usleep(1000000);

	// swap pinmux from GPIO back to uart
	rc_set_pinmux_mode(DSM_PIN, PINMUX_UART);
	
	// all done
	printf("\n\n\nYour receiver should now be blinking. If not try again.\n");
	printf("Now turn on your transmitter in bind mode.\n");
	printf("Use test_dsm to confirm functionality.\n\n");
	
	return 0;
}

/*******************************************************************************
* int load_default_calibration()
*
* internal function, writes default values to the config file. Generally only
* used once the first time rc_initialize_dsm is called after a clean install
*******************************************************************************/
int load_default_calibration(){
	int i;
	for(i=0;i<MAX_DSM_CHANNELS;i++){
		rc_mins[i]=DEFAULT_MIN;
		rc_maxes[i]=DEFAULT_MAX;
	}
	return 0;
}

/*******************************************************************************
* void *calibration_listen_func(void *ptr)
*
* this is started as a background thread by rc_calibrate_dsm_routine(). 
* Only used during calibration to monitor data as it comes in.
*******************************************************************************/
void *calibration_listen_func(void *ptr){
	//wait for data to start
	printf("waiting for dsm connection");
	rc_get_dsm_ch_raw(1); // flush the data ready flag with a read
	while(!rc_is_new_dsm_data()){
		if(rc_get_state()==EXITING || listening==0){
			return 0;
		}
		rc_usleep(5000); 
	}
	
	//start limits at first value
	int j;
	for(j=0;j<MAX_DSM_CHANNELS;j++){
		rc_mins[j]=rc_get_dsm_ch_raw(j+1);
		rc_maxes[j]=rc_get_dsm_ch_raw(j+1);
	}
	
	// record limits until user presses enter
	while(listening && rc_get_state()!=EXITING){
		printf("\r");
		if (rc_is_new_dsm_data()){
			for(j=0;j<MAX_DSM_CHANNELS;j++){
				if(rc_get_dsm_ch_raw(j+1) > 0){ //record only non-zero channels
					if(rc_get_dsm_ch_raw(j+1)>rc_maxes[j]){
						rc_maxes[j] = rc_get_dsm_ch_raw(j+1);
					}
					else if(rc_get_dsm_ch_raw(j+1)<rc_mins[j]){
						rc_mins[j] = rc_get_dsm_ch_raw(j+1);
					}
					printf("%d:%d ",j+1,rc_get_dsm_ch_raw(j+1));
				}
			}
			fflush(stdout);
		}
		rc_usleep(10000); 
	}
	(void)ptr; // shut up gcc
	return 0;
}

/*******************************************************************************
* int rc_calibrate_dsm_routine()
*
* routine for measuring the min and max values from a transmitter on each
* channel and save to disk for future use.
* If a channel isn't used by the transmitter then default values are saved.
* if the user forgot to move one of the channels during the calibration process
* then defualt values are also saved.
*******************************************************************************/
int rc_calibrate_dsm_routine(){
	int i,ret;
	
	dsm_frame_rate = 0; // zero until mode is detected on first packet
	running = 1; // lets uarts 4 thread know it can run
	num_channels = 0;
	last_time = 0;
	rc_is_dsm_active_flag = 0;
	rc_set_dsm_data_func(&rc_null_func);
	
	if(rc_uart_init(DSM_UART_BUS, DSM_BAUD_RATE, 0.1)){
		printf("Error, failed to initialize UART%d for dsm\n", DSM_UART_BUS);
	}
	
	pthread_create(&serial_parser_thread, NULL, serial_parser, (void*) NULL);
		
	// display instructions
	printf("\nRaw dsm data should display below if the transmitter and\n");
	printf("receiver are paired and working. Move all channels through\n");
	printf("their range of motion and the minimum and maximum values will\n");
	printf("be recorded. When you are finished moving all channels,\n");
	printf("press ENTER to save the data or any other key to abort.\n\n");		
	
	// start listening
	listening = 1;
	pthread_t  listening_thread;
	pthread_create(&listening_thread, NULL, calibration_listen_func, (void*) NULL);
	
	// wait for user to hit enter
	ret = rc_continue_or_quit();
	
	//stop listening
	listening=0;
	rc_stop_dsm_service();
	pthread_join(listening_thread, NULL);
	
	
	// abort if user hit something other than enter
	if(ret<0){
		printf("aborting calibrate_dsm routine\n");
		return -1;
	}
	
	//if it looks like new data came in, write calibration file
	if((rc_mins[0]==0) || (rc_mins[0]==rc_maxes[0])){ 
		printf("no new data recieved, exiting\n");
		return -1;
	}
	
	// construct a new file path string
	char file_path[100];
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, DSM_CAL_FILE);
	
	// open for writing
	FILE* cal;
	cal = fopen(file_path, "w");
	if (cal == NULL) {
		mkdir(CONFIG_DIRECTORY, 0777);
		cal = fopen(file_path, "w");
		if (cal == 0){
			printf("could not open config directory\n");
			printf(CONFIG_DIRECTORY);
			printf("\n");
			return -1;
		}
	}

	// if new data was captures for a channel, write data to cal file
	// otherwise fill in defaults for unused channels in case
	// a higher channel radio is used in the future with this cal file
	for(i=0;i<MAX_DSM_CHANNELS;i++){
		if((rc_mins[i]==0) || (rc_mins[i]==rc_maxes[i])){
			fprintf(cal, "%d %d\n",DEFAULT_MIN, DEFAULT_MAX);
		}
		else{
			fprintf(cal, "%d %d\n", rc_mins[i], rc_maxes[i]);
		}
	}
	fclose(cal);
	printf("New calibration file written\n");
	printf("use rc_test_dsm to confirm\n");
	return 0;
}


