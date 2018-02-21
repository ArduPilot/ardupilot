#include "RCOutput_CANZero.h"

#include "ifaddrs.h"
#include "stdint.h"
#include "unistd.h"
#include "net/if.h"
#include "stdio.h"
#include "time.h"
#include "sys/poll.h"
//#include "dirent.h"
#include "string"
#include "linux/can/raw.h"
#include "AP_Math/AP_Math.h"
#include "AP_Param/AP_Param.h"
#include "AP_BoardConfig/AP_BoardConfig.h"
#include "AP_Vehicle/AP_Vehicle_Type.h"

//extern const AP_HAL::HAL& hal;

//
//#define CAN_SYNC_MSG "080#00" // Message to send during initialization to get the node IDs of all available CAN devices.

#define CAN_SYNC_ID (0x080)
#define CAN_SYNC_DATA_TYPE uint8_t
#define CAN_SYNC_DATA (0x00)
#define CAN_SET_RPM_ID (0x600)
#define CAN_SET_RPM_DATA_TYPE int32_t
#define CAN_SET_RPM_META (0x23FF6000)//(0x60FF) 001 0 00 1 1 1111111101100000 00000000
#define CAN_SET_RPMPS_ID (0x600)
#define CAN_SET_RPMPS_DATA_TYPE uint32_t
#define CAN_SET_RPMPS_META (0x23836000)//(0x6083)
#define CAN_SET_CTL_ID (0x600)
#define CAN_SET_CTL_DATA_TYPE uint16_t
#define CAN_SET_CTL_META (0x23406000)//(0x6040) 00100011010000000110000000000000
//#define CAN_SET_CTL_ON_DATA (0x0003)
#define CAN_SET_CTL_OFF_DATA (0x0000)

#define CAN_GET_RPM_REQ_ID (0x600)
#define CAN_GET_RPM_RESP_ID (0x580)
#define CAN_GET_RPM_DATA_TYPE int32_t
#define CAN_GET_RPM_META (0x406C6000)//(0x606C)
#define CAN_GET_MAX_RPMPS_REQ_ID (0x600)
#define CAN_GET_MAX_RPMPS_RESP_ID (0x580)
#define CAN_GET_MAX_RPMPS_DATA_TYPE float
#define CAN_GET_MAX_RPMPS_META (4001200F)//(0x2001.F)

#define CAN_SET_MSG_LEN(L) (((uint32_t)(4-L))<<26) // | with META to get correct metadata.
#define CAN_GET_MSG_LEN(M) (4-(((uint32_t)M)>>26))
//

//#define CANID_DELIM '#'
//#define DATA_SEPERATOR '.'
//#define CAN_MAX_DLC 8
//#define CAN_MAX_DLEN 8
#define CAN_MTU (sizeof(struct can_frame))
//#define CANFD_MAX_DLEN 64
//#define CANFD_MTU (sizeof(struct canfd_frame))

//#define SOL_CAN_BASE 100
//#define SOL_CAN_RAW (SOL_CAN_BASE + CAN_RAW)

#define SCAN_TIMEOUT_TOTAL ((float)3.0) //sec
#define SCAN_TIMEOUT_MSG ((int)200) //ms
#define CAN_RECV_TIMEOUT_TOTAL ((float)0.01) //sec
#define CAN_RECV_TIMEOUT_MSG ((int)1) //ms

#define PWM_CHIP_PATH "/sys/class/pwm/"
#define PWM_CHIP_BASE_NAME "pwmchip"

uint16_t previous_ppm[4] = {0,0,0,0};

namespace Linux {

	RCOutput_CANZero::RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_ch_count, uint8_t can_ch_count)
	{
		// Initialize sysfs to use PWM.
		sysfs_out = new RCOutput_Sysfs(pwm_chip, pwm_channel_base, pwm_ch_count);
		this->pwm_channel_count_max = pwm_ch_count;
		this->can_channel_count_max = can_ch_count;
		this->channel_count_max = pwm_ch_count+can_ch_count;
		this->ch_inf = (ChannelInfo*)calloc(channel_count_max, sizeof(ChannelInfo));
		this->_pending = (uint16_t*)calloc(channel_count_max, sizeof(uint16_t));
	}

	RCOutput_CANZero::~RCOutput_CANZero()
	{
		if(can_socket != 0) close(can_socket);
		if(ch_inf != NULL) free(ch_inf);
		if(_pending != NULL) free(_pending);
		if(sysfs_out != NULL) (*sysfs_out).~RCOutput_Sysfs();
	}

	void RCOutput_CANZero::init()
	{
		if(AP_Param::initialised()){
			AP_Param::load_all(false);
			//printf("Parameters initialized.\n");
			AP_BoardConfig* BoardParams = (AP_BoardConfig*)AP_Param::find_object("BRD_");
			ctl_on = (uint16_t)BoardParams->CANZero_CTL;
			max_rpm = (int32_t)BoardParams->CANZero_SPD;
			rpmps = (int32_t)BoardParams->CANZero_ACC;
			//printf("max_rpm: %d\n", (int32_t)BoardParams->CANZero_SPD);
		}

		struct ifaddrs if_list;
		struct ifaddrs* if_list_ptr = &if_list;
		getifaddrs(&if_list_ptr); // Get list of available interfaces.

		// Searching for a CAN interface.
		struct ifaddrs* ifa_current = if_list_ptr;
		while(ifa_current != NULL){
			char* pos = strstr(ifa_current->ifa_name, "can");
			if(pos != NULL && strlen(ifa_current->ifa_name) < 7){
				strncpy(can_ifa_name, ifa_current->ifa_name, 8);
				break;
			}
			ifa_current = ifa_current->ifa_next;
		}

		// Creating a socket connection and binding it to the available interface.
		//printf("Connecting to socket and binding to interface \"%s\".\n", ifa_current->ifa_name);
		can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		can_addr_output.can_family = AF_CAN;
		can_addr_output.can_ifindex = if_nametoindex(can_ifa_name);
		bind(can_socket, (struct sockaddr *)&can_addr_output, sizeof(can_addr_output));

		std::map<uint8_t, uint8_t> ids;
		scan_devices(&ids);

		// Discard any frames received, so that the buffer is empty when a response to a specific request is expected.
		set_default_filter();
		clear_socket_buffer(can_socket);

		//printf("Discovered CAN devices:");
		std::pair<int,std::map<uint8_t,uint8_t>::iterator> it(0,ids.begin());
		for(; it.first < can_channel_count_max && it.second != ids.end();
				it.first++, it.second++){
			ch_inf[it.first].hw_chan = it.second->first; // Copying ID of the device to the channel info.
			ch_inf[it.first].can = 1; // Marking the channel as a CAN channel.
			//printf(" [%x|%x]", ch_inf[it.first].hw_chan, ch_inf[it.first].can);
		}
		can_channel_count = it.first;
		//printf("\n");

		sysfs_out->init();

		for(int i = 0; i < pwm_channel_count_max; i++){
			ch_inf[i+can_channel_count].hw_chan = i;
			ch_inf[i+can_channel_count].can = 0;
		}
		pwm_channel_count = pwm_channel_count_max;
		channel_count = can_channel_count + pwm_channel_count;

		set_acceleration_all(rpmps);

		freeifaddrs(if_list_ptr);
	}

	void RCOutput_CANZero::set_freq(uint32_t chmask, uint16_t freq_hz)
	{
	    for (uint8_t i = 0; i < channel_count; i++) {
	        if (chmask & 1 << i) {
	        	ch_inf[i].freq_hz = freq_hz;
	        	if(ch_inf[i].can){
	        		// Do nothing.
	        	}else{
	        		sysfs_out->set_freq(chmask >> can_channel_count, freq_hz);
	        	}
	        }
	    }
	}

	uint16_t RCOutput_CANZero::get_freq(uint8_t ch)
	{
		if(ch >= channel_count){
			return 0;
		}
		if(ch_inf[ch].can){
			// Do nothing.
		}else{
			ch_inf[ch].freq_hz = sysfs_out->get_freq(ch_inf[ch].hw_chan);
		}
		return ch_inf[ch].freq_hz;
	}

	void RCOutput_CANZero::enable_ch(uint8_t ch)
	{
		if(ch >= channel_count){
			return;
		}
		if(ch_inf[ch].can){
			//printf("Enabling channel %d.\n", ch_inf[ch].hw_chan);
			can_frame frame_output;
			generate_frame<CAN_SET_CTL_DATA_TYPE>(&frame_output, CAN_SET_CTL_ID, ch_inf[ch].hw_chan, CAN_SET_CTL_META, ctl_on);
			::write(can_socket, &frame_output, CAN_MTU);
		}else{
			sysfs_out->enable_ch(ch_inf[ch].hw_chan);
		}
	}

	void RCOutput_CANZero::disable_ch(uint8_t ch)
	{
		if(ch >= channel_count){
			return;
		}
		if(ch_inf[ch].can){
			can_frame frame_output;
			generate_frame<CAN_SET_CTL_DATA_TYPE>(&frame_output, CAN_SET_CTL_ID, ch_inf[ch].hw_chan, CAN_SET_CTL_META, CAN_SET_CTL_OFF_DATA);
			::write(can_socket, &frame_output, CAN_MTU);
		}else{
			sysfs_out->disable_ch(ch_inf[ch].hw_chan);
		}
	}

	void RCOutput_CANZero::write(uint8_t ch, uint16_t period_us)
	{
		if(ch >= channel_count){
			return;
		}
		//printf("ppm: %d\nmax rpm: %d\n_corked: %d\n", period_us, max_rpm, _corked);
		//if(!_corked) printf("not corked\n");
		if(_corked){
	        _pending[ch] = period_us;
	        _pending_mask |= (1U<<ch);
		}else{
			_write(ch, period_us);
		}
	}

	void RCOutput_CANZero::_write(uint8_t ch, uint16_t period_us)
	{
		ch_inf[ch].ppm = period_us;
		if(ch_inf[ch].can){
			CAN_SET_RPM_DATA_TYPE rpm = ppm_to_rpm<CAN_SET_RPM_DATA_TYPE>(period_us);

//			if(ch <= 3 && previous_ppm[ch] != period_us){
//				printf("Channel: %02X\nReceived ppm: %d\nCorresponding rpm: %d\n", ch, period_us, rpm);
//			}

			// Ignore negative rpm when using copter.
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
			if(rpm < 0){
				rpm = 0;
			}
#endif

			can_frame frame_output;
			generate_frame<CAN_SET_RPM_DATA_TYPE>(&frame_output, CAN_SET_RPM_ID, ch_inf[ch].hw_chan, CAN_SET_RPM_META, rpm);

//			if(ch <= 3 && previous_ppm[ch] != period_us){
//				printf("CAN frame data: %02X %02X %02X %02X\n", frame_output.data[4], frame_output.data[5], frame_output.data[6], frame_output.data[7]);
//				previous_ppm[ch] = period_us;
//			}

			::write(can_socket, &frame_output, CAN_MTU);
		}else{
			sysfs_out->write(ch_inf[ch].hw_chan, period_us);
		}
	}

	uint16_t RCOutput_CANZero::read(uint8_t ch)
	{
		if(ch >= channel_count){
			return 1000;
		}
		uint16_t ppm;
		if(ch_inf[ch].can){
//			can_frame frame_output;
//			can_frame frame_input;
//			generate_frame(&frame_output, CAN_GET_RPM_REQ_ID, (uint16_t)ch_inf[ch].hw_chan, CAN_GET_RPM_META, (uint32_t)0);
//			recv_filtered(&frame_output, &frame_input, CAN_GET_RPM_RESP_ID, (uint16_t)ch_inf[ch].hw_chan, CAN_GET_RPM_META);
//			CAN_GET_RPM_DATA_TYPE rpm;
//			data_array_to_var(frame_input.data+4, &rpm);
//			ppm = rpm_to_ppm(rpm);
			ppm = ch_inf[ch].ppm;
		}else{
			ppm = sysfs_out->read(ch_inf[ch].hw_chan);
		}
		return ppm;
	}

	void RCOutput_CANZero::read(uint16_t *period_us, uint8_t len)
	{
	    for (int i = 0; i < MIN(len, channel_count); i++) {
	        period_us[i] = read(i);
	    }
	    for (int i = channel_count; i < len; i++) {
	        period_us[i] = 1000;
	    }
	}

	void RCOutput_CANZero::cork(void)
	{
		//printf("cork()\n");
		_corked = true;
		sysfs_out->cork();
	}

	void RCOutput_CANZero::push(void)
	{
	    if (!_corked) {
	        return;
	    }
	    //printf("push()\n");
	    for (uint8_t i=0; i<channel_count; i++) {
	        if (((1U<<i) & _pending_mask)/* && ch_inf[i].can*/) {
				_write(i, _pending[i]);
	        }
	    }
	    sysfs_out->push();
	    _pending_mask = 0;
	    _corked = false;
	}

	template<typename T> void RCOutput_CANZero::set_acceleration(uint8_t ch, T acc)
	{
		if(ch_inf[ch].can){
			can_frame frame_output;
			generate_frame<CAN_SET_RPMPS_DATA_TYPE>(&frame_output, CAN_SET_RPMPS_ID, ch_inf[ch].hw_chan, CAN_SET_RPMPS_META, acc);
			::write(can_socket, &frame_output, CAN_MTU);
		}
	}

	template<typename T> void RCOutput_CANZero::set_acceleration_all(T acc)
	{
		for(uint8_t ch = 0; ch < channel_count; ch++){
			if(ch_inf[ch].can){
				can_frame frame_output;
				generate_frame<CAN_SET_RPMPS_DATA_TYPE>(&frame_output, CAN_SET_RPMPS_ID, ch_inf[ch].hw_chan, CAN_SET_RPMPS_META, acc);
				::write(can_socket, &frame_output, CAN_MTU);
			}
		}
	}

	// Returns 0 if the single message timeout was reached (likely no more devices online in the network or they reply very slowly).
	// Returns 1 if the total timeout was reached (possibly more devices available and/or the network is being flooded by some nodes).
	int RCOutput_CANZero::scan_devices(std::map<uint8_t, uint8_t> *ids)
	{
		int ret = 1;
		unsigned int addr_len = sizeof(struct sockaddr_can);
		//struct sockaddr_can can_addr_output;
		//struct sockaddr_can can_addr_input;
		struct can_frame frame_output;
		struct can_frame frame_input;
		generate_frame<CAN_SYNC_DATA_TYPE>(&frame_output, CAN_SYNC_ID, 0, 0, CAN_SYNC_DATA, 1);

		::write(can_socket, &frame_output, CAN_MTU);

		struct pollfd fds = {can_socket, POLLIN, 0};
		struct timespec tps = {0, 0}; // Start time.
		float start_time = 0; // Start time in s.
		struct timespec tpe = {0, 0}; // Current time/end time.
		float dt = 0; // Time difference in s.
		const float nsps = 1000000000; // nanoseconds per second

		clock_gettime(CLOCK_MONOTONIC, &tps);
		start_time = (float(tps.tv_nsec)/nsps + tps.tv_sec);
		while(dt < SCAN_TIMEOUT_TOTAL){
			if(0 < poll(&fds, 1, SCAN_TIMEOUT_MSG)){
				::recvfrom(can_socket, &frame_input, sizeof(struct can_frame), 0, (struct sockaddr*)&can_addr_input, &addr_len);
				(*ids)[frame_input.can_id & 0x7F] = frame_input.can_id & 0x7F; // Creates an entry for the given ID if not already present.
			}else{
				ret = 0;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC, &tpe);
			dt = float((float(tpe.tv_nsec)/nsps + tpe.tv_sec) - start_time);
		}

		return ret;
	}

	template<typename T> void RCOutput_CANZero::generate_frame(can_frame *frame, uint16_t base_id, uint16_t node_id, uint32_t meta, T value, uint8_t ignore_meta)
	{
		frame->can_id = base_id | node_id;

		if(!ignore_meta){
			frame->can_dlc = sizeof(meta) + sizeof(value);
		}else{
			frame->can_dlc = sizeof(value);
		}

		int pos = 0;
		if(!ignore_meta){
			meta |= CAN_SET_MSG_LEN(sizeof(value));
			for(; pos < sizeof(meta); pos++){
				frame->data[pos] = (meta >> (8*(sizeof(meta)-1-pos))) & 0xFF;
			}
		}

		for(int i = pos; i < frame->can_dlc; i++){
			frame->data[i] = (value >> (8*(i-pos))) & 0xFF;
		}

		// *** Send a complete 8 byte frame.
		for(int i = frame->can_dlc; i < 8; i++){
			frame->data[i] = 0x00;
		}

		frame->can_dlc = 8;
		// ***
	}

	template<typename T> T RCOutput_CANZero::ppm_to_rpm(uint16_t pulse_width)
	{
		T rpm = ((float(max_rpm))/500)*(float(pulse_width)-1500);
		//printf("ppm: %d\nrpm: %d\nmax rpm: %d\n", pulse_width, rpm, max_rpm);
		return rpm;//((float(max_rpm))/500)*(float(pulse_width)-1500);
	}

	template<typename T> uint16_t RCOutput_CANZero::rpm_to_ppm(T rpm)
	{
		return (float(rpm)/float(max_rpm))*500+1500;
	}

	template<typename T> void RCOutput_CANZero::data_array_to_var(uint8_t *data, T *var){
		*var = 0;
		for(int i = 0; i < sizeof(T); i++){
			//printf("%02x", data[i]);
			*var |= (T)((((uint64_t)data[i])&0xFF)<<i*8);
		}
		//printf("\n");
	}

	int RCOutput_CANZero::recv_filtered(can_frame *frame_output, can_frame *frame_input, uint16_t base_id, uint16_t node_id, uint32_t meta)
	{
		int ret = 0;
		struct pollfd fds = {can_socket, POLLIN, 0};
		struct timespec tps = {0, 0}; // Start time.
		float start_time = 0; // Start time in s.
		struct timespec tpe = {0, 0}; // Current time/end time.
		float dt = 0; // Time difference in s.
		const float nsps = 1000000000; // nanoseconds per second
		unsigned int addr_len = sizeof(struct sockaddr_can);
		//struct sockaddr_can can_addr_input;

		struct can_filter rfilter;
		rfilter.can_id = base_id | node_id;
		rfilter.can_mask = 0x07FF;

		setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

		::write(can_socket, frame_output, CAN_MTU);

		clock_gettime(CLOCK_MONOTONIC, &tps);
		start_time = (float(tps.tv_nsec)/nsps + tps.tv_sec);
		while(dt < CAN_RECV_TIMEOUT_TOTAL){
			if(0 < (ret = poll(&fds, 1, CAN_RECV_TIMEOUT_MSG))){
				::recvfrom(can_socket, frame_input, sizeof(struct can_frame), 0, (struct sockaddr*)&can_addr_input, &addr_len);
				printf("%x: %02x%02x%02x%02x %02x%02x%02x%02x\n", (uint32_t)frame_input->can_id, frame_input->data[0], frame_input->data[1], frame_input->data[2], frame_input->data[3], frame_input->data[4], frame_input->data[5], frame_input->data[6], frame_input->data[7]);
			}else{
				ret = 0;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC, &tpe);
			dt = float((float(tpe.tv_nsec)/nsps + tpe.tv_sec) - start_time);
		}

		// Setting the filter back to the default setting.
		set_default_filter();

		return ret;
	}

	void RCOutput_CANZero::clear_socket_buffer(int socket){
		struct pollfd fds = {socket, POLLIN, 0};
		struct can_frame frame_input;
		int data_available = poll(&fds, 1, 0);
		data_available = data_available<0?0:data_available; // Make sure errors are evaluated to false.
		while(data_available){
			::recv(socket, &frame_input, sizeof(frame_input), 0);
			data_available = poll(&fds, 1, 0);
			data_available = data_available<0?0:data_available; // Make sure errors are evaluated to false.
		}
	}

	void RCOutput_CANZero::set_default_filter(){
		setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
	}
}
