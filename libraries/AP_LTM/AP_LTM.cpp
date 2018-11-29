/* #################################################################################################################
 * LightTelemetry protocol (LTM)
 *
 * Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds). 
 *			   
 * Protocol details: 3 different frames, little endian.
 *   G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
 *    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0   
 *     $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
 *   A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
 *     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0   
 *      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
 *   S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
 *     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0     
 *      $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
 * ################################################################################################################# */

#include "AP_LTM.h"
//


extern const AP_HAL::HAL& hal;

// Constructord
AP_LTM::AP_LTM(const AP_AHRS &ahrs, const AP_BattMonitor &battery):
	_ahrs(ahrs),
    _battery(battery)
    {}

// init - perform require initialisation including detecting which protocol to use
void AP_LTM::init(const AP_SerialManager &serial_manager, const uint8_t mav_type){
	_uav.mav_type = mav_type;
    // check for LTM_Port
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LTM, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_LTM_BAUD, 
								AP_SERIALMANAGER_LTM_BUFSIZE_RX, 
								AP_SERIALMANAGER_LTM_BUFSIZE_TX);
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_LTM::tick, void));		
    }
}	
		
void AP_LTM::send_LTM_frame(uint8_t LTPacket[], uint8_t LTPacket_size){
    //calculate Checksum
    uint8_t LTCrc = 0x00;
    for (uint8_t i = 3; i < LTPacket_size-1; i++) LTCrc ^= LTPacket[i];
    LTPacket[LTPacket_size-1]=LTCrc;
    for (uint8_t i = 0; i<LTPacket_size; i++) _port->write(LTPacket[i]);
}

// GPS frame
void AP_LTM::send_Gframe(void){
	
	get_position();

    uint8_t LTBuff[LTM_GFRAME_SIZE];
    //protocol: START(2 bytes)FRAMEID(1byte)LAT(cm,4 bytes)LON(cm,4bytes)SPEED(m/s,1bytes)ALT(cm,4bytes)SATS(6bits)FIX(2bits)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x47; // G ( gps frame at 5hz )
    //PAYLOAD
    LTBuff[3]=(_uav.lat >> 8*0) & 0xFF;
    LTBuff[4]=(_uav.lat >> 8*1) & 0xFF;
    LTBuff[5]=(_uav.lat >> 8*2) & 0xFF;
    LTBuff[6]=(_uav.lat >> 8*3) & 0xFF;
    LTBuff[7]=(_uav.lon >> 8*0) & 0xFF;
    LTBuff[8]=(_uav.lon >> 8*1) & 0xFF;
    LTBuff[9]=(_uav.lon >> 8*2) & 0xFF;
    LTBuff[10]=(_uav.lon >> 8*3) & 0xFF;
    LTBuff[11]=(_uav.gndspeed >> 8*0) & 0xFF;
    LTBuff[12]=(_uav.alt >> 8*0) & 0xFF;
    LTBuff[13]=(_uav.alt >> 8*1) & 0xFF;
    LTBuff[14]=(_uav.alt >> 8*2) & 0xFF;
    LTBuff[15]=(_uav.alt >> 8*3) & 0xFF;
    LTBuff[16]= ((_uav.sats_visible << 2 )& 0xFF) | (_uav.fix_type & 0b00000011) ; // last 6 bits: sats number, first 2:fix type (0,1,2,3)
	send_LTM_frame(LTBuff, LTM_GFRAME_SIZE);
    _ltm_scheduler++;
}

// Sensors frame
void AP_LTM::send_Sframe(void){
	
    get_sense();
	
	uint8_t LTBuff[LTM_SFRAME_SIZE];
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x53; //S 
    //PAYLOAD
    LTBuff[3]=(_uav.volt >> 8*0) & 0xFF;                                                      // vbat converted in mv
    LTBuff[4]=(_uav.volt >> 8*1) & 0xFF;
    LTBuff[5]=(_uav.amp >> 8*0) & 0xFF;                                                    // actual current instead of consumed mah in regular LTM
    LTBuff[6]=(_uav.amp >> 8*1) & 0xFF;
    LTBuff[7]=(_uav.rssi >> 8*0) & 0xFF;                                                   
    LTBuff[8]=(_uav.airspeed >> 8*0) & 0xFF;                                              
    LTBuff[9]= ((_uav.flightmode << 2)& 0xFF ) | ((_uav.failsafe << 1)& 0b00000010 ) | (_uav.armstat & 0b00000001) ; // last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: Arm status.
    send_LTM_frame(LTBuff, LTM_SFRAME_SIZE);
    _ltm_scheduler++;
}

// Attitude frame
void AP_LTM::send_Aframe(void){
	
	get_attitude();
	
    uint8_t LTBuff[LTM_AFRAME_SIZE];   
    //A Frame: $T(2 bytes)A(1byte)PITCH(2 bytes)ROLL(2bytes)HEADING(2bytes)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x41; //A 
    //PAYLOAD
    LTBuff[3]=(_uav.pitch >> 8*0) & 0xFF;
    LTBuff[4]=(_uav.pitch >> 8*1) & 0xFF;
    LTBuff[5]=(_uav.roll >> 8*0) & 0xFF;
    LTBuff[6]=(_uav.roll >> 8*1) & 0xFF;
    LTBuff[7]=(_uav.heading >> 8*0) & 0xFF;
    LTBuff[8]=(_uav.heading >> 8*1) & 0xFF;
	send_LTM_frame(LTBuff, LTM_AFRAME_SIZE);
    _ltm_scheduler++;
}

// Send LTM
void AP_LTM::send_LTM(void){
        if (_ltm_scheduler & 1) {    // is odd
            send_Aframe();
        } else {					// is even
            if (_ltm_scheduler % 4 == 0) send_Sframe();
            else send_Gframe();
        }
        if (_ltm_scheduler > 9 ) _ltm_scheduler = 1;       
}

// get data for G frame
void AP_LTM::get_position(void){
    const AP_GPS &gps = AP::gps();
    Location loc;
	
	float alt;
	_ahrs.get_relative_position_D_home(alt);
		
    if(_ahrs.get_position(loc)) {
        _uav.lat = loc.lat;      
        _uav.lon = loc.lng;		
        _uav.gndspeed = (uint8_t)roundf(gps.ground_speed());
		_uav.alt = (int32_t)roundf(alt); 
		_uav.sats_visible = (uint8_t)AP::gps().num_sats(); 		
		_uav.fix_type = (uint8_t)AP::gps().status();		      
		                                 
	} else {
        _uav.lat = 0;      
        _uav.lon = 0;  
		_uav.gndspeed = 0;
		_uav.alt = (int32_t)roundf(alt);
		_uav.sats_visible = (uint8_t)AP::gps().num_sats(); 		
		_uav.fix_type = (uint8_t)AP::gps().status();		
    }
}

// get data for S frame
void AP_LTM::get_sense(void){ 
	
	const AP_Airspeed *aspeed = _ahrs.get_airspeed();
	AP_RSSI *ap_rssi = AP_RSSI::get_instance();
	      
    _uav.volt = (uint16_t)roundf(_battery.voltage() * 10.0f);
    _uav.amp = (uint16_t)roundf(_battery.current_amps() * 10.0f);
		
	// airspeed in m/s if available and enabled - even if not used - otherwise send 0
    if (aspeed && aspeed->enabled())  _uav.airspeed = (uint8_t)roundf(aspeed->get_airspeed() / 10.0f); 
    else _uav.airspeed = 0;
  
    _uav.flightmode = AP_Notify::flags.flight_mode;
       
    if (ap_rssi) {
        uint8_t rssiv = ap_rssi->read_receiver_rssi_uint8();
        _uav.rssi = (rssiv * 99) / 255;                                                                                               
    } else _uav.rssi = 0;
    
    _uav.armstat = AP_Notify::flags.armed;
    _uav.failsafe = AP_Notify::flags.failsafe_radio;
}

// get data for A frame
void AP_LTM::get_attitude(void){	
	_uav.pitch = (uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & PITCH_LIMIT;
	_uav.roll = (uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & ROLL_LIMIT;
	_uav.heading = (uint16_t)roundf((_ahrs.yaw_sensor / 100.0)) & YAW_LIMIT;
}	

// gets called by scheduler
void AP_LTM::tick(void){
	uint32_t now = AP_HAL::millis();
    if (now - _last_frame_ms >= 50) {
        _last_frame_ms = now;
        send_LTM();
    }	
}

