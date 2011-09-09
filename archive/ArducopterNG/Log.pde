/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Log.pde
 Version  : v1.0, Aug 27, 2010
 Author(s): ArduCopter Team
             Ted Carancho (aeroquad), Jose Julio, Jordi Muñoz,
             Jani Hirvinen, Ken McEwans, Roberto Navoni,          
             Sandro Benigno, Chris Anderson
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.

* ************************************************************** *
ChangeLog:


* ************************************************************** *
TODO:


* ************************************************************** */


// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#define HEAD_BYTE1 0xA3    // Decimal 163
#define HEAD_BYTE2 0x95    // Decimal 149
#define END_BYTE 0xBA      // Decimal 186

#define LOG_ATTITUDE_MSG		0x01
#define LOG_GPS_MSG			0x02
#define LOG_MODE_MSG			0X03
#define LOG_CONTROL_TUNING_MSG	        0X04
#define LOG_NAV_TUNING_MSG		0X05
#define LOG_PERFORMANCE_MSG		0X06
#define LOG_RAW_MSG			0x07
#define LOG_RADIO_MSG                   0x08
#define LOG_SENSOR_MSG                  0x09
#define LOG_PID_MSG                     0x0A
#define LOG_RANGEFINDER_MSG             0x0B

#define LOG_MAX_ERRORS                  50   // when reading logs, give up after 50 sequential failures to find HEADBYTE1

void Log_Erase(void)
{
    SerPri("Erasing logs...");
    for(int i=0; i<4000; i++ )
        DataFlash.PageErase(i);
    SerPrln("Done!");
}


//  Function to display available logs and solicit action from the user.
//  --------------------------------------------------------------------
void Print_Log_Menu(void)
{
	int log_start;
	int log_end;
	eeprom_busy_wait();
	byte last_log_num = eeprom_read_byte((uint8_t *) EE_LAST_LOG_NUM);			eeprom_busy_wait();	
	if (last_log_num == 0) {
		SerPriln("No logs available for download");
	} else {
		SerPri(last_log_num,DEC);
		SerPriln(" logs available for download");
		for(int i=1;i<last_log_num+1;i++) {
			log_start = eeprom_read_word((uint16_t *) (EE_LOG_1_START+(i-1)*0x02));			eeprom_busy_wait();	
			log_end = eeprom_read_word((uint16_t *) (EE_LOG_1_START+(i)*0x02))-1;			eeprom_busy_wait();	
			if (i == last_log_num) {	
				log_end = eeprom_read_word((uint16_t *) EE_LAST_LOG_PAGE);			eeprom_busy_wait();	
			}	
			SerPri("Log number ");
			SerPri(i);
			SerPri(",    start page ");
			SerPri(log_start);
			SerPri(",   end page ");
			SerPriln(log_end);
		}
	}
	SerPriln(" ");
	SerPriln("Input 1 <enter> to dump a log");
	SerPriln("Input 2 <enter> to erase all logs");
	SerPriln("Input 3 <enter> to leave log mode");
	SerPriln(" ");
}
	
	
//  Function to get desired action from the user.
//  ---------------------------------------------
byte Get_User_Log_Command(void)
{

	byte step=0;
	byte char_1;
	byte char_2;
	byte char_3;
	
	Serial.flush();
	
	while(step<3)
	{
		if(Serial.available()){
			char_1 = Serial.read();
			char_2 = Serial.read();
			char_3 = Serial.read();

			if (char_1<0x30 || char_1>0x39) {
				return 0;
			} else if (char_2 == 0xFF) {
				return (char_1 - 0x30);
  			} else if (char_2<0x30 || char_2>0x39) {
				return 0;
			} else if (char_2 != 0xFF) {
				return 0;
			} else {
				return ((char_1 - 0x30)*10 + char_2 - 0x30);
			}

		} else {
			delay(20);
		}
	}
}
	


// Write an attitude packet. Total length : 10 bytes
#if LOG_ATTITUDE
void Log_Write_Attitude(int log_roll, int log_pitch, int log_yaw)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_ATTITUDE_MSG);
	DataFlash.WriteInt(log_roll);
	DataFlash.WriteInt(log_pitch);
	DataFlash.WriteInt(log_yaw);
	DataFlash.WriteByte(END_BYTE);
}
#endif


#ifdef LOG_SEN
// Write a Sensor Raw data packet
void Log_Write_Sensor(int s1, int s2, int s3,int s4, int s5, int s6, int s7)
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteByte(LOG_SENSOR_MSG);
  DataFlash.WriteInt(s1);
  DataFlash.WriteInt(s2);
  DataFlash.WriteInt(s3);
  DataFlash.WriteInt(s4);
  DataFlash.WriteInt(s5);
  DataFlash.WriteInt(s6);
  DataFlash.WriteInt(s7);
  DataFlash.WriteByte(END_BYTE);
}
#endif

// Write a PID control info
void Log_Write_PID(byte num_PID, int P, int I,int D, int output)
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteByte(LOG_PID_MSG);
  DataFlash.WriteByte(num_PID);
  DataFlash.WriteInt(P);
  DataFlash.WriteInt(I);
  DataFlash.WriteInt(D);
  DataFlash.WriteInt(output);
  DataFlash.WriteByte(END_BYTE);
}

// Write a Radio packet
void Log_Write_Radio(int ch1, int ch2, int ch3,int ch4, int ch5, int ch6)
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteByte(LOG_RADIO_MSG);
  DataFlash.WriteInt(ch1);
  DataFlash.WriteInt(ch2);
  DataFlash.WriteInt(ch3);
  DataFlash.WriteInt(ch4);
  DataFlash.WriteInt(ch5);
  DataFlash.WriteInt(ch6);
  DataFlash.WriteByte(END_BYTE);
}

#if LOG_PM
// Write a performance monitoring packet. Total length : 19 bytes
void Log_Write_Performance()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_PERFORMANCE_MSG);
	DataFlash.WriteLong(millis() - perf_mon_timer);
	DataFlash.WriteInt(mainLoop_count);
	DataFlash.WriteInt(G_Dt_max);
	DataFlash.WriteByte(gyro_sat_count);
	DataFlash.WriteByte(adc_constraints);
	DataFlash.WriteByte(renorm_sqrt_count);
	DataFlash.WriteByte(renorm_blowup_count);
	DataFlash.WriteByte(gps_fix_count);
	DataFlash.WriteInt((int)(imu_health*1000));
	DataFlash.WriteByte(END_BYTE);
}
#endif

#if LOG_CTUN
// Write a control tuning packet. Total length : 22 bytes
void Log_Write_Control_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_CONTROL_TUNING_MSG);
	DataFlash.WriteInt((int)(servo_out[CH_ROLL]*100));
	DataFlash.WriteInt((int)nav_roll);
	DataFlash.WriteInt((int)roll_sensor);
	DataFlash.WriteInt((int)(servo_out[CH_PITCH]*100));
	DataFlash.WriteInt((int)nav_pitch);
	DataFlash.WriteInt((int)pitch_sensor);
	DataFlash.WriteInt((int)(servo_out[CH_THROTTLE]*100));
	DataFlash.WriteInt((int)(servo_out[CH_RUDDER]*100));
	DataFlash.WriteInt((int)AN[5]);
	DataFlash.WriteByte(END_BYTE);
}
#endif

#if LOG_NTUN
// Write a navigation tuning packet. Total length : 18 bytes
void Log_Write_Nav_Tuning()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_NAV_TUNING_MSG);
	DataFlash.WriteInt((int)yaw_sensor);
	DataFlash.WriteInt((int)wp_distance);
	DataFlash.WriteInt((int)target_bearing);
	DataFlash.WriteInt((int)nav_bearing);
	DataFlash.WriteInt(altitude_error);
	DataFlash.WriteInt((int)airspeed_error);
	DataFlash.WriteInt((int)(nav_gain_scaler*1000));
	DataFlash.WriteByte(END_BYTE);
}
#endif

#if LOG_MODE
// Write a mode packet. Total length : 5 bytes
void Log_Write_Mode(byte mode)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_MODE_MSG);
	DataFlash.WriteByte(mode);
	DataFlash.WriteByte(END_BYTE);
}
#endif

#if LOG_GPS
// Write an GPS packet. Total length : 30 bytes
void Log_Write_GPS(	long log_Time, long log_Lattitude, long log_Longitude, long log_mix_alt, long log_gps_alt, 
					long log_Ground_Speed, long log_Ground_Course, byte log_Fix, byte log_NumSats)
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_GPS_MSG);
	DataFlash.WriteLong(log_Time);
	DataFlash.WriteByte(log_Fix);
	DataFlash.WriteByte(log_NumSats);
	DataFlash.WriteLong(log_Lattitude);
	DataFlash.WriteLong(log_Longitude);
	DataFlash.WriteLong(log_mix_alt);
	DataFlash.WriteLong(log_gps_alt);
	DataFlash.WriteLong(log_Ground_Speed);
	DataFlash.WriteLong(log_Ground_Course);
	DataFlash.WriteByte(END_BYTE);
}
#endif

#if LOG_RAW
// Write an raw accel/gyro data packet. Total length : 28 bytes
void Log_Write_Raw()
{
	DataFlash.WriteByte(HEAD_BYTE1);
	DataFlash.WriteByte(HEAD_BYTE2);
	DataFlash.WriteByte(LOG_RAW_MSG);
	for(int i=0;i<6;i++) 
		DataFlash.WriteLong((long)(AN[i]*1000.0));
	DataFlash.WriteByte(END_BYTE);
}
#endif

#if LOG_RANGEFINDER
// Write a Sensor Raw data packet
void Log_Write_RangeFinder(int rf1, int rf2, int rf3,int rf4, int rf5, int rf6)
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteByte(LOG_RANGEFINDER_MSG);
  DataFlash.WriteInt(rf1);
  DataFlash.WriteInt(rf2);
  DataFlash.WriteInt(rf3);
  DataFlash.WriteInt(rf4);
  DataFlash.WriteInt(rf5);
  DataFlash.WriteInt(rf6);
  DataFlash.WriteByte(END_BYTE);
}
#endif


// Read an control tuning packet
void Log_Read_Control_Tuning()
{	
	float logvar;

	SerPri("CTUN:");
	for (int y=1;y<10;y++) {
		logvar = DataFlash.ReadInt();
		if(y < 9) logvar = logvar/100.f;
		SerPri(logvar);
		SerPri(",");
	}
	SerPriln(" ");
}

// Read a nav tuning packet
void Log_Read_Nav_Tuning()
{	
	float logvar;

	SerPri("NTUN:");
	for (int y=1;y<8;y++) {
		logvar = DataFlash.ReadInt();
		if(y > 6) logvar = logvar/1000.f;
		SerPri(logvar);
		SerPri(",");
	}
	SerPriln(" ");
}

// Read a performance packet
void Log_Read_Performance()
{	
	long pm_time;
	int logvar;

	SerPri("PM:");
	pm_time = DataFlash.ReadLong();
	SerPri(pm_time);
	SerPri(",");
	for (int y=1;y<9;y++) {
		if(y<3 || y>7) logvar = DataFlash.ReadInt();
		else logvar = DataFlash.ReadByte();
		if(y > 7) logvar = logvar/1000.f;
		SerPri(logvar);
		SerPri(",");
	}
	SerPriln(" ");
}

// Read an attitude packet
void Log_Read_Attitude()
{	
	int log_roll;
	int log_pitch;
	int log_yaw;

	log_roll = DataFlash.ReadInt();
	log_pitch = DataFlash.ReadInt();
	log_yaw = DataFlash.ReadInt(); 
	SerPri("ATT:");
	SerPri(log_roll);
	SerPri(",");
	SerPri(log_pitch);
	SerPri(",");
	SerPri(log_yaw);
	SerPriln();
}

// Read a Sensor packet
void Log_Read_Sensor()
{   
    SerPri("SENSOR:");
    SerPri(DataFlash.ReadInt());  SerPri(",");
    SerPri(DataFlash.ReadInt());  SerPri(",");
    SerPri(DataFlash.ReadInt());  SerPri(",");
    SerPri(DataFlash.ReadInt());  SerPri(",");
    SerPri(DataFlash.ReadInt());  SerPri(",");
    SerPri(DataFlash.ReadInt());  SerPri(",");
    SerPri(DataFlash.ReadInt());
    SerPriln();
}

// Read a Sensor raw data packet
void Log_Read_PID()
{  
  SerPri("PID:");
  SerPri((int)DataFlash.ReadByte());  // NUM_PID
  SerPri(",");
  SerPri(DataFlash.ReadInt());  // P
  SerPri(",");
  SerPri(DataFlash.ReadInt());  // I
  SerPri(",");
  SerPri(DataFlash.ReadInt());  // D
  SerPri(",");
  SerPri(DataFlash.ReadInt());  // output
  SerPriln();
}

// Read an Radio packet
void Log_Read_Radio()
{  
  SerPri("RADIO:");
  SerPri(DataFlash.ReadInt());
  SerPri(",");
  SerPri(DataFlash.ReadInt());
  SerPri(",");
  SerPri(DataFlash.ReadInt());
  SerPri(",");
  SerPri(DataFlash.ReadInt());
  SerPri(",");
  SerPri(DataFlash.ReadInt());
  SerPri(",");
  SerPri(DataFlash.ReadInt());
  SerPriln();
}

// Read a mode packet
void Log_Read_Mode()
{	
	byte mode;

	mode = DataFlash.ReadByte();
	SerPri("MOD:");
	switch (mode) {
		case 0:
			SerPriln("Manual mode");
			break;
		case 1:
			SerPriln("Stabilize mode");
			break;
		case 5:
			SerPriln("Fly By Wire A mode");
			break;
		case 6:
			SerPriln("Fly By Wire B mode");
			break;
		case 10:
			SerPriln("AUTO mode");
			break;
		case 11:
			SerPriln("RTL mode");
			break;
		case 12:
			SerPriln("Loiter mode");
			break;
		case 98:
			SerPriln("Air Start Complete");
			break;
		case 99:
			SerPriln("Ground Start Complete");
			break;
			}
}

// Read a GPS packet
void Log_Read_GPS()
{
	long log_Time;
	byte log_Fix;
	byte log_NumSats;
	long log_Lattitude;
	long log_Longitude;
	long log_gps_alt;
	long log_mix_alt;
	float log_Ground_Speed;
	float log_Ground_Course;
 
	log_Time = DataFlash.ReadLong();
	log_Fix = DataFlash.ReadByte();
	log_NumSats = DataFlash.ReadByte();
	log_Lattitude = DataFlash.ReadLong();
	log_Longitude = DataFlash.ReadLong();
	log_mix_alt = DataFlash.ReadLong();
	log_gps_alt = DataFlash.ReadLong();
	log_Ground_Speed = DataFlash.ReadLong();
	log_Ground_Course = DataFlash.ReadLong();

	SerPri("GPS:");
	SerPri(log_Time);
	SerPri(",");
	SerPri((int)log_Fix);
	SerPri(",");
	SerPri((int)log_NumSats);
	SerPri(",");
	SerPri(log_Lattitude);
	SerPri(",");
	SerPri(log_Longitude);
	SerPri(",");
	SerPri(log_mix_alt/100.0);
	SerPri(",");
	SerPri(log_gps_alt/100.0);
	SerPri(",");
	SerPri(log_Ground_Speed/100.0f);			
	SerPri(",");
	SerPri(log_Ground_Course/100.0);	 //	This is incorrect!!!!!
	SerPriln();

}

// Read a raw accel/gyro packet
void Log_Read_Raw()
{	
	float logvar;

	SerPri("RAW:");
	for (int y=0;y<6;y++) {
		logvar = DataFlash.ReadLong()/1000.f;
		SerPri(logvar);
		SerPri(",");
	}
	SerPriln(" ");
}

// Read a RangeFinder packet
void Log_Read_RangeFinder()
{
	SerPri("RF:");
        SerPri(DataFlash.ReadInt());
        SerPri(",");
        SerPri(DataFlash.ReadInt());
        SerPri(",");
        SerPri(DataFlash.ReadInt());
        SerPri(",");
        SerPri(DataFlash.ReadInt());
        SerPri(",");
        SerPri(DataFlash.ReadInt());
        SerPri(",");
        SerPri(DataFlash.ReadInt());   
	SerPriln(" ");
}

// Read the DataFlash log memory : Packet Parser
void Log_Read(int start_page, int end_page)
{
	byte data;
	byte log_step=0;
	int packet_count=0; 
	int flash_led = 1;
        int numErrors = 0;

	DataFlash.StartRead(start_page);
	while (DataFlash.GetPage() < end_page && numErrors < LOG_MAX_ERRORS)
	{
		data = DataFlash.ReadByte();
		switch(log_step)		 //This is a state machine to read the packets
		{
			case 0:	
				if(data==HEAD_BYTE1) {	// Head byte 1
					log_step++;
                                        numErrors = 0;
                                }else
                                        numErrors++;
				break; 
			case 1:
				if(data==HEAD_BYTE2)	// Head byte 2
					log_step++;
				break; 
			case 2:
				if(data==LOG_ATTITUDE_MSG){
					Log_Read_Attitude();
					log_step++;

				}else if(data==LOG_GPS_MSG){
					Log_Read_GPS();
					log_step++;
				
				}else if(data==LOG_MODE_MSG){
					Log_Read_Mode();
					log_step++;

				}else if(data==LOG_CONTROL_TUNING_MSG){
					Log_Read_Control_Tuning();
					log_step++;

				}else if(data==LOG_NAV_TUNING_MSG){
					Log_Read_Nav_Tuning();
					log_step++;

				}else if(data==LOG_PERFORMANCE_MSG){
					Log_Read_Performance();
					log_step++;
					
				}else if(data==LOG_RAW_MSG){
					Log_Read_Raw();
					log_step++;

				}else if(data==LOG_RADIO_MSG){
					Log_Read_Radio();
					log_step++;

				}else if(data==LOG_SENSOR_MSG){
					Log_Read_Sensor();
					log_step++;

                                }else if(data==LOG_PID_MSG){
					Log_Read_PID();
					log_step++;

                                }else if(data==LOG_RANGEFINDER_MSG) {
                                        Log_Read_RangeFinder();
                                        log_step++;
                                        
				}else{
					SerPri("Error Reading Packet: ");
					SerPri(packet_count); 
					log_step=0;	 // Restart, we have a problem...
				}
				break;
			case 3:
				if(data==END_BYTE){
					 packet_count++;
				}else{
					SerPri("Error Reading END_BYTE  ");
					SerPriln(data,DEC);
				}
				log_step=0;			// Restart sequence: new packet...
				if(flash_led > 0) {
					digitalWrite(A_LED_PIN,HIGH);
					flash_led++;
					if(flash_led>100) flash_led=-1;
				} else {
					digitalWrite(A_LED_PIN,LOW);
					flash_led--;
					if(flash_led<-100) flash_led=1;
				}
				break;
		}
	}
	SerPri("Number of packets read: ");
	SerPriln(packet_count);
	digitalWrite(A_LED_PIN,HIGH);
}


