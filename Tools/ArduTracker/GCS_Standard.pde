// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

///
/// @file	GCS_Standard.pde
/// @brief	GCS driver for the APM binary protocol
///

#if GCS_PROTOCOL == GCS_PROTOCOL_STANDARD

// constructor
GCS_STANDARD::GCS_STANDARD(BinComm::MessageHandler GCS_MessageHandlers[]) :
	_binComm(GCS_MessageHandlers)
{
}

void
GCS_STANDARD::init(BetterStream *port)
{
	GCS_Class::init(port);
	_binComm.init(port);
}

void
GCS_STANDARD::update()
{
	_binComm.update();
}

void 
GCS_STANDARD::acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2)
{
	_binComm.send_msg_acknowledge(id, sum1, sum2);
	gcs_messages_sent++;
}

void
GCS_STANDARD::send_message(uint8_t id, uint32_t param)
{
	byte mess_buffer[54];
	byte mess_ck_a = 0;
	byte mess_ck_b = 0;
	int tempint;
	int ck;
	long templong;
			
	switch(id) {
	case MSG_MODE_CHANGE:
	case MSG_HEARTBEAT:
		_binComm.send_msg_heartbeat(control_mode,			// current control mode
									millis() / 1000,		// seconds since power-up
									battery_voltage1 * 100,	// battery voltage * 100
									command_must_index);	// command index (waypoint #)
		break;
		
	case MSG_ATTITUDE:
		_binComm.send_msg_attitude(roll_sensor,
								   pitch_sensor,
								   yaw_sensor);
		break;
		
	case MSG_LOCATION:
		_binComm.send_msg_location(current_loc.lat,
								   current_loc.lng,
								   GPS.altitude / 100,
								   GPS.ground_speed,
								   yaw_sensor,
								   GPS.time);
		break;
		
	case MSG_PRESSURE:
		_binComm.send_msg_pressure(current_loc.alt / 10,
								   airspeed / 100);
		break;
		
	case MSG_PERF_REPORT:
		_binComm.send_msg_perf_report(millis() - perf_mon_timer,
									  mainLoop_count,
									  G_Dt_max & 0xff,
									  gyro_sat_count,
									  adc_constraints,
									  renorm_sqrt_count,
									  renorm_blowup_count,
									  gps_fix_count,
									  imu_health * 100,
									  gcs_messages_sent);
		break;
	
	case MSG_CPU_LOAD:
		//TODO: Implement appropriate BinComm routine here
		
	case MSG_VALUE:
		switch(param) {
			//case 0x00:		templong = roll_mode;				break;
			//case 0x01:		templong = pitch_mode;				break;
			//case 0x02:		templong = throttle_mode;			break;
		case 0x03:		templong = yaw_mode;				break;
		case 0x04:		templong = elevon1_trim;			break;
		case 0x05:		templong = elevon2_trim;			break;
		/* TODO: implement for new PID lib
		case 0x10:		templong = integrator[0] * 1000;	break;
		case 0x11:		templong = integrator[1] * 1000;	break;
		case 0x12:		templong = integrator[2] * 1000;	break;
		case 0x13:		templong = integrator[3] * 1000;	break;
		case 0x14:		templong = integrator[4] * 1000;	break;
		case 0x15:		templong = integrator[5] * 1000;	break;
		case 0x16:		templong = integrator[6] * 1000;	break;
		case 0x17:		templong = integrator[7] * 1000;	break;
		*/
		case 0x1a:		templong = kff[0];					break;
		case 0x1b:		templong = kff[1];					break;
		case 0x1c:		templong = kff[2];					break;
		case 0x20:		templong = target_bearing;			break;
		case 0x21:		templong = nav_bearing;				break;
		case 0x22:		templong = bearing_error;			break;
		case 0x23:		templong = crosstrack_bearing;		break;
		case 0x24:		templong = crosstrack_error;		break;
		case 0x25:		templong = altitude_error;			break;
		case 0x26:		templong = wp_radius;				break;
		case 0x27:		templong = loiter_radius;			break;
			//			case 0x28:		templong = wp_mode;					break;
			//			case 0x29:		templong = loop_commands;			break;
		case 0x2a:		templong = nav_gain_scaler;			break;
		}
		_binComm.send_msg_value(param,
								templong);
		break;
		
	case MSG_COMMAND_LIST:
		tell_command = get_wp_with_index((int)param);
		_binComm.send_msg_command_list(param,
									   wp_total,
									   tell_command.id,
									   tell_command.p1,
									   tell_command.alt,
									   tell_command.lat,
									   tell_command.lng);
		break;

	case MSG_TRIM_STARTUP:
		_binComm.send_msg_trim_startup(radio_trim);
		break;
		
	case MSG_TRIM_MIN:
		_binComm.send_msg_trim_min(radio_min);
		break;
		
	case MSG_TRIM_MAX:
		_binComm.send_msg_trim_max(radio_max);
		break;
		
		/* TODO: implement for new PID lib
	case MSG_PID:								// PID Gains message
		_binComm.send_msg_pid(param,
							  kp[param] * 1000000,
							  ki[param] * 1000000,
							  kd[param] * 1000000,
							  integrator_max[param]);
		break;
		*/

        case MSG_SERVO_OUT:
                _binComm.send_msg_servo_out(servo_out);
                break;
                
        case MSG_RADIO_OUT:
                _binComm.send_msg_radio_out(radio_out);
                break;      
                
        default:
                GCS.send_text(SEVERITY_LOW,"<send_message> unknown message ID");
	}
	gcs_messages_sent++;
}

void 
GCS_STANDARD::send_text(byte severity, const char *str)
{
	if (severity >= DEBUG_LEVEL) {
		char	text[50];		// XXX magic numbers

		strncpy(text, str, 50);
		_binComm.send_msg_status_text(severity, text);
		gcs_messages_sent++;
	}
}

void
receive_message(void * arg, uint8_t id, uint8_t messageVersion, void * messageData)
{
  // process command
  switch(id) {
    
  case MSG_STATUS_TEXT:
      {
          char str[50];
          uint8_t severity;
          GCS.getBinComm().unpack_msg_status_text(severity,str);
          SendDebug(str);
          SendDebug(" severity: "); SendDebugln(severity);
      }
      break;
    
  case MSG_VERSION_REQUEST:
      break;
      
  case MSG_VALUE_REQUEST:
      break;
      
  case MSG_VALUE_SET:
      break;
      
  case MSG_PID_REQUEST:
      break;
      
  case MSG_PID_SET:
      break;
      
  case MSG_EEPROM_REQUEST:
      break;
      
  case MSG_EEPROM_SET:
      break;
      
  case MSG_PIN_REQUEST:
      break;
      
  case MSG_PIN_SET:
      break;
      
  case MSG_DATAFLASH_REQUEST:
      break;
      
  case MSG_DATAFLASH_SET:
      break;
      
  case MSG_COMMAND_REQUEST:
      uint16_t commandIndex;
      GCS.getBinComm().unpack_msg_command_request(commandIndex);
      tell_command = get_wp_with_index(commandIndex);
      GCS.getBinComm().send_msg_command_list(commandIndex,uint16_t(wp_total),tell_command.id,
        tell_command.p1,tell_command.alt,tell_command.lat,tell_command.lng);
      break;
      
  case MSG_COMMAND_UPLOAD:
      uint8_t action; // 0 -insert in list, 1- execute immediately
      uint16_t itemNumber; // item number ( i.e. waypoint number)
      uint16_t listLength; // list length
      struct Location temp;
      GCS.getBinComm().unpack_msg_command_upload(action,itemNumber,listLength,temp.id,temp.p1,temp.alt,temp.lat,temp.lng);
      wp_total=listLength;
      if (action == 0) // insert in list
      {
        // save waypoint total to eeprom at start of the list
        if (itemNumber == 1) save_EEPROM_waypoint_info();
        set_wp_with_index(temp, itemNumber);
      }
      else if (action == 1)
      {
        next_command = temp;
        process_now();
      }
      break;
      
  case MSG_ACKNOWLEDGE:
      break;
      
  default:
      {
          char str[50];
          sprintf(str,"<receive_message> unknown messageID:%x",id);
          GCS.send_text(SEVERITY_LOW,str);
      }
  }
}

#endif // GCS_PROTOCOL_STANDARD
