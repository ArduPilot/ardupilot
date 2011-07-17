/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if 0

#define INPUT_BUF_LEN 40
char input_buffer[INPUT_BUF_LEN];

static void readCommands(void)
{
	static byte header[2];
	const byte read_GS_header[] 	= {0x21, 0x21}; //!! Used to verify the payload msg header

	if(Serial.available()){
		//Serial.println("Serial.available");
		byte bufferPointer;

		header[0] = Serial.read();
		header[1] = Serial.read();

		if((header[0] == read_GS_header[0]) && (header[1] == read_GS_header[1])){

			// Block until we read full command
			// --------------------------------
			delay(20);
			byte incoming_val = 0;

			// Ground Station communication
			// ----------------------------
			while(Serial.available() > 0)
			{
				incoming_val = Serial.read();

				if (incoming_val != 13 && incoming_val != 10 ) {
					input_buffer[bufferPointer++] = incoming_val;
				}

				if(bufferPointer >= INPUT_BUF_LEN){
					Serial.println("Big buffer overrun");
					bufferPointer = 0;
					input_buffer[0] = 1;
					Serial.flush();
					memset(input_buffer,0,sizeof(input_buffer));
					return;
				}
			}
			parseCommand(input_buffer);

			// clear buffer of old data
			// ------------------------
			memset(input_buffer,0,sizeof(input_buffer));

		}else{
			Serial.flush();
		}
	}
}

// Commands can be sent as !!a:100|b:200|c:1
// -----------------------------------------
static void parseCommand(char *buffer)
{
	Serial.println("got cmd ");
	Serial.println(buffer);
	char *token, *saveptr1, *saveptr2;

	for (int j = 1;; j++, buffer = NULL) {
		token = strtok_r(buffer, "|", &saveptr1);
		if (token == NULL) break;

		char * cmd 		= strtok_r(token, ":", &saveptr2);
		long value		= strtol(strtok_r (NULL,":", &saveptr2), NULL,0);

		///*
		Serial.print("cmd ");
		Serial.print(cmd[0]);
		Serial.print("\tval ");
		Serial.println(value);
		Serial.println("");
		//*/
		///*
		switch(cmd[0]){
			case 'P':
				g.pid_stabilize_roll.kP((float)value / 1000);
				g.pid_stabilize_pitch.kP((float)value / 1000);
				g.pid_stabilize_pitch.save_gains();
				break;

			case 'I':
				g.pid_stabilize_roll.kI((float)value / 1000);
				g.pid_stabilize_pitch.kI((float)value / 1000);
				g.pid_stabilize_pitch.save_gains();
				break;

			case 'D':
				//g.pid_stabilize_roll.kD((float)value / 1000);
				//g.pid_stabilize_pitch.kD((float)value / 1000);
				break;

			case 'X':
				g.pid_stabilize_roll.imax(value * 100);
				g.pid_stabilize_pitch.imax(value * 100);
				g.pid_stabilize_pitch.save_gains();
				break;

			case 'R':
				//g.stabilize_dampener.set_and_save((float)value / 1000);
				break;
		}
		//*/
	}
}

#endif
