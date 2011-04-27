
void writePoints()
{
    int mem;
	struct Location loc;


    for (byte i = 0; i < waypoint_total; i++){

		loc.id 			= (uint8_t) mission[i][0];
		loc.options 	= (uint8_t) mission[i][1];
		loc.p1 			= (uint8_t) mission[i][2];
	    loc.alt 		= (long)(mission[i][3] * 100);
		loc.lat 		= (long)(mission[i][4] * t7);
    	loc.lng 		= (long)(mission[i][5] * t7);

		switch(loc.id){
			case MAV_CMD_NAV_WAYPOINT:
    			//loc.p1 = (byte)mission[i][4];// wp_radius
    			//loc.p1 = WP_RADIUS;
    			break;

			case MAV_CMD_CONDITION_YAW:
		    	loc.alt = (long)mission[i][3];	// speed
			    loc.lat = (long)mission[i][4];	// rotation direction
    			loc.lng = (long)mission[i][5];	// target yaw in deg
    			break;
		}

		set_wp_with_index(loc,(i+1));

		/*
    	Serial.print((i+1),DEC);
    	Serial.print(": ");
    	Serial.print(loc.id,DEC);
    	Serial.print(", ");
    	Serial.print(loc.p1,DEC);
    	Serial.print(", ");
    	Serial.print(loc.alt,DEC);
    	Serial.print(", ");
    	Serial.print(loc.lat,DEC);
    	Serial.print(", ");
    	Serial.println(loc.lng,DEC);
		*/
	}
}


struct Location get_wp_with_index(int i)
{
	struct Location temp;
	long mem;

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i > waypoint_total) {
		temp.id = CMD_BLANK;
	}else{
		// read WP position
		mem = (WP_START_BYTE) + (i * WP_SIZE);
		temp.id = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.options = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.p1 = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.alt = (long)eeprom_read_dword((uint32_t*)mem);	// alt is stored in CM!

		mem += 4;
		temp.lat = (long)eeprom_read_dword((uint32_t*)mem);

		mem += 4;
		temp.lng = (long)eeprom_read_dword((uint32_t*)mem);
	}
	return temp;
}

void set_wp_with_index(struct Location temp, int i)
{
	i = constrain(i, 0, waypoint_total.get());
	uint32_t mem = WP_START_BYTE + (i * WP_SIZE);

	eeprom_write_byte((uint8_t *)	mem, temp.id);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.options);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.p1);

	mem++;
	eeprom_write_dword((uint32_t *)	mem, temp.alt);	// alt is stored in CM!

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lat);

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lng);
}



