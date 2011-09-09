// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//////////////////////////////////////////////////////////////////////
// parameter get/set functions 
//////////////////////////////////////////////////////////////////////

// calculate memory starting location for eeprom
static uint16_t floatMemStart  = 0;
static uint16_t uint8MemStart  = floatMemStart  + PARAM_FLOAT_COUNT  * sizeof(float);
static uint16_t uint16MemStart = uint8MemStart  + PARAM_UINT8_COUNT  * sizeof(uint8_t);
static uint16_t int16MemStart  = uint16MemStart + PARAM_UINT16_COUNT  * sizeof(uint16_t);
static uint16_t uint32MemStart = int16MemStart  + PARAM_INT16_COUNT * sizeof(int16_t);

// read parameters
static uint8_t get(uint8_param_t id) __attribute__((noinline));
static uint8_t get(uint8_param_t id)
{
	return eeprom_read_byte((uint8_t *)((id-PARAM_FIRST_UINT8)*sizeof(uint8_t)+uint8MemStart));
}

static uint16_t get(uint16_param_t id) __attribute__((noinline));
static uint16_t get(uint16_param_t id)
{
	return eeprom_read_word((uint16_t *)((id-PARAM_FIRST_UINT16)*sizeof(uint16_t)+uint16MemStart));
}

static int16_t get(int16_param_t id) __attribute__((noinline));
static int16_t get(int16_param_t id)
{
	return (int16_t)eeprom_read_word((uint16_t *)((id-PARAM_FIRST_INT16)*sizeof(int16_t)+int16MemStart));
}

static float get(float_param_t id) __attribute__((noinline));
static float get(float_param_t id)
{
	float value;
	eeprom_read_block((void*)&value,(const void*)((id-PARAM_FIRST_FLOAT)*sizeof(float)+floatMemStart),sizeof(value));
	return value;
}

static float get(uint32_param_t id) __attribute__((noinline));
static float get(uint32_param_t id)
{
	uint32_t value;
	eeprom_read_block((void*)&value,(const void*)((id-PARAM_FIRST_UINT32)*sizeof(uint32_t)+uint32MemStart),sizeof(value));
	return value;
}



// write parameters
static void set(uint8_param_t id, uint8_t val) __attribute__((noinline));
static void set(uint8_param_t id, uint8_t val)
{
	eeprom_write_byte((uint8_t *)((id-PARAM_FIRST_UINT8)*sizeof(uint8_t)+uint8MemStart),val);
}

static void set(uint16_param_t id, uint16_t val) __attribute__((noinline));
static void set(uint16_param_t id, uint16_t val)
{
	eeprom_write_word((uint16_t *)((id-PARAM_FIRST_UINT16)*sizeof(uint16_t)+uint16MemStart),val);
}

static void set(int16_param_t id, int16_t val) __attribute__((noinline));
static void set(int16_param_t id, int16_t val)
{
	eeprom_write_word((uint16_t *)((id-PARAM_FIRST_INT16)*sizeof(int16_t)+int16MemStart),(uint16_t)val);
}

static void set(float_param_t id, float val) __attribute__((noinline));
static void set(float_param_t id, float val)
{
	eeprom_write_block((const void *)&val,(void *)((id-PARAM_FIRST_FLOAT)*sizeof(float)+floatMemStart),sizeof(val));
}

static void set(uint32_param_t id, uint32_t val) __attribute__((noinline));
static void set(uint32_param_t id, uint32_t val)
{
	eeprom_write_block((const void *)&val,(void *)((id-PARAM_FIRST_UINT32)*sizeof(uint32_t)+uint32MemStart),sizeof(val));
}

static void setParamAsFloat(uint16_t id, float value)
{
	if (id < PARAM_FIRST_UINT8) set((float_param_t)id,value);
	else if (id < PARAM_FIRST_UINT16) set((uint8_param_t)id,(uint8_t)value);
	else if (id < PARAM_FIRST_INT16) set((uint16_param_t)id,(uint16_t)value);
	else if (id < PARAM_FIRST_UINT32) set((int16_param_t)id,(int16_t)value);
	else if (id < PARAM_COUNT) set((uint32_param_t)id,(uint32_t)value);
	// XXX: uint32 won't have full bitwise precision
}

static float getParamAsFloat(uint16_t id)
{
	// name
	if (id < PARAM_FIRST_UINT8) return (float)get((float_param_t)id);
	else if (id < PARAM_FIRST_UINT16) return (float)get((uint8_param_t)id);
	else if (id < PARAM_FIRST_INT16) return (float)get((uint16_param_t)id);
	else if (id < PARAM_FIRST_UINT32) return (float)get((int16_param_t)id);
	else if (id < PARAM_COUNT) return (float)get((uint32_param_t)id);
	// XXX: uint32 won't have full bitwise precision
}

static const prog_char *getParamName(uint16_t id)
{
	return (const prog_char *)pgm_read_word(&param_nametab[id]);
}

global_struct::global_struct() :
	// parameters
	// note, all values not explicitly initialised here are zeroed
	param_count(PARAM_COUNT),
	waypoint_send_timeout(1000), // 1 second
	waypoint_receive_timeout(1000), // 1 second

	// stream rates
	streamRateRawSensors(1),
	streamRateExtendedStatus(1),
	streamRateRCChannels(1),
	streamRateRawController(1),
	streamRateRawSensorFusion(1),
	streamRatePosition(1),
	streamRateExtra1(1),
	streamRateExtra2(1),
	streamRateExtra3(1)
{
}

// Print
static void printParam(BetterStream & serial, uint16_t id) 
{
 serial.printf_P(PSTR("id %d %S, %f addr %d\n"), id, getParamName(id), getParamAsFloat(id), getAddress(id));
}

static void printAllParams(BetterStream & serial) 
{
	for (int i=0;i<global_data.param_count;i++)
		printParam(serial,i);
}

static uint16_t getAddress(int id)
{
	if (id < PARAM_FIRST_UINT8)		return (id - PARAM_FIRST_FLOAT)  * sizeof(float)    + floatMemStart;
	if (id < PARAM_FIRST_UINT16)	return (id - PARAM_FIRST_UINT8)  * sizeof(uint8_t)  + uint8MemStart;
	if (id < PARAM_FIRST_INT16)		return (id - PARAM_FIRST_UINT16) * sizeof(uint16_t) + uint16MemStart;
	if (id < PARAM_FIRST_UINT32)	return (id - PARAM_FIRST_INT16)  * sizeof(int16_t)  + int16MemStart;
	/*if (id < PARAM_COUNT)*/		return (id - PARAM_FIRST_UINT32) * sizeof(uint32_t) + uint32MemStart;
}

// Array interfaces
static uint8_t flight_mode(int i) 
{
	return get(uint8_param_t(PARAM_FLIGHT_MODE_1+i));
}

static void set_flight_mode(int i, uint8_t value) 
{
	set(uint8_param_t(PARAM_FLIGHT_MODE_1+i),value);
}

static uint16_t radio_min(int i) 
{
	return get(uint16_param_t(PARAM_RADIOMIN_CH1+i));
}

static void set_radio_min(int i, uint16_t value) 
{
	set(uint16_param_t(PARAM_RADIOMIN_CH1+i),value);
}

static uint16_t radio_max(int i) 
{
	return get(uint16_param_t(PARAM_RADIOMAX_CH1+i));
}

static void set_radio_max(int i, uint16_t value) 
{
	set(uint16_param_t(PARAM_RADIOMAX_CH1+i),value);
}

static uint16_t radio_trim(int i) 
{
	return get(uint16_param_t(PARAM_RADIOTRIM_CH1+i));
}

static void set_radio_trim(int i, uint16_t value) 
{
	set(uint16_param_t(PARAM_RADIOTRIM_CH1+i),value);
}
