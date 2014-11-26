/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <AP_HAL.h>
#include "AP_OSD_MAX7456.h"
#include <AP_Math.h>


extern const AP_HAL::HAL& hal;


/******* MAX7456 DATASHEET BEGIN*******/
#define NTSC 0
#define PAL 1
#define MAX7456_MODE_MASK_PAL 0x40 //PAL mask 01000000
#define MAX7456_CENTER_PAL 0x8

#define MAX7456_MODE_MASK_NTCS 0x00 //NTSC mask 00000000 ("|" will do nothing)
#define MAX7456_CENTER_NTSC 0x6

//MAX7456 reg read addresses
#define MAX7456_OSDBL_reg_read 0xec //black level
#define MAX7456_STAT_reg_read  0xa0 //0xa[X] Status

//MAX7456 reg write addresses
#define MAX7456_VM0_reg   0x00
#define MAX7456_VM1_reg   0x01
#define MAX7456_DMM_reg   0x04
#define MAX7456_DMAH_reg  0x05
#define MAX7456_DMAL_reg  0x06
#define MAX7456_DMDI_reg  0x07
#define MAX7456_OSDM_reg  0x0c //not used. Is to set mix
#define MAX7456_OSDBL_reg 0x6c //black level

//MAX7456 reg write addresses to recording NVM process
#define MAX7456_CMM_reg   0x08
#define MAX7456_CMAH_reg  0x09
#define MAX7456_CMAL_reg  0x0a
#define MAX7456_CMDI_reg  0x0b
#define MAX7456_CMDO_reg  0xc0

//DMM commands
#define MAX7456_CLEAR_display 0x04
#define MAX7456_CLEAR_display_vert 0x06

#define MAX7456_INCREMENT_auto 0x03
#define MAX7456_SETBG_local 0x20 //00100000 force local BG to defined brightness level VM1[6:4]

#define MAX7456_END_string 0xff

//VM0 commands mixed with mode NTSC or PAL mode
#define MAX7456_ENABLE_display_vert 0x0c //mask with NTSC/PAL
#define MAX7456_RESET 0x02 //mask with NTSC/PAL
#define MAX7456_DISABLE_display 0x00 //mask with NTSC/PAL

//VM0 command modifiers
#define MAX7456_SYNC_autosync 0x10
#define MAX7456_SYNC_internal 0x30
#define MAX7456_SYNC_external 0x20
//VM1 command modifiers
#define MAX7456_WHITE_level_80 0x03
#define MAX7456_WHITE_level_90 0x02
#define MAX7456_WHITE_level_100 0x01
#define MAX7456_WHITE_level_120 0x00

#define NVM_ram_size 0x36
#define WRITE_nvr 0xa0
#define STATUS_reg_nvr_busy 0x20

//#define isPAL

////If PAL
//#ifdef isPAL
//#define MAX7456_screen_size 480 //16x30
//#define MAX7456_screen_rows 15
//#else
//#define MAX7456_screen_size 390 //13x30
//#define MAX7456_screen_rows 12
//#endif

/******* MAX7456 DATASHEET END*******/

const AP_Param::GroupInfo AP_OSD_MAX7456::var_info[] PROGMEM = {

	AP_GROUPINFO("SPEED", 0, AP_OSD_MAX7456, _bEnableSpeed, 1),

	AP_GROUPINFO("ALTITUDE", 1, AP_OSD_MAX7456, _bEnableAlt, 1),

	AP_GROUPINFO("THROTTLE", 2, AP_OSD_MAX7456, _bEnableThrottle, 1),

	AP_GROUPINFO("PITCH", 3, AP_OSD_MAX7456, _bEnablePitch, 1),

	AP_GROUPINFO("ROLL", 4, AP_OSD_MAX7456, _bEnableRoll, 1),

	AP_GROUPINFO("HOME", 5, AP_OSD_MAX7456, _bEnableHome, 1),

	AP_GROUPINFO("MODE", 6, AP_OSD_MAX7456, _bEnableMode, 1),

	AP_GROUPINFO("TIME", 7, AP_OSD_MAX7456, _bEnableTime, 1),

	AP_GROUPINFO("HORIZON", 8, AP_OSD_MAX7456, _bEnableHorizon, 1),

	AP_GROUPINFO("GPS_SATS", 9, AP_OSD_MAX7456, _bEnableGPSSats, 1),

	AP_GROUPINFO("GPS_COORD", 10, AP_OSD_MAX7456, _bEnableGPSCoord, 1),

	AP_GROUPINFO("BATT_VOL", 11, AP_OSD_MAX7456, _bEnableBattVol, 1),

	AP_GROUPINFO("BATT_CUR", 12, AP_OSD_MAX7456, _bEnableBattCur, 1),

	AP_GROUPINFO("BATT_PER", 13, AP_OSD_MAX7456, _bEnableBattPercent, 1),

	AP_GROUPINFO("WP", 14, AP_OSD_MAX7456, _bEnableWP, 1),

	AP_GROUPINFO("HEAD", 15, AP_OSD_MAX7456, _bEnableHead, 1),
	AP_GROUPINFO("HEAD_ROSE", 16, AP_OSD_MAX7456, _bEnableHeadRose, 1),

	AP_GROUPINFO("VIDEO_MODE", 17, AP_OSD_MAX7456, _iMode, 1),
	AP_GROUPINFO("RSSI", 18, AP_OSD_MAX7456, _iEnableRSSI, 1),
	AP_GROUPINFO("BATT_CON", 19, AP_OSD_MAX7456, _iEnableCurConsume, 1),

	AP_GROUPEND
};

AP_OSD_MAX7456::AP_OSD_MAX7456()
:_spi(NULL),
 _spi_sem(NULL),
 _osd_vars(NULL),
 _startTime(0)
{
	AP_Param::setup_object_defaults(this, var_info);
	
	//default
	_video_mode = MAX7456_MODE_MASK_PAL;
}

// SPI should be initialized externally
bool AP_OSD_MAX7456::init()
{
	_spi = hal.spi->device(AP_HAL::SPIDevice_MAX7456);
	_spi_sem = _spi->get_semaphore();
	_osd_vars = new AP_OSD_Vars();

	//TTTest
	//read_one_char_from_NVM(1);

	if (!_spi_sem->take(100)){
		hal.scheduler->panic(PSTR("PANIC: AP_OSD_MAX7456: failed to take "
			"serial semaphore for init"));
		return false; /* never reached */
	}


	// set mode. we will do auto bottom-align, because NTCS mode only has 12 rows.
	if(_iMode ==0)
	{
		_video_mode = MAX7456_MODE_MASK_NTCS;
	}
	else
	{
		_video_mode = MAX7456_MODE_MASK_PAL;
		_osd_vars->_panGPSSats_XY[1] = 11;
		_osd_vars->_panGPSCoord_XY[1] = 12;
		
		_osd_vars->_panBatteryVol_XY[1] = 10;
		_osd_vars->_panBatteryCurrent_XY[1] = 11;
		_osd_vars->_panBatteryConsume_XY[1] = 12;
		_osd_vars->_panBatteryPercent_XY[1] = 13;
	}

	_spi->cs_assert();
	{
		//read black level register
		_spi->transfer(MAX7456_OSDBL_reg_read);
		uint8_t osdbl_r = _spi->transfer(0xff);

		_spi->transfer(MAX7456_VM0_reg);
		_spi->transfer(MAX7456_RESET | _video_mode);
		hal.scheduler->delay(50);

		//set black level
		uint8_t osdbl_w = (osdbl_r & 0xef);	//Set bit 4 to zero 11101111
		_spi->transfer(MAX7456_OSDBL_reg); //black level write register
		_spi->transfer(osdbl_w);

		// set all rows to same charactor white level, 90%
		// no matter what the video mode, we just assume the max rows is 15(PAL mode)
		for (uint8_t x = 0; x < 15; x++)
		{
			_spi->transfer(x + 0x10);
			_spi->transfer(MAX7456_WHITE_level_120);
		}
	}
	_spi->cs_release();

	_spi->cs_assert();
	{
		// define sync (auto,int,ext) and making sure the Max7456 is enabled
		_spi->transfer(MAX7456_VM0_reg);
		_spi->transfer((MAX7456_ENABLE_display_vert | _video_mode) | MAX7456_SYNC_autosync);
	}
	_spi->cs_release();

	// now that we have initialised, we set the SPI bus speed to high
	// (8MHz on APM2)
	//_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

	_spi_sem->give();

	for(_HorizonHitIndex=0;_HorizonHitIndex < 12; _HorizonHitIndex++)
	{
		_lastHorizonColHit[_HorizonHitIndex] = _osd_vars->_panHorizon_XY[0] + 1;
		_lastHorizonRowHit[_HorizonHitIndex] = _osd_vars->_panHorizon_XY[1];
	}
	
	clear();

	_startTime = hal.scheduler->millis();
	_lastUpdate10HZ = _startTime;
	_lastUpdate3HZ = _startTime;
	_lastUpdate1HZ = _startTime;

	return true;
}

// clear the screen
void AP_OSD_MAX7456::clear()
{
	if (!_spi_sem->take(10)){
		hal.console->printf_P(PSTR("TTTest - AP_OSD_MAX7456::clear() can not get sem\n"));
		return ;
	}

	_spi->cs_assert();
	{
		_spi->transfer(MAX7456_DMM_reg);
		_spi->transfer(MAX7456_CLEAR_display);
	}
	_spi->cs_release();

	//hal.scheduler->delay(50);

	setPanel(_osd_vars->_panHorizon_XY[0], _osd_vars->_panHorizon_XY[1]);
	openPanel();
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB|"));
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB|"));
	printf_P(PSTR("\xD8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xD9|"));
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB|"));
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB"));
	closePanel();

	_spi_sem->give();
}

void AP_OSD_MAX7456::setPanel(uint8_t st_col, uint8_t st_row)
{
	start_col = st_col;
	start_row = st_row;
	col = st_col;
	row = st_row;
}

void AP_OSD_MAX7456::openPanel(void)
{
	unsigned int linepos;
	uint8_t settings, char_address_hi, char_address_lo;

	//find [start address] position
	linepos = row*30+col;

	// divide 16 bits into hi & lo uint8_t
	char_address_hi = linepos >> 8;
	char_address_lo = linepos;

	//Auto increment turn writing fast (less SPI commands).
	//No need to set next char address. Just send them
	settings = MAX7456_INCREMENT_auto; //To Enable DMM Auto Increment
	
	_spi->cs_assert();

	_spi->transfer(MAX7456_DMM_reg); //dmm
	_spi->transfer(settings);

	_spi->transfer(MAX7456_DMAH_reg); // set start address high
	_spi->transfer(char_address_hi);

	_spi->transfer(MAX7456_DMAL_reg); // set start address low
	_spi->transfer(char_address_lo);
}

void AP_OSD_MAX7456::closePanel(void){  
	_spi->transfer(MAX7456_DMDI_reg);
	_spi->transfer(MAX7456_END_string); //This is needed "trick" to finish auto increment
	
	_spi->cs_release();

	row++; //only after finish the auto increment the new row will really act as desired
}

void AP_OSD_MAX7456::openSingle(uint8_t x, uint8_t y){
	unsigned int linepos;
	uint8_t char_address_hi, char_address_lo;

	//find [start address] position
	linepos = y*30+x;

	// divide 16 bits into hi & lo uint8_t
	char_address_hi = linepos >> 8;
	char_address_lo = linepos;

	_spi->cs_assert();

	_spi->transfer(MAX7456_DMAH_reg); // set start address high
	_spi->transfer(char_address_hi);

	_spi->transfer(MAX7456_DMAL_reg); // set start address low
	_spi->transfer(char_address_lo);
}

size_t AP_OSD_MAX7456::write(uint8_t c)
{
	if(c == '|'){
		closePanel(); //It does all needed to finish auto increment and change current row
		openPanel(); //It does all needed to re-enable auto increment
	}
	else{
		_spi->transfer(MAX7456_DMDI_reg);
		_spi->transfer(c);
	}
	return 1;
}

void AP_OSD_MAX7456::updateScreen()
{
	uint32_t nowTime = hal.scheduler->millis();
	
	if((nowTime - _lastUpdate10HZ) > 100)
	{
		showAt10HZ();
		_lastUpdate10HZ = nowTime;
	}

	if((nowTime - _lastUpdate3HZ) > 330)
	{
		showAt3HZ();
		_lastUpdate3HZ = nowTime;
	}

	if((nowTime - _lastUpdate1HZ) > 1000)
	{
		showAt1HZ();
		_lastUpdate1HZ = nowTime;
	}
}

void AP_OSD_MAX7456::showAt10HZ()
{
	// SPI select max7456 
	if (!_spi_sem->take_nonblocking()){
		hal.console->printf_P(PSTR("TTTest - AP_OSD_MAX7456::showAt10HZ() can not get sem\n"));
		return ;
	}

	// heading degree
	if(_bEnableHead)
	{
		setPanel(12, 5);
		openPanel();
		printf("%4.0f%c", (double)_osd_vars->_heading, 0xb0);
		closePanel();
	}

	//heading direction
	static char buf_show[12];
	static const char buf_Rule[36] = { 0xf2,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0xf4,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0xf3,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0xf5,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0};
	if(_bEnableHeadRose)
	{
		int8_t start;
		start = round((_osd_vars->_heading * 36)/360);
		start -= 5;
		if(start < 0) start += 36;
		for(int8_t x=0; x <= 10; x++){
			buf_show[x] = buf_Rule[start];
			if(++start > 35) start = 0;
		}
		buf_show[11] = '\0';
		setPanel(8, 3);
		openPanel();
		printf("%s|%c%s%c", "\x20\xf0\xf0\xf0\xf0\xf0\xf7\xf0\xf0\xf0\xf0\xf0\x20", 0xf8, buf_show, 0xf9);
		closePanel();
	}

	// pitch, size 1 x 6
	// -+ value of current Pitch from vehicle with degree symbols and pitch symbol
	if(_bEnablePitch)
	{
		setPanel(_osd_vars->_panPitch_XY[0], _osd_vars->_panPitch_XY[1]);
		openPanel();
		printf("%4i%c%c",_osd_vars->_pitch,0xb0,0xb1);
		closePanel();
	}

	// roll, size 1 x 6
	// -+ value of current Roll from vehicle with degree symbols and roll symbol
	if(_bEnableRoll)
	{
		setPanel(_osd_vars->_panRoll_XY[0], _osd_vars->_panRoll_XY[1]);
		openPanel();
		printf("%4i%c%c",_osd_vars->_roll, 0xb0, 0xb2);
		closePanel();
	}

	if(_bEnableHorizon)
	{
		showHorizon(_osd_vars->_panHorizon_XY[0] + 1, _osd_vars->_panHorizon_XY[1]);
		_spi->cs_release();
	}

	//SPI release
	_spi_sem->give();
}


void AP_OSD_MAX7456::showAt3HZ()
{
	// SPI select max7456 
	if (!_spi_sem->take_nonblocking()){
		hal.console->printf_P(PSTR("TTTest - AP_OSD_MAX7456::showAt3HZ() can not get sem\n"));
		return ;
	}

	// velocity, size 1 x 7
	if(_bEnableSpeed)
	{
		setPanel(_osd_vars->_panSpeed_XY[0], _osd_vars->_panSpeed_XY[1]);
		openPanel();
		printf("%c%c%3.0f%c%c",0xBC, 0xBD, (double)(_osd_vars->_groundSpeed * 3.6), 0xD1, 0xD2);
		closePanel();
	}

	// throttle, size 1 x 7
	if(_bEnableThrottle)
	{
		setPanel(_osd_vars->_panThrottle_XY[0], _osd_vars->_panThrottle_XY[1]);
		openPanel();
		printf("%c%c%3.0i%c",0xBE, 0xBF, _osd_vars->_throttle, 0x25);
		closePanel();
	}

	// altitude, size 1 x 7
	if(_bEnableAlt)
	{
		setPanel(_osd_vars->_panVehicleAlt_XY[0], _osd_vars->_panVehicleAlt_XY[1]);
		openPanel();
		printf("%c%c%4.0f%c",0xC0, 0xBD, (double)_osd_vars->_altitude, 0x8D);
		closePanel();
	}

	// number of locked satellites, size 1 x 5
	if(_bEnableGPSSats)
	{
		setPanel(_osd_vars->_panGPSSats_XY[0], _osd_vars->_panGPSSats_XY[1]);
		openPanel();
		printf("%c%2i", 0x0f, _osd_vars->_GPSSats);
		closePanel();
	}
	
	// GPS Longitude and Latitude, size 2 x 12
	if(_bEnableGPSCoord)
	{
		setPanel(_osd_vars->_panGPSCoord_XY[0], _osd_vars->_panGPSCoord_XY[1]);
		openPanel();
		printf("%c%c%11.6f|%c%c%11.6f", 0xC1, 0xBD, (double)_osd_vars->_GPSLongitudePrint*1000, 0xC2, 0xBD, (double)_osd_vars->_GPSLatitudePrint*1000);
		closePanel();
	}


	//SPI release
	_spi_sem->give();
}

void AP_OSD_MAX7456::showAt1HZ()
{
	// SPI select max7456 
	if (!_spi_sem->take_nonblocking()){
		hal.console->printf_P(PSTR("TTTest - AP_OSD_MAX7456::showAt1HZ() can not get sem\n"));
		return ;
	}

	if(_bEnableHome)
	{
		// home direction, size 1 x 5
		setPanel(_osd_vars->_panHomeDir_XY[0], _osd_vars->_panHomeDir_XY[1]);
		openPanel();
		uint8_t home_bearing = round(((float)_osd_vars->_homeDirection - _osd_vars->_heading)/360.0 * 16.0) + 1; //Convert to int 0-16 
		if(home_bearing < 0 ) home_bearing += 16; //normalize
		showArrow((uint8_t)home_bearing,0);
		closePanel();

		// home distance, size 1 x 5
		setPanel(_osd_vars->_panHomeDist_XY[0], _osd_vars->_panHomeDist_XY[1]);
		openPanel();
		printf("%5.0f%c",(double)(_osd_vars->_homeDistance*0.01f), 0x8D);
		closePanel();
	}

	// flight mode 
	// TODO - ugly? fixme!
	setPanel(_osd_vars->_panMode_XY[0], _osd_vars->_panMode_XY[1]);
	openPanel();
	char* mode_str = "";
	if (_osd_vars->_flyMode == 0)       mode_str = "stab"; //Stabilize: hold level position
	else if (_osd_vars->_flyMode == 1)  mode_str = "acro"; //Acrobatic: rate control
	else if (_osd_vars->_flyMode == 2)  mode_str = "alth"; //Altitude Hold: auto control
	else if (_osd_vars->_flyMode == 3)  mode_str = "auto"; //Auto: auto control
	else if (_osd_vars->_flyMode == 4)  mode_str = "guid"; //Guided: auto control
	else if (_osd_vars->_flyMode == 5)  mode_str = "loit"; //Loiter: hold a single location
	else if (_osd_vars->_flyMode == 6)  mode_str = "retl"; //Return to Launch: auto control
	else if (_osd_vars->_flyMode == 7)  mode_str = "circ"; //Circle: auto control
	else if (_osd_vars->_flyMode == 8)  mode_str = "posi"; //Position: auto control
	else if (_osd_vars->_flyMode == 9)  mode_str = "land"; //Land:: auto control
	else if (_osd_vars->_flyMode == 10) mode_str = "oflo"; //OF_Loiter: hold a single location using optical flow sensor
	else if (_osd_vars->_flyMode == 11) mode_str = "drif"; //Drift mode: 
	else if (_osd_vars->_flyMode == 13) mode_str = "sprt"; //Sport: earth frame rate control
	else if (_osd_vars->_flyMode == 14) mode_str = "flip"; //Flip: flip the vehicle on the roll axis
	else if (_osd_vars->_flyMode == 15) mode_str = "atun"; //Auto Tune: autotune the vehicle's roll and pitch gains
	else if (_osd_vars->_flyMode == 16) mode_str = "hybr"; //Hybrid: position hold with manual override
	printf("%c%c%c%s", 0xC7, 0xC8, 0x20, mode_str);
	closePanel();

	//  flight time from start
	if(_bEnableTime)
	{
		setPanel(_osd_vars->_panTime_XY[0], _osd_vars->_panTime_XY[1]);
		openPanel();
		_osd_vars->_startTime = hal.scheduler->millis()*0.001f;
		printf("%c%2i%c%02i", 0xB3, ((int)_osd_vars->_startTime/60)%60,0x3A,(int)_osd_vars->_startTime%60);
		closePanel();
	}

	// Total battery current consume since start up in amp/h
	if(_iEnableCurConsume)
	{
		setPanel(_osd_vars->_panBatteryConsume_XY[0], _osd_vars->_panBatteryConsume_XY[1]);
		openPanel();
		printf("%c%c%5i%c%c%c", 0xeb, 0xcb, (int)_osd_vars->_BatteryConsum, 0xb6, 0xb7, 0xc9);
		closePanel();
	}

	// battery voltage, size 1 x 8
	if(_bEnableBattVol)
	{
		setPanel(_osd_vars->_panBatteryVol_XY[0], _osd_vars->_panBatteryVol_XY[1]);
		openPanel();
		printf("%c%c%5.2f%c", 0xCB, 0xCC, (double)_osd_vars->_BatteryVol, 0x8e);
		closePanel();
	}

	// battery current, size 1 x 8
	if(_bEnableBattCur)
	{
		setPanel(_osd_vars->_panBatteryCurrent_XY[0], _osd_vars->_panBatteryCurrent_XY[1]);
		openPanel();
		printf("%c%c%5.2f%c", 0xCB, 0xCD, (float(_osd_vars->_BatteryCurrent) * .01), 0x8F);
		closePanel();
	}

	// battery percent, size 1 x 8
	if(_bEnableBattPercent)
	{
		setPanel(_osd_vars->_panBatteryPercent_XY[0], _osd_vars->_panBatteryPercent_XY[1]);
		openPanel();
		printf("%c%c%3.0i%c", 0xCB, 0xCE, _osd_vars->_BatteryPercent, 0x25);
		closePanel();
	}

	if(_bEnableWP)
	{
		// waypoint direction, size 1 x 5
		setPanel(_osd_vars->_panWPDir_XY[0], _osd_vars->_panWPDir_XY[1]);
		openPanel();
		uint8_t wp_target_bearing = round(((float)_osd_vars->_WPDirection - _osd_vars->_heading)/360.0 * 16.0) + 1; //Convert to int 0-16 
		if(wp_target_bearing < 0 ) wp_target_bearing += 16; //normalize
		showArrow((uint8_t)wp_target_bearing,1);
		closePanel();

		// waypoint distance, size 1 x 5
		setPanel(_osd_vars->_panWPDist_XY[0], _osd_vars->_panWPDist_XY[1]);
		openPanel();
		printf("%5.0f%c",(double)(_osd_vars->_WPDistance*0.01f), 0x8D);
		closePanel();
	}

	//SPI release
	_spi_sem->give();
}

void AP_OSD_MAX7456::showArrow(uint8_t rotate_arrow, uint8_t mode) 
{ 
	char arrow_set1 = 0x90;
	char arrow_set2 = 0x91;
	if((1< rotate_arrow) && (rotate_arrow < 17))
	{
		arrow_set1 = 0x90+(uint8_t)((rotate_arrow-1)*2);
		arrow_set2 = arrow_set1+0x1;
	}

	//char arrow_set1 = 0x0;
	//char arrow_set2 = 0x0;

	//switch(rotate_arrow) {
	//case 0: 
	//	arrow_set1 = 0x90;
	//	arrow_set2 = 0x91;
	//	break;
	//case 1: 
	//	arrow_set1 = 0x90;
	//	arrow_set2 = 0x91;
	//	break;
	//case 2: 
	//	arrow_set1 = 0x92;
	//	arrow_set2 = 0x93;
	//	break;
	//case 3: 
	//	arrow_set1 = 0x94;
	//	arrow_set2 = 0x95;
	//	break;
	//case 4: 
	//	arrow_set1 = 0x96;
	//	arrow_set2 = 0x97;
	//	break;
	//case 5: 
	//	arrow_set1 = 0x98;
	//	arrow_set2 = 0x99;
	//	break;
	//case 6: 
	//	arrow_set1 = 0x9A;
	//	arrow_set2 = 0x9B;
	//	break;
	//case 7: 
	//	arrow_set1 = 0x9C;
	//	arrow_set2 = 0x9D;
	//	break;
	//case 8: 
	//	arrow_set1 = 0x9E;
	//	arrow_set2 = 0x9F;
	//	break;
	//case 9: 
	//	arrow_set1 = 0xA0;
	//	arrow_set2 = 0xA1;
	//	break;
	//case 10: 
	//	arrow_set1 = 0xA2;
	//	arrow_set2 = 0xA3;
	//	break;
	//case 11: 
	//	arrow_set1 = 0xA4;
	//	arrow_set2 = 0xA5;
	//	break;
	//case 12: 
	//	arrow_set1 = 0xA6;
	//	arrow_set2 = 0xA7;
	//	break;
	//case 13: 
	//	arrow_set1 = 0xA8;
	//	arrow_set2 = 0xA9;
	//	break;
	//case 14: 
	//	arrow_set1 = 0xAA;
	//	arrow_set2 = 0xAB;
	//	break;
	//case 15: 
	//	arrow_set1 = 0xAC;
	//	arrow_set2 = 0xAd;
	//	break;
	//case 16: 
	//	arrow_set1 = 0xAE;
	//	arrow_set2 = 0xAF;
	//	break;
	//} 

	if(mode == 0)		printf("%c%c%c", 0x1F, arrow_set1, arrow_set2);			//home icon
	else if(mode == 1)	printf("%c%c%c%c", 0xCF, 0xD0, arrow_set1, arrow_set2);	//waypoint
	else if(mode == 2)	printf("%c%c", arrow_set1, arrow_set2);	//heading
}

// Calculate and shows Artificial Horizon
void AP_OSD_MAX7456::showHorizon(uint8_t start_col, uint8_t start_row) 
{ 
	int x, nose, row, minval, hit, subval = 0;
	const int cols = 12;
	const int rows = 5;
	int col_hit[cols];
	float  pitch, roll;

	(abs(_osd_vars->_pitch) == 90)?pitch = 89.99 * (90/_osd_vars->_pitch) * -0.017453293:pitch = _osd_vars->_pitch * -0.017453293;
	(abs(_osd_vars->_roll) == 90)?roll = 89.99 * (90/_osd_vars->_roll) * 0.017453293:roll = _osd_vars->_roll * 0.017453293;

	nose = round(tan(pitch) * (rows*9));
	for(int col=1;col <= cols;col++){
		x = (col * 12) - (cols * 6) - 6;//center X point at middle of each col
		col_hit[col-1] = (tan(roll) * x) + nose + (rows*9) - 1;//calculating hit point on Y plus offset to eliminate negative values
		//col_hit[(col-1)] = nose + (rows * 9);
	}
	
	//clear the last display
	for(_HorizonHitIndex=0;_HorizonHitIndex < cols; _HorizonHitIndex++)
	{
		openSingle(_lastHorizonColHit[_HorizonHitIndex], _lastHorizonRowHit[_HorizonHitIndex]);
		printf("%c", 0x20);
	}
	_HorizonHitIndex = 0;
	for(int col=0;col < cols; col++){
		hit = col_hit[col];
		if(hit > 0 && hit < (rows * 18)){
			row = rows - ((hit-1)/18);
			minval = rows*18 - row*18 + 1;
			subval = hit - minval;
			subval = round((subval*9)/18);
			if(subval == 0) subval = 1;
			printHit(start_col + col, start_row + row - 1, subval);
		}
	}
}

void AP_OSD_MAX7456::printHit(uint8_t col, uint8_t row, uint8_t subval)
{
	_lastHorizonColHit[_HorizonHitIndex] = col;
	_lastHorizonRowHit[_HorizonHitIndex] = row;
	_HorizonHitIndex++;

	openSingle(col, row);
	char subval_char = 0x05 + subval;
	//switch (subval){
	//	case 1:
	//		subval_char = 0x06;
	//		break;
	//	case 2:
	//		subval_char = 0x07; 
	//		break;
	//	case 3:
	//		subval_char = 0x08;
	//		break;
	//	case 4:
	//		subval_char = 0x09;
	//		break;
	//	case 5:
	//		subval_char = 0x0a; 
	//		break;
	//	case 6:
	//		subval_char = 0x0b;
	//		break;
	//	case 7:
	//		subval_char = 0x0c;
	//		break;
	//	case 8:
	//		subval_char = 0x0d;
	//		break;
	//	case 9:
	//		subval_char = 0x0e;
	//		break;
	//}
	printf("%c", subval_char);

}

void AP_OSD_MAX7456::write_NVM(uint32_t font_count, uint8_t *character_bitmap)
{
	uint8_t x;
	uint8_t char_address_hi, char_address_lo;
	uint8_t screen_char;

	char_address_hi = font_count;
	char_address_lo = 0;
	  
	if (!_spi_sem->take_nonblocking()) {
		hal.console->printf_P(PSTR("TTTest - AP_OSD_MAX7456::write_NVM() can not get sem\n"));
		return;
	}

	// disable display
	_spi->cs_assert();
	_spi->transfer(MAX7456_VM0_reg); 
	_spi->transfer(MAX7456_DISABLE_display);

	_spi->transfer(MAX7456_CMAH_reg); // set start address high
	_spi->transfer(char_address_hi);

	for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) uint8_ts of character to shadow ram
	{
		screen_char = character_bitmap[x];
		_spi->transfer(MAX7456_CMAL_reg); // set start address low
		_spi->transfer(x);
		_spi->transfer(MAX7456_CMDI_reg);
		_spi->transfer(screen_char);
	}

	// transfer a 54 uint8_ts from shadow ram to NVM
	_spi->transfer(MAX7456_CMM_reg);
	_spi->transfer(WRITE_nvr);

	// wait until bit 5 in the status register returns to 0 (12ms)
	while ((_spi->transfer(MAX7456_STAT_reg_read) & STATUS_reg_nvr_busy) != 0x00);

	_spi->transfer(MAX7456_VM0_reg); // turn on screen next vertical
	_spi->transfer(MAX7456_ENABLE_display_vert);
	_spi->cs_release(); 

	_spi_sem->give();
}


//void AP_OSD_MAX7456::read_one_char_from_NVM(uint32_t font_count)
//{
//	uint8_t x;
//	uint8_t character_bitmap[NVM_ram_size];
//	uint8_t char_address_hi, char_address_lo;
//
//	char_address_hi = font_count;
//	char_address_lo = 0;  
//
//	if (!_spi_sem->take_nonblocking()) {
//		hal.console->printf_P(PSTR("TTTest - AP_OSD_MAX7456::read_one_char_from_NVM() can not get sem\n"));
//		return;
//	}
//
//	// disable display
//	_spi->cs_assert();
//	{
//		_spi->transfer(MAX7456_VM0_reg); 
//		_spi->transfer(MAX7456_DISABLE_display);
//
//		_spi->transfer(MAX7456_CMAH_reg); // set start address high
//		_spi->transfer(char_address_hi);
//
//		_spi->transfer(MAX7456_CMM_reg); // set start address low
//		_spi->transfer(0x50);
//	}
//	_spi->cs_release();
//
//	// wait until bit 5 in the status register returns to 0 (12ms)
//	while ((_spi->transfer(MAX7456_STAT_reg_read) & STATUS_reg_nvr_busy) != 0x00);
//
//	for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) uint8_ts of character to shadow ram
//	{  
//		_spi->cs_assert();
//		{
//			_spi->transfer(MAX7456_CMAL_reg); // set start address low
//			_spi->transfer(x);
//
//			_spi->transfer(MAX7456_CMDO_reg);
//			character_bitmap[x] = _spi->transfer(0xff);
//		}
//		_spi->cs_release();
//	}
//
//	_spi->cs_assert();
//	{
//		_spi->transfer(MAX7456_VM0_reg); // turn on screen next vertical
//		_spi->transfer(MAX7456_ENABLE_display_vert);
//	}
//	_spi->cs_release();
//
//
//	_spi_sem->give();
//
//	//for testing
//	for(x = 0; x < NVM_ram_size; x++)
//	{
//		hal.console->printf_P(PSTR("TTTest - 4-pixel data: %u\n"), character_bitmap[x]);
//	}
//	
//}